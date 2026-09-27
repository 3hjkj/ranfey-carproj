#!/usr/bin/env python3
"""把容器里转发出来的点云重新发布成 ROS2 话题。

为什么需要这个节点
------------------
车机上**不需要它**。车机是 ARM，厂商那套库原生匹配，rfans_driver 直接
publish /rfans_points，rviz2 就能订阅。

但在 PC 上我们是用 qemu 模拟 aarch64 来跑那份 ARM 二进制的，qemu 对组播
socket 选项支持不全，DDS 的 SDP 发现走不通 —— 节点在容器里活得好好的、
话题也在发，外面就是发现不了。

所以让容器里的驱动在帧末（锁外）把点云原样 UDP 单播丢给宿主，由这个节点
在**原生 x86-64 的 ROS2** 里重新发布。宿主这边 DDS 是原生的，rviz2、
ros2 topic hz/echo 全部正常。

用法
----
在**宿主**上直接跑，别用 `ros2 run` —— 本包只在容器的 aarch64 环境里构建过
（厂商库只有 ARM 版），宿主上并没有安装它。这脚本是纯 Python，直接执行即可：

    source /opt/ros/humble/setup.bash
    ./rfans_relay.py                          # 默认听 0.0.0.0:7500，发 /rfans_points
    ./rfans_relay.py --ros-args -p frame_id:=rflink    # 与驱动 launch 对齐

驱动的对应配置（launch 参数）：net_forward: true / net_host / net_port。
PC 上用 launch/pc_relay.launch.py 已经把这三项配好了。
"""

import argparse
import socket
import struct
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField

# 驱动端 TransClound_S 的布局（#pragma pack(1)，40 字节），见 point_types.h。
# PointCloud2 允许自定义 point_step，所以这里一个点都不用转换 —— 收到 UDP
# 直接整块 memcpy 进 msg.data 就行。
POINT_DTYPE = np.dtype([
    ('x',        '<f4'),
    ('y',        '<f4'),
    ('z',        '<f4'),
    ('intent',   '<f4'),
    ('vangle',   '<f4'),
    ('hangle',   '<f4'),
    ('range',    '<f4'),
    ('timeflag', '<f8'),
    ('laserid',  '<i4'),
])
assert POINT_DTYPE.itemsize == 40, POINT_DTYPE.itemsize

# 对外字段名必须和驱动 InitPointcloud2() 里的逐字一致 —— 结构体成员叫 intent /
# timeflag / vangle / hangle，但驱动发布的字段名是 intensity / timestamp /
# v_angle / h_angle。这里若不一致，同一个雷达在"直连"和"经 relay"两条路上
# 会在 rviz2 里显示成不同的字段，排查起来很费劲。
#
# (名字, 结构体成员, PointField 常量)  —— 偏移由 dtype 推出，不手写，改结构体时自动跟上
_FIELD_MAP = [
    ('x',         'x',        PointField.FLOAT32),
    ('y',         'y',        PointField.FLOAT32),
    ('z',         'z',        PointField.FLOAT32),
    ('intensity', 'intent',   PointField.FLOAT32),
    ('v_angle',   'vangle',   PointField.FLOAT32),
    ('h_angle',   'hangle',   PointField.FLOAT32),
    ('range',     'range',    PointField.FLOAT32),
    ('timestamp', 'timeflag', PointField.FLOAT64),
    ('laserid',   'laserid',  PointField.INT32),
]
FIELDS = [PointField(name=pub, offset=POINT_DTYPE.fields[mem][1], datatype=dt, count=1)
          for pub, mem, dt in _FIELD_MAP]
# 偏移必须和驱动那套吻合：x0 y4 z8 intensity12 v_angle16 h_angle20 range24
# timestamp28(8字节) laserid36(4字节) → 合计 40
assert [f.offset for f in FIELDS] == [0, 4, 8, 12, 16, 20, 24, 28, 36], \
    [f.offset for f in FIELDS]

MAGIC = 0x314E4652          # 'RFN1'，与 rfans_driver.cpp 的 netFrame() 对应
HDR = struct.Struct('<IIHHII')      # magic, frame_id, pkt_idx, pkt_total, n_points, reserved
assert HDR.size == 20, HDR.size

# 收到半截帧超过这么久就丢掉：宁可漏一帧，也不让残缺帧一直占着内存
FRAME_TIMEOUT = 2.0


class Frame:
    __slots__ = ('total', 'n_points', 'chunks', 'got', 'first_at')

    def __init__(self, total, n_points, now):
        self.total = total
        self.n_points = n_points
        self.chunks = {}
        self.got = 0
        self.first_at = now


class RfansRelay(Node):
    def __init__(self, port, topic, host):
        super().__init__('rfans_relay')
        # RELIABLE 是刻意选的：RELIABLE 的发布者既能被 RELIABLE 也能被
        # BEST_EFFORT 的订阅者连上，反过来则连不上。rviz2 的 PointCloud2
        # 默认就是 Reliable，用 BEST_EFFORT 发会静默地"订阅不上"，很难查。
        qos = QoSProfile(depth=2,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(PointCloud2, topic, qos)

        # 和驱动一样从 frame_id 参数取，默认 "world"（见 InitPointcloud2）。
        # 车机和 PC 用同一个值，rviz2 里才不用改配置。
        self.frame_id = self.declare_parameter('frame_id', 'world').value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 点云是 10 MB/s 的连续流，缓冲区给小了内核直接丢包，表现为"帧永远收不齐"
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
        self.sock.bind((host, port))
        self.sock.settimeout(0.5)

        self.frames = {}
        self.published = 0
        self.incomplete = 0
        self.bad_packets = 0
        self.bytes_in = 0
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._recv_loop, daemon=True)

        self.get_logger().info(
            f'监听 udp://{host}:{port}，发布到 {topic} '
            f'(SO_RCVBUF={self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)})')
        self.create_timer(5.0, self._report)

    def start(self):
        self.thread.start()

    def shutdown(self):
        self._stop.set()
        self.thread.join(timeout=2.0)

    # ---- 收包线程 ----------------------------------------------------
    def _recv_loop(self):
        while not self._stop.is_set():
            try:
                pkt, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                self._expire()
                continue
            except OSError:
                if self._stop.is_set():
                    break
                raise
            self.bytes_in += len(pkt)
            self._handle(pkt)
            self._expire()

    def _handle(self, pkt):
        if len(pkt) < HDR.size:
            self.bad_packets += 1
            return
        magic, fid, idx, total, n_points, _ = HDR.unpack_from(pkt, 0)
        if magic != MAGIC or total == 0 or idx >= total:
            self.bad_packets += 1
            return

        # frame_id 是驱动那边的递增计数器，回绕就当新帧处理，不做特殊照顾
        # loopback 上同一帧的片是按序到达的，所以见到未知 fid 直接建帧即可。
        # 不做重排：少一片就整帧丢（在 _expire 里超时清掉），不做部分发布。
        fr = self.frames.get(fid)
        if fr is None:
            fr = self.frames[fid] = Frame(total, n_points, time.monotonic())
        if idx not in fr.chunks:
            fr.chunks[idx] = pkt[HDR.size:]
            fr.got += 1

        if fr.got == fr.total:
            del self.frames[fid]
            self._publish(fr)

    def _expire(self):
        now = time.monotonic()
        for fid in [f for f, fr in self.frames.items()
                    if now - fr.first_at > FRAME_TIMEOUT]:
            fr = self.frames.pop(fid)
            self.incomplete += 1
            if self.incomplete % 20 == 1:
                self.get_logger().warn(
                    f'帧 {fid} 只收到 {fr.got}/{fr.total} 片，丢弃（累计 {self.incomplete} 帧）')

    def _publish(self, fr):
        raw = b''.join(fr.chunks[i] for i in range(fr.total))
        want = fr.n_points * POINT_DTYPE.itemsize
        if len(raw) < want:
            self.bad_packets += 1
            return
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = fr.n_points
        msg.fields = FIELDS
        msg.is_bigendian = False
        msg.point_step = POINT_DTYPE.itemsize
        msg.row_step = want
        msg.is_dense = False      # 与驱动一致；返回值里本来就有被滤成 0 的点
        msg.data = raw[:want]
        self.pub.publish(msg)
        self.published += 1

    def _report(self):
        if self.published:
            self.get_logger().info(
                f'已发布 {self.published} 帧   {self.bytes_in/1e6:.1f} MB  '
                f'丢帧 {self.incomplete}  坏包 {self.bad_packets}')
        elif self.bytes_in == 0:
            self.get_logger().warn('还没收到任何数据 —— 驱动那边 net_forward 开了吗？')


def main():
    ap = argparse.ArgumentParser(description='R-Fans-32 点云 UDP → ROS2 中继')
    ap.add_argument('--port', type=int, default=7500, help='监听端口（默认 7500）')
    ap.add_argument('--host', default='0.0.0.0', help='监听地址（默认 0.0.0.0）')
    ap.add_argument('--topic', default='/rfans_points', help='发布的话题名')
    # 必须用 parse_known_args：文档里推荐的 `--ros-args -p frame_id:=rflink`
    # 是给 rclpy 解析的，argparse 不认识，用 parse_args 会直接报错退出。
    # 剩下的那些原样留在 sys.argv 里交给下面的 rclpy.init()。
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = RfansRelay(args.port, args.topic, args.host)
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
