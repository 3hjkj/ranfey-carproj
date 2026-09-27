#!/usr/bin/env python3
"""把驱动 dump 出来的 xyz 文件回放成 ROS2 点云话题。

为什么需要这个节点
------------------
雷达网线拔了 / 车机坏了 / 手上只有一份录好的 xyz 时，用它把数据灌回
/rfans_points，下游（perception 聚类、rviz2）完全不知道自己吃的是回放，
照常跑。这样调聚类参数、修聚类 bug 就不用每次都守着雷达。

在**宿主**上直接跑，别用 `ros2 run` —— 本包只在容器的 aarch64 环境里构建过
（厂商库只有 ARM 版），宿主上并没有安装它。这脚本是纯 Python，直接执行即可。

    source /opt/ros/humble/setup.bash
    ./rfans_replay.py --file /tmp/person.xyz              # 发 /rfans_points
    ./rfans_replay.py --file /tmp/person.xyz --dry-run    # 只统计，不发

关于 xyz 文件
-------------
列序是 `x, y, z, hangle, range, intent`，见 rfans_driver.cpp 里 dumpFrame()
的 `snprintf("%.6g,%.6g,%.6g,%.6g,%.6g,%.6g\\n", x, y, z, hangle, range, intent)`。
**没有 laserid**，要靠仰角反推：实测 `atan2(z, hypot(x,y))` 与整度数的偏差
max 0.0001°，所以 `laserid = round(仰角) + 20`（0 号环最低，视场 −20°~+11°）。

怎么切圈（这是本脚本最容易写错的地方）
--------------------------------------
点按**时间**排，不是按方位角排，而且解码器是**跨 laserid 交织输出**的，
相邻点的 hangle 会在两组之间来回跳 ±10°~13°。所以直接 `diff(hangle) < -180`
切圈会切错：实测那样切出 66 段，其中 18 段是残圈。

正确做法是**按 laserid 分别解缠**：同一个 laserid 的点在文件里天然按时间排，
它的 hangle 严格单调 mod 360（实测步进 +0.2550°，负步进占比 0.000）。
对 32 个子序列各自解缠，就得到真正的连续转台角，再按 360° 切。

用解缠法实测得到 59 圈，其中 57 圈覆盖满 1408 个方位步（残圈只剩 2 个）。

数据质量的坑（务必知道）
------------------------
录这份 xyz 时 `save_xyz` 走的是逐点 `snprintf`，会把完整帧打散（见
rfans-save-path-critical-section）。实测**每圈的 32 路回波平均少了约 21%**
（满圈 45056 点 = 1408 步 × 32 线，实测中位只有 35730）。
所以拿回放数据去评估聚类，指标会偏悲观 —— 要拿准数得先把 save_xyz 改成
二进制落盘重录一份。本脚本如实回放，不做任何插值补齐。
"""

import argparse
import io
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField

# 驱动端 TransClound_S 的布局（#pragma pack(1)，40 字节），见 point_types.h。
# 与 rfans_relay.py 用的是同一套 —— 回放出来的消息必须和真雷达逐字节同构，
# 否则同一个下游在"回放"和"实车"两条路上行为不一致，调参就没有意义了。
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

# 字段名逐字对齐驱动 InitPointcloud2()：结构体成员叫 intent/timeflag/vangle/hangle，
# 对外字段名是 intensity/timestamp/v_angle/h_angle。
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
assert [f.offset for f in FIELDS] == [0, 4, 8, 12, 16, 20, 24, 28, 36], \
    [f.offset for f in FIELDS]

# 每圈方位步数。实时满帧 las_num = 45056 = 1408 × 32，据此取 1408；
# 单 laserid 实测步进 0.2550°（360/0.2550 ≈ 1412）略有出入，是量化取整，
# 以实时那个 1408 为准。
STEPS_PER_REV = 1408
LINES = 32
FULL_FRAME_POINTS = STEPS_PER_REV * LINES      # 45056
LIDAR_MIN_ELEV = -20.0                          # 视场下沿，也是 laserid 0 的仰角


def wrap180(a):
    """把角度差折进 (-180, 180]，用于 mod 360 的单调解缠。"""
    return (a + 180.0) % 360.0 - 180.0


def read_xyz(path):
    """读 xyz 文本，自动丢掉被 SIGTERM 截断的最后半行。

    save_xyz 是攒够 1MB 才落盘的，进程被 kill 时尾部那不足 1MB 的部分不写，
    文件末行是半行 —— 直接 np.loadtxt 会在这一行上报错。
    """
    with open(path, 'rb') as f:
        blob = f.read()
    cut = blob.rfind(b'\n')
    if cut == -1:
        raise ValueError(f'{path} 里没有完整的一行')
    if cut != len(blob) - 1:
        print(f'  [警告] 末行是半行（{len(blob)-cut-1} 字节），已丢弃')
        blob = blob[:cut + 1]
    return np.loadtxt(io.BytesIO(blob), delimiter=',')


def split_revolutions(data, rps, min_steps):
    """按 laserid 分别解缠，切成整圈。

    返回 (圈列表, 统计 dict)。圈列表里每项是 `(角度数组, 数据下标数组)`，
    角度是解缠后的连续转台角，用它算圈内时间戳。
    """
    x, y, z = data[:, 0], data[:, 1], data[:, 2]
    h = data[:, 3]

    # 仰角反推 laserid：实测与整度数偏差 max 0.0001°，可以放心取整
    elev = np.degrees(np.arctan2(z, np.hypot(x, y)))
    laserid = np.rint(elev - LIDAR_MIN_ELEV).astype(np.int32)
    if laserid.min() < 0 or laserid.max() >= LINES:
        raise ValueError(f'laserid 越界 [{laserid.min()},{laserid.max()}]，'
                         f'这份文件不是 R-Fans-32 的？')

    # 逐 laserid 解缠。子序列天然按时间排，hangle 单调 mod 360，
    # 所以累加 wrap180(diff) 就是真实转角。
    ang = np.empty(len(h), dtype=np.float64)
    for lid in range(LINES):
        idx = np.flatnonzero(laserid == lid)
        if len(idx) == 0:
            continue
        hh = h[idx]
        ang[idx] = hh[0] + np.concatenate(
            ([0.0], np.cumsum(wrap180(np.diff(hh)))))

    # 按 360° 切圈。minshift 消除浮点误差导致的边界抖动
    ang0 = ang.min()
    rev = np.floor((ang - ang0) / 360.0 + 1e-9).astype(np.int64)
    n_rev = int(rev.max()) + 1

    step_idx = np.floor(((ang - ang0) % 360.0) / (360.0 / STEPS_PER_REV)
                        ).astype(np.int32)

    frames = []
    dropped = []
    for k in range(n_rev):
        sel = np.flatnonzero(rev == k)
        nsteps = len(np.unique(step_idx[sel]))
        if nsteps < min_steps:
            dropped.append((k, len(sel), nsteps))
            continue
        frames.append((ang[sel], sel))

    stats = {
        'n_points': len(data),
        'n_rev': n_rev,
        'n_kept': len(frames),
        'dropped': dropped,
        'points_per_frame': [len(s) for _, s in frames],
        'steps_per_frame': [len(np.unique(step_idx[s])) for _, s in frames],
    }
    return frames, laserid, stats


def build_frames(data, frames, laserid, rps, vangle_from_laserid):
    """把每圈打成驱动格式的字节块，发布时直接赋值，不做逐点处理。"""
    x, y, z = data[:, 0], data[:, 1], data[:, 2]
    h, rng, it = data[:, 3], data[:, 4], data[:, 5]

    # v_angle 默认如实回放 0：驱动 rfans_driver.cpp:419 写的是
    #   out_ros.vangle = dec_input->turn_angle;   // 转台角度
    # 而 R-Fans-32 没有转台角度这个量（旋转产生的就是水平方位角，已经在
    # h_angle 里），LaserPoint_S 的构造函数也没初始化 turn_angle，所以实测
    # 全是 0。回放必须跟着是 0，否则"回放"和"实车"两条路行为不一致。
    # 加 --vangle-from-laserid 才会填真实仰角（= laserid − 20，单位度，
    # 与 h_angle 一致），用来验证修掉这个 bug 之后下游有没有变化。
    if vangle_from_laserid:
        vangle = (laserid.astype(np.float32) + LIDAR_MIN_ELEV)
    else:
        vangle = np.zeros(len(x), dtype=np.float32)

    out = []
    us_per_rev = 1e6 / rps
    for k, (ang, sel) in enumerate(frames):
        pts = np.empty(len(sel), dtype=POINT_DTYPE)
        pts['x'] = x[sel]
        pts['y'] = y[sel]
        pts['z'] = z[sel]
        pts['intent'] = it[sel]
        pts['vangle'] = vangle[sel]
        pts['hangle'] = h[sel]
        pts['range'] = rng[sel]
        # timeflag 是微秒。用解缠角算圈内时刻 —— 角度决定时间，
        # 这比按点数均分更贴近真实（丢步的地方不会把时间也拉偏）。
        phase = (ang - ang[0]) / 360.0
        pts['timeflag'] = k * us_per_rev + phase * us_per_rev
        pts['laserid'] = laserid[sel]
        out.append(pts.tobytes())
    return out


class RfansReplay(Node):
    def __init__(self, frames, topic, frame_id, rps, loop):
        super().__init__('rfans_replay')
        # RELIABLE：perception 用 create_subscription(topic, 10, cb) 默认就是
        # RELIABLE，rviz2 也是。用 BEST_EFFORT 发会静默地"订阅不上"。
        qos = QoSProfile(depth=2,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(PointCloud2, topic, qos)
        self.frames = frames
        self.frame_id = frame_id
        self.rps = rps
        self.loop = loop
        self.idx = 0
        self.sent = 0
        self.t0 = time.monotonic()
        self.timer = self.create_timer(1.0 / rps, self._tick)
        self.get_logger().info(
            f'回放到 {topic}：{len(frames)} 圈，{rps} Hz，'
            f'帧长 {len(frames[0])//POINT_DTYPE.itemsize} 点，'
            f'{"循环" if loop else "单次"}，frame_id={frame_id}')

    def _tick(self):
        if self.idx >= len(self.frames):
            if not self.loop:
                # 单次回放：打完最后一帧就把定时器摘掉，进程留着不退 ——
                # 下游（perception/rviz2）通常还在跑，这个节点先退会让它们
                # 跟着进入无数据状态，反而看不清是哪里断的。
                dt = time.monotonic() - self.t0
                self.get_logger().info(
                    f'回放结束：{self.sent} 帧 / {dt:.1f}s '
                    f'（应到 {len(self.frames)} 帧 / '
                    f'{len(self.frames)/self.rps:.1f}s）')
                self.destroy_timer(self.timer)
                self.timer = None
                return
            self.idx = 0
            return
        raw = self.frames[self.idx]
        self.idx += 1
        self.sent += 1

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = len(raw) // POINT_DTYPE.itemsize
        msg.fields = FIELDS
        msg.is_bigendian = False
        msg.point_step = POINT_DTYPE.itemsize
        msg.row_step = len(raw)
        msg.is_dense = False       # 与驱动一致；本来就有被滤成 0 的点
        msg.data = raw
        self.pub.publish(msg)


def main():
    ap = argparse.ArgumentParser(
        description='R-Fans-32 xyz 回放 → ROS2 PointCloud2')
    ap.add_argument('--file', required=True, help='xyz 文件（列序 x,y,z,hangle,range,intent）')
    ap.add_argument('--topic', default='/rfans_points', help='发布的话题名')
    ap.add_argument('--frame-id', default='rflink', help='header.frame_id')
    ap.add_argument('--rps', type=float, default=10.0, help='转/秒，同时决定发布频率')
    ap.add_argument('--min-steps', type=int, default=STEPS_PER_REV - 28,
                    help=f'一圈至少覆盖多少方位步才算数（满 {STEPS_PER_REV}）')
    ap.add_argument('--vangle-from-laserid', action='store_true',
                    help='把 v_angle 填成真实仰角（默认如实回放驱动的 0）')
    ap.add_argument('--loop', action='store_true', help='循环回放')
    ap.add_argument('--dry-run', action='store_true', help='只统计不发布')
    # 必须 parse_known_args：`--ros-args -p xxx:=yyy` 是给 rclpy 解析的，
    # argparse 不认识，用 parse_args 会直接报错退出。
    args, _ = ap.parse_known_args()

    t0 = time.time()
    print(f'读入 {args.file} ...')
    data = read_xyz(args.file)
    print(f'  {len(data)} 点，耗时 {time.time()-t0:.1f}s')

    frames, laserid, stats = split_revolutions(data, args.rps, args.min_steps)
    print(f'\n=== 分圈（按 laserid 解缠）===')
    print(f'  解缠出 {stats["n_rev"]} 圈，保留 {stats["n_kept"]} 圈'
          f'（丢掉 {len(stats["dropped"])} 个残圈）')
    for k, npts, nsteps in stats['dropped']:
        print(f'    丢 圈{k}: {npts} 点，只覆盖 {nsteps}/{STEPS_PER_REV} 步')
    ppp = stats['points_per_frame']
    if ppp:
        print(f'  每圈点数: 中位 {np.median(ppp):.0f}  min {min(ppp)}  max {max(ppp)}'
              f'   （满圈 {FULL_FRAME_POINTS}）')
        print(f'  密度中位数 {np.median(ppp)/FULL_FRAME_POINTS*100:.1f}%'
              f' —— 低于 100% 是 save_xyz 丢点，不是回放的问题')
    print(f'  laserid 分布: {np.bincount(laserid, minlength=LINES)}')

    if args.dry_run or not frames:
        if not frames:
            print('\n没有可回放的整圈，退出。')
        return 1 if not frames else 0

    built = build_frames(data, frames, laserid, args.rps,
                         args.vangle_from_laserid)

    rclpy.init()
    node = RfansReplay(built, args.topic, args.frame_id, args.rps, args.loop)
    node._last_rps = args.rps
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
