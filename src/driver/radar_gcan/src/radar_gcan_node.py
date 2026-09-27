#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""毫米波雷达驱动节点：广成 GCAN-202 转换机（TCP）-> radar_msgs/SensorData。

数据流：

    雷达 --CAN--> GCAN-202 转换机 --TCP 192.168.5.137:4001--> 本节点
                                                            |
                                              /sensorRawData (radar_msgs/SensorData)
                                                            |
                                    perception 融合 / decision_planning 紧急停车

与 radar_adapter 的关系：radar_adapter 的 sim 模式往**同一个** /sensorRawData 发假
目标，两者不能同时跑。本节点启动时会检查该话题上有没有别的发布者并报错。

坐标系（沿用 radar_adapter 的约定）：车辆系，x 前、y 侧向，单位米；
vx 纵向速度（远离为正）、vy 横向速度。

**量纲**：int.h 只有位布局、没有 factor/offset，解出的是原始计数。必须标定。
未标定时本节点只解码、不发布（见 publish_uncalibrated）。

线程模型：一个接收线程（socket recv 是阻塞的）+ ROS 定时器。线程间只用一把锁
保护槽位字典，临界区里只做 dict 更新和快照拷贝，不做任何 IO。
"""

import json
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from radar_msgs.msg import SensorData
from std_msgs.msg import String

from gcan_protocol import (FRAME_LEN, ID_RADAR_SPD, ID_RADAR_YAW, decode_frame,
                           decode_radar, decode_speed, decode_yaw,
                           is_heartbeat, parse_heartbeat, slot_to_obj_id)

CONNECT_TIMEOUT = 2.0
RECV_TIMEOUT = 0.5          # 保活用：让收线程能定期检查退出标志，关节点时不被 read 卡住
RECONNECT_MIN = 1.0
RECONNECT_MAX = 10.0

# 原始字段 -> 换算系数参数名。只列真正参与发布的四个：a_long/a_lat/width 目前
# 下游 SensorData 里没有对应字段，解出来只进 stats 供诊断，换算它们没有意义。
FACTOR_OF = {
    'l_long': 'factor_long_m',
    'l_lat': 'factor_lat_m',
    'v_long': 'factor_vlong_mps',
    'v_lat': 'factor_vlat_mps',
}

FACTOR_PARAMS = ('factor_long_m', 'factor_lat_m',
                 'factor_vlong_mps', 'factor_vlat_mps')

# 这几个系数没标定就不该发布：x 和 vy 直接参与 decision_planning 的紧急停车判据
# （`x < 12 && |vy| > 0.3`），填错系数的距离值比不发更危险。
REQUIRED_FACTORS = FACTOR_PARAMS

HEARTBEAT_HINT = (
    '只收到心跳、没有 CAN 数据 —— 转换机和网络是好的，问题在 CAN 侧。按顺序查：'
    '1) 转换机 CAN 波特率 vs 雷达波特率（通常 500k）'
    '2) 雷达接的是哪一路（心跳第 5 字节 1=CAN1 / 2=CAN2；'
    '端口 4001=CAN1、9998=CAN2，用 -p port:= 切换）'
    '3) CAN_H/CAN_L 是否接反  4) 120Ω 终端电阻  5) 雷达供电')

MISALIGN_HINT = (
    '收到的帧里有大量非法 DLC —— TCP 字节流可能没对齐在 13 字节边界上，'
    '解出来的目标全是垃圾。检查转换机是不是在连接建立时吐了半截历史缓冲')


class RadarGcan(Node):

    def __init__(self):
        super().__init__('radar_gcan')

        self.declare_parameter('host', '192.168.5.137')
        self.declare_parameter('port', 4001)
        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('stale_s', 0.5)
        self.declare_parameter('publish_uncalibrated', False)
        # 雷达的侧向符号约定仓库里没有文档，装反了会导致左右镜像。
        # 真车对标时如果发现目标左右颠倒，把这个改成 -1.0。
        self.declare_parameter('lateral_sign', 1.0)
        # 雷达在车上的安装位置（米）。装在车头前 0.5 m 就填 0.5。
        self.declare_parameter('offset_x', 0.0)
        self.declare_parameter('offset_y', 0.0)
        # 标定后的合理性门限：超出这个距离的目标直接丢弃并记数。作用是让标定
        # 系数填错一个数量级时立刻显形（比如把厘米当米的系数填成 0.01）。
        # **未标定时不生效**，否则原始计数（12 位，最大 4095）会被全部丢掉。
        self.declare_parameter('max_range_m', 300.0)
        # 每次连上后先丢这么多秒的帧（转换机可能缓存了断线期间的历史数据）
        self.declare_parameter('warmup_s', 0.5)
        # stats 话题周期，0 = 关闭
        self.declare_parameter('stats_period_s', 1.0)
        for p in FACTOR_PARAMS:
            self.declare_parameter(p, 0.0)

        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.stale_s = float(self.get_parameter('stale_s').value)
        self.publish_uncalibrated = bool(
            self.get_parameter('publish_uncalibrated').value)
        self.lateral_sign = float(self.get_parameter('lateral_sign').value)
        self.offset_x = float(self.get_parameter('offset_x').value)
        self.offset_y = float(self.get_parameter('offset_y').value)
        self.max_range_m = float(self.get_parameter('max_range_m').value)
        self.warmup_s = float(self.get_parameter('warmup_s').value)
        self.stats_period_s = float(self.get_parameter('stats_period_s').value)

        self.factors = {p: float(self.get_parameter(p).value)
                        for p in FACTOR_PARAMS}
        self._check_calibration()

        self.pub = self.create_publisher(SensorData, 'sensorRawData', 10)
        self.pub_stats = (self.create_publisher(String, '~/stats', 10)
                          if self.stats_period_s > 0.0 else None)

        # ── 共享状态：收线程写，定时器回调读 ──
        self._lock = threading.Lock()
        # (family, slot) -> dict，见 _store()
        self._slots = {}
        self._n_frames = 0
        self._n_heartbeats = 0
        self._n_bogus_dlc = 0
        self._n_pub = 0
        self._n_dropped_range = 0
        self._unknown = {}        # can_id -> 帧数
        self._connected = False
        self._hb_ch = None
        self._hb_sn = None
        self._t0 = time.monotonic()
        self._t_last_hint = 0.0
        self._t_last_misalign_hint = 0.0
        self._t_last_conflict_check = 0.0
        self._t_warmup_until = 0.0

        self._stop = threading.Event()
        self._rx = threading.Thread(target=self._rx_loop, daemon=True,
                                    name='radar_gcan_rx')
        self._rx.start()

        period = 1.0 / max(0.1, self.publish_hz)
        self.create_timer(period, self._publish)
        if self.pub_stats is not None:
            self.create_timer(self.stats_period_s, self._stats)
        self.get_logger().info(
            'radar_gcan 启动：%s:%d -> sensorRawData @ %.1f Hz'
            % (self.host, self.port, self.publish_hz))

    def _check_calibration(self):
        missing = [p for p in REQUIRED_FACTORS if self.factors[p] == 0.0]
        if not missing:
            return
        if not self.publish_uncalibrated:
            self.get_logger().error(
                '未标定：%s 是 0。节点只解码、**不发布**。原始值会周期打印，'
                '标定后把这些参数填上再启。要在台架上强制跑通链路，'
                '加 publish_uncalibrated:=true（此时原始计数当米/当 m/s 直接发）。'
                % ', '.join(missing))
            return
        # 未标定直通模式下还填了一部分系数 = 一半字段是物理量、一半是原始计数，
        # 混在一起的 x/y 比全不填更难看懂。
        partial = [p for p in REQUIRED_FACTORS if self.factors[p] != 0.0]
        if partial:
            self.get_logger().warning(
                'publish_uncalibrated=true，但 %s 已经填了系数，而 %s 没有 —— '
                '发布出来的 x/y/vx/vy 单位是混的（部分物理量、部分原始计数），'
                '只适合看链路是否通，不要用来判断数值。'
                % (', '.join(partial), ', '.join(missing)))

    # ───────────────────────── 接收侧 ─────────────────────────

    def _rx_loop(self):
        backoff = RECONNECT_MIN
        while not self._stop.is_set() and rclpy.ok():
            try:
                sock = socket.create_connection((self.host, self.port),
                                                timeout=CONNECT_TIMEOUT)
            except OSError as e:
                self.get_logger().warning(
                    '连不上 %s:%d（%s），%.0f 秒后重试'
                    % (self.host, self.port, e, backoff),
                    throttle_duration_sec=5.0)
                self._connected = False
                if self._stop.wait(backoff):
                    return
                backoff = min(backoff * 2.0, RECONNECT_MAX)
                continue

            backoff = RECONNECT_MIN
            self._connected = True
            # 13 字节的小帧，Nagle 攒包 + 对端延时确认会凭空加上几十毫秒
            try:
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError:
                pass
            self.get_logger().info('已连接 %s:%d' % (self.host, self.port))
            sock.settimeout(RECV_TIMEOUT)
            self._t_warmup_until = time.monotonic() + self.warmup_s
            buf = b''
            try:
                while not self._stop.is_set() and rclpy.ok():
                    try:
                        chunk = sock.recv(8192)
                    except socket.timeout:
                        continue
                    if not chunk:
                        self.get_logger().warning('对端关闭连接，准备重连')
                        break
                    buf = self._consume(buf + chunk)
            except OSError as e:
                self.get_logger().warning('接收出错：%s，准备重连' % e)
            finally:
                try:
                    sock.close()
                except OSError:
                    pass
                self._connected = False
                # 断线必须清空槽位，否则重连后的第一帧前会拿断线前的旧值继续发布
                self._reset_slots()

    def _consume(self, buf):
        """按 13 字节切帧。返回没切完的尾巴。

        TCP 不丢字节也不重排，所以这里不需要重新同步 —— 只要起始对齐，
        之后就是死切。真断了连接 buf 会随 sock 一起丢掉重建。
        """
        now = time.monotonic()
        warmup = now < self._t_warmup_until
        while len(buf) >= FRAME_LEN:
            b, buf = buf[:FRAME_LEN], buf[FRAME_LEN:]
            if warmup:
                continue                    # 预热窗口内只对齐、不入槽

            if is_heartbeat(b):
                ch, sn = parse_heartbeat(b)
                self._n_heartbeats += 1
                self._hb_ch, self._hb_sn = ch, sn
                self._maybe_hint()
                continue

            f = decode_frame(b)
            if f.dlc > 8:
                self._n_bogus_dlc += 1      # DLC 不可能大于 8，说明流没对齐

            d = decode_radar(f.can_id, f.data)
            if d is None:
                # 0x130/0x3E9 是整车量不是目标，单独存；其余计入未知上报
                if f.can_id == ID_RADAR_YAW:
                    self._store('YAW', 0, decode_yaw(f.data))
                elif f.can_id == ID_RADAR_SPD:
                    self._store('SPD', 0, decode_speed(f.data))
                else:
                    self._unknown[f.can_id] = self._unknown.get(f.can_id, 0) + 1
                continue

            self._store(d['family'], d['slot'], d)
            self._n_frames += 1
        return buf

    def _store(self, family, slot, fields):
        """把一帧的字段并进它所属的槽位。

        槽位字典的键：
          fields   累积的语义字段（A/B/C 各带一部分，合并后才是完整目标）
          t_a/t_b  A 帧、B 帧各自的到达时刻（monotonic）
          seq      收到过的 A/B 帧计数，用来判断"自上次发布以来有没有新数据"
          pub_seq  上次发布时的 seq
          t        任意帧的最后到达时刻（stats 用）
        """
        if not fields:
            return
        now = time.monotonic()
        with self._lock:
            s = self._slots.setdefault(
                (family, slot),
                {'fields': {}, 't_a': 0.0, 't_b': 0.0,
                 'seq': 0, 'pub_seq': 0, 't': 0.0})
            s['fields'].update(fields)
            s['t'] = now
            frame = fields.get('frame')
            if frame == 'A':
                s['t_a'] = now
                s['seq'] += 1
            elif frame == 'B':
                s['t_b'] = now
                s['seq'] += 1

    def _reset_slots(self):
        with self._lock:
            self._slots.clear()

    def _maybe_hint(self):
        """有心跳、无数据帧 = CAN 侧问题。每 10 秒提醒一次。"""
        now = time.monotonic()
        if self._n_frames or now - self._t_last_hint < 10.0:
            return
        self._t_last_hint = now
        self.get_logger().warning(
            '%s（心跳 %d 条，通道 %s，SN %s）'
            % (HEARTBEAT_HINT, self._n_heartbeats, self._hb_ch, self._hb_sn))

    # ───────────────────────── 发布侧 ─────────────────────────

    def _factor(self, field):
        """取该字段的换算系数。未标定但允许发布时按 1.0（原始计数直通）。"""
        name = FACTOR_OF[field]
        v = self.factors[name]
        if v != 0.0:
            return v
        return 1.0 if self.publish_uncalibrated else 0.0

    def _publish(self):
        if not self._calibrated():
            return

        now = time.monotonic()
        with self._lock:
            snap = [(k, dict(v['fields']), v['t_a'], v['t_b'],
                     v['seq'], v['pub_seq']) for k, v in self._slots.items()]

        now_wall = self.get_clock().now().nanoseconds * 1e-9
        gate_range = self.max_range_m > 0.0 and not self.publish_uncalibrated
        n_dropped = 0

        for (family, slot), f, t_a, t_b, seq, pub_seq in snap:
            if family not in ('RT', 'RTS'):
                continue                    # YAW / SPD 不是目标

            # 只在槽位收到过**新的** A/B 帧时才发。用定时器无条件重发的话，
            # 下游融合每周期都会对同一个观测再做一次卡尔曼更新 —— 同一份测量
            # 被当成多次独立测量，协方差被虚假收紧，滤波器会变得过度自信。
            if seq == pub_seq:
                continue

            # A 帧和 B 帧都必须新鲜。track_id 来自 B 帧（RTS 来自 A 帧，但两族
            # 都有 B），如果 B 帧停了而 A 帧还在更新，我们会一直拿着一个冻结的
            # track_id 发目标。
            if now - t_a > self.stale_s or now - t_b > self.stale_s:
                continue
            if f.get('live', 0) == 0:
                continue                    # LiveCount=0 表示雷达自己认为该目标已丢
            if f.get('track_id', 0) == 0:
                continue
            l_long = f.get('l_long')
            if l_long is None:
                continue

            x = l_long * self._factor('l_long') + self.offset_x
            if x <= 0.0:
                continue                    # 车尾方向/零距离的目标，融合侧用不上
            y = (f.get('l_lat', 0) * self._factor('l_lat')
                 * self.lateral_sign + self.offset_y)

            if gate_range and (x > self.max_range_m
                               or abs(y) > self.max_range_m):
                n_dropped += 1
                continue

            msg = SensorData()
            msg.sensor_type = 1             # 1 = radar
            # 用 A 帧的到达时刻，而不是定时器的当前时刻：观测时间戳应该是
            # "这个测量是什么时候采到的"，不是"我们什么时候决定转发它"。
            msg.timestamp = now_wall - (now - t_a)
            # obj_id 用**物理槽位**编码而非雷达 track_id，理由见
            # gcan_protocol.slot_to_obj_id 的注释（下游 radar_objs 从不清理）
            msg.obj_id = slot_to_obj_id(family, slot)
            msg.x = float(x)
            msg.y = float(y)
            msg.vx = float(f.get('v_long', 0) * self._factor('v_long'))
            msg.vy = float(f.get('v_lat', 0) * self._factor('v_lat')
                           * self.lateral_sign)
            self.pub.publish(msg)

            with self._lock:
                s = self._slots.get((family, slot))
                if s is not None:
                    s['pub_seq'] = seq
            self._n_pub += 1

        if n_dropped:
            self._n_dropped_range += n_dropped
            self.get_logger().warning(
                '%d 个目标超出 max_range_m=%.0f m 被丢弃 —— 换算系数可能填错了一个'
                '数量级' % (n_dropped, self.max_range_m), throttle_duration_sec=5.0)

    def _calibrated(self):
        if self.publish_uncalibrated:
            return True
        return all(self.factors[p] != 0.0 for p in REQUIRED_FACTORS)

    # ───────────────────────── 自省 ─────────────────────────

    def _check_topic_conflict(self):
        """启动时若已有别人在发 /sensorRawData，两条流会交替覆盖同一个话题，
        下游看到的是真假掺半的目标 —— 这种故障很难从数据上认出来，所以要吼。"""
        try:
            infos = self.get_publishers_info_by_topic('sensorRawData')
        except Exception:                   # noqa: BLE001 图查询失败不该拖死节点
            return
        others = {i.node_name for i in infos if i.node_name != self.get_name()}
        if others:
            self.get_logger().error(
                'sensorRawData 上已经有别的发布者：%s。radar_adapter 的 sim 模式会'
                '往同一话题发**假目标**，同时跑的话下游拿到的是真假交替的数据。'
                '请先停掉它。' % ', '.join(sorted(others)))

    def _stats(self):
        now = time.monotonic()
        with self._lock:
            slots = {}
            for (family, slot), v in self._slots.items():
                if family in ('YAW', 'SPD'):
                    key = family
                else:
                    key = '%s%d(obj_id=%d)' % (family, slot,
                                               slot_to_obj_id(family, slot))
                slots[key] = {
                    'age_s': round(now - v['t'], 2),
                    'new': v['seq'] != v['pub_seq'],
                    **{k: val for k, val in v['fields'].items()
                       if k not in ('family', 'slot', 'frame')},
                }
            n_frames = self._n_frames
            n_hb = self._n_heartbeats
            n_bogus = self._n_bogus_dlc
            n_pub = self._n_pub
            n_drop = self._n_dropped_range
            unknown = dict(self._unknown)

        if now - self._t_last_conflict_check > 5.0:
            self._t_last_conflict_check = now
            self._check_topic_conflict()

        dt = max(1e-6, now - self._t0)
        payload = {
            'connected': self._connected,
            'frames': n_frames,
            'fps': round(n_frames / dt, 2),
            'published': n_pub,
            'dropped_range': n_drop,
            'heartbeats': n_hb,
            'bogus_dlc': n_bogus,
            'hb_channel': self._hb_ch, 'hb_sn': self._hb_sn,
            'unknown_ids': {'0x%03X' % k: v for k, v in sorted(unknown.items())},
            'calibrated': self._calibrated(),
            'publish_uncalibrated': self.publish_uncalibrated,
            'slots': slots,
        }
        if self.pub_stats is not None:
            self.pub_stats.publish(
                String(data=json.dumps(payload, ensure_ascii=False)))

        # 流没对齐的话解出来的全是垃圾，比"没数据"更危险 —— 有数据但全是错的
        if n_bogus > 10 and n_bogus > 0.1 * max(1, n_frames + n_bogus):
            if now - self._t_last_misalign_hint > 30.0:
                self._t_last_misalign_hint = now
                self.get_logger().error(
                    '%s（非法 DLC %d 帧）' % (MISALIGN_HINT, n_bogus))

        # 未标定时把原始值打出来，标定就靠它
        if not self._calibrated() and slots:
            self.get_logger().info(
                '未标定，当前原始值：%s'
                % json.dumps(slots, ensure_ascii=False), throttle_duration_sec=5.0)

    def destroy_node(self):
        self._stop.set()
        self._rx.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RadarGcan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
