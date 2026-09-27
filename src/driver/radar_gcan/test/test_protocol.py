#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gcan_protocol 的合成帧单测。

可以直接跑（不需要 pytest）：

    python3 src/driver/radar_gcan/test/test_protocol.py

也可以被 pytest / colcon test 收集。

重点回归的是几位历史上解错的地方：RTS 族的 A/B 语义与 RT 相反、以及各处
LiveCount / DetectionSenson 的位位置。这些错误不会让程序崩，只会让打印出来的
数字看着"差不多但不对"，所以必须有测试钉住。
"""

import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'src'))

import gcan_protocol as P    # noqa: E402


# ───────────────────── 构造合成帧的小工具 ─────────────────────

def _b12(v):
    """有符号 12 位 -> 12 位无符号。"""
    return v & 0xFFF


def _u12(v):
    return v & 0xFFF


def gcan(can_id, data, dlc=8, ff=False, rtr=False):
    """组成 13 字节的转换机帧。"""
    info = (0x80 if ff else 0) | (0x40 if rtr else 0) | (dlc & 0x0F)
    body = bytes(data).ljust(8, b'\x00')[:8]
    return bytes([info]) + can_id.to_bytes(4, 'big') + body


def rt_a(l_long, v_long, l_lat, v_lat, a_long, sens=0, live=0):
    d = bytearray(8)
    d[0:3] = (_b12(l_long) | (_b12(v_long) << 12)).to_bytes(3, 'little')
    d[3:6] = (_b12(l_lat) | (_b12(v_lat) << 12)).to_bytes(3, 'little')
    d[6] = a_long & 0xFF
    d[7] = ((live & 3) << 6) | ((sens & 3) << 4)
    return bytes(d)


def rt_b(track_id, status=0, a_lat=0, movement=0, live=0):
    d = bytearray(8)
    d[0] = track_id & 0xFF
    d[2] = (status & 0x0F) << 4
    v56 = ((a_lat & 0xFF) << 4) | ((movement & 0x0F) << 12)
    d[5:7] = v56.to_bytes(2, 'little')
    d[7] = (live & 3) << 6
    return bytes(d)


def rt_c(width, obj_class, vis_trk_id=0, live=0):
    d = bytearray(8)
    d[0] = width & 0x1F
    d[3:5] = ((obj_class & 0x0F) << 7).to_bytes(2, 'little')
    d[6:8] = (((vis_trk_id & 0x0F) << 6) | ((live & 3) << 14)).to_bytes(2, 'little')
    return bytes(d)


def rts_a(l_long, l_lat, track_id, status=0, a_long=0, sens=0, live=0):
    d = bytearray(8)
    v = (_u12(l_long) | (_u12(l_lat) << 12) | ((track_id & 0xFF) << 24))
    d[0:4] = v.to_bytes(4, 'little')
    v67 = ((status & 0x0F) | ((a_long & 0xFF) << 4)
           | ((sens & 3) << 12) | ((live & 3) << 14))
    d[6:8] = v67.to_bytes(2, 'little')
    return bytes(d)


def rts_b(v_long, v_lat, a_lat=0, movement=0, live=0):
    d = bytearray(8)
    d[0:4] = (_b12(v_long) | (_b12(v_lat) << 12)).to_bytes(4, 'little')
    d[5:7] = (((a_lat & 0xFF) << 4) | ((movement & 0x0F) << 12)).to_bytes(2, 'little')
    d[7] = (live & 3) << 6
    return bytes(d)


# ───────────────────────── 转换机层 ─────────────────────────

def test_decode_frame_standard_id():
    f = P.decode_frame(gcan(0x760, rt_a(1, 2, 3, 4, 5)))
    assert f.can_id == 0x760
    assert f.ff is False and f.rtr is False
    assert f.dlc == 8 and len(f.data) == 8


def test_decode_frame_masks_extended_bits_on_standard_frame():
    # 标准帧只有低 11 位有效，高位的扩展/错误标志必须被掩掉。
    # 0x80000760 这种 can_id 在 raw socket 上很常见。
    raw = bytes([0x08]) + (0x80000760).to_bytes(4, 'big') + b'\x00' * 8
    f = P.decode_frame(raw)
    assert f.ff is False
    assert f.can_id == 0x760


def test_decode_frame_keeps_extended_id():
    raw = bytes([0x88]) + (0x0C0228D1).to_bytes(4, 'big') + b'\x00' * 8
    f = P.decode_frame(raw)
    assert f.ff is True
    assert f.can_id == 0x0C0228D1


def test_decode_frame_clamps_insane_dlc():
    # 设备偶尔把 DLC 写成大于 8 的垃圾值，不能因此越界
    f = P.decode_frame(bytes([0x0F]) + (0x760).to_bytes(4, 'big') + b'\x11' * 8)
    assert f.dlc == 0x0F
    assert len(f.data) == 8


def test_decode_frame_respects_short_dlc():
    f = P.decode_frame(bytes([0x03]) + (0x760).to_bytes(4, 'big') + b'\x11' * 8)
    assert len(f.data) == 3


def test_heartbeat():
    # 心跳是**整帧 13 字节**，magic 在偏移 0，不是塞在某个正常帧的数据场里
    hb = P.HEARTBEAT_MAGIC + b'1' + b'21102914'
    assert len(hb) == P.FRAME_LEN
    assert P.is_heartbeat(hb) is True
    ch, sn = P.parse_heartbeat(hb)
    assert ch == '1' and sn == '21102914'
    # 第二路通道
    assert P.parse_heartbeat(P.HEARTBEAT_MAGIC + b'2' + b'21102914')[0] == '2'


def test_normal_frame_is_not_mistaken_for_heartbeat():
    """正常帧的数据场如果恰好以 AA 00 FF 00 开头，不能把整帧误判成心跳 ——
    判据是帧首 4 字节，不是数据场首 4 字节。"""
    f = gcan(0x760, b'\xaa\x00\xff\x00' + b'\x11' * 4)
    assert P.is_heartbeat(f) is False
    assert P.decode_frame(f).can_id == 0x760


def test_heartbeat_is_not_a_data_frame():
    # 心跳的前 4 字节 AA00FF00 如果被当成帧信息+ID 解，会得到荒唐的 ID，
    # 所以调用方必须先判心跳再 decode_frame。
    hb = P.HEARTBEAT_MAGIC + b'1' + b'21102914'
    assert P.decode_frame(hb).can_id != 0x760


# ───────────────────────── ID 映射 ─────────────────────────

def test_radar_id_map():
    assert P.RADAR_IDS[0x760] == ('RT', 1, 'A')
    assert P.RADAR_IDS[0x762] == ('RT', 1, 'C')
    assert P.RADAR_IDS[0x765] == ('RT', 2, 'A')
    assert P.RADAR_IDS[0x77B] == ('RT', 6, 'C')
    assert P.RADAR_IDS[0x740] == ('RTS', 1, 'A')
    assert P.RADAR_IDS[0x751] == ('RTS', 4, 'C')
    assert len(P.RADAR_IDS) == 6 * 3 + 4 * 3
    # 0x130 / 0x3E9 不是"目标"ID，不该出现在映射里
    assert P.ID_RADAR_YAW not in P.RADAR_IDS
    assert P.ID_RADAR_SPD not in P.RADAR_IDS


def test_slot_to_obj_id():
    assert [P.slot_to_obj_id('RT', n) for n in range(1, 7)] == [1, 2, 3, 4, 5, 6]
    assert [P.slot_to_obj_id('RTS', n) for n in range(1, 5)] == [7, 8, 9, 10]


# ───────────────────────── 有符号工具 ─────────────────────────

def test_signed_helpers():
    assert P.s12(0x7FF) == 2047          # 上界
    assert P.s12(0x800) == -2048         # 下界
    assert P.s12(0xFFF) == -1
    assert P.s8(0x7F) == 127
    assert P.s8(0x80) == -128
    assert P.s8(0xFF) == -1


# ───────────────────────── RT 族 ─────────────────────────

def test_rt_a_roundtrip():
    f = P.decode_radar(0x760, rt_a(120, -35, -8, 3, -2))
    assert (f['family'], f['slot'], f['frame']) == ('RT', 1, 'A')
    assert f['l_long'] == 120 and f['v_long'] == -35
    assert f['l_lat'] == -8 and f['v_lat'] == 3
    assert f['a_long'] == -2


def test_rt_a_negative_values_do_not_bleed_across_fields():
    # 12 位字段紧邻打包，负数的高位如果没被正确截断会污染隔壁字段
    f = P.decode_radar(0x760, rt_a(-1, -1, -1, -1, -1))
    assert (f['l_long'], f['v_long'], f['l_lat'], f['v_lat']) == (-1, -1, -1, -1)


def test_rt_a_extremes():
    f = P.decode_radar(0x760, rt_a(2047, -2048, 2047, -2048, 127))
    assert f['l_long'] == 2047 and f['v_long'] == -2048
    assert f['l_lat'] == 2047 and f['v_lat'] == -2048
    assert f['a_long'] == 127
    f2 = P.decode_radar(0x760, rt_a(0, 0, 0, 0, -128))
    assert f2['a_long'] == -128


def test_rt_a_live_and_sens_bit_positions():
    """回归：sniffer 把 LiveCount 读成 bit0~1、DetectionSenson 读成 bit2~3。
    按 int.h 的 DATA7 = Reserved:4 | DetectionSenson:2 | LiveCount:2，
    正确位置是 bit4~5 和 bit6~7。"""
    f = P.decode_radar(0x760, rt_a(10, 0, 0, 0, 0, sens=2, live=3))
    assert f['live'] == 3
    assert f['sens'] == 2
    # 具体钉住字节值：3<<6 | 2<<4 = 0xE0
    assert rt_a(10, 0, 0, 0, 0, sens=2, live=3)[7] == 0xE0
    f0 = P.decode_radar(0x760, rt_a(10, 0, 0, 0, 0, sens=0, live=0))
    assert f0['live'] == 0 and f0['sens'] == 0
    f1 = P.decode_radar(0x760, rt_a(10, 0, 0, 0, 0, sens=1, live=1))
    assert f1['live'] == 1 and f1['sens'] == 1


def test_rt_b_fields():
    f = P.decode_radar(0x761, rt_b(42, status=7, a_lat=-3, movement=5, live=2))
    assert (f['family'], f['slot'], f['frame']) == ('RT', 1, 'B')
    assert f['track_id'] == 42
    assert f['status'] == 7
    assert f['a_lat'] == -3
    assert f['movement'] == 5
    assert f['live'] == 2


def test_rt_b_carries_no_position():
    # B 帧没有距离字段，别让 A 帧的值从别处漏进来
    f = P.decode_radar(0x761, rt_b(1, 0, 0, 0, 0))
    assert 'l_long' not in f and 'v_long' not in f


def test_rt_c_fields():
    f = P.decode_radar(0x762, rt_c(width=17, obj_class=5, vis_trk_id=9, live=2))
    assert (f['family'], f['slot'], f['frame']) == ('RT', 1, 'C')
    assert f['width'] == 17
    assert f['obj_class'] == 5
    assert f['vis_trk_id'] == 9
    assert f['live'] == 2


def test_rt_c_live_bit_position():
    """回归：sniffer 把 LiveCount 读成 bytes[6:8] 的 bit4~5，
    按 int.h 的 DATA67 应该是 bit14~15。"""
    assert rt_c(0, 0, 0, live=3)[6:8] == (3 << 14).to_bytes(2, 'little')
    for v in (0, 1, 2, 3):
        assert P.decode_radar(0x762, rt_c(0, 0, 0, live=v))['live'] == v


def test_rt_c_width_is_five_bits():
    # Width 只有 5 位，高 3 位是同字节的 Reserved，不能一起读进来
    f = P.decode_radar(0x762, rt_c(width=31, obj_class=0))
    assert f['width'] == 31
    d = bytearray(rt_c(31, 0))
    d[0] |= 0xE0                     # 把高位 Reserved 全置 1
    assert P.decode_radar(0x762, bytes(d))['width'] == 31


def test_rt_c_obj_class_spans_byte_boundary():
    # ObjectClass 起始于 byte3 的 bit7，跨 3/4 字节边界，最容易解错
    for cls in range(16):
        assert P.decode_radar(0x762, rt_c(0, obj_class=cls))['obj_class'] == cls


# ───────────────────────── RTS 族 ─────────────────────────

def test_rts_a_has_track_id_and_no_velocity():
    """回归（最重要的一条）：RTS 的 A 帧有 TrackID、没有速度；
    RT 的 A 帧有速度、没有 TrackID。sniffer 把两族当同一套解，RTS 全错。"""
    f = P.decode_radar(0x740, rts_a(88, -12, track_id=33, status=4,
                                    a_long=-1, sens=1, live=2))
    assert (f['family'], f['slot'], f['frame']) == ('RTS', 1, 'A')
    assert f['track_id'] == 33
    assert f['l_long'] == 88 and f['l_lat'] == -12
    assert f['status'] == 4 and f['a_long'] == -1
    assert f['sens'] == 1 and f['live'] == 2
    assert 'v_long' not in f and 'v_lat' not in f


def test_rts_b_is_the_velocity_frame():
    f = P.decode_radar(0x741, rts_b(-25, 4, a_lat=-2, movement=1, live=3))
    assert (f['family'], f['slot'], f['frame']) == ('RTS', 1, 'B')
    assert f['v_long'] == -25 and f['v_lat'] == 4
    assert f['a_lat'] == -2 and f['movement'] == 1 and f['live'] == 3
    assert 'track_id' not in f and 'l_long' not in f


def test_rts_c_matches_rt_c():
    a = P.decode_radar(0x762, rt_c(11, 3, 5, 1))
    b = P.decode_radar(0x742, rt_c(11, 3, 5, 1))
    for k in ('width', 'obj_class', 'vis_trk_id', 'live'):
        assert a[k] == b[k]
    assert b['family'] == 'RTS'


def test_rts_slots():
    assert P.decode_radar(0x74F, rts_a(1, 1, 1))['slot'] == 4
    assert P.decode_radar(0x751, rt_c(0, 0))['slot'] == 4


def test_rt_and_rts_do_not_share_ids():
    """RT 的 slot n 和 RTS 的 slot n 是不同的物理目标，槽位必须分得开。"""
    assert P.decode_radar(0x760, rt_a(1, 0, 0, 0, 0))['slot'] == 1
    assert P.decode_radar(0x740, rts_a(1, 0, 1))['slot'] == 1
    assert (P.slot_to_obj_id('RT', 1) != P.slot_to_obj_id('RTS', 1))


# ───────────────────────── 其它 ID ─────────────────────────

def test_yaw_and_speed():
    d = bytearray(8)
    d[0:2] = (1234).to_bytes(2, 'little')
    d[2] = 0x20                       # bit5 = 有效位
    y = P.decode_yaw(bytes(d))
    assert y['yaw_raw'] == 1234 and y['yaw_valid'] == 1

    d2 = bytearray(8)
    d2[0:2] = (0x1234 | (1 << 15)).to_bytes(2, 'little')
    s = P.decode_speed(bytes(d2))
    assert s['speed_raw'] == 0x1234 and s['speed_valid'] == 1


# ───────────────────────── 异常输入 ─────────────────────────

def test_unknown_id_returns_none():
    assert P.decode_radar(0x123, rt_a(1, 2, 3, 4, 5)) is None
    assert P.decode_radar(P.ID_RADAR_YAW, b'\x00' * 8) is None


def test_short_payload_returns_none():
    # 半截帧不该被当成有效目标；宁可丢一帧也不要解出垃圾
    assert P.decode_radar(0x760, b'\x00' * 7) is None
    assert P.decode_radar(0x760, b'') is None
    assert P.decode_yaw(b'\x00' * 3) is None
    assert P.decode_speed(b'\x00' * 3) is None


def test_full_pipeline_from_bytes():
    """端到端：一串字节流 -> 切帧 -> 解出目标，模拟节点里的路径。"""
    stream = (gcan(0x760, rt_a(150, -20, -5, 0, 0, live=1))
              + gcan(0x761, rt_b(7, status=1, live=1))
              + P.HEARTBEAT_MAGIC + b'1' + b'21102914'
              + gcan(0x762, rt_c(18, 2, live=1)))
    assert len(stream) == 4 * P.FRAME_LEN

    got, hearts = [], 0
    for i in range(0, len(stream), P.FRAME_LEN):
        b = stream[i:i + P.FRAME_LEN]
        if P.is_heartbeat(b):
            hearts += 1
            continue
        f = P.decode_frame(b)
        d = P.decode_radar(f.can_id, f.data)
        if d:
            got.append(d)
    assert hearts == 1
    assert [d['frame'] for d in got] == ['A', 'B', 'C']
    assert got[0]['l_long'] == 150 and got[1]['track_id'] == 7
    assert got[2]['width'] == 18


# ───────────────────────── 独立运行入口 ─────────────────────────

def _main():
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith('test_') and callable(f)]
    failed = []
    for name, fn in tests:
        try:
            fn()
        except AssertionError as e:
            failed.append(name)
            print('FAIL  %s  %s' % (name, e))
        except Exception as e:                       # noqa: BLE001
            failed.append(name)
            print('ERROR %s  %s: %s' % (name, type(e).__name__, e))
    print('\n%d/%d 通过' % (len(tests) - len(failed), len(tests)))
    if failed:
        print('失败：' + ', '.join(failed))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(_main())
