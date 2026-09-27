#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""广成 GCAN-202 以太网-CAN 转换机 + 毫米波雷达报文解码（纯协议层，不依赖 ROS）。

分两层，别混在一起切：

  一、转换机层（每帧固定 13 字节）
      [0]     帧信息：bit7=FF(1=扩展帧) bit6=RTR(1=远程帧) bit3~0=DLC
      [1:5]   CAN ID，大端（标准帧只有低 11 位有效）
      [5:13]  数据 8 字节，不足补 0
      心跳帧：该路 CAN 无数据时每 2 秒一条，AA 00 FF 00 + 通道号('1'/'2') + 8 字节 SN。
              看到心跳 = 转换机和网络都正常，问题在 CAN 侧。

  二、雷达层（CAN 数据场 8 字节）
      位布局来自 can_com/include/can_com/int.h 的 Radar_RT*/Radar_RTS* 结构体。
      注意 int.h 里每个结构体开头还有 MsgLengh(1) + MsgID(4) 共 5 字节 —— 那是
      生成器产物的软件内部帧头，**不在 CAN 线上**；DATA* 成员的编号（DATA012、
      DATA345…）从数据场第 0 字节重新起算。两者是独立的两层，不要叠加。

两族的语义差别是本文件最关键的地方：

  RT  （0x760 起，6 个目标）：A 帧 = 位置 + **速度** + 纵向加速度；TrackID 在 B 帧
  RTS （0x740 起，4 个目标）：A 帧 = 位置 + **TrackID**；**速度在 B 帧**

  即两族的 A/B 帧语义是反过来的。can_com/scripts/radar_sniffer.py 把 RTS 当成 RT
  解，它打印的 RTS 数据全是错的 —— 本模块不重复这个错误。

本模块只输出**原始计数**，不做任何量纲换算。int.h 里没有 factor/offset（整个仓库
也没有 .dbc/.arxml），换算系数必须实测标定，由调用方（节点参数）负责。
"""

import collections

# ───────────────────────── 转换机层 ─────────────────────────

FRAME_LEN = 13
HEARTBEAT_MAGIC = b'\xaa\x00\xff\x00'

# 帧信息字节的位定义
FF_MASK = 0x80          # 扩展帧
RTR_MASK = 0x40         # 远程帧
DLC_MASK = 0x0F

CAN_SFF_MASK = 0x7FF
CAN_EFF_MASK = 0x1FFFFFFF

FrameInfo = collections.namedtuple('FrameInfo', 'ff rtr dlc can_id data')


def decode_frame(b):
    """13 字节 -> FrameInfo。传入的必须正好是 13 字节。"""
    info = b[0]
    ff = bool(info & FF_MASK)
    rtr = bool(info & RTR_MASK)
    dlc = info & DLC_MASK
    can_id = int.from_bytes(b[1:5], 'big')
    if not ff:
        can_id &= CAN_SFF_MASK
    # DLC 可能被设备写成 0~15 的垃圾值，夹到 8 以内避免越界
    n = dlc if dlc <= 8 else 8
    return FrameInfo(ff, rtr, dlc, can_id, b[5:5 + n])


def is_heartbeat(b):
    """心跳帧：AA 00 FF 00 + 通道号 + SN。"""
    return b[:4] == HEARTBEAT_MAGIC


def parse_heartbeat(b):
    """-> (通道号 '1'/'2', 设备 SN 字符串)。"""
    ch = b[4:5].decode('ascii', 'replace')
    sn = b[5:13].decode('ascii', 'replace')
    return ch, sn


# ───────────────────────── ID 布局 ─────────────────────────

RT_BASE, RT_TARGETS, RT_STEP = 0x760, 6, 5
RTS_BASE, RTS_TARGETS, RTS_STEP = 0x740, 4, 5

ID_RADAR_YAW = 0x130
ID_RADAR_SPD = 0x3E9

FRAME_SUFFIXES = 'ABC'


def radar_ids():
    """-> {can_id: (family, slot, frame)}，family ∈ {'RT','RTS'}，frame ∈ 'ABC'。"""
    m = {}
    for base, count, step, family in ((RT_BASE, RT_TARGETS, RT_STEP, 'RT'),
                                      (RTS_BASE, RTS_TARGETS, RTS_STEP, 'RTS')):
        for n in range(1, count + 1):
            for i, suf in enumerate(FRAME_SUFFIXES):
                m[base + (n - 1) * step + i] = (family, n, suf)
    return m


RADAR_IDS = radar_ids()


def slot_to_obj_id(family, slot):
    """物理槽位 -> SensorData.obj_id（1~10）。

    刻意**不用**雷达自己的 track_id 当 obj_id：perception 的 cbRadar 按 obj_id
    合并进 radar_objs.objs，而那个 vector 全仓库无人清理。track_id 会随目标进出
    不停回收重发，用它会把这个列表越撑越大；用绑定物理槽位的固定值可以把集合
    钉死在 ≤10 条，顺带避免跨槽位 track_id 相撞时下游把两个目标并成一个。

    RT1~RT6  -> 1~6
    RTS1~RTS4 -> 7~10
    """
    if family == 'RT':
        return slot
    return RT_TARGETS + slot


# ───────────────────────── 位运算工具 ─────────────────────────

def s12(v):
    """12 位二补码 -> 有符号。横向距离/速度会是负的。"""
    return v - 4096 if v & 0x800 else v


def s8(v):
    """8 位二补码 -> 有符号。"""
    return v - 256 if v & 0x80 else v


def _u24(d):
    return int.from_bytes(d, 'little')


def _u16(d):
    return int.from_bytes(d, 'little')


def _u32(d):
    return int.from_bytes(d, 'little')


# ───────────────────────── 雷达层解码 ─────────────────────────
#
# 每个解码函数返回一个扁平的语义字典（原始计数，未换算），缺字段就是该帧不携带它。
# 调用方按槽位 update() 合并即可，不必关心两族谁把速度放哪一帧。

def _decode_rt_a(data):
    """RT*A：位置 + 速度 + 纵向加速度。
    DATA012 = L_LongObj:12 | V_LongObj:12
    DATA345 = L_LatObj:12  | V_LatObj:12
    DATA6   = A_LongObj:8
    DATA7   = Reserved:4 | DetectionSenson:2 | LiveCount:2

    ── 两处推断，标定时留意 ──
    1. 有符号性：int.h 里容器声明是 `uint32_t L_LongObj : 12`（无符号容器，gcc
       不会给它做符号扩展），但横向距离/速度必然有正负，所以这里按 12 位二补码
       解。若实测发现负值方向不对，改的就是这一处。
    2. L_LatObj 的起始字节：RT*A 的位域声明容器宽度加起来超过 8 字节，按 32 位容器
       排下来 L_LatObj 会落在 byte4 而不是 byte3。两种排法对 L_LongObj/V_LongObj
       的结果**完全相同**（低 24 位一致），只影响横向量。这里采信 DATA* 成员的
       字节范围命名（DATA345 从第 3 字节起算），因为那是生成器给出的权威划分。
       标定 factor_lat_m 时如果横向距离怎么都对不上，先回来查这一条。
    """
    v1 = _u24(data[0:3])
    v2 = _u24(data[3:6])
    return {
        'l_long': s12(v1 & 0xFFF), 'v_long': s12((v1 >> 12) & 0xFFF),
        'l_lat': s12(v2 & 0xFFF), 'v_lat': s12((v2 >> 12) & 0xFFF),
        'a_long': s8(data[6]),
        'sens': (data[7] >> 4) & 0x03,      # bit4~5
        'live': (data[7] >> 6) & 0x03,      # bit6~7
    }


def _decode_rt_b(data):
    """RT*B：TrackID / Status / 横向加速度 / Movement。
    DATA0  = TrackID:8
    DATA2  = Reserved:4 | Status:4
    DATA56 = Reserved1:4 | A_LatObj:8 | Movement:4
    DATA7  = Reserved:6 | LiveCount:2
    """
    v56 = _u16(data[5:7])
    return {
        'track_id': data[0],
        'status': (data[2] >> 4) & 0x0F,
        'a_lat': s8((v56 >> 4) & 0xFF),
        'movement': (v56 >> 12) & 0x0F,
        'live': (data[7] >> 6) & 0x03,
    }


def _decode_c(data):
    """RT*C 与 RTS*C 逐字节相同：宽度 / 类别 / VisTrkID。
    DATA0  = Width:5
    DATA34 = Reserved1:7 | ObjectClass:4 | Reserved:5
    DATA67 = Reserved1:6 | VisTrkID:4 | Reserved:4 | LiveCount:2
    """
    v34 = _u16(data[3:5])
    v67 = _u16(data[6:8])
    return {
        'width': data[0] & 0x1F,
        'obj_class': (v34 >> 7) & 0x0F,
        'vis_trk_id': (v67 >> 6) & 0x0F,
        'live': (v67 >> 14) & 0x03,
    }


def _decode_rts_a(data):
    """RTS*A：位置 + TrackID（**没有速度**，速度在 B 帧）。
    DATA0123 = L_LongRel:12 | L_LatRel:12 | TrackID:8
    DATA67   = Status:4 | A_LongObj:8 | DetectionSenson:2 | LiveCount:2
    """
    v = _u32(data[0:4])
    v67 = _u16(data[6:8])
    return {
        'l_long': s12(v & 0xFFF), 'l_lat': s12((v >> 12) & 0xFFF),
        'track_id': (v >> 24) & 0xFF,
        'status': v67 & 0x0F,
        'a_long': s8((v67 >> 4) & 0xFF),
        'sens': (v67 >> 12) & 0x03,
        'live': (v67 >> 14) & 0x03,
    }


def _decode_rts_b(data):
    """RTS*B：速度帧。
    DATA0123 = V_LongObj:12 | V_LatObj:12
    DATA56   = Reserved1:4 | A_LatObj:8 | Movement:4
    DATA7    = Reserved:6 | LiveCount:2
    """
    v = _u32(data[0:4])
    v56 = _u16(data[5:7])
    return {
        'v_long': s12(v & 0xFFF), 'v_lat': s12((v >> 12) & 0xFFF),
        'a_lat': s8((v56 >> 4) & 0xFF),
        'movement': (v56 >> 12) & 0x0F,
        'live': (data[7] >> 6) & 0x03,
    }


def decode_radar(can_id, data):
    """统一入口。-> dict（含 family/slot/frame）或 None（不是已知雷达 ID）。

    data 长度不足 8 字节时返回 None —— 半截帧不该被当成有效目标。
    """
    t = RADAR_IDS.get(can_id)
    if t is None:
        return None
    if len(data) < 8:
        return None
    family, slot, frame = t
    if family == 'RT':
        fields = (_decode_rt_a(data) if frame == 'A' else
                  _decode_rt_b(data) if frame == 'B' else _decode_c(data))
    else:
        fields = (_decode_rts_a(data) if frame == 'A' else
                  _decode_rts_b(data) if frame == 'B' else _decode_c(data))
    fields['family'] = family
    fields['slot'] = slot
    fields['frame'] = frame
    return fields


def decode_yaw(data):
    """0x130 Radar_Yaw：13 位偏航角，bit5 是有效位。"""
    if len(data) < 8:
        return None
    v = _u16(data[0:2])
    return {'yaw_raw': v & 0x1FFF, 'yaw_valid': (data[2] >> 5) & 0x01}


def decode_speed(data):
    """0x3E9 Radar_Spd：15 位车速，bit15 是有效位（报文里重复了两份，取第一份）。"""
    if len(data) < 8:
        return None
    v = _u16(data[0:2])
    return {'speed_raw': v & 0x7FFF, 'speed_valid': (v >> 15) & 0x01}


def describe(fields):
    """给日志/诊断用的一行摘要（原始计数，不带单位）。"""
    if fields is None:
        return '—'
    fam, slot, frame = fields['family'], fields['slot'], fields['frame']
    head = '%s%d%s' % (fam, slot, frame)
    bits = []
    for k in ('track_id', 'l_long', 'l_lat', 'v_long', 'v_lat',
              'a_long', 'a_lat', 'width', 'obj_class', 'status',
              'movement', 'live'):
        if k in fields:
            bits.append('%s=%s' % (k, fields[k]))
    return '%s  %s' % (head, ' '.join(bits))
