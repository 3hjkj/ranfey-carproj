#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
毫米波雷达 CAN 数据嗅探器（广成 GCAN-202 / CANET-2E 以太网-CAN 转换器）

用途：在把雷达接进 ROS 之前，先确认雷达到底有没有在发数据、发的是哪些 ID、
      原始值长什么样。缩放系数拿不到，所以这里只打印原始值 + 按 int.h 的位布局
      解出的字段，方便实测反推。

用法：
    python3 radar_sniffer.py                          # 连 192.168.5.137:4001，跑 30 秒
    python3 radar_sniffer.py --host 192.168.5.3 -t 60 # 换台设备跑 60 秒
    python3 radar_sniffer.py --raw                    # 连心跳都逐帧打印

协议（广成 GCAN-202，每帧固定 13 字节）：
    [0]      帧信息：bit7=FF(1=扩展帧) bit6=RTR(1=远程帧) bit3~0=DLC(数据长度)
    [1:5]    CAN ID，大端（标准帧低 11 位有效）
    [5:13]   数据 8 字节，不足补 0

心跳帧：CAN 无数据时每 2 秒一条，AA 00 FF 00 + 通道号('1'/'2') + 8 字节 SN。
        看到它 = 那路 CAN 上没数据，这是排查时最重要的信号。
"""

import argparse
import collections
import socket
import sys
import time

# ---- 雷达 CAN ID 布局（来自 can_com/include/can_com/int.h）------------------
# 6 个目标，每个目标 3 帧：A=位置速度 B=TrackID/Status C=宽度/类别
#   RT1A=0x760 RT1B=0x761 RT1C=0x762
#   RT2A=0x765 ...                    RT6C=0x77B
#   步长 5：base(n) = 0x760 + (n-1)*5
RT_BASE = 0x760
RT_STEP = 5
RT_TARGETS = 6

# 短距雷达 RTS 系列：RTS1A=0x740 ... RTS4C=0x751，同样步长 5
RTS_BASE = 0x740
RTS_TARGETS = 4

# 其它已知雷达相关 ID
RADAR_EXTRA = {0x130: "Radar_Yaw", 0x3E9: "Radar_Spd"}

HEARTBEAT_MAGIC = b"\xaa\x00\xff\x00"


def rt_ids():
    """返回 {CAN_ID: (目标号, 'A'/'B'/'C', 前缀)} —— RT 和 RTS 两族。"""
    m = {}
    for n in range(1, RT_TARGETS + 1):
        base = RT_BASE + (n - 1) * RT_STEP
        for i, suf in enumerate("ABC"):
            m[base + i] = (n, suf, "RT")
    for n in range(1, RTS_TARGETS + 1):
        base = RTS_BASE + (n - 1) * RT_STEP
        for i, suf in enumerate("ABC"):
            m[base + i] = (n, suf, "RTS")
    return m


RT_IDS = rt_ids()


def decode_frame(b):
    """13 字节 -> (帧信息, can_id, dlc, data)。"""
    info = b[0]
    ff = bool(info & 0x80)          # 扩展帧
    rtr = bool(info & 0x40)         # 远程帧
    dlc = info & 0x0F
    can_id = int.from_bytes(b[1:5], "big")
    if not ff:
        can_id &= 0x7FF             # 标准帧只有低 11 位有效
    # DLC 可能被设备写成大于 8 的垃圾值，这里夹一下避免越界
    n = dlc if 0 <= dlc <= 8 else 8
    return ff, rtr, dlc, can_id, b[5:5 + n]


def is_heartbeat(b):
    """心跳帧：AA 00 FF 00 + 通道号 + SN。"""
    return b[:4] == HEARTBEAT_MAGIC


def describe_heartbeat(b):
    ch = b[4:5].decode("ascii", "replace")
    sn = b[5:13].decode("ascii", "replace")
    return "心跳  通道=%s  SN=%s" % (ch, sn)


def u24(d):
    return int.from_bytes(d, "little")


def s12(v):
    """12 位有符号（二补码）—— 横向距离/速度会是负的。"""
    return v - 4096 if v & 0x800 else v


def s8(v):
    return v - 256 if v & 0x80 else v


def describe_radar(can_id, data):
    """按 int.h 的位布局解雷达帧，只给原始值（缩放系数待实测反推）。"""
    t = RT_IDS.get(can_id)
    if t is None:
        if can_id in RADAR_EXTRA:
            return RADAR_EXTRA[can_id] + "  " + data.hex(" ")
        return None
    n, suf, pre = t
    if len(data) < 8:
        return "%s%d%s  数据不足 8 字节" % (pre, n, suf)

    if suf == "A":
        # RT*A: L_LongObj:12|V_LongObj:12 (data[0:3])
        #       L_LatObj:12 |V_LatObj:12 (data[3:6])
        #       A_LongObj:8 (data[6])  flags(data[7])
        v1 = u24(data[0:3])
        v2 = u24(data[3:6])
        l_long = s12(v1 & 0xFFF)
        v_long = s12((v1 >> 12) & 0xFFF)
        l_lat = s12(v2 & 0xFFF)
        v_lat = s12((v2 >> 12) & 0xFFF)
        a_long = s8(data[6])
        live = data[7] & 0x03
        sens = (data[7] >> 2) & 0x03
        return ("%s%d 纵距=%5d 纵速=%5d 横距=%5d 横速=%5d 纵加速=%4d "
                "LiveCount=%d Sensor=%d" %
                (pre, n, l_long, v_long, l_lat, v_lat, a_long, live, sens))
    if suf == "B":
        # RT*B: TrackID(data[0])  Status(data[2] bits4~7)
        return ("%s%d TrackID=%3d Status=%d" %
                (pre, n, data[0], (data[2] >> 4) & 0x0F))
    # suf == 'C': Width:5(data[0])  ObjectClass:4(data[3..4] bits7~10)
    #             VisTrkID:4|LiveCount:2(data[6..7])
    w = data[0] & 0x1F
    obj_cls = (int.from_bytes(data[3:5], "little") >> 7) & 0x0F
    v67 = int.from_bytes(data[6:8], "little")
    vis = (v67 >> 6) & 0x0F
    live = (v67 >> 4) & 0x03
    return ("%s%d 宽度=%2d 类别=%2d VisTrkID=%2d LiveCount=%d" %
            (pre, n, w, obj_cls, vis, live))


def main():
    ap = argparse.ArgumentParser(description="广成 CANET 雷达 sniff 工具")
    ap.add_argument("--host", default="192.168.5.137", help="转换机 IP")
    ap.add_argument("--port", type=int, default=4001,
                    help="工作端口：4001=CAN1，9998=CAN2（实测；手册写的 4002 未开放）")
    ap.add_argument("-t", "--duration", type=float, default=30.0,
                    help="采集秒数，0 表示一直跑")
    ap.add_argument("--raw", action="store_true", help="逐帧打印，含心跳")
    args = ap.parse_args()

    try:
        s = socket.socket()
        s.settimeout(2.0)
        s.connect((args.host, args.port))
    except Exception as e:
        print("连不上 %s:%d -> %s" % (args.host, args.port, e))
        print("提示：确认网线/交换机、转换机 IP，以及端口是否已开放。")
        return 1

    print("已连接 %s:%d，采集 %.0f 秒...\n" % (args.host, args.port, args.duration))

    buf = b""
    counts = collections.Counter()      # CAN ID -> 帧数
    last = {}                           # CAN ID -> 最近一次解码文本
    hb_count = 0
    lost = 0                            # 非 13 字节整数倍的残留

    t0 = time.time()
    try:
        while args.duration <= 0 or time.time() - t0 < args.duration:
            try:
                chunk = s.recv(8192)
            except socket.timeout:
                continue
            if not chunk:
                print("对端关闭连接")
                break
            buf += chunk
            while len(buf) >= 13:
                b, buf = buf[:13], buf[13:]
                if is_heartbeat(b):
                    hb_count += 1
                    if args.raw:
                        print("  [心跳] %s" % describe_heartbeat(b))
                    continue
                ff, rtr, dlc, can_id, data = decode_frame(b)
                counts[can_id] += 1
                txt = describe_radar(can_id, data)
                if txt:
                    last[can_id] = txt
                    if args.raw or counts[can_id] <= 3:
                        print("  0x%03X  %s" % (can_id, txt))
                elif args.raw:
                    print("  0x%03X  %s" % (can_id, data.hex(" ")))
    except KeyboardInterrupt:
        pass
    finally:
        s.close()
        lost = len(buf)

    dt = time.time() - t0
    print("\n" + "=" * 62)
    print("采集 %.1f 秒，共 %d 帧，心跳 %d 条，残留 %d 字节"
          % (dt, sum(counts.values()), hb_count, lost))

    if hb_count and not counts:
        print("\n!! 只收到心跳，没有 CAN 数据。")
        print("   这表示转换机工作正常，但那路 CAN 总线上没有报文。")
        print("   按顺序查：1) 转换机 CAN 波特率 vs 雷达波特率（通常 500k）")
        print("             2) 雷达接的是哪一路 CAN（心跳第 5 字节 31=CAN1 / 32=CAN2）")
        print("             3) CAN_H/CAN_L 是否接反  4) 120Ω 终端电阻  5) 雷达供电")
        return 2

    if not counts:
        print("\n!! 一帧都没收到，连心跳都没有。检查网络和端口。")
        return 3

    print("\n各 CAN ID 帧数：")
    for cid, c in counts.most_common():
        tag = ""
        if cid in RT_IDS:
            n, suf, pre = RT_IDS[cid]
            tag = "  <- %s%d%s 雷达目标" % (pre, n, suf)
        elif cid in RADAR_EXTRA:
            tag = "  <- %s" % RADAR_EXTRA[cid]
        print("  0x%03X  %6d 帧%s" % (cid, c, tag))

    if last:
        print("\n雷达目标最新原始值（缩放系数待定，单位不是米）：")
        for cid in sorted(last):
            print("  0x%03X  %s" % (cid, last[cid]))
    else:
        print("\n没有匹配到雷达 ID（0x740~0x77B）。")
        print("如果上面有其它 ID，可能是别的雷达协议，把这份输出发我。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
