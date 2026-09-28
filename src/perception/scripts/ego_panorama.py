#!/usr/bin/env python3
"""自车全景图：让肉眼判断"哪一块是自车"，用来定 preprocess.json 的车体框。

画四块：
  ① 俯视 x-y，1 m 网格，叠当前车体框和传感器位置
  ② 侧视 x-z，标出传感器高度和地面
  ③ 沿 +x / -x 的切片点数直方图（找自车的前后边界）
  ④ 沿 ±y 的切片点数直方图（找自车的左右边界）

注意实测已经发现 +x 方向 x∈(1.0,2.0) 整段无点，这类"断口"要在图上看清楚，
不能想当然地把框一路推过去。
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

# WSL 里 fontconfig 是空的，中文得直接借 Windows 的字体
try:
    font_manager.fontManager.addfont('/mnt/c/Windows/Fonts/simhei.ttf')
    plt.rcParams['font.sans-serif'] = ['SimHei']
    plt.rcParams['axes.unicode_minus'] = False
    ZH = True
except Exception:
    ZH = False


def L(zh, en):
    return zh if ZH else en


sys.path.insert(0, '/home/qq/carProj/src/perception/scripts')
from cluster_view3d import RAW_DTYPE

# 当前值，来自 preprocess.json（obj 路径实际读的就是这个文件）
CAR = dict(xmin=-5.0, xmax=1.0, ymin=-2.0, ymax=2.0)
MOUNT_Z = -1.76          # 地面在传感器坐标系下的高度


def grab(timeout=25.0):
    rclpy.init()
    node = Node('ego_panorama')
    got = {}
    q = QoSProfile(depth=3, reliability=ReliabilityPolicy.RELIABLE,
                   history=HistoryPolicy.KEEP_LAST)
    node.create_subscription(PointCloud2, '/rfans_points',
                             lambda m: got.setdefault('m', m), q)
    t0 = time.time()
    while 'm' not in got and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    return got.get('m')


def main():
    m = grab()
    if m is None:
        print('没抓到 /rfans_points')
        return 1
    raw = np.frombuffer(m.data, dtype=RAW_DTYPE)
    p = np.stack([raw['x'], raw['y'], raw['z']], 1).astype(np.float64)
    print(f'{len(p):,} 点')

    fig = plt.figure(figsize=(20, 13))

    # ── ① 俯视 x-y ───────────────────────────────────────────
    ax = fig.add_subplot(2, 2, 1)
    lim = 16.0
    s = p[(np.abs(p[:, 0]) < lim) & (np.abs(p[:, 1]) < lim)]
    # 按高度分色：离地面近的（<-1.4）画浅灰当背景，立体物画深色
    bg = s[s[:, 2] < MOUNT_Z + 0.3]
    fg = s[s[:, 2] >= MOUNT_Z + 0.3]
    ax.scatter(bg[:, 1], bg[:, 0], s=0.3, c='#d8d5cd', linewidths=0,
               label=L('贴地/低矮', 'near-ground'))
    ax.scatter(fg[:, 1], fg[:, 0], s=0.5, c='#3d3b37', linewidths=0,
               label=L('离地 >0.3 m', 'above 0.3 m'))
    ax.add_patch(plt.Rectangle((CAR['ymin'], CAR['xmin']),
                               CAR['ymax']-CAR['ymin'], CAR['xmax']-CAR['xmin'],
                               fill=False, ec='#d55181', lw=2.5,
                               label=L('当前车体框', 'current ego box')))
    ax.scatter([0], [0], marker='*', s=420, c='#c98500', zorder=6,
               label=L('传感器', 'sensor'))
    ax.set_xticks(np.arange(-15, 16, 5)); ax.set_yticks(np.arange(-15, 16, 5))
    ax.grid(alpha=.3)
    ax.set_title(L('① 俯视 x-y（1 m 网格）', '① top view x-y'))
    ax.set_xlabel('y (m)'); ax.set_ylabel('x (m)')
    ax.legend(loc='lower left', fontsize=9); ax.set_aspect('equal')

    # ── ② 侧视 x-z ───────────────────────────────────────────
    ax = fig.add_subplot(2, 2, 2)
    s2 = p[np.abs(p[:, 0]) < 20]
    ax.scatter(s2[:, 0], s2[:, 2], s=0.4, c='#3d3b37', linewidths=0)
    ax.axhline(0, color='#c98500', lw=1.6)
    ax.text(19, 0.08, L('传感器 z=0', 'sensor z=0'), ha='right',
            fontsize=9, color='#c98500')
    ax.axhline(MOUNT_Z, color='#3987e5', lw=1.6)
    ax.text(19, MOUNT_Z+0.08, L('地面 z=-1.76', 'ground z=-1.76'), ha='right',
            fontsize=9, color='#3987e5')
    ax.axvspan(CAR['xmin'], CAR['xmax'], color='#d55181', alpha=.15)
    ax.text((CAR['xmin']+CAR['xmax'])/2, 1.6, L('当前框', 'box'),
            ha='center', fontsize=9, color='#d55181')
    ax.set_title(L('② 侧视 x-z', '② side view x-z'))
    ax.set_xlabel('x (m)'); ax.set_ylabel('z (m)')
    ax.grid(alpha=.3); ax.set_xlim(-20, 20)

    # ── ③ 沿 x 的切片点数 ────────────────────────────────────
    ax = fig.add_subplot(2, 2, 3)
    band = p[(np.abs(p[:, 1]) < 2.5) & (p[:, 2] > MOUNT_Z + 0.05)]
    edges = np.arange(-12, 12.5, 0.5)
    idx = np.digitize(band[:, 0], edges) - 1
    cnt = np.zeros(len(edges)-1, int)
    for i in idx:
        if 0 <= i < len(cnt):
            cnt[i] += 1
    ctr = (edges[:-1] + edges[1:]) / 2
    bars = ax.bar(ctr, cnt, width=0.45,
                  color=['#d55181' if CAR['xmin'] <= c <= CAR['xmax'] else '#3987e5'
                         for c in ctr])
    ax.axvline(0, color='#c98500', lw=2)
    ax.text(0.2, ax.get_ylim()[1]*0.9, L('传感器', 'sensor'), fontsize=9,
            color='#c98500')
    ax.set_title(L('③ 沿 x 切片点数（|y|<2.5 m，离地点）\n'
                   '红=当前框内（会被剔除）  蓝=框外（会留成目标）',
                   '③ point count vs x'))
    ax.set_xlabel('x (m)'); ax.set_ylabel(L('点数', 'points'))
    ax.grid(alpha=.3, axis='y')

    # ── ④ 沿 y 的切片点数 ────────────────────────────────────
    ax = fig.add_subplot(2, 2, 4)
    band2 = p[(np.abs(p[:, 0]) < 6) & (p[:, 2] > MOUNT_Z + 0.05)]
    edges2 = np.arange(-10, 10.25, 0.5)
    idx2 = np.digitize(band2[:, 1], edges2) - 1
    cnt2 = np.zeros(len(edges2)-1, int)
    for i in idx2:
        if 0 <= i < len(cnt2):
            cnt2[i] += 1
    ctr2 = (edges2[:-1] + edges2[1:]) / 2
    ax.bar(ctr2, cnt2, width=0.45,
           color=['#d55181' if CAR['ymin'] <= c <= CAR['ymax'] else '#3987e5'
                  for c in ctr2])
    ax.set_title(L('④ 沿 y 切片点数（|x|<6 m，离地点）',
                   '④ point count vs y'))
    ax.set_xlabel('y (m)'); ax.set_ylabel(L('点数', 'points'))
    ax.grid(alpha=.3, axis='y')

    fig.suptitle(L('自车全景图 —— 对着实物指出哪一块是自车',
                   'ego panorama'), fontsize=15)
    fig.tight_layout()
    out = '/home/qq/carProj/ego_panorama.png'
    fig.savefig(out, dpi=105)
    print(f'图已存到 {out}')

    # 同时打一份数字，方便在图上定位
    print('\n沿 x 切片（|y|<2.5，离地点 z>-1.71）：')
    for a in np.arange(-8, 8, 1.0):
        sel = band[(band[:, 0] >= a) & (band[:, 0] < a+1.0)]
        tag = ' ← 框内' if (CAR['xmin'] <= a < CAR['xmax']) else ''
        print(f'  x {a:>5.1f}~{a+1:<5.1f} {len(sel):>8,}{tag}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
