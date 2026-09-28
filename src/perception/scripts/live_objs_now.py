#!/usr/bin/env python3
"""实时结果现状图：三维 eps=0.30 之后，目标长什么样、判型判成什么样。

配套 dim2_vs_dim3.py 看：那张图说明"为什么"要补 z，这张说明补完之后"还剩什么问题"。
"""
import time, collections
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from lidar_msgs.msg import Objects

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Polygon
try:
    font_manager.fontManager.addfont('/mnt/c/Windows/Fonts/simhei.ttf')
    plt.rcParams['font.sans-serif'] = ['SimHei']
except Exception:
    pass
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['font.size'] = 9.5

SURF, INK, INK2 = '#fcfcfb', '#0b0b0b', '#52514e'
S1, S2, S3 = '#2a78d6', '#eb6834', '#1baf7a'
OTHER = '#c9ccd1'
TYPE = {0: '未知', 1: '卡车', 2: '车', 3: '人', 4: '骑行者', 5: '墙'}
ARC = np.pi / 180.0
GZ = -1.76

rclpy.init(); node = Node('lon'); got = {}
q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
node.create_subscription(PointCloud2, '/live/lidar_cluster_labels', lambda m: got.__setitem__('c', m), q)
node.create_subscription(Objects, '/live/lidar_objs', lambda m: got.__setitem__('o', m), q)
t = time.time()
while ('c' not in got or 'o' not in got) and time.time() - t < 30:
    rclpy.spin_once(node, timeout_sec=0.2)
rclpy.shutdown()

m = got['c']; off = {f.name: f.offset for f in m.fields}
nn_ = m.width * m.height
buf = np.frombuffer(m.data, dtype=np.uint8).reshape(nn_, m.point_step)
col = lambda k, dt: np.ascontiguousarray(buf[:, off[k]:off[k]+4]).view(dt).ravel()
xyz = np.stack([col(k, np.float32) for k in ('x', 'y', 'z')], 1).astype(np.float64)
lab = col('label', np.int32)
objs = list(got['o'].objs)

fig = plt.figure(figsize=(16.6, 8.4), facecolor=SURF)
gs = fig.add_gridspec(2, 3, width_ratios=[1.75, 1, 1], hspace=.42, wspace=.28,
                      left=.05, right=.985, top=.845, bottom=.09)

# A. 俯视：点按簇着色（前 3 大 + 其余），叠加每个目标的 OBB
axA = fig.add_subplot(gs[:, 0])
sz = collections.Counter(lab[lab >= 0].tolist())
top3 = [k for k, _ in sz.most_common(3)]
for i, k in enumerate(top3):
    s = lab == k
    axA.scatter(xyz[s, 0], xyz[s, 1], s=5, c=[S1, S2, S3][i], lw=0, alpha=.9)
axA.scatter(xyz[~np.isin(lab, top3), 0], xyz[~np.isin(lab, top3), 1],
            s=2.8, c=OTHER, lw=0, alpha=.85)
for o in objs:
    if max(o.length, o.width) < 0.5:
        continue
    th = o.rel_heading * ARC
    c, s_ = np.cos(th), np.sin(th)
    hl, hw = o.length / 2, o.width / 2
    box = np.array([[-hl, -hw], [hl, -hw], [hl, hw], [-hl, hw]])
    R = np.array([[c, -s_], [s_, c]])
    box = box @ R.T + np.array([o.rel_x, o.rel_y])
    axA.add_patch(Polygon(box, closed=True, fc='none', ec='#8a4fbe', lw=1.1, alpha=.85))
axA.plot(0, 0, marker='*', ms=14, c=INK, mec=SURF, mew=.9, zorder=6)
axA.set_aspect('equal'); axA.grid(alpha=.2, lw=.6, color='#dddedb'); axA.set_axisbelow(True)
for sp in ('top', 'right'):
    axA.spines[sp].set_visible(False)
for sp in ('left', 'bottom'):
    axA.spines[sp].set_color('#dddedb')
axA.set_xlabel('x 向前 (m)'); axA.set_ylabel('y 向左 (m)')
axA.set_title(f'本帧俯视：{len(objs)} 个目标，最大 {max((max(o.length,o.width) for o in objs), default=0):.2f} m'
              f'（三维 eps=0.30）', fontsize=11, pad=8)
axA.legend(handles=[
    plt.Line2D([], [], marker='o', ls='', ms=7, mfc=S1, mec='none', label='第 1 大簇'),
    plt.Line2D([], [], marker='o', ls='', ms=7, mfc=S2, mec='none', label='第 2'),
    plt.Line2D([], [], marker='o', ls='', ms=7, mfc=S3, mec='none', label='第 3'),
    plt.Line2D([], [], marker='o', ls='', ms=6, mfc=OTHER, mec='none', label='其余簇'),
    plt.Line2D([], [], ls='-', c='#8a4fbe', lw=1.1, label='目标外接框（≥0.5 m）'),
], loc='upper right', frameon=False, fontsize=8.6, ncol=2)

# B. 类型分布
axB = fig.add_subplot(gs[0, 1])
cnt = collections.Counter(o.type for o in objs)
ks = sorted(cnt)
axB.bar([TYPE.get(k, '?') for k in ks], [cnt[k] for k in ks], color=S1, width=.68)
for i, k in enumerate(ks):
    axB.text(i, cnt[k], str(cnt[k]), ha='center', va='bottom', fontsize=9, color=INK2)
axB.set_ylabel('目标数'); axB.set_title('判型结果', fontsize=10.5, pad=6)
axB.grid(alpha=.2, lw=.6, axis='y', color='#dddedb'); axB.set_axisbelow(True)
for sp in ('top', 'right'):
    axB.spines[sp].set_visible(False)
for sp in ('left', 'bottom'):
    axB.spines[sp].set_color('#dddedb')
plt.setp(axB.get_xticklabels(), fontsize=9)

# C. 尺寸散布：碎块与实体的分界
axC = fig.add_subplot(gs[1, 1:])
mw = np.array([max(o.length, o.width) for o in objs])
ht = np.array([o.height for o in objs])
axC.scatter(mw, ht, s=26, c=S1, alpha=.75, lw=0)
axC.axvline(0.5, ls='--', lw=1.2, c='#b23c17')
axC.axhline(0.8, ls='--', lw=1.2, c='#8a4fbe')
axC.annotate('0.5 m：碎块/实体', (0.5, ht.max() * .97), xytext=(4, 0),
             textcoords='offset points', fontsize=8.6, color='#b23c17')
axC.annotate('0.8 m：判型的高度门槛', (mw.max(), 0.8), xytext=(-6, 5),
             textcoords='offset points', fontsize=8.6, color='#6b3a96', ha='right')
axC.set_xscale('log')
axC.set_xlabel('max(长, 宽)  (m，对数轴)'); axC.set_ylabel('高 (m)')
axC.set_title(f'目标尺寸：{int((mw<0.5).sum())} 个小于 0.5 m（碎块），{int((mw>=0.5).sum())} 个实体',
              fontsize=10.5, pad=6)
axC.grid(alpha=.2, lw=.6, color='#dddedb'); axC.set_axisbelow(True)
for sp in ('top', 'right'):
    axC.spines[sp].set_visible(False)
for sp in ('left', 'bottom'):
    axC.spines[sp].set_color('#dddedb')

fig.suptitle('补上 z 之后：巨块没了，但换成了碎块 —— 半径没有中间档', fontsize=15.5, y=.955, color=INK)
fig.text(.5, .895,
         '同一帧的实时输出。/live/lidar_cluster_labels 与 /live/lidar_objs。'
         '细扫证明 eps 在 0.30→0.33 之间是一跳：0.30 时 66 簇最大 125 点，0.33 时最大簇回到 714 点、跨 9.3×8.9 m。',
         ha='center', fontsize=9.8, color=INK2)

out = '/home/qq/carProj/live_objs_now.png'
fig.savefig(out, dpi=140, facecolor=SURF)
print('写出', out)
print('目标数', len(objs), '类型', {TYPE.get(k, k): v for k, v in sorted(cnt.items())})
print('max(l,w) 最大', mw.max().round(2), ' <0.5m 的', int((mw < 0.5).sum()))
