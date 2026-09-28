#!/usr/bin/env python3
"""邻域半径的取舍图：为什么"把墙拆开"和"把车保住"不能同时做到。

上一版 eps_compare.py 我用方块模型扫的，是错的：dbscan.h:146 的比较是
`GetDist(...) <= eps_`，而 GetDist 只在 |dx|>0.5 或 |dy|>0.5 时才返回 10，
所以 **eps<=0.5 时有效邻域就是半径 eps 的正圆**，那个 0.5 只在 eps>0.5 时才卡人。
本图用精确语义重算，横轴就是 objs.json 里能直接填的 dbscan_eps —— 改它不用重编。

本图要回答的问题是"要不要把 eps 调小"，而不是"调小了好看不好看"：
把 eps 调小确实能把墙拆开，代价是远处物体的扫描线之间本来就隔得比 eps 宽，
一调小就整片碎掉。右边那张点间距图是这件事的直接证据。
"""
import sys, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from scipy.spatial import cKDTree

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Rectangle
try:
    font_manager.fontManager.addfont('/mnt/c/Windows/Fonts/simhei.ttf')
    plt.rcParams['font.sans-serif'] = ['SimHei']
except Exception:
    pass
plt.rcParams['axes.unicode_minus'] = False

rclpy.init(); node = Node('ct'); got = {}
q = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
node.create_subscription(PointCloud2, '/live/lidar_cluster_labels', lambda m: got.setdefault('m', m), q)
t = time.time()
while 'm' not in got and time.time() - t < 30:
    rclpy.spin_once(node, timeout_sec=0.2)
m = got['m']; off = {f.name: f.offset for f in m.fields}
nn_ = m.width * m.height
buf = np.frombuffer(m.data, dtype=np.uint8).reshape(nn_, m.point_step)
col = lambda k, dt: np.ascontiguousarray(buf[:, off[k]:off[k]+4]).view(dt).ravel()
xyz = np.stack([col(k, np.float32) for k in ('x', 'y', 'z')], 1).astype(np.float64)
lab = col('label', np.int32)
rclpy.shutdown()
P = xyz[:, :2]; GZ = -1.76


def dbscan_circle(xy, eps, min_pts=3):
    n = len(xy); tree = cKDTree(xy)
    pr = tree.query_pairs(2 * min(eps, 0.5) + 0.01, output_type='ndarray')
    if len(pr):
        d = xy[pr[:, 0]] - xy[pr[:, 1]]; ad = np.abs(d); eu = np.hypot(d[:, 0], d[:, 1])
        pr = pr[(ad[:, 0] <= 0.5 + 1e-9) & (ad[:, 1] <= 0.5 + 1e-9) & (eu <= eps + 1e-9)]
    adj = [[] for _ in range(n)]
    for a, b in pr:
        adj[a].append(b); adj[b].append(a)
    l = np.full(n, -1, np.int32); c = 0
    for i in range(n):
        if l[i] != -1 or len(adj[i]) + 1 < min_pts:
            continue
        st = [i]; l[i] = c
        while st:
            u = st.pop()
            if len(adj[u]) + 1 < min_pts:
                continue
            for v in adj[u]:
                if l[v] == -1:
                    l[v] = c; st.append(v)
        c += 1
    return l, c


EPS = np.round(np.arange(0.15, 0.65, 0.025), 3)
nclu, bigsz = [], []
for e in EPS:
    l, c = dbscan_circle(P, e)
    sz = np.bincount(l[l >= 0], minlength=c) if c else np.array([], int)
    nclu.append(int((sz >= 4).sum()))
    bigsz.append(int(sz.max()) if sz.size else 0)

# 远处真实物体在多大半径下开始碎：跟节点 eps=0.6 下的每个非巨块簇比
ks = sorted(set(lab.tolist()) - {-1}, key=lambda k: -int((lab == k).sum()))
frag_e = np.round(np.arange(0.20, 0.61, 0.05), 3)
frag = {k: [] for k in ks[1:7]}
for e in frag_e:
    for k in frag:
        mask = lab == k
        l, c = dbscan_circle(P[mask], e)
        sz = np.bincount(l[l >= 0], minlength=c) if c else np.array([], int)
        frag[k].append(int((sz >= 4).sum()))

# 点间距 vs 距离：直接说明为什么小半径会碎
tree = cKDTree(P)
d1, i1 = tree.query(P, k=2)
d1 = d1[:, 1]
r = np.hypot(P[:, 0], P[:, 1])
edges = np.arange(1, 19, 1.0)
which = np.digitize(r, edges)
med = [np.median(d1[which == b]) if (which == b).sum() > 8 else np.nan
       for b in range(1, len(edges))]
ctr = (edges[:-1] + edges[1:]) / 2

BIG = int(np.argmax(bigsz))
fig = plt.figure(figsize=(16.5, 10.4))
gs = fig.add_gridspec(2, 3, height_ratios=[1.28, 1.0], hspace=0.42, wspace=0.26)

# A. 巨块俯视
axA = fig.add_subplot(gs[0, 0])
gm = l == int(np.argmax(np.bincount(l[l >= 0])))if False else None
l6, c6 = dbscan_circle(P, 0.60)
sz6 = np.bincount(l6[l6 >= 0], minlength=c6); big6 = int(np.argmax(sz6))
axA.scatter(P[l6 != big6, 0], P[l6 != big6, 1], s=3, c='#c3c8cf', lw=0)
axA.scatter(P[l6 == big6, 0], P[l6 == big6, 1], s=5, c='#d1780a', lw=0)
gq = P[l6 == big6]
axA.add_patch(Rectangle((gq[:, 0].min(), gq[:, 1].min()),
                        gq[:, 0].ptp(), gq[:, 1].ptp(),
                        fc='none', ec='#a35a00', lw=1.4, ls='--'))
axA.plot(0, 0, marker='*', ms=13, c='#1d4f8f', mec='white', mew=.8, zorder=5)
axA.set_aspect('equal'); axA.grid(alpha=.22, lw=.6); axA.set_axisbelow(True)
for sp in ('top', 'right'): axA.spines[sp].set_visible(False)
axA.set_title(f'当前 eps=0.60：最大簇 {int(sz6[big6])} 点\n跨度 {gq[:,0].ptp():.1f} × {gq[:,1].ptp():.1f} m',
              fontsize=10.5, pad=7)
axA.set_xlabel('x 向前 (m)', fontsize=9); axA.set_ylabel('y 向左 (m)', fontsize=9)

# B. 巨块侧视：它是不是贴地铺开的
axB = fig.add_subplot(gs[0, 1])
qz = xyz[l6 == big6]
axB.scatter(qz[:, 0], qz[:, 2], s=4, c='#d1780a', lw=0, alpha=.75)
axB.axhline(GZ, c='#2c7a3f', lw=1.6)
axB.annotate('地面 −1.76 m', (qz[:, 0].min(), GZ), xytext=(3, 4),
             textcoords='offset points', fontsize=9, color='#2c7a3f')
axB.set_xlabel('x 向前 (m)', fontsize=9); axB.set_ylabel('z (m)', fontsize=9)
axB.grid(alpha=.22, lw=.6); axB.set_axisbelow(True)
for sp in ('top', 'right'): axB.spines[sp].set_visible(False)
axB.set_title(f'巨块的侧视：离地 {qz[:,2].min()-GZ:.2f}–{qz[:,2].max()-GZ:.2f} m\n'
              f'贴地(离地<0.1 m)的点占 {100*np.mean(qz[:,2]<GZ+0.1):.0f}% —— 不是地面漏点',
              fontsize=10.5, pad=7)

# C. 远处真实物体俯视
axC = fig.add_subplot(gs[0, 2])
k2 = ks[1]
axC.scatter(P[lab != k2, 0], P[lab != k2, 1], s=3, c='#c3c8cf', lw=0)
axC.scatter(P[lab == k2, 0], P[lab == k2, 1], s=5, c='#3987e5', lw=0)
q2 = P[lab == k2]
axC.add_patch(Rectangle((q2[:, 0].min(), q2[:, 1].min()), q2[:, 0].ptp(), q2[:, 1].ptp(),
                        fc='none', ec='#1d4f8f', lw=1.4, ls='--'))
axC.plot(0, 0, marker='*', ms=13, c='#1d4f8f', mec='white', mew=.8, zorder=5)
axC.set_aspect('equal'); axC.grid(alpha=.22, lw=.6); axC.set_axisbelow(True)
for sp in ('top', 'right'): axC.spines[sp].set_visible(False)
rr = np.hypot(q2[:, 0], q2[:, 1])
axC.set_title(f'第 2 大簇（{int((lab==k2).sum())} 点）：{q2[:,0].ptp():.1f} × {q2[:,1].ptp():.1f} m\n'
              f'距离 {rr.min():.1f}–{rr.max():.1f} m', fontsize=10.5, pad=7)
axC.set_xlabel('x 向前 (m)', fontsize=9)

def style(ax):
    ax.grid(alpha=.22, lw=.6); ax.set_axisbelow(True)
    for sp in ('top', 'right'): ax.spines[sp].set_visible(False)

# D. 簇数 / 最大簇点数 随 eps  # 单系列各一格
axD = fig.add_subplot(gs[1, 0])
axD.plot(EPS, nclu, '-o', c='#3987e5', lw=2, ms=4.5, mfc='white', mew=1.5)
axD.axvline(0.60, ls='--', lw=1.3, c='#d1780a')
axD.annotate('在线', (0.60, max(nclu)), xytext=(-30, -6), textcoords='offset points',
             color='#a35a00', fontsize=9)
axD.set_xlabel('dbscan_eps (m)', fontsize=9); axD.set_ylabel('簇数（≥4 点）', fontsize=9)
axD.set_title('簇数：0.25→0.30 之间有一道坎', fontsize=10.5, pad=6); style(axD)

axE = fig.add_subplot(gs[1, 1])
axE.plot(EPS, bigsz, '-o', c='#3987e5', lw=2, ms=4.5, mfc='white', mew=1.5)
axE.axvline(0.60, ls='--', lw=1.3, c='#d1780a')
axE.set_yscale('log')
axE.set_xlabel('dbscan_eps (m)', fontsize=9); axE.set_ylabel('最大簇点数（对数轴）', fontsize=9)
axE.set_title('最大簇：跨过那道坎就涨到 600+ 点', fontsize=10.5, pad=6); style(axE)

axF = fig.add_subplot(gs[1, 2])
COLS = ['#3987e5', '#c98500', '#2c7a3f', '#8a4fbe', '#c0392b', '#0d7f8c']
for i, (k, ys) in enumerate(frag.items()):
    axF.plot(frag_e, ys, '-o', c=COLS[i], lw=1.8, ms=4, mfc='white', mew=1.3,
             label=f'{int((lab==k).sum())} 点')
axF.axhline(1, c='#2c7a3f', lw=1.2, ls=':', alpha=.8)
axF.annotate('完整', (frag_e[0], 1), xytext=(2, 5), textcoords='offset points',
             fontsize=8.5, color='#2c7a3f')
axF.axvline(0.60, ls='--', lw=1.3, c='#d1780a')
axF.set_yscale('log'); axF.set_yticks([1, 2, 5, 10, 20]); axF.set_yticklabels(['1', '2', '5', '10', '20'])
axF.set_xlabel('dbscan_eps (m)', fontsize=9); axF.set_ylabel('碎成几瓣', fontsize=9)
axF.set_title('代价：远处的真实物体在同样半径下碎掉', fontsize=10.5, pad=6)
axF.legend(fontsize=8.2, frameon=False, ncol=3, loc='upper center', title='物体点数',
           title_fontsize=8.2)
style(axF)

fig.suptitle('调小 eps 拆得开墙，但也拆得碎车 —— 半径不是这个问题的解法', fontsize=15.5, y=.975)
fig.text(.5, .928,
         f"同一帧的真实 DBSCAN 输入点（{len(P):,} 点）。左三格说明巨块是连续立面而非地面漏点；"
         f"右下说明远处物体的扫描线间距本来就跟小半径同量级。",
         ha='center', fontsize=10, color='#555')

out = '/home/qq/carProj/cluster_tradeoff.png'
fig.savefig(out, dpi=145, bbox_inches='tight', facecolor='white')
print('写出', out)
print('eps 扫描:', dict(zip(EPS.tolist(), bigsz)))
print('碎瓣:', {int((lab==k).sum()): frag[k] for k in frag})
