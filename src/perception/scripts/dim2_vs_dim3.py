#!/usr/bin/env python3
"""2-D 投影聚类 vs 含 z 的三维聚类：把"全聚成墙"的真因摆出来。

代码事实（dbscan.cpp:96-103 / :101-102 / dbscan.h:37-46）：
DbscanType 只有 x、y 两个坐标，Init() 只把 data_in[i].x/.y 拷进去，z 被整个丢掉。
GetDist 也只看 |dx|、|dy|。**聚类是纯 2-D 的。**

后果：竖直墙面上一整列点（同一方位、不同高度）在 x-y 上几乎重合，
所以整面墙投影成一条曲线。房间的四壁连成一条闭合曲线 -> 一块巨簇。

本图对照 2-D 与 3-D 在三个 eps 下的结果。配色按 dataviz 技能：散点属 --pairs all，
只有前三个色槽能过全部色对门槛，所以最多三色，其余归"其他"（灰）。
"""
import numpy as np, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from scipy.spatial import cKDTree

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.lines import Line2D

try:
    font_manager.fontManager.addfont('/mnt/c/Windows/Fonts/simhei.ttf')
    plt.rcParams['font.sans-serif'] = ['SimHei']
except Exception:
    pass
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['font.size'] = 9.5

# dataviz 参考调色板（light）：surface-1 / text / 前三个分类槽
SURF, INK, INK2 = '#fcfcfb', '#0b0b0b', '#52514e'
S1, S2, S3 = '#2a78d6', '#eb6834', '#1baf7a'
OTHER = '#c9ccd1'
GZ = -1.76

rclpy.init(); node = Node('d23'); got = {}
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
rclpy.shutdown()


def dbscan(pts, eps, min_pts=3):
    """精确照抄 GetDist 语义（含 |dx|,|dy|<=0.5 的方形上限），维度由传入列数决定。"""
    n = len(pts); pr = cKDTree(pts).query_pairs(2 * min(eps, 0.5) + 0.01, output_type='ndarray')
    if len(pr):
        d = pts[pr[:, 0]] - pts[pr[:, 1]]; ad = np.abs(d); eu = np.linalg.norm(d, axis=1)
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


EPS = [0.30, 0.40, 0.60]
RES = {}
for dim in (2, 3):
    for e in EPS:
        P = xyz if dim == 3 else xyz[:, :2]
        l, c = dbscan(P, e)
        sz = np.bincount(l[l >= 0], minlength=c) if c else np.array([], int)
        order = np.argsort(sz)[::-1]
        RES[(dim, e)] = (l, sz, order)

fig = plt.figure(figsize=(16.2, 13.6), facecolor=SURF)
gs = fig.add_gridspec(3, 3, height_ratios=[1, 1, 0.92], hspace=0.33, wspace=0.20,
                      left=.05, right=.985, top=.885, bottom=.05)

xl = (xyz[:, 0].min() - 0.8, min(xyz[:, 0].max() + 0.8, 13))
yl = (xyz[:, 1].min() - 0.8, xyz[:, 1].max() + 0.8)


def panel(ax, dim, e, annot=True):
    l, sz, order = RES[(dim, e)]
    hues = [S1, S2, S3]
    big3 = [int(k) for k in order[:3] if sz[k] >= 4]
    for i, k in enumerate(big3):
        s = l == k
        ax.scatter(xyz[s, 0], xyz[s, 1], s=5.5, c=hues[i], lw=0, alpha=.92)
    rest = ~np.isin(l, big3)
    ax.scatter(xyz[rest, 0], xyz[rest, 1], s=3, c=OTHER, lw=0, alpha=.9)
    ax.plot(0, 0, marker='*', ms=13, c=INK, mec=SURF, mew=.9, zorder=6)
    ax.set_xlim(*xl); ax.set_ylim(*yl); ax.set_aspect('equal')
    ax.grid(alpha=.2, lw=.6, color='#dddedb'); ax.set_axisbelow(True)
    for sp in ('top', 'right'):
        ax.spines[sp].set_visible(False)
    for sp in ('left', 'bottom'):
        ax.spines[sp].set_color('#dddedb')
    if annot:
        gk = int(order[0]); gq = xyz[l == gk]
        nclu = int((sz >= 4).sum())
        crit = sz[gk] > 400
        ax.text(.028, .972,
                f"簇数 {nclu}\n最大簇 {int(sz[gk])} 点\n跨度 {gq[:,0].ptp():.1f}×{gq[:,1].ptp():.1f} m",
                transform=ax.transAxes, va='top', ha='left', fontsize=9,
                bbox=dict(fc=SURF, ec='#dddedb', lw=.8, boxstyle='round,pad=0.42'))
        if crit:
            ax.text(.028, .105, '整片连成一块', transform=ax.transAxes, fontsize=9.4,
                    color='#b23c17', fontweight='bold')
    ax.set_xlabel('x 向前 (m)')
    return ax


for j, e in enumerate(EPS):
    ax = fig.add_subplot(gs[0, j]); panel(ax, 2, e)
    ax.set_title(f'eps = {e:.2f} m', fontsize=11.5, color=INK, pad=7)
    if j == 0:
        ax.set_ylabel('y 向左 (m)')
    else:
        ax.tick_params(labelleft=False)

for j, e in enumerate(EPS):
    ax = fig.add_subplot(gs[1, j]); panel(ax, 3, e)
    ax.set_title(f'eps = {e:.2f} m', fontsize=11.5, color=INK, pad=7)
    if j == 0:
        ax.set_ylabel('y 向左 (m)')
    else:
        ax.tick_params(labelleft=False)

# 第三行：侧视 x-z，直观看 z 有没有参与
for j, (dim, e, note) in enumerate([
        (2, 0.60, 'z 被丢弃：最大簇在高度上铺满 3.35 m'),
        (3, 0.60, '补上 z 但 eps 仍 0.60：墙在三维里本来就是连通的，一样成块'),
        (3, 0.30, '补上 z + eps 收到 0.30：最大簇缩成 2.11×1.71×1.57 m')]):
    ax = fig.add_subplot(gs[2, j])
    l, sz, order = RES[(dim, e)]
    gk = int(order[0]); s = l == gk
    ax.scatter(xyz[~s, 0], xyz[~s, 2], s=2.6, c=OTHER, lw=0, alpha=.8)
    ax.scatter(xyz[s, 0], xyz[s, 2], s=4.5, c=(S2 if dim == 2 else S1), lw=0, alpha=.9)
    ax.axhline(GZ, c='#1baf7a', lw=1.5, zorder=1)
    ax.annotate('地面 −1.76 m', (xl[0], GZ), xytext=(3, 4), textcoords='offset points',
                fontsize=8.6, color='#0f7a55')
    ax.set_xlim(*xl); ax.set_ylim(-2.1, 3.4)
    ax.grid(alpha=.2, lw=.6, color='#dddedb'); ax.set_axisbelow(True)
    for sp in ('top', 'right'):
        ax.spines[sp].set_visible(False)
    for sp in ('left', 'bottom'):
        ax.spines[sp].set_color('#dddedb')
    ax.set_xlabel('x 向前 (m)')
    if j == 0:
        ax.set_ylabel('z (m)')
    else:
        ax.tick_params(labelleft=False)
    ax.set_title(note, fontsize=9.8, color=INK2, pad=6)

fig.text(.015, .905, 'x-y 二维聚类（现在在用）', fontsize=12.5, color=INK, fontweight='bold')
fig.text(.015, .607, 'x-y-z 三维聚类（补上被丢掉的 z）', fontsize=12.5, color=INK, fontweight='bold')
fig.text(.015, .283, '侧视 x–z：同一批点，看 z 有没有参与', fontsize=12.5, color=INK, fontweight='bold')

fig.suptitle('"全聚成墙"的真因：聚类只用了 x 和 y，z 被丢掉了', fontsize=16, y=.965, color=INK)
fig.text(.5, .928,
         f"同一帧的真实 DBSCAN 输入点（{len(xyz):,} 点，来自 /live/lidar_cluster_labels）。"
         f"竖直墙面上一整列点在同一方位上 x-y 几乎重合，整圈墙因此投影成一条闭合曲线，必然连成一块。",
         ha='center', fontsize=10, color=INK2)

fig.legend(handles=[
    Line2D([], [], marker='o', ls='', ms=7, mfc=S1, mec='none', label='最大簇'),
    Line2D([], [], marker='o', ls='', ms=7, mfc=S2, mec='none', label='第 2 大簇'),
    Line2D([], [], marker='o', ls='', ms=7, mfc=S3, mec='none', label='第 3 大簇'),
    Line2D([], [], marker='o', ls='', ms=6, mfc=OTHER, mec='none', label='其余簇'),
    Line2D([], [], marker='*', ls='', ms=11, mfc=INK, mec=SURF, label='传感器'),
], loc='upper right', bbox_to_anchor=(.99, .905), ncol=5, frameon=False, fontsize=9.3)

out = '/home/qq/carProj/dim2_vs_dim3.png'
fig.savefig(out, dpi=140, facecolor=SURF)
print('写出', out)
for dim in (2, 3):
    for e in EPS:
        l, sz, order = RES[(dim, e)]
        gq = xyz[l == int(order[0])]
        print(f'  {"2D" if dim==2 else "3D"} eps={e:.2f}: 簇数(>=4)={int((sz>=4).sum()):>3} '
              f'最大={int(sz[order[0]]):>4} 跨度 {gq[:,0].ptp():5.2f}×{gq[:,1].ptp():5.2f}×{gq[:,2].ptp():5.2f} m')
