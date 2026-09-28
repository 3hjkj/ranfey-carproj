#!/usr/bin/env python3
"""邻域半径扫描图：整圈墙是一次"渗流"连起来的，还是一个真实物体？

拿 /live/lidar_cluster_labels 里**节点自己那一帧的 DBSCAN 输入点**（1,839 点）离线重跑
DBSCAN，只改邻域半径，画出各半径下的俯视聚类结果。目的是让"全聚成墙"这件事可见：
巨簇是在某个半径上**突然**出现的（阈值相变），不是随半径慢慢长大。

邻域语义严格照抄 dbscan.h 的 GetDist：返回 10 的条件是 |dx|>HALF 或 |dy|>HALF，
所以有效邻域是切比雪夫半宽 HALF 的正方形（外加 euclid<eps 把四角切掉）。
横轴标的就是这个 HALF。HALF=0.50 那一档与节点在线结果逐项吻合（946/343/272），
可作为语义复刻正确的证据。
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

try:
    font_manager.fontManager.addfont('/mnt/c/Windows/Fonts/simhei.ttf')
    plt.rcParams['font.sans-serif'] = ['SimHei']
except Exception:
    pass
plt.rcParams['axes.unicode_minus'] = False


def grab_points(topic, timeout=30):
    rclpy.init()
    n = Node('eps_cmp'); got = {}
    q = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                   history=HistoryPolicy.KEEP_LAST)
    n.create_subscription(PointCloud2, topic, lambda m: got.setdefault('m', m), q)
    t = time.time()
    while 'm' not in got and time.time() - t < timeout:
        rclpy.spin_once(n, timeout_sec=0.2)
    if 'm' not in got:
        rclpy.shutdown(); raise SystemExit(f'没收到 {topic}')
    m = got['m']
    off = {f.name: f.offset for f in m.fields}
    nn = m.width * m.height
    buf = np.frombuffer(m.data, dtype=np.uint8).reshape(nn, m.point_step)
    col = lambda k, dt: np.ascontiguousarray(buf[:, off[k]:off[k] + 4]).view(dt).ravel()
    xyz = np.stack([col(k, np.float32) for k in ('x', 'y', 'z')], 1).astype(np.float64)
    lab = col('label', np.int32)
    rclpy.shutdown()
    return xyz, lab


def dbscan_cheb(xy, half, min_pts=3):
    """照抄 GetDist 的方形邻域。返回 labels, 每簇点数（降序）。"""
    n = len(xy)
    tree = cKDTree(xy)
    pr = tree.query_pairs(2 * half, output_type='ndarray')
    if len(pr):
        d = np.abs(xy[pr[:, 0]] - xy[pr[:, 1]])
        pr = pr[(d[:, 0] <= half + 1e-9) & (d[:, 1] <= half + 1e-9)]
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
    sizes = np.bincount(l[l >= 0], minlength=c) if c else np.array([], int)
    return l, np.sort(sizes)[::-1]


xyz, lab_node = grab_points('/live/lidar_cluster_labels')
P = xyz[:, :2]
ks = sorted(set(lab_node.tolist()) - {-1})
node_sizes = np.array(sorted([int((lab_node == k).sum()) for k in ks], reverse=True))
print(f'节点输入 {len(xyz):,} 点，报 {len(ks)} 簇，最大 {node_sizes[:5].tolist()}')

RA = [0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50]
res = {}
for h in RA:
    l, s = dbscan_cheb(P, h)
    nclu = int((s >= 4).sum())
    big = l == int(np.argmax([(l == k).sum() for k in range(s.size)])) if s.size else np.zeros(len(P), bool)
    span = (P[big][:, 0].ptp(), P[big][:, 1].ptp()) if big.any() else (0, 0)
    res[h] = dict(l=l, sizes=s, nclu=nclu, big=big, span=span,
                  big_n=int(big.sum()))
    print(f'  HALF={h:.2f}  簇数(≥4点)={nclu:3d}  最大簇={int(s[0]) if s.size else 0:4d}点  '
          f'跨度 {span[0]:5.2f} × {span[1]:5.2f} m')

# 用最大的三簇跟节点在线结果对表，反推节点实际在用的半径
best = min(RA, key=lambda h: np.abs(res[h]['sizes'][:3] - node_sizes[:3]).sum())
print(f'\n与节点在线结果最吻合的半径：HALF={best:.2f}  '
      f'(我的前三 {res[best]["sizes"][:3].tolist()} vs 节点 {node_sizes[:3].tolist()})')

# ── 画图 ──────────────────────────────────────────────────────────────
PANELS = [0.20, 0.25, 0.30, best]
GRAY, HOT = '#c3c8cf', '#d1780a'
fig = plt.figure(figsize=(15.5, 9.6))
gs = fig.add_gridspec(2, 4, height_ratios=[2.35, 1.0], hspace=0.34, wspace=0.22)

allP = P
xl = (allP[:, 0].min() - 1.0, min(allP[:, 0].max() + 1.0, 16))
yl = (allP[:, 1].min() - 1.0, allP[:, 1].max() + 1.0)

for i, h in enumerate(PANELS):
    ax = fig.add_subplot(gs[0, i])
    d = res[h]
    ax.scatter(P[~d['big'], 0], P[~d['big'], 1], s=3.5, c=GRAY, lw=0, alpha=.85)
    ax.scatter(P[d['big'], 0], P[d['big'], 1], s=5.5, c=HOT, lw=0)
    ax.plot(0, 0, marker='*', ms=13, c='#1d4f8f', mec='white', mew=.8, zorder=5)
    ax.set_xlim(*xl); ax.set_ylim(*yl); ax.set_aspect('equal')
    ax.grid(alpha=.22, lw=.6); ax.set_axisbelow(True)
    for sp in ('top', 'right'):
        ax.spines[sp].set_visible(False)
    tag = '  ← 在线在用' if h == best else ''
    ax.set_title(f'邻域半宽 {h:.2f} m{tag}', fontsize=11.5,
                 color=('#a35a00' if h == best else '#222'), pad=8)
    ax.text(.03, .965,
            f"簇数 {d['nclu']}\n最大簇 {d['big_n']} 点\n跨度 {d['span'][0]:.1f}×{d['span'][1]:.1f} m",
            transform=ax.transAxes, va='top', ha='left', fontsize=9.2,
            bbox=dict(fc='white', ec='#d5d8dd', lw=.8, boxstyle='round,pad=0.42'))
    ax.set_xlabel('x 向前 (m)', fontsize=9.5)
    if i == 0:
        ax.set_ylabel('y 向左 (m)', fontsize=9.5)
    else:
        ax.tick_params(labelleft=False)

fig.suptitle('整圈墙是一次"渗流"：邻域半宽超过 ≈0.27 m，全场就突然连成一块',
             fontsize=15, y=.975)
fig.text(.5, .925,
         f"同一帧的真实 DBSCAN 输入点（{len(P):,} 点，取自 /live/lidar_cluster_labels）。"
         f"橙 = 最大簇，灰 = 其余簇。在线在用的半径由前三簇点数与节点上报值对表反推。",
         ha='center', fontsize=10, color='#555')

# 底排两张单系列折线：簇数 / 最大簇跨度 随半径的变化（不共轴，各画各的）
for j, (key, ylab, ttl) in enumerate([
        ('nclu', '簇数（≥4 点）', '簇数：半径一到 0.30，簇数从 59 塌到 18'),
        ('span', '最大簇跨度 (m)', '最大簇跨度：同一处从 3.3 m 跳到 12.0 m')]):
    ax = fig.add_subplot(gs[1, j * 2:j * 2 + 2])
    if key == 'nclu':
        ys = [res[h]['nclu'] for h in RA]
    else:
        ys = [max(res[h]['span']) for h in RA]
    ax.plot(RA, ys, '-o', c='#3987e5', lw=2, ms=6, mfc='white', mew=1.8)
    ax.axvline(best, ls='--', lw=1.3, c='#d1780a')
    ax.annotate('在线', (best, max(ys)), xytext=(4, -12), textcoords='offset points',
                color='#a35a00', fontsize=9.5)
    ax.set_xlabel('邻域半宽 (m)', fontsize=9.5)
    ax.set_ylabel(ylab, fontsize=9.5)
    ax.set_title(ttl, fontsize=10.5, pad=6)
    ax.grid(alpha=.22, lw=.6); ax.set_axisbelow(True)
    for sp in ('top', 'right'):
        ax.spines[sp].set_visible(False)

out = '/home/qq/carProj/eps_compare.png'
fig.savefig(out, dpi=145, bbox_inches='tight', facecolor='white')
print(f'\n写出 {out}')
