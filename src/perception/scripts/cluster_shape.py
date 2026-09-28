#!/usr/bin/env python3
"""聚类的**结构指标** —— 不依赖真值，所以能当判据用。

为什么不用 cluster_eval.py 的六指标
-----------------------------------
那份"真值"是用和 perception 同一套配方（体素 0.3 + DBSCAN eps 0.6）自动生成的，
它自己就把一面 11.54 m 长的墙当成**一个物体**：最大真值簇 631 点占 35.7%，
前三大簇占 73.0%。在这把尺子下，把两个物体拆开会被"完整性"扣分 ——
历史值 H=0.968 / C=0.508 就是这个签名。**拿六指标做优化目标，等于把 perception
往"合并"推**，正对着用户抱怨的方向。

所以这里换一套不依赖真值的量：直接看聚类结果自身的几何形态。
判据是「大簇覆盖了多少点」——用户抱怨的"把很多物品合在一起"，
在几何上就表现为少数几个跨十几米的簇吃掉了大部分点。

报什么
------
- 点数 / 簇数 / 噪点率
- **最大簇点占比** ← 现状实测 50.4%
- **跨度分档表**：按每簇最小面积外接矩形的**最长边**分档
  ≤1 / 1~2 / 2~3 / >3 m，各档列**簇数**与**覆盖点占比** ← 本轮核心指标
  中间那档（2~3 m）单独留着：实测它几乎是空的，是「物体 / 墙」分界的证据，
  但别把结论硬编码进来，让它自己显示
- 每簇一行：点数、最小外接矩形的最长边/短边、z 跨度、竖直度 |n_z|

为什么用最小面积外接矩形而不是 AABB 跨度
-----------------------------------------
斜放 45° 的物体，AABB 会把它的跨度放大到 √2 倍，于是"1.5 m 的物体"看起来
2.1 m，正好跨过 2 m 分界。旋转卡壳（rotating calipers）给出的最长边与
min_rotate_rect.cpp 里 max(length, width) 同义。AABB 跨度也一并打印，便于对照。

两种取数
--------
--live  抓 cluster_live.py 正在推给浏览器的那一帧（SSE）。
        为什么不用 ros2 topic echo：这台机器上给饱和话题新加订阅者收不到数据，
        而 SSE 流是页面已经在收的东西 —— 取到的就是用户眼睛看到的。
--xyz   离线。链条与 perception 对齐（已验证最大簇跨度与实时帧吻合）：
        voxel(0.1) → ROR(0.5,5) → remove_ground → voxel(V) → ROR(0.5,5) → DBSCAN(eps,3)
        注意与 cluster_eval.build_gt 的链条不同（那条省掉了前面 0.1 细体素和 ROR），
        所以点数会有差，绝对值只用于横向比较。

用法
----
    python3 cluster_shape.py --live                 # 抓一帧实时数据
    python3 cluster_shape.py --xyz /tmp/person.xyz --rev 27 --voxel 0.3 --eps 0.7
"""
import argparse
import base64
import json
import os
import sys

import numpy as np
from scipy.spatial import ConvexHull, QhullError

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cluster_eval import (                                  # noqa: E402
    dbscan_kdtree, radius_filter, remove_ground, voxel_downsample)

# 分档边界（米）。实测跨度分布是双峰的：≤2 m 与 >3 m 两档，中间几乎是空的。
BANDS = [1.0, 2.0, 3.0]


def min_area_rect_longest(xy):
    """旋转卡壳：最小面积外接矩形。返回 (最长边, 短边, 角度弧度, 面积)。

    点太少（1/2 个）时矩形退化，直接按点距给一个等效跨度。
    """
    n = len(xy)
    if n == 0:
        return 0.0, 0.0, 0.0, 0.0
    if n == 1:
        return 0.0, 0.0, 0.0, 0.0
    if n == 2:
        d = float(np.linalg.norm(xy[0] - xy[1]))
        return d, 0.0, 0.0, 0.0
    try:
        hull = ConvexHull(xy)
    except QhullError:
        # 共线点：退化成一条线段
        d = float(np.max(np.linalg.norm(xy[:, None] - xy[None, :], axis=2)))
        return d, 0.0, 0.0, 0.0
    h = xy[hull.vertices]
    edges = np.roll(h, -1, axis=0) - h
    best = None
    for a in np.arctan2(edges[:, 1], edges[:, 0]):
        c, s = np.cos(-a), np.sin(-a)
        # 把凸包转到该边与 x 轴平行，再取 AABB
        qx = h[:, 0] * c - h[:, 1] * s
        qy = h[:, 0] * s + h[:, 1] * c
        w = float(qx.max() - qx.min())
        hh = float(qy.max() - qy.min())
        if best is None or w * hh < best[3]:
            best = (max(w, hh), min(w, hh), float(a), w * hh)
    return best


def verticality(pts):
    """|n_z| —— 用 PCA 最小特征向量当法向。**只对单面簇有意义。**

    单个平面（一面墙、柜体侧面）的 |n_z| 接近 0。实测簇 37（11.86 m 长、
    0.13 m 厚的那面墙）报 0.034，正确。

    但对**合并簇**这个量会误导：簇 36 是四面墙连成的一个环，几何上是个筒，
    PCA 的最小方差方向落在 z 轴，于是报 1.000 —— 一个竖直墙环被报成"水平面"。
    所以判"是不是墙"要看**薄度 = 短边/最长边**（见 describe 里的 thin），
    单面墙约 0.01，环状合并簇约 0.9。这个量在轴对齐或斜放时都成立。
    """
    if len(pts) < 3:
        return float('nan')
    c = pts - pts.mean(axis=0)
    cov = c.T @ c / len(pts)
    w, v = np.linalg.eigh(cov)
    return float(abs(v[2, 0]))          # 最小特征值对应的特征向量


def describe(pts, labels):
    """把一帧的 (points, labels) 算成结构指标字典。labels < 0 视为噪点。"""
    labels = np.asarray(labels)
    noise_mask = labels < 0
    ids = np.unique(labels[~noise_mask])

    clusters = []
    for cid in ids:
        sel = labels == cid
        p = pts[sel]
        span, side, ang, area = min_area_rect_longest(p[:, :2])
        clusters.append({
            'id': int(cid),
            'n': int(sel.sum()),
            'span': span,              # 最小外接矩形最长边 = 与 min_rotate_rect 同义
            'side': side,              # 短边
            # 薄度：单面墙 ~0.01，环状合并簇 ~0.9。判"是不是墙"看这个，
            # 不看 nz —— 理由见 verticality 的注释。
            'thin': (side / span) if span > 1e-9 else float('nan'),
            'area': area,
            'zspan': float(p[:, 2].max() - p[:, 2].min()) if len(p) else 0.0,
            'aabb_x': float(p[:, 0].max() - p[:, 0].min()) if len(p) else 0.0,
            'aabb_y': float(p[:, 1].max() - p[:, 1].min()) if len(p) else 0.0,
            'nz': verticality(p),
        })
    clusters.sort(key=lambda c: -c['n'])

    n_total = len(pts)
    n_noise = int(noise_mask.sum())

    # 分档：桶式（(0,1] (1,2] (2,3] (3,∞)），不是累积式
    edges = [0.0] + BANDS + [np.inf]
    bands = []
    for i in range(len(edges) - 1):
        lo, hi = edges[i], edges[i + 1]
        sel = [c for c in clusters if lo < c['span'] <= hi]
        bands.append({
            'lo': lo, 'hi': hi,
            'n_clusters': len(sel),
            'n_points': int(sum(c['n'] for c in sel)),
            'pt_frac': (sum(c['n'] for c in sel) / n_total) if n_total else 0.0,
        })

    largest = clusters[0]['n'] if clusters else 0
    return {
        'n_points': n_total,
        'n_clusters': len(clusters),
        'n_noise': n_noise,
        'noise_frac': (n_noise / n_total) if n_total else 0.0,
        'largest_frac': (largest / n_total) if n_total else 0.0,
        'largest_n': largest,
        'bands': bands,
        'clusters': clusters,
    }


def fmt_band(lo, hi):
    if hi == np.inf:
        return '> %.0f m' % lo
    if lo == 0.0:
        return '<= %.0f m' % hi
    return '%.0f~%.0f m' % (lo, hi)


def report(d, title=''):
    out = []
    if title:
        out.append(title)
    out.append('  点数 %d   簇数 %d   噪点 %d (%.1f%%)   最大簇 %d 点 = %.1f%%'
               % (d['n_points'], d['n_clusters'], d['n_noise'],
                  d['noise_frac'] * 100, d['largest_n'], d['largest_frac'] * 100))
    out.append('')
    out.append('  跨度分档（按最小外接矩形最长边）')
    out.append('    %-10s %8s %10s %10s' % ('档', '簇数', '点数', '覆盖占比'))
    for b in d['bands']:
        star = ''
        if b['lo'] == 2.0 and b['hi'] == 3.0:
            star = '   <- 物体/墙分界（实测几乎为空）'
        out.append('    %-10s %8d %10d %9.1f%%%s'
                   % (fmt_band(b['lo'], b['hi']), b['n_clusters'],
                      b['n_points'], b['pt_frac'] * 100, star))
    out.append('')
    out.append('  各簇（按点数降序）')
    out.append('    %5s %7s %9s %9s %8s %7s %8s %8s'
               % ('簇', '点数', '最长边', '短边', '薄度', 'z跨度', 'AABBx', '竖直度'))
    for c in d['clusters']:
        nz = '  nan' if np.isnan(c['nz']) else '%8.3f' % c['nz']
        out.append('    %5d %7d %9.2f %9.2f %8.3f %7.2f %8.2f%s'
                   % (c['id'], c['n'], c['span'], c['side'], c['thin'],
                      c['zspan'], c['aabb_x'], nz))
    out.append('')
    out.append('  注：薄度 = 短边/最长边，单面墙约 0.01、环状合并簇约 0.9，'
               '判"是不是墙"看它；')
    out.append('      竖直度 |n_z| 只对单面簇有意义，合并簇会退化（见 verticality 注释）。')
    return '\n'.join(out)


# ───────────────────────── 取数 ─────────────────────────

def grab_live(n_frames, host='127.0.0.1', port=8765):
    """从 cluster_live.py 的 SSE 流里抓帧。

    标签是**追踪后的 id**（cluster_live 推的是 tracker 的输出），不是原始簇号。
    对结构指标无影响 —— 追踪不改变一簇的点集，只给它一个跨帧稳定的号。
    """
    import http.client

    def dec(b64, dt='<i2'):
        if not b64:
            return np.empty(0, dtype=dt)
        return np.frombuffer(base64.b64decode(b64), dtype=dt)

    c = http.client.HTTPConnection(host, port, timeout=30)
    c.request('GET', '/stream')
    r = c.getresponse()
    if r.status != 200:
        raise SystemExit('SSE 返回 HTTP %s —— cluster_live.py 起了吗？' % r.status)

    buf, frames = b'', []
    while len(frames) < n_frames:
        chunk = r.read(4096)
        if not chunk:
            break
        buf += chunk
        while b'\n\n' in buf:
            raw, buf = buf.split(b'\n\n', 1)
            for line in raw.split(b'\n'):
                if line.startswith(b'data: '):
                    frames.append(json.loads(line[6:]))
    c.close()
    if not frames:
        raise SystemExit('SSE 流里没读到帧')

    last = frames[-1]
    pt = dec(last['pt']).astype(np.float64).reshape(-1, 3) / 1000.0
    lab = dec(last['lab']).astype(np.int64)
    if len(pt) != len(lab):
        raise SystemExit('点数 %d 与标签数 %d 不对齐' % (len(pt), len(lab)))
    return pt, lab, 'SSE 第 %d 帧（共抓 %d 帧）' % (len(frames) - 1, len(frames))


def chain_xyz(path, rev, voxel, eps, min_pts, cache='/tmp/_xyz.npz'):
    """离线复现 perception 的聚类输入，再跑 DBSCAN。

    第一段（0.1 细体素 + ROR）是 perception 在降采样之前做的，build_gt 那条链
    省掉了它，所以这里必须补上，否则点数与实时对不上。
    """
    if os.path.exists(cache):
        d = np.load(cache)
        X, ha = d['X'], d['ha']
    else:
        raw = np.loadtxt(path, delimiter=',', usecols=(0, 1, 2, 3))
        X, ha = raw[:, :3], raw[:, 3]
        np.savez(cache, X=X, ha=ha)

    cuts = np.flatnonzero(np.diff(ha) < -180) + 1
    segs = [s for s in np.split(np.arange(len(ha)), cuts) if len(s) > 20000]
    if not segs:
        raise SystemExit('切不出一圈（点数太少？）')
    if rev >= len(segs):
        raise SystemExit('--rev %d 超范围，只有 %d 圈' % (rev, len(segs)))
    pts = X[segs[rev]]

    p, _ = voxel_downsample(pts, 0.1)
    p = radius_filter(p, 0.5, 5)
    p, _ = remove_ground(p)
    p, _ = voxel_downsample(p, voxel)
    p = radius_filter(p, 0.5, 5)
    lab = dbscan_kdtree(p, eps, min_pts)
    return p, lab, '第 %d 圈  体素 %.2f  eps %.2f  min_pts %d' % (rev, voxel, eps, min_pts)


def main():
    ap = argparse.ArgumentParser(description='聚类结构指标（不依赖真值）')
    ap.add_argument('--live', action='store_true', help='从 SSE 抓实时帧')
    ap.add_argument('--xyz', help='离线：驱动的 xyz dump')
    ap.add_argument('--rev', type=int, default=27, help='取第几圈（--xyz）')
    ap.add_argument('--voxel', type=float, default=0.30)
    ap.add_argument('--eps', type=float, default=0.70)
    ap.add_argument('--min-pts', type=int, default=3)
    ap.add_argument('--frames', type=int, default=3, help='SSE 抓几帧（取最后一帧）')
    ap.add_argument('--json', help='把指标另存成 json')
    a = ap.parse_args()

    if a.live and a.xyz:
        raise SystemExit('--live 与 --xyz 只能选一个')
    if a.live:
        pts, lab, title = grab_live(a.frames)
    elif a.xyz:
        pts, lab, title = chain_xyz(a.xyz, a.rev, a.voxel, a.eps, a.min_pts)
    else:
        raise SystemExit('要指定 --live 或 --xyz <文件>')

    d = describe(pts, lab)
    print(report(d, title))
    if a.json:
        with open(a.json, 'w') as f:
            json.dump(d, f, indent=1, ensure_ascii=False)
        print('\n  指标已写到 %s' % a.json)
    return 0


if __name__ == '__main__':
    sys.exit(main())
