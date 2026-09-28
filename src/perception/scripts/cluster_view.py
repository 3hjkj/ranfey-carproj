#!/usr/bin/env python3
"""把 perception 的聚类结果画出来，和真值并排比 —— 指标表格之外的"看一眼"。

为什么要这张图
--------------
cluster_eval.py 给的是六个数字。数字能告诉你**有没有变好**，但告诉不了你
**错在哪**：过分割还是欠分割？错在墙上还是在目标上？这些必须看空间分布。

编码方式（这张图唯一重要的决定）
--------------------------------
两个面板**用同一套颜色**，颜色 = 该点所属的**真值簇**，这样任何色块错位
都是分歧。具体：

  左：真值自己的点（57 圈累积），按真值簇上色
  右：perception 的点（单帧），按"这个 perception 簇主要落在哪个真值簇上"
      继承同一个颜色

关键是**继承规则**：一个真值簇的颜色只发给与它重叠最多的那一个 perception
簇（"嫡系"）。于是：

    真值的一个簇被切成两块 → 右图只有一块是彩色的，另一块**变灰**
    ↓
    彩色区域旁边/里面出现灰块 = **过分割**（perception 把一整块切碎了）
    两个颜色的交界处右图是一个颜色 = **欠分割**（两个目标被并成一个）

灰在右图里就是"嫌疑"标记，不需要额外画边界线。

放大行会**按框内的簇重新排一次序**再上色，否则小目标（人）在这里是灰的
——它点数少，进不了全场景前八。标题里写了这件事，别把它当成两行同色。

为什么最多八种颜色
------------------
这张图是散点，任意两个色块都可能在空间上挨着。dataviz 的配色参考里，
默认色板的八槽只保证了**相邻对**达标（栈图/柱图那种有序的相邻），
所以这里按"点数排序 + 固定槽序"发色，让最可能挨着的几个大簇拿到色板里
分离度最好的几个槽，而不是随手乱发。超过八个折叠成灰。

用法
----
    source /opt/ros/humble/setup.bash
    source ~/carProj/install/setup.bash
    # 需要 perception 在跑（订阅 /rfans_points_replay）
    ./cluster_view.py --xyz /tmp/person.xyz --out /tmp/cluster_view.png

依赖回放节点提供 /rfans_points_replay（做浅灰背景层用），--no-raw 可关。
"""

import argparse
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.lines import Line2D

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cluster_eval import LABEL_DTYPE, LABEL_TOPIC, build_gt, evaluate  # noqa: E402

RAW_TOPIC = '/rfans_points_replay'

# 驱动的 40 字节点布局，抄自 rfans_replay.py
RAW_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'),
                      ('v_angle', '<f4'), ('h_angle', '<f4'), ('range', '<f4'),
                      ('timestamp', '<f8'), ('laserid', '<i4')])

# ---- 配色：dataviz/palette.md 默认实例的分类色板，槽序照抄（槽序本身是
#      CVD 安全机制的一部分，不要重排）----
SERIES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100',
          '#e87ba4', '#008300', '#4a3aa7', '#e34948']
OTHER = '#9a988f'      # 折叠成"其他 / 可疑"的灰
NOISE = '#0b0b0b'      # 噪点
SURFACE = '#fcfcfb'
GRID = '#e1e0d9'
INK = '#0b0b0b'
INK2 = '#52514e'
BACKDROP = '#d8d7d0'   # 原始帧背景层

# 这台机器上 fontconfig 里没有 CJK 字体，但 /mnt/c 下有 Windows 的。
CJK_CANDIDATES = [
    '/mnt/c/Windows/Fonts/simhei.ttf',
    '/mnt/c/Windows/Fonts/msyh.ttc',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',
]


def setup_cjk_font():
    for p in CJK_CANDIDATES:
        if os.path.exists(p):
            font_manager.fontManager.addfont(p)
            name = font_manager.FontProperties(fname=p).get_name()
            plt.rcParams['font.family'] = ['sans-serif']
            plt.rcParams['font.sans-serif'] = [name, 'DejaVu Sans']
            plt.rcParams['axes.unicode_minus'] = False
            return name
    print('警告：没找到中文字体，图上中文会是方框（装 fonts-wqy-zenhei，'
          '或在 CJK_CANDIDATES 里加路径）')
    return None


class Capture(Node):
    """抓一帧标签 + 一帧原始点云。场景静止，不严格对齐时间。"""

    def __init__(self, want_raw):
        super().__init__('cluster_view')
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST)
        self.lab = None
        self.raw = None
        self.want_raw = want_raw
        self.create_subscription(PointCloud2, LABEL_TOPIC, self._on_lab, q)
        if want_raw:
            self.create_subscription(PointCloud2, RAW_TOPIC, self._on_raw, q)

    def _on_lab(self, msg):
        if self.lab is None:
            self.lab = np.frombuffer(msg.data, dtype=LABEL_DTYPE).copy()

    def _on_raw(self, msg):
        if self.raw is None:
            self.raw = np.frombuffer(msg.data, dtype=RAW_DTYPE).copy()

    def done(self):
        return self.lab is not None and (self.raw is not None or not self.want_raw)


def assign_colors(gt_labels, plab, pgt, gt_in, pred_in, n_slots=len(SERIES)):
    """按"真值排序发色 + 嫡系继承"给两套标签上色。

    gt_in / pred_in 是布尔掩码，限定"拿哪些点来排序"——放大行只按框内的簇
    排序，否则小目标永远进不了前八，放大行就白放大了。

    返回 (gt_color, pred_color, top, stats)。pred_color 里 None 表示灰。
    """
    uniq, cnt = np.unique(gt_labels[gt_in & (gt_labels >= 0)], return_counts=True)
    order = [int(c) for c in uniq[np.argsort(-cnt)]]
    top = order[:n_slots]
    gt_color = {c: SERIES[i] for i, c in enumerate(top)}   # 第九名开外不给色

    # 列联表：每个 perception 簇落在各个真值簇上的点数
    m = pred_in & (plab >= 0) & (pgt >= 0)
    overlap = {}
    for p, g in zip(plab[m].tolist(), pgt[m].tolist()):
        overlap[(p, g)] = overlap.get((p, g), 0) + 1
    best = {}          # pred 簇 -> (重叠点数, 主要落在的 gt 簇)
    for (p, g), n in overlap.items():
        if n > best.get(p, (0, -1))[0]:
            best[p] = (n, g)

    # 每个真值簇只认一个嫡系：与它重叠最多的 perception 簇
    owner = {}         # gt 簇 -> (重叠点数, 嫡系 pred 簇)
    for p, (n, g) in best.items():
        if g in gt_color and (g not in owner or n > owner[g][0]):
            owner[g] = (n, p)
    heir = {n_p: g for g, (n, n_p) in owner.items()}

    pred_color = {}
    frag, small, orphan = 0, 0, 0
    for p in np.unique(plab[pred_in & (plab >= 0)]).tolist():
        p = int(p)
        g = best.get(p, (0, -1))[1]
        if heir.get(p) is not None:
            pred_color[p] = gt_color[g]
        else:
            pred_color[p] = None
            if g in gt_color:
                frag += 1      # 落在前八真值簇上，但不是嫡系 → 过分割碎片
            elif g >= 0:
                small += 1     # 落在小真值簇上 → 只是没排进前八
            else:
                orphan += 1    # 压根对不上任何真值簇
    return gt_color, pred_color, top, dict(frag=frag, small=small, orphan=orphan)


def paint(labels, cmap, none_color=OTHER):
    """把 {簇号: 颜色或 None} 展开成逐点颜色数组。None 用灰。"""
    lut = {c: (col if col else none_color) for c, col in cmap.items()}
    return np.array([lut.get(int(c), none_color) for c in labels])


def pick_zoom(gt_pts, gt_labels, max_span, min_pts, half):
    """找小目标最集中的地方做放大框：取点数最多的那个小簇，开一个 2*half 的窗。"""
    best = None
    for c in np.unique(gt_labels[gt_labels >= 0]):
        p = gt_pts[gt_labels == c]
        if len(p) < min_pts or p[:, 0].ptp() > max_span or p[:, 1].ptp() > max_span:
            continue
        if best is None or len(p) > best[0]:
            best = (len(p), p[:, 0].mean(), p[:, 1].mean(), int(c))
    if best is None:
        return None
    _, cx, cy, c = best
    return (cx - half, cx + half, cy - half, cy + half, c)


def main():
    ap = argparse.ArgumentParser(description='聚类结果可视化对照')
    ap.add_argument('--xyz', required=True, help='原始 xyz（用于取真值）')
    ap.add_argument('--out', default='/tmp/cluster_view.png')
    ap.add_argument('--no-raw', action='store_true', help='不画原始帧背景层')
    ap.add_argument('--gt-voxel', type=float, default=0.30)
    ap.add_argument('--gt-radius', type=float, default=0.5)
    ap.add_argument('--gt-search-num', type=int, default=5)
    ap.add_argument('--gt-eps', type=float, default=0.60)
    ap.add_argument('--gt-min-pts', type=int, default=3)
    ap.add_argument('--gt-max-dist', type=float, default=0.30)
    ap.add_argument('--zoom-span', type=float, default=3.0,
                    help='跨度小于这个值的真值簇算"小目标"')
    ap.add_argument('--zoom-half', type=float, default=4.0,
                    help='放大框的半边长（米）')
    args, _ = ap.parse_known_args()

    print(f'中文字体: {setup_cjk_font()}')

    stem = os.path.splitext(os.path.basename(args.xyz))[0]
    cache = (f'/tmp/gt_{stem}_ng_r{args.gt_radius}_{args.gt_search_num}'
             f'_v{args.gt_voxel}_e{args.gt_eps}_m{args.gt_min_pts}.npz')
    gt_pts, gt_labels = build_gt(args.xyz, cache, args.gt_voxel, args.gt_eps,
                                 args.gt_min_pts, radius=args.gt_radius,
                                 search_num=args.gt_search_num)
    tree = cKDTree(gt_pts)

    rclpy.init()
    node = Capture(not args.no_raw)
    print('等待一帧 /perception/lidar_cluster_labels ...')
    while rclpy.ok() and not node.done():
        rclpy.spin_once(node, timeout_sec=0.5)
    if node.lab is None:
        print('没收到标签。perception 在跑吗？')
        return 1
    arr, raw = node.lab, node.raw
    node.destroy_node()
    rclpy.try_shutdown()

    pxyz = np.stack([arr['x'], arr['y'], arr['z']], 1).astype(np.float64)
    plab = arr['label'].astype(np.int64)
    print(f'拿到 {len(pxyz)} 个带标签的点，簇 {len(np.unique(plab[plab >= 0]))} 个；'
          f'背景帧 {0 if raw is None else len(raw)} 点')

    # perception 点回投真值：两侧都是 0.3m 体素质心，不重合，取最近邻
    dist, idx = tree.query(pxyz, k=1)
    good = dist <= args.gt_max_dist
    pgt = np.full(len(pxyz), -1, dtype=np.int64)
    pgt[good] = gt_labels[idx[good]]

    valid = pgt >= 0
    met = evaluate(pxyz[valid], plab[valid], pgt[valid]) if valid.sum() >= 10 else {}

    # ---- 视野：全场景 + 小目标放大 ----
    allp = np.vstack([gt_pts[:, :2], pxyz[:, :2]])
    rows = [('全场景', (allp[:, 0].min() - 1, allp[:, 0].max() + 1,
                        allp[:, 1].min() - 1, allp[:, 1].max() + 1))]
    z = pick_zoom(gt_pts, gt_labels, args.zoom_span, 15, args.zoom_half)
    if z:
        rows.append(('小目标放大', (z[0], z[1], z[2], z[3])))
        print(f'放大框以真值簇 {z[4]} 为中心: x[{z[0]:.1f},{z[1]:.1f}] '
              f'y[{z[2]:.1f},{z[3]:.1f}]')

    fig, axes = plt.subplots(len(rows), 2, figsize=(13.5, 5.9 * len(rows)),
                             facecolor=SURFACE, squeeze=False)
    notes = []
    row_tops = []
    for r, (rname, (bx0, bx1, by0, by1)) in enumerate(rows):
        inbox = lambda p: ((p[:, 0] >= bx0) & (p[:, 0] <= bx1) &
                           (p[:, 1] >= by0) & (p[:, 1] <= by1))
        gtc, predc, top, st = assign_colors(gt_labels, plab, pgt,
                                            inbox(gt_pts), inbox(pxyz))
        row_tops.append(top)
        notes.append(f'{rname}: 前八真值簇 ' +
                     '/'.join(str(c) for c in top) +
                     f'；perception 灰掉的簇 {st["frag"]} 个过分割碎片、'
                     f'{st["small"]} 个小目标、{st["orphan"]} 个对不上')

        for c, (who, pts, lab, cmap) in enumerate((
                ('真值', gt_pts, gt_labels, gtc),
                ('perception', pxyz, pgt, predc))):
            ax = axes[r][c]
            ax.set_facecolor(SURFACE)
            if raw is not None:
                ax.scatter(raw['x'], raw['y'], s=0.25, c=BACKDROP,
                           linewidths=0, rasterized=True, zorder=1)

            if c == 0:
                ok = lab >= 0
                col = paint(lab[ok], cmap)          # 按**真值**簇号查 gtc
            else:
                # 右图按 **perception 簇号**查 predc —— 查错了键就会整片掉进
                # 灰色兜底，看着像"全被切碎"，其实只是字典没命中。这里的 lab
                # 是 pgt（真值簇号），不能拿来当 predc 的键，要用 plab。
                ok = plab >= 0
                col = np.array([predc.get(int(p)) or OTHER for p in plab[ok]])
            ax.scatter(pts[ok, 0], pts[ok, 1], s=9, c=col, linewidths=0,
                       rasterized=True, zorder=3)
            if (~ok).any():
                ax.scatter(pts[~ok, 0], pts[~ok, 1], s=6, c=NOISE, linewidths=0,
                           alpha=0.75, zorder=4)

            if c == 0:
                sub = f'真值（57 圈累积，{len(pts)} 点）'
                graynote = '灰 = 未进前八的簇'
            else:
                sub = f'perception（单帧，{len(pts)} 点，' \
                      f'{len(np.unique(plab[plab >= 0]))} 簇）'
                graynote = '灰 = 过分割碎片/小目标'

            ax.set_xlim(bx0, bx1)
            ax.set_ylim(by0, by1)
            ax.set_aspect('equal', adjustable='box')
            ax.grid(True, color=GRID, linewidth=0.6, zorder=0)
            ax.set_axisbelow(True)
            for s in ('top', 'right'):
                ax.spines[s].set_visible(False)
            for s in ('left', 'bottom'):
                ax.spines[s].set_color('#c3c2b7')
            ax.tick_params(colors=OTHER, labelsize=8, length=3)
            ranknote = '' if r == 0 else '　※框内重排上色'
            ax.set_title(f'{rname} — {sub}\n{graynote}{ranknote}', color=INK,
                         fontsize=11, pad=6, linespacing=1.5)
            if r == len(rows) - 1:
                ax.set_xlabel('x (m)', color=INK2, fontsize=10)
            if c == 0:
                ax.set_ylabel('y (m)', color=INK2, fontsize=10)

    # 图例只讲第一行（全场景）的映射。放大行按框内重排过，颜色对不上，
    # 那句说明写在放大行的标题里 —— 之前这里用了循环残留的 top，图例和
    # 画面是两张皮，别再用循环变量。
    handles = [Line2D([], [], marker='o', ls='', ms=7, mfc=SERIES[i], mec='none',
                      label=f'真值簇 #{topi}（全场景第 {i+1} 大）')
               for i, topi in enumerate(row_tops[0][:len(SERIES)])]
    handles += [
        Line2D([], [], marker='o', ls='', ms=4, mfc=NOISE, mec='none',
               label='真值噪点'),
        Line2D([], [], marker='x', ls='', ms=6, mfc=NOISE, mew=0.9,
               label='perception 噪点'),
    ]
    fig.legend(handles=handles, loc='lower center', ncol=5, frameon=False,
               fontsize=9, labelcolor=INK2, bbox_to_anchor=(0.5, 0.018))

    if met:
        txt = (f"本帧 {int(valid.sum())} 点对上真值，perception {len(np.unique(plab[plab >= 0]))} 簇 / "
               f"真值 {len(np.unique(pgt[valid]))} 簇　|　均一性 {met['homogeneity']:.3f}　"
               f"完整性 {met['completeness']:.3f}　V-measure {met['v_measure']:.3f}　"
               f"ARI {met['ARI']:.3f}　AMI {met['AMI']:.3f}　"
               f"轮廓系数 {met.get('silhouette', float('nan')):.3f}")
    else:
        txt = '点太少，本帧算不了指标'

    fig.suptitle('perception 聚类 vs 多圈累积真值　—　同色 = 同一个真值簇；'
                 '右图出现灰块 = 被切碎了', color=INK, fontsize=13, y=0.982)
    fig.text(0.5, 0.095, txt, ha='center', color=INK2, fontsize=9.5)
    fig.tight_layout(rect=(0, 0.14, 1, 0.962))
    fig.savefig(args.out, dpi=150, facecolor=SURFACE)
    print(f'图写到 {args.out}')
    for n in notes:
        print('  ' + n)
    if met:
        print('  ' + '  '.join(f'{k}={v:.4f}' for k, v in met.items()
                               if isinstance(v, float) and not np.isnan(v)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
