#!/usr/bin/env python3
"""近场解剖图：判断 x 2~4 m 那坨是"车盖"还是"别的东西"。

判别思路：
  - 车盖是**近水平**的面，且相对车辆中线**左右对称**
  - 墙/别的车是**立起来**的面，或左右不对称
所以画：放大俯视（按高度分层上色）+ 左/右两侧分别的侧视 + 两处 y-z 剖面。
把框内那坨（已知是自车）和 x2~4 那坨的剖面并排放，形状一样就是同一类东西。
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
from matplotlib.colors import LinearSegmentedColormap, BoundaryNorm

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

CAR = dict(xmin=-5.0, xmax=1.0, ymin=-2.0, ymax=2.0)
GZ = -1.76          # 地面在传感器坐标系下的高度

# 高度分层：顺序色阶（单一色相由浅到深），高度是量而不是身份
BANDS = [0.0, 0.3, 0.6, 1.0, 1.4, 2.0, 3.0]
BAND_LBL = ['0~0.3', '0.3~0.6', '0.6~1.0', '1.0~1.4', '1.4~2.0', '2.0~3.0']
CMAP = LinearSegmentedColormap.from_list(
    'h', ['#cfe3f7', '#8fbdea', '#3987e5', '#1d4f8f', '#c98500', '#8a5a00'])
NORM = BoundaryNorm(BANDS, CMAP.N)


def grab(timeout=25.0):
    rclpy.init()
    node = Node('ego_zoom')
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
    h = p[:, 2] - GZ                        # 离地高度
    print(f'{len(p):,} 点')

    fig = plt.figure(figsize=(19, 16))
    XL, YL = 8.0, 6.0
    YC = 3.0        # 剖面只看到 ±3 m，再宽就把旁边的侧墙卷进来了

    # ── A 放大俯视，按离地高度分层 ────────────────────────────
    ax = fig.add_subplot(3, 2, 1)
    s = p[(np.abs(p[:, 0]) < XL) & (np.abs(p[:, 1]) < YL) & (h > 0.05)]
    sh = h[(np.abs(p[:, 0]) < XL) & (np.abs(p[:, 1]) < YL) & (h > 0.05)]
    sc = ax.scatter(s[:, 1], s[:, 0], s=2.2, c=sh, cmap=CMAP, norm=NORM,
                    linewidths=0)
    cb = fig.colorbar(sc, ax=ax, ticks=BANDS[:-1] + [BANDS[-1]])
    cb.set_label(L('离地高度 (m)', 'height (m)'))
    ax.add_patch(plt.Rectangle((CAR['ymin'], CAR['xmin']),
                               CAR['ymax']-CAR['ymin'], CAR['xmax']-CAR['xmin'],
                               fill=False, ec='#d55181', lw=2.5))
    ax.scatter([0], [0], marker='*', s=380, c='#c98500', zorder=6)
    ax.set_xticks(np.arange(-6, 6.1, 1)); ax.set_yticks(np.arange(-8, 8.1, 1))
    ax.set_xticks(np.arange(-6, 6.1, 0.5), minor=True)
    ax.set_yticks(np.arange(-8, 8.1, 0.5), minor=True)
    ax.grid(which='major', alpha=.45)
    ax.grid(which='minor', alpha=.16, lw=.5)
    # 把 x1~2 那段真空框出来，这才是要用户判断的地方
    ax.add_patch(plt.Rectangle((-YC, 1.0), 2*YC, 1.0, color='#199e70',
                               alpha=.12, zorder=0))
    ax.annotate(L('x 1~2 m 真空', 'x 1-2 m gap'), (YC-0.2, 1.5),
                ha='right', fontsize=9, color='#0f6b4c')
    ax.set_xlim(-6, 6); ax.set_ylim(-8, 8)
    ax.set_title(L('A 俯视放大（按离地高度上色，0.5 m 网格）',
                   'A zoomed top view, colored by height'))
    ax.set_xlabel('y (m)'); ax.set_ylabel('x (m)'); ax.set_aspect('equal')

    # ── B/C/D 侧视：全部 / 左侧(y<0) / 右侧(y>0) ─────────────
    def side(axx, sel, title):
        q = p[sel]
        hh = h[sel]
        axx.scatter(q[:, 0], q[:, 2], s=2.0, c=hh, cmap=CMAP, norm=NORM,
                    linewidths=0)
        axx.axhline(0, color='#c98500', lw=1.4)
        axx.axhline(GZ, color='#3987e5', lw=1.4)
        axx.axvspan(CAR['xmin'], CAR['xmax'], color='#d55181', alpha=.13)
        axx.axvline(1.0, color='#d55181', lw=1.2, ls='--')
        axx.axvline(2.0, color='#199e70', lw=1.2, ls='--')
        axx.set_xlim(-XL, XL); axx.set_ylim(-2.2, 3.2)
        axx.grid(alpha=.3)
        axx.set_title(title); axx.set_xlabel('x (m)'); axx.set_ylabel('z (m)')

    m_all = np.abs(p[:, 0]) < XL
    side(fig.add_subplot(3, 2, 2), m_all & (np.abs(p[:, 1]) < 1.0),
         L('B 侧视 x-z（中心走廊 |y|<1）\n粉=当前框 红虚线=x1 绿虚线=x2',
           'B side view, |y|<1'))
    side(fig.add_subplot(3, 2, 3), m_all & (p[:, 1] < -0.5) & (p[:, 1] > -YC),
         L('C 侧视 x-z（左侧 -3<y<-0.5）', 'C side view, -3<y<-0.5'))
    side(fig.add_subplot(3, 2, 4), m_all & (p[:, 1] > 0.5) & (p[:, 1] < YC),
         L('D 侧视 x-z（右侧 0.5<y<3）', 'D side view, 0.5<y<3'))

    # ── E/F y-z 剖面：把"框内那坨"和"x2~4 那坨"并排 ──────────
    def prof(axx, sel, title):
        q = p[sel]
        hh = h[sel]
        axx.scatter(q[:, 1], q[:, 2], s=2.2, c=hh, cmap=CMAP, norm=NORM,
                    linewidths=0)
        axx.axhline(0, color='#c98500', lw=1.4)
        axx.axhline(GZ, color='#3987e5', lw=1.4)
        axx.axvline(0, color='#199e70', lw=1.2, ls='--')
        axx.set_xlim(-YC, YC); axx.set_ylim(-2.2, 3.2)
        axx.grid(alpha=.3)
        axx.set_title(title)
        axx.set_xlabel('y (m)'); axx.set_ylabel('z (m)')

    prof(fig.add_subplot(3, 2, 5),
         (p[:, 0] > -5.0) & (p[:, 0] < 1.0) & (np.abs(p[:, 1]) < YC),
         L('E y-z 剖面：x∈(-5,1) 已知自车那坨（|y|<3）',
           'E cross-section, x in (-5,1)'))
    prof(fig.add_subplot(3, 2, 6),
         (p[:, 0] > 1.5) & (p[:, 0] < 4.5) & (np.abs(p[:, 1]) < YC),
         L('F y-z 剖面：x∈(1.5,4.5) 待判那坨（|y|<3）',
           'F cross-section, x in (1.5,4.5)'))

    fig.suptitle(L('近场解剖：E 和 F 的剖面形状像不像？',
                   'near-field anatomy'), fontsize=15)
    fig.tight_layout()
    out = '/home/qq/carProj/ego_zoom.png'
    fig.savefig(out, dpi=100)
    print(f'图已存到 {out}')

    # 数字对照：两坨的剖面统计
    print('\n两坨的剖面统计（|y|<3，离地高度分布）：')
    for name, sel in [('框内 x∈(-5,1)  ', (p[:, 0] > -5) & (p[:, 0] < 1)),
                      ('待判 x∈(1.5,4.5)', (p[:, 0] > 1.5) & (p[:, 0] < 4.5))]:
        sel = sel & (np.abs(p[:, 1]) < YC)
        q, hh = p[sel], h[sel]
        if not len(q):
            continue
        print(f'  {name}  {len(q):>7,} 点   '
              f'y {np.percentile(q[:,1],1):6.2f}..{np.percentile(q[:,1],99):6.2f}'
              f'（1~99%）   离地 {hh.min():5.2f}..{hh.max():5.2f} m   '
              f'中位 {np.median(hh):5.2f} m')

    # 中心走廊逐 0.5 m 扫一遍，把"真空"和"那坨"精确定位
    print('\n中心走廊 |y|<1 沿 x 逐 0.5 m（离地点 z>-1.71）：')
    band = p[(np.abs(p[:, 1]) < 1.0) & (p[:, 2] > GZ + 0.05)]
    for a in np.arange(-8, 8, 0.5):
        c = band[(band[:, 0] >= a) & (band[:, 0] < a + 0.5)]
        tag = '  ← 框内' if (CAR['xmin'] <= a < CAR['xmax']) else ''
        zr = (f'z {c[:,2].min():5.2f}..{c[:,2].max():5.2f}'
              if len(c) else ' ' * 16)
        print(f'  x {a:>5.1f}~{a+0.5:<5.1f} {len(c):>7,}   {zr}{tag}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
