#!/usr/bin/env python3
"""照 grid_cluster.cpp 的顺序，在一帧真实点云上逐级复现预处理，数出每级吃掉多少。

对应关系（源码：architecture/lidar_objs/src/grid_cluster.cpp:62-93）：
    VoxelFilter(0.1)  → RadiusFilter(0.5,5) → GroundPoints(0.2,5)
    → VoxelFilter(0.3) → RadiusFilter(0.5,5) → DBSCAN

配置**来自两个文件**，别混（见下面 CFG 的注释）。
ROI 裁剪和车体框剔除都裹在 VoxelFilter 里（lidar_preprocess2.cpp:109-116）。
"""
import sys
import numpy as np
from scipy.spatial import cKDTree

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, '/home/qq/carProj/src/perception/scripts')
from cluster_view3d import RAW_DTYPE

# 两份配置是分开用的，别混：
#   裁剪范围 / 车体框 / 地面格网边长  →  preprocess.json（源码里的 proprecess_config_）
#   体素、半径滤波、地面高度阈值       →  cell.json（源码里的 objs_config_）
CFG = dict(xmin=-80.0, xmax=80.0, ymin=-25.0, ymax=25.0, zmin=-2.0, zmax=2.0,
           car_xmin=-1.5, car_xmax=3.9, car_ymin=-2.4, car_ymax=0.5,
           voxel_size=0.1, radius_search=0.5, search_num=5,
           threshold_h=0.2, points_num=5, cell_size_x=0.5, cell_size_y=0.5)


def grab_frame(timeout=20.0):
    rclpy.init()
    node = Node('trace_pipeline')
    box = {}
    q = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE,
                   history=HistoryPolicy.KEEP_LAST)
    node.create_subscription(PointCloud2, '/rfans_points',
                             lambda m: box.setdefault('m', m), q)
    import time
    t0 = time.time()
    while 'm' not in box and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    m = box.get('m')
    out = None if m is None else np.frombuffer(m.data, dtype=RAW_DTYPE).copy()
    node.destroy_node()
    rclpy.shutdown()
    return out


def crop(pts):
    """ROI 裁剪 + 车体框剔除 —— 和 VoxelFilter 里那两段 continue 一致。"""
    c = CFG
    keep = ((pts[:, 0] >= c['xmin']) & (pts[:, 0] <= c['xmax']) &
            (pts[:, 1] >= c['ymin']) & (pts[:, 1] <= c['ymax']) &
            (pts[:, 2] >= c['zmin']) & (pts[:, 2] <= c['zmax']))
    pts = pts[keep]
    in_car = ((pts[:, 0] > c['car_xmin']) & (pts[:, 0] < c['car_xmax']) &
              (pts[:, 1] > c['car_ymin']) & (pts[:, 1] < c['car_ymax']))
    return pts[~in_car]


def voxel(pts, size):
    """PCL VoxelGrid 语义：每个叶子里取质心。"""
    if len(pts) == 0:
        return pts
    keys = np.floor((pts - pts.min(0)) / size).astype(np.int64)
    _, inv = np.unique(keys, axis=0, return_inverse=True)
    n = inv.max() + 1
    out = np.zeros((n, 3))
    cnt = np.bincount(inv)
    for k in range(3):
        out[:, k] = np.bincount(inv, weights=pts[:, k]) / cnt
    return out


def radius_filter(pts, r, min_pts):
    """PCL RadiusOutlierRemoval：半径 r 内的邻居数（含自身）>= min_pts 才留。"""
    if len(pts) == 0:
        return pts
    tree = cKDTree(pts)
    cnt = np.array([len(x) for x in tree.query_ball_point(pts, r)])
    return pts[cnt >= min_pts]


def ground_split(pts, threshold_h, points_num):
    """GetGridZmin + GetGroundPoints：0.3x0.3 格，点数 <=5 的格子判无效、
    里面的点**全当地面丢掉**；有效格子按 z - zmin > threshold_h 分地面/非地面。"""
    c = CFG
    if len(pts) == 0:
        return pts, pts
    gx = np.floor((pts[:, 0] - c['xmin']) / c['cell_size_x']).astype(np.int64)
    gy = np.floor((pts[:, 1] - c['ymin']) / c['cell_size_y']).astype(np.int64)
    key = gx * 100003 + gy
    order = np.argsort(key)
    key_s, z_s = key[order], pts[order, 2]
    bounds = np.flatnonzero(np.r_[True, key_s[1:] != key_s[:-1], True])
    ground = np.zeros(len(pts), bool)
    for i in range(len(bounds) - 1):
        idx = order[bounds[i]:bounds[i + 1]]
        if len(idx) <= points_num:          # 无效格子：整格丢掉
            ground[idx] = True
        else:
            ground[idx] = (pts[idx, 2] - pts[idx, 2].min()) <= threshold_h
    return pts[ground], pts[~ground]


def main():
    raw = grab_frame()
    if raw is None:
        print('没抓到 /rfans_points 的帧')
        return 1
    xyz = np.stack([raw['x'], raw['y'], raw['z']], 1).astype(np.float64)
    total = len(xyz)
    print(f"{'原始点云':<34}{total:>9,}  100.0%")

    def step(name, pts):
        print(f"{name:<34}{len(pts):>9,}  {len(pts)/total*100:5.1f}%"
              f"   本级去掉 {prev[0]-len(pts):>7,}")
        prev[0] = len(pts)
        return pts

    prev = [total]
    p = crop(xyz);                              step('  ① ROI裁剪 + 去车体框', p)
    p = voxel(p, CFG['voxel_size']);            step(f"  ② 体素 {CFG['voxel_size']} m", p)
    p = radius_filter(p, CFG['radius_search'],
                      CFG['search_num']);       step(f"  ③ 半径滤波 {CFG['radius_search']} m/{CFG['search_num']} 邻居", p)
    g, p = ground_split(p, CFG['threshold_h'],
                        CFG['points_num']);     step(f"  ④ 去地面 (地面 {len(g):,} 点被丢)", p)
    p = voxel(p, 0.3);                          step('  ⑤ 体素 0.3 m', p)
    p = radius_filter(p, CFG['radius_search'],
                      CFG['search_num']);       step('  ⑥ 半径滤波 (再来一遍)', p)
    print(f"\n进 DBSCAN 的点数 = {len(p):,}   （标签话题上实测约 1,768）")
    return 0


if __name__ == '__main__':
    sys.exit(main())
