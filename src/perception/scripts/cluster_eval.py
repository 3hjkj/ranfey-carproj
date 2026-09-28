#!/usr/bin/env python3
"""评估 perception 的激光雷达聚类质量。

指标
----
均一性 homogeneity / 完整性 completeness / V-measure / ARI / AMI / 轮廓系数。
前五个是"两套标签的对比"，必须有真值；轮廓系数不需要真值。

真值怎么来的（以及它的局限，务必先读）
--------------------------------------
这份数据没有任何人工标注，所以真值是**多圈累积自动生成**的，流程刻意
和 perception 自己那条链**逐级对齐**：

    person.xyz 全部点（约 218 万，含 57 个完整圈）
      → 去地面（复刻 lidar_preprocess 的栅格 zmin 法）  ← 这一步不能省
      → VoxelFilter(0.3) → RadiusFilter(0.5, 5)
      → DBSCAN(0.6, 3)   ← 与 lidar_cluster.cpp 用的参数相同
      → 每个点的簇号 = 该处的真值标签

于是两边**唯一的差别只剩输入**：真值吃 57 圈，perception 吃单圈。指标回答
的就是"单圈聚类相对稠密视角退化多少"这一个问题。

**去地面那一步漏掉过一次，后果很严重，记在这里：** 直接在未去地面的累积云上
跑 DBSCAN，地面/墙这类连续面必然连成几个横跨十几米的巨簇，把 perception 的
目标点吸进去 —— 实测 89.6% 的点落在跨度 >3m 的环境簇上，六个指标全部失真
（当时算出来 V=0.84，看着挺好，其实是在和一团糊比）。所以真值缓存名里带了
`ng` 标记，两种真值不会互相覆盖；`--no-ground-removal` 可复现那个错误结果。

汇报时仍然不要说成"准确率" —— 真值本身也是聚类算法的产物。

为什么真值不能用 perception 那个 DBSCAN 类来算
-----------------------------------------------
`DBSCAN::CheckNearPoints()` 是 O(n²) 的（还带 0.5m 门限），218 万点要
4×10^12 次比较，跑不动。这里改用 scipy 的 cKDTree 重写了一个等价语义的
DBSCAN，只在离线建真值时用。

用法
----
    source /opt/ros/humble/setup.bash
    source ~/carProj/install/setup.bash
    ./cluster_eval.py --xyz /tmp/person.xyz --frames 20

真值第一次构建要一两分钟，结果缓存到 /tmp/gt_<数据名>.npz，之后秒开。
需要 perception 正在订阅回放数据并往 /perception/lidar_cluster_labels 发标签。
"""

import argparse
import os
import sys
import time

import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components
from sklearn.metrics import (adjusted_mutual_info_score, adjusted_rand_score,
                             homogeneity_completeness_v_measure,
                             silhouette_score)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2


class _EnoughFrames(Exception):
    """采够帧数后从回调里抛出来，让 rclpy.spin() 干净地返回。

    为什么不用 rclpy.shutdown()：它要在回调里 join 执行器，而执行器正等着这个
    回调返回 —— 死锁。进程打完表就再也不退出，任何用 timeout/顺序等待它的脚本
    都会卡死。"""


LABEL_TOPIC = '/perception/lidar_cluster_labels'

# 去地面参数：逐字抄自 perception 的配置，两边必须一致，否则标签空间对不上。
#   preprocess.json → ROI 裁剪 / 车体框 / 栅格尺寸
#   objs.json       → threshold_h / points_num（grid_cluster.cpp:70 传的就是这两个）
# 为什么不一致会出事：真值要在**同一个语义**上和 perception 比。perception 聚类
# 前先去地面，要是真值不去，地面就会变成几个横跨十几米的巨簇，把 perception 的
# 目标点全吸进去 —— 实测 89.6% 的点会落到这种簇上，指标彻底失去意义。
GROUND_CFG = {
    'xmin': -80.0, 'xmax': 80.0, 'ymin': -25.0, 'ymax': 25.0,
    'zmin': -2.0, 'zmax': 2.0,
    'car_xmin': -1.5, 'car_xmax': 3.9, 'car_ymin': -2.4, 'car_ymax': 0.5,
    'cell_size_x': 0.5, 'cell_size_y': 0.5,
    'threshold_h': 0.2,   # objs.json
    'points_num': 5,      # objs.json：格内点数不足这个数，整格按地面处理
}
# 逐点标签点云的布局，见 lidar_cluster.cpp 的 PublishLabels()
LABEL_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('label', '<i4')])
assert LABEL_DTYPE.itemsize == 16, LABEL_DTYPE.itemsize


def read_xyz(path):
    """读驱动的 xyz dump，列序 x,y,z,hangle,range,intent。

    只取 xyz：真值只需要几何。末行可能是半行（进程被 kill 时攒够 1MB 才落盘），
    先截掉再解析。
    """
    with open(path, 'rb') as f:
        blob = f.read()
    cut = blob.rfind(b'\n')
    if cut != len(blob) - 1:
        print(f'  [警告] 末行是半行，已丢弃')
        blob = blob[:cut + 1]
    import io
    data = np.loadtxt(io.BytesIO(blob), delimiter=',', usecols=(0, 1, 2))
    return np.ascontiguousarray(data, dtype=np.float64)


def voxel_downsample(pts, voxel):
    """按体素取质心。累积云里同一位置有几十个重复点，降采样既提速又降噪。"""
    keys = np.floor(pts / voxel).astype(np.int64)
    _, inv, counts = np.unique(keys, axis=0, return_inverse=True,
                               return_counts=True)
    sums = np.zeros((len(counts), 3), dtype=np.float64)
    np.add.at(sums, inv, pts)
    return sums / counts[:, None], counts


def remove_ground(pts, cfg=GROUND_CFG):
    """复刻 perception 的栅格地面分割，返回 (非地面点, 地面点)。

    对应 lidar_preprocess.cpp 的 GetGridZmin + GetGroundPoints：
      1. 裁掉 ROI 之外的点，以及车体框内的点（车自己不算障碍物）；
      2. 按 cell_size 分格，每格记下 z 最小值 —— 那就是该处的局部地面；
      3. 点比所在格的地面高出超过 threshold_h 才算非地面；
      4. 格内点数不超过 points_num 的**整格算地面**（C++ 里的 vaild 标志），
         这是为了不让一两个噪点把地面抬高。
    """
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]

    inside = ((x >= cfg['xmin']) & (x <= cfg['xmax']) &
              (y >= cfg['ymin']) & (y <= cfg['ymax']) &
              (z >= cfg['zmin']) & (z <= cfg['zmax']))
    in_car = ((x > cfg['car_xmin']) & (x < cfg['car_xmax']) &
              (y > cfg['car_ymin']) & (y < cfg['car_ymax']))
    keep = inside & ~in_car

    cx = np.floor((x - cfg['xmin']) / cfg['cell_size_x']).astype(np.int64)
    cy = np.floor((y - cfg['ymin']) / cfg['cell_size_y']).astype(np.int64)
    key = np.where(keep, cx * 100000 + cy, -1)

    # 逐格求 z 最小值与点数。np.minimum.at 是唯一能按 key 做 "min 散射" 的
    # 向量化手段（np.minimum.at 比 pandas groupby 在这种规模上快得多）。
    uk, inv = np.unique(key[keep], return_inverse=True)
    zmin = np.full(len(uk), np.inf)
    np.minimum.at(zmin, inv, z[keep])
    cnt = np.bincount(inv, minlength=len(uk))

    valid_cell = cnt > cfg['points_num']
    # 有效格且高出地面 → 非地面；其余（无效格、贴地点、ROI 外的点）全算地面
    is_no_ground = np.zeros(len(pts), dtype=bool)
    is_no_ground[keep] = (valid_cell[inv] &
                          (z[keep] - zmin[inv] > cfg['threshold_h']))
    return pts[is_no_ground], pts[keep & ~is_no_ground]


def radius_filter(pts, radius, min_neighbors):
    """pcl::RadiusOutlierRemoval(radius, min_neighbors)：邻域内点数不够就删。

    注意 PCL 的 radiusSearch **把查询点自己算进去**，所以 min_neighbors=5
    的实际含义是"除自己外还要有 4 个邻居"。这个函数在整条链里是把墙、地面
    这类连续稀疏面打散的关键一步 —— 没有它，稠密累积云上的 DBSCAN 一定会
    把整面墙连成一个簇。
    """
    tree = cKDTree(pts)
    nb = tree.query_ball_point(pts, radius, return_sorted=False)
    cnt = np.fromiter((len(x) for x in nb), dtype=np.int64, count=len(pts))
    return pts[cnt >= min_neighbors]


def dbscan_kdtree(pts, eps, min_pts):
    """KD-tree 版 DBSCAN，语义与 perception 的 DBSCAN 一致。

    核心点：eps 邻域内点数 >= min_pts；从核心点出发做连通分量，分量里只要
    含核心点就是一个簇，其余点（孤立点、够不到任何核心点的边界点）算噪点。
    返回逐点标签，-1 = 噪点。
    """
    n = len(pts)
    tree = cKDTree(pts)
    nb = tree.query_ball_point(pts, eps, return_sorted=False)
    # 减 1 是必须的：scipy 的 query_ball_point **把查询点自己算在结果里**，
    # 而 C++ 那边 adj_points_count 统计的是 j != i，不含自己。
    # 不减就等于 C++ 的 DBSCAN(eps, min_pts-1)，核心点会多出一批 —— 和
    # radius_filter 上面记的那个坑是同一个。实测差一个会让真值的核心点
    # 比例虚高，指标跟着偏。
    core = np.fromiter((len(x) - 1 >= min_pts for x in nb), dtype=bool, count=n)
    print(f'    核心点 {core.sum()} / {n} ({core.sum()/n*100:.1f}%)')

    # 只从核心点连边：非核心点不主动连，靠被核心点连上成为边界点
    rows, cols = [], []
    for i in np.flatnonzero(core):
        for j in nb[i]:
            if j != i:
                rows.append(i)
                cols.append(j)
    if not rows:
        return np.full(n, -1, dtype=np.int64)
    adj = csr_matrix((np.ones(len(rows), dtype=np.int8), (rows, cols)),
                     shape=(n, n))
    ncomp, comp = connected_components(adj, directed=False)

    # 含核心点的分量才是簇，其余全是噪点
    labels = np.full(n, -1, dtype=np.int64)
    nxt = 0
    for c in range(ncomp):
        m = comp == c
        if core[m].any():
            labels[m] = nxt
            nxt += 1
    return labels


def build_gt(xyz_path, cache_path, voxel, eps, min_pts,
             with_ground_removal=True, radius=0.5, search_num=5):
    """建真值并缓存。返回 (真值点云 Nx3, 逐点真值标签)。"""
    if os.path.exists(cache_path):
        z = np.load(cache_path)
        print(f'真值缓存命中 {cache_path}：{len(z["pts"])} 个体素，'
              f'{len(np.unique(z["labels"][z["labels"] >= 0]))} 个簇')
        return z['pts'], z['labels']

    print(f'构建真值（首次较慢）...')
    t0 = time.time()
    pts_all = read_xyz(xyz_path)
    print(f'  读入 {len(pts_all)} 点  {time.time()-t0:.1f}s')

    if with_ground_removal:
        nog, gnd = remove_ground(pts_all)
        print(f'  去地面：非地面 {len(nog)} 点，地面/裁掉 {len(gnd)} 点'
              f'（{len(gnd)/len(pts_all)*100:.1f}%）  {time.time()-t0:.1f}s')
        pts_all = nog

    pts, counts = voxel_downsample(pts_all, voxel)
    print(f'  体素 {voxel}m 降采样 → {len(pts)} 点'
          f'（平均每体素 {counts.mean():.1f} 个原始点）  {time.time()-t0:.1f}s')

    if radius > 0:
        pts = radius_filter(pts, radius, search_num)
        print(f'  RadiusFilter({radius}m, {search_num}) → {len(pts)} 点'
              f'  {time.time()-t0:.1f}s')

    labels = dbscan_kdtree(pts, eps, min_pts)
    n_cluster = len(np.unique(labels[labels >= 0]))
    n_noise = int((labels < 0).sum())
    print(f'  DBSCAN(eps={eps}, min_pts={min_pts}) → {n_cluster} 簇，'
          f'噪点 {n_noise} ({n_noise/len(pts)*100:.1f}%)  {time.time()-t0:.1f}s')

    sizes = np.bincount(labels[labels >= 0]) if n_cluster else np.array([])
    if len(sizes):
        order = np.argsort(sizes)[::-1]
        print(f'  最大的几个簇（体素数）: '
              f'{sizes[order[:12]].tolist()}')

    np.savez_compressed(cache_path, pts=pts, labels=labels)
    print(f'  真值已缓存到 {cache_path}  总耗时 {time.time()-t0:.1f}s')
    return pts, labels


def evaluate(xyz, pred, gt):
    """算六个指标。xyz 是这些点的坐标，pred/gt 是两套标签，-1 表示噪点。

    轮廓系数需要真实坐标 —— 它量的是簇内距离 vs 最近邻簇距离，没有坐标
    就没有意义。其余五个只看标签的列联表，与坐标无关。
    """
    out = {}
    h, c, v = homogeneity_completeness_v_measure(gt, pred)
    out['homogeneity'] = h
    out['completeness'] = c
    out['v_measure'] = v
    out['ARI'] = adjusted_rand_score(gt, pred)
    out['AMI'] = adjusted_mutual_info_score(gt, pred)

    # 轮廓系数：
    #  - 噪点(-1)不构成"簇"，混进去会把"噪点"当成一个巨大而松散的簇，污染结果；
    #  - 只含 1 个点的簇轮廓系数没有定义，sklearn 会直接报错。
    # 两种都剔除，并如实报出剔了多少。
    keep = pred >= 0
    p, x = pred[keep], xyz[keep]
    if len(p):
        _, cnt = np.unique(p, return_counts=True)
        big = np.where(cnt >= 2)[0]
        ok = np.isin(p, big)
        out['sil_excluded'] = int((~ok).sum()) + int((~keep).sum())
        if ok.sum() >= 2 and len(np.unique(p[ok])) >= 2:
            out['silhouette'] = float(silhouette_score(x[ok], p[ok]))
    return out


class ClusterEval(Node):
    def __init__(self, gt_tree, gt_labels, max_frames, gt_max_dist,
                 gt_pts=None, topic=LABEL_TOPIC):
        super().__init__('cluster_eval')
        self.gt_tree = gt_tree
        self.gt_labels = gt_labels
        self.max_frames = max_frames
        self.gt_max_dist = gt_max_dist
        self.rows = []
        self.bad_match = 0
        self.mismatch_dist = []

        # 归因表：真值的每个簇有多大、跨多大范围，以及它一共"接住"了多少
        # 个 perception 点。这用来回答一个必须回答的问题 —— 指标里那些
        # 分歧，到底是 perception 聚类不好，还是真值本身把地面/墙面也当成
        # 了一个"目标"？真值是稠密云上跑的 DBSCAN，没有去地面这一步，
        # 而 perception 在聚类前是**先去地面**的。两者标签空间不一致的话，
        # 指标会系统性地亏待 perception。
        self.gt_size = {}
        self.gt_span = {}
        if gt_pts is not None:
            uniq, cnt = np.unique(gt_labels[gt_labels >= 0], return_counts=True)
            for c, n in zip(uniq, cnt):
                m = gt_labels == c
                self.gt_size[int(c)] = int(n)
                p = gt_pts[m]
                self.gt_span[int(c)] = (p[:, 0].ptp(), p[:, 1].ptp(),
                                        p[:, 2].ptp(),
                                        float(np.hypot(p[:, 0], p[:, 1]).min()))
        self.gt_hit = {}

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2, topic, self._cb, qos)
        self.get_logger().info(
            f'监听 {topic}，采满 {max_frames} 帧后出结果')

    def _cb(self, msg):
        if self.max_frames and len(self.rows) >= self.max_frames:
            return
        arr = np.frombuffer(msg.data, dtype=LABEL_DTYPE)
        if len(arr) == 0:
            return
        xyz = np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float64)
        pred = arr['label'].astype(np.int64)

        # 把 DBSCAN 实际吃到的点映射回真值：这些点是 0.3m 体素的质心，
        # 和原始点不重合，所以取最近邻。距离分布要盯着 —— 太远说明对错了对象。
        dist, idx = self.gt_tree.query(xyz, k=1)
        good = dist <= self.gt_max_dist
        self.bad_match += int((~good).sum())
        if good.any():
            self.mismatch_dist.append(float(np.median(dist[good])))

        gt = np.full(len(pred), -1, dtype=np.int64)
        gt[good] = self.gt_labels[idx[good]]

        # 点名：每个真值簇接住了多少个 perception 点
        for c, n in zip(*np.unique(gt[gt >= 0], return_counts=True)):
            self.gt_hit[int(c)] = self.gt_hit.get(int(c), 0) + int(n)

        # 真值里是噪点的位置不参与比较（真值自己都不确定那是啥）
        valid = gt >= 0
        if valid.sum() < 10:
            self.get_logger().warn(f'本帧只有 {valid.sum()} 个点能对上真值，跳过')
            return

        row = evaluate(xyz[valid], pred[valid], gt[valid])
        row['n_points'] = int(valid.sum())
        row['n_pred_clusters'] = int(len(np.unique(pred[valid][pred[valid] >= 0])))
        row['n_gt_clusters'] = int(len(np.unique(gt[valid])))
        row['pred_noise_ratio'] = float((pred[valid] < 0).mean())
        self.rows.append(row)

        k = len(self.rows)
        self.get_logger().info(
            f'[{k}/{self.max_frames}] 点 {row["n_points"]}  '
            f'预测簇 {row["n_pred_clusters"]} / 真值簇 {row["n_gt_clusters"]}  '
            f'h={row["homogeneity"]:.3f} c={row["completeness"]:.3f} '
            f'V={row["v_measure"]:.3f} ARI={row["ARI"]:.3f} '
            f'AMI={row["AMI"]:.3f} sil={row.get("silhouette", float("nan")):.3f}')

        if self.max_frames and len(self.rows) >= self.max_frames:
            self.report()
            # 采够就退。留着的话 _cb 会在开头 return，进程白占 CPU 又不干活。
            #
            # **不要改成 rclpy.shutdown()**：在回调里调它会死锁 —— 它要 join 执行器，
            # 而执行器正在等这个回调返回。实测结果是表已经打完了、进程却永不退出
            # （/tmp/ab_metrics.sh 就是被这个卡住的，第二轮永远等不到）。抛异常让
            # spin() 返回才是安全的：Executor._execute_task 会把回调里的异常重新抛出来。
            raise _EnoughFrames()

    def report(self):
        if not self.rows:
            print('\n没采到任何帧。')
            return
        keys = ['homogeneity', 'completeness', 'v_measure', 'ARI', 'AMI',
                'silhouette', 'n_points', 'n_pred_clusters', 'n_gt_clusters',
                'pred_noise_ratio']
        print(f'\n{"="*72}')
        print(f'{"指标":<16}{"均值":>10}{"中位":>10}{"最小":>10}{"最大":>10}{"标准差":>10}')
        print('-' * 72)
        for k in keys:
            vals = np.array([r[k] for r in self.rows if k in r and
                             not np.isnan(r.get(k, np.nan))], dtype=float)
            if len(vals) == 0:
                continue
            print(f'{k:<16}{vals.mean():>10.4f}{np.median(vals):>10.4f}'
                  f'{vals.min():>10.4f}{vals.max():>10.4f}{vals.std():>10.4f}')
        print('=' * 72)
        print(f'共 {len(self.rows)} 帧')
        if self.mismatch_dist:
            print(f'回投真值的最近邻距离中位: '
                  f'{np.mean(self.mismatch_dist):.4f} m（应远小于 0.3m 体素）')
        if self.bad_match:
            print(f'超出匹配半径被丢弃的点: {self.bad_match}')
        self.report_attribution()
        print('\n注意：真值是多圈累积聚类自动生成的，指标衡量的是与稠密视角的'
              '\n一致性，不是绝对准确率 —— 真值本身也是算法产物。')

    def report_attribution(self):
        """perception 的点都落到真值的哪些簇上。

        真值是在**没去地面**的稠密云上跑的 DBSCAN，所以最大的几个"簇"往往
        是地面和墙面。perception 聚类前先去地面，本来就不该输出它们。如果
        这些环境簇吃掉了大量 perception 点，那指标里的分歧就有相当一部分是
        "标签空间不一致"造成的，不能算 perception 的账。
        """
        if not self.gt_hit:
            return
        total = sum(self.gt_hit.values())
        print(f'\n{"="*72}')
        print(f'归因：perception 的点落到了真值的哪些簇上（共 {total} 点次）')
        print('-' * 72)
        print(f'{"GT簇":>5}{"接住点次":>9}{"占比":>8}{"GT体素":>9}'
              f'{"xyz跨度":>20}{"最近距":>8}')
        for c, n in sorted(self.gt_hit.items(), key=lambda kv: -kv[1])[:12]:
            sp = self.gt_span.get(c)
            s = (f'{sp[0]:>6.2f}x{sp[1]:>6.2f}x{sp[2]:>5.2f}' if sp else ' ' * 20)
            near = f'{sp[3]:>8.2f}' if sp else ' ' * 8
            print(f'{c:>5}{n:>9}{n/total*100:>7.1f}%'
                  f'{self.gt_size.get(c, 0):>9}{s:>20}{near:>8}')
        big = sum(n for c, n in self.gt_hit.items()
                  if self.gt_span.get(c, (0, 0, 0, 0))[0] > 3.0
                  or self.gt_span.get(c, (0, 0, 0, 0))[1] > 3.0)
        print(f'\n跨度 >3m 的环境簇吃掉的点次: {big} / {total} = {big/total*100:.1f}%')


def main():
    ap = argparse.ArgumentParser(description='perception 聚类质量评估')
    ap.add_argument('--xyz', required=True, help='原始 xyz（用于建真值）')
    ap.add_argument('--frames', type=int, default=20, help='采多少帧后出结果')
    # 下面这几个默认值刻意取得和 perception 一致（objs.json：voxel_size 0.3、
    # radius_search 0.5、search_num 5；lidar_cluster.cpp：DBSCAN(0.6, 3)）。
    # 两边同链路、同参数，唯一差别就只剩输入是 57 圈还是单圈 —— 指标回答的
    # 就是"单圈聚类相对稠密视角退化多少"这一个问题。
    ap.add_argument('--gt-voxel', type=float, default=0.30, help='真值体素边长')
    ap.add_argument('--gt-radius', type=float, default=0.5,
                    help='真值 RadiusFilter 半径，<=0 关闭')
    ap.add_argument('--gt-search-num', type=int, default=5,
                    help='真值 RadiusFilter 最少邻居数')
    ap.add_argument('--gt-eps', type=float, default=0.60, help='真值 DBSCAN eps')
    ap.add_argument('--gt-min-pts', type=int, default=3, help='真值 DBSCAN minPts')
    ap.add_argument('--gt-max-dist', type=float, default=0.30,
                    help='点回投真值的最大距离，超过算对不上')
    ap.add_argument('--rebuild-gt', action='store_true', help='强制重建真值')
    ap.add_argument('--no-ground-removal', action='store_true',
                    help='建真值时不先去地面（对照组，指标会失真）')
    ap.add_argument('--topic', default=LABEL_TOPIC)
    args, _ = ap.parse_known_args()

    stem = os.path.splitext(os.path.basename(args.xyz))[0]
    gtag = 'ng' if not args.no_ground_removal else 'raw'
    rtag = f'r{args.gt_radius}_{args.gt_search_num}' if args.gt_radius > 0 else 'nor'
    # g2 = 真值链的第 2 版语义。改 dbscan_kdtree 的核心点判据（去掉查询点自己）
    # 时旧缓存不会失效、会被静默复用，所以版本号必须进键。
    cache = (f'/tmp/gt_{stem}_{gtag}_{rtag}_v{args.gt_voxel}_e{args.gt_eps}'
             f'_m{args.gt_min_pts}_g2.npz')
    if args.rebuild_gt and os.path.exists(cache):
        os.remove(cache)

    gt_pts, gt_labels = build_gt(args.xyz, cache, args.gt_voxel,
                                 args.gt_eps, args.gt_min_pts,
                                 with_ground_removal=not args.no_ground_removal,
                                 radius=args.gt_radius,
                                 search_num=args.gt_search_num)
    tree = cKDTree(gt_pts)

    rclpy.init()
    node = ClusterEval(tree, gt_labels, args.frames, args.gt_max_dist,
                       gt_pts=gt_pts, topic=args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.report()
    except _EnoughFrames:
        pass          # 表已经在回调里打过了
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
