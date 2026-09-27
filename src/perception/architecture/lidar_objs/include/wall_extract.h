#ifndef _WALL_EXTRACT_H__
#define _WALL_EXTRACT_H__

/* 摘墙：把墙面从聚类输入里分出去，让墙不再充当「物体之间的桥」。

   为什么需要它
   ------------
   本场景是一整圈连续的墙。DBSCAN 用大半径（eps 0.6~0.7）才能保住远处物体的完整，
   而那个半径足以让整圈墙连成一块、并把离墙 0.6 m 以内的真物体一起吸进去
   （objs.json 的 _note_split_off 里已经写明这个代价）。反过来把半径收小又会把
   物体切碎 —— 实测一个 344 点的物体在 0.25 m 下裂成 17 瓣。**墙和物体在几何上
   都是连续曲面，距离判据分不开它们。**

   lidar_cluster.cpp 里已经有半边机制（max(长,宽) > wall_size_m 判 type=5），
   但它**只贴标签、不摘点**，墙的点仍然和物体待在同一个簇里。本模块补上摘点这一步。

   判据（纯几何，不用法向）
   ------------------------
   1. 先在聚类输入上跑一次 DBSCAN(seg_eps, seg_min_pts) 做**预分组** —— 只是连通片
   2. 每片算：点数、z 跨度、XY 最小面积外接矩形的长边 span 与短边 side
   3. 判墙：点数 >= min_points 且 z跨度 > min_zspan
            且 (薄度 side/span < max_thin 或 span > min_span)

   **为什么不用法向**（三种试过的判据都失败了，别再走回去）
   - 全局平面拟合 + 厚度：对**一整圈**墙做平面拟合没有意义。实测最要紧的两片
     （637 点的墙环、270 点）都被判成「不是墙」，只摘掉 28% 的点。
   - 局部法向 + 区域生长：kNN 法向在**极薄的墙**上病态 —— 实测一片 11.64 m 长、
     厚 0.02 m 的平面，中位 |n_z| 却算出 0.309（12 个最近邻沿墙铺开、协方差病态），
     刚好越过阈值。25° 区域生长切出 226 片，按那套阈值只认出 9.5% 的点。
   - |n_z| 当竖直度：合并簇上会退化 —— 四面墙连成的环，PCA 最小方向落在 z 轴，
     报 1.000，一个竖直墙环被报成「水平面」。见 cluster_shape.py 的 verticality()。

   实测效果（4 帧，seg_eps = 0.30*1.10 = 0.33，见 plans/deep-cuddling-grove.md）
   --------------------------------------------------------------------------
   最大簇占**全帧**比例   50.4% / 39.2% / 53.1% / 39.4%  →  7.0% / 9.4% / 12.1% / 9.1%
   最大簇点数             1037 / 556 / 570 / 564        →  143 / 133 / 130 / 131
   非墙簇数               14 / 16 / 12 / 17             →  17 / 18 / 13 / 21
   跨 >3 m 的簇           3 / 4 / 2 / 4                 →  1 / 2 / 1 / 2
   摘掉的点               66% / 63% / 67% / 62%

   注意分母：摘墙后的最大簇「占剩余点」是 20.4% / 25.3% / 37.0% / 23.8%，
   那个数会随着摘掉越多而虚高，**不要**拿它跟摘墙前的比例直接比。
   与摘墙前可比的是占**全帧**那一行。

   seg_eps 必须跟着 cluster_voxel_size 走
   --------------------------------------
   实测 eps 恰好等于体素格距时（体素 0.30 / eps 0.30）面邻接在浮点边界上断掉，
   只摘掉 5.6% 的点。所以绝对值和体素绑死，调用方按
   cluster_voxel_size * wall_seg_eps_ratio 算好 seg_eps 传进来。

   **ratio 的安全带实测是 [1.06, 1.15]，默认取 1.10**（四帧扫描
   ratio 1.04~1.30 步进 0.02，见 plans/deep-cuddling-grove.md）：

   - **下限 1.06**：1.04 时实时帧最大非墙簇还有 **260** 点、1.08 时 185 点，
     说明墙还在粘着物体没摘干净；到 1.10 才掉到地板值 143。
   - **上限 ~1.16**：实时帧上那个质心 (6.17, 2.99)、125 点、XY 2.65×1.39、
     z 跨 3.37 m 的真物体，在 1.18 还在、**1.20 被吞**；非墙簇数也在 1.16 之后
     开始掉（实时 17→16→14，离线 27 圈 18→16→15）。
   - 1.10 在四帧上的非墙簇数是 **17 / 18 / 13 / 21**，是每一帧的最高或并列最高。

   注意：1.10 与 1.14 的差别只有 4~13 点的薄碎片（薄度 0.01~0.23，是墙屑不是物体），
   真正有区别的是 1.10 比 1.15 多保住 2 个簇（离线 33 圈 21 对 19）。
   **改 cluster_voxel_size 必须重测这个 ratio。** */

#include <vector>

#include "dbscan.h"

namespace perception
{

struct WallConfig
{
    bool   enable     = true;
    double seg_eps    = 0.33;   // = cluster_voxel_size * wall_seg_eps_ratio，调用方算好
    int    seg_min_pts = 3;     // 预分组的连通性门槛
    int    min_points = 30;     // 片点数下限。刻意偏严：宁可漏判（少摘）不可误判（多摘）
    double min_zspan  = 1.5;    // 片 z 跨度下限，把水平面（地面残点、桌面）挡在外面
    double max_thin   = 0.15;   // 薄度上限。单面墙约 0.01，环状合并簇约 0.5~0.9
    double min_span   = 3.0;    // XY 跨度上限分支：环状墙的薄度大，靠跨度这一支进来
};

/* XY 平面上的最小面积外接矩形（凸包 + 旋转卡壳）。
   输出最长边 span 与短边 side，与 min_rotate_rect.cpp 的 max(length,width) /
   min(length,width) 同义，所以薄度 = side/span 可以直接和那里的判定对照。

   **不要换成 min_rotate_rect 的 MinRotateRect**：那一版是 O(m²)
   （实测 1037 点 19.8 ms），而这里每帧要对每一片都算一次。

   本函数内层是 O(h²)（h = 凸包点数），不是教科书里的线性旋转卡壳 —— 刻意如此：
   cluster_shape.py:64-95 的 min_area_rect_longest() 就是同一个写法，保持等价才能
   拿两边互相验证（该 Python 版已用生产 MinRotateRect 对过大簇：10.7036 对
   10.70379638671875）。h 只有几十到一两百，O(h²) 是微秒级，不值得为常数优化
   破坏这个可验证性。 */
void MinAreaRectXY(const std::vector<points>& pts,
                   const std::vector<int>&    sel,
                   double&                    span,
                   double&                    side);

/* 摘墙。返回墙片，每个元素是该片各点在 data 里的**原下标**（升序）。
   is_wall[i] 为 1 表示第 i 个点属于某个墙片（用 char 而不是 vector<bool>）。

   内部自建 DBSCAN(seg_eps, seg_min_pts) 做预分组 —— DBSCAN 的构造函数只存两个
   double，自建比让调用方持有更干净，也让本模块能脱离 PointsCluster 单测。 */
std::vector<std::vector<int>> ExtractWalls(const std::vector<points>& data,
                                           const WallConfig&          cfg,
                                           std::vector<char>&         is_wall);

}  // namespace perception

#endif  // _WALL_EXTRACT_H__
