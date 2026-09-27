#ifndef LIDAR_OBJS_TYPE_H
#define LIDAR_OBJS_TYPE_H
#include "data_pool.h"
#include "common/tool/read_config.h"
// #include "common/read_config.h"
namespace perception
{
    namespace lidar_objs
    {

        struct objs_config
        {
            int ground_is;
            float zmax;
            float zmin;
            float xmin;
            float xmax;
            float ymin;
            float ymax;
            float car_xmin;
            float car_xmax;
            float car_ymin;
            float car_ymax;
            float cell_size_x;
            float cell_size_y;
            float voxel_size;
            float radius_search;
            int search_num;
            float threshold_h;
            int points_num;

            double in_max_cluster_distance; // setRadiusSearch

            /* ── 聚类门槛（原来全写死在 lidar_cluster.cpp 里，现在接出来）──
               这里用**类内初始化**给默认值，不要挪进下面的 reset()：reset()
               全仓没有任何调用点，靠它兜底等于没兜底。默认值刻意与原来那三个
               字面量一致（0.6/3、4 点、0.2 m），所以配置文件缺键时行为不变。 */
            double dbscan_eps     = 0.6;   // DBSCAN 邻域半径 (m)
            int    dbscan_min_pts = 3;     // 核心点所需的最少邻居数
            int    min_cluster_points = 4; // 成为目标所需的最少点数（远场下限）
            double min_obj_height = 0.2;   // 目标最低高度，低于此算噪点 (m)
            /* 近场小碎片抑制：最小点数随距离线性缩放，
               r=0 时用 min_pts_near，到 min_pts_rmax 处降回 min_cluster_points。
               近处一个实物动辄几百点，4 个点的小簇必是碎片；远处一个行人本来
               就只有七八个点，一刀切会把真人切掉。 */
            int    min_pts_near   = 12;    // 近场最小点数
            double min_pts_rmax   = 20.0;  // 缩放过渡半径 (m)
            /* 墙 / 建筑：用 max(长,宽) 判定，因为 length/width 会因为最小面积
               搜索的平局处理逐帧互换，max 对互换不变。 */
            double wall_size_m    = 6.0;   // 超过此尺寸且高度够，判为墙

            /* 巨簇事后切分。为什么需要它：本场景是一圈连续的墙，墙在三维里
               本来就是**真连通**的曲面，任何"能把一辆车连成一体"的半径都大到
               足以让整圈墙连成一块 —— 实测 eps 在 0.30 与 0.33 之间是一跳
               （0.30 时最大簇 125 点，0.33 时 714 点），中间没有可用的档位。
               所以反过来做：**用大半径保证真物体完整，只对跨度超限的簇用小半径
               再切一遍**。真物体小，永远进不了这个分支；墙再被切成片。
               代价是多一趟聚类，且只切一层（eps=split_eps 下最大簇约 4 m，
               不会还超限，所以不做递归）。 */
            double split_eps      = 0.3;   // 切分巨簇时用的第二半径 (m)
            double split_size_m   = 6.0;   // 跨度超过此值才触发切分 (m)；<=0 关闭

            /* ── 摘墙（wall_extract.h）──────────────────────────────────
               巨簇切分（上面那两条）**只切不摘**：墙的点仍然和物体待在同一个
               簇里，离墙 0.6 m 以内的真物体会被一起吸进去。这一组旋钮的作用是
               在聚类**之前**把墙点分离出去，墙单独发 type=5，剩下的点再走主聚类。
               判据、三次失败尝试的实测数据、以及 ratio 安全带的推导全在
               wall_extract.h 的注释里，改这里之前先读它。 */
            bool  wall_extract        = true;  // 总开关，真机 A/B 用
            /* 预分组 eps = cluster_voxel_size × 此值。**必须跟着体素走**：
               实测 eps 恰好等于体素格距时（体素 0.30 / eps 0.30）面邻接在浮点
               边界上断掉，只摘掉 5.6% 的点；到 0.30×√2=0.424 起开始吞物体。
               实测安全带 [1.06, 1.15]，默认 1.10。改 cluster_voxel_size 必须重测。 */
            double wall_seg_eps_ratio = 1.10;
            int    wall_seg_min_pts   = 3;     // 预分组的连通性门槛，与主聚类分开
            int    wall_min_points    = 30;    // 片点数下限。刻意偏严：宁可漏判不可误判
            double wall_min_zspan     = 1.5;   // 片 z 跨度下限，把地面残点/桌面挡在外面
            double wall_max_thin      = 0.15;  // 薄度 side/span 上限；单面墙约 0.01
            double wall_min_span      = 3.0;   // XY 跨度分支；环状墙薄度大，靠这支进来

            /* ── 地面分割 / 体素分辨率 ──────────────────────────────────
               地面分割与聚类用**两个不同的体素**。原因：PMF 的形态学开运算拿窗口
               内点集的 min z 当地面曲面（pcl/filters/impl/morphological_filter.hpp），
               需要地面被真正采样到；实测本场景 voxel 0.3 时全场景只剩 645 点、
               近地点仅 46 个，1.5 m 窗口里只有一两个点，"曲面"就是这几个点本身，
               PMF 会退化。voxel 0.10 时 4532 点、近地点 168 个，才有料可算。
               地面点分完就丢，所以细体素只多花一次 voxel + 一次 PMF，不影响
               DBSCAN 速度（DBSCAN 吃的是后面那次粗体素）。 */
            int   ground_method      = 2;      // 1=旧栅格最低点法 2=PMF
            float ground_voxel_size  = 0.10f;  // 地面分割前的体素边长 (m)，要比聚类细
            float cluster_voxel_size = 0.30f;  // 聚类前的体素边长 (m)，原来是 grid_cluster.cpp 的字面量

            /* PMF 参数，逐个对应 PCL 1.12 的 setter。max_window 单位是**米**
               （PCL 内部把窗口按 cell_size 乘出来再和它比），虽然 setter 收 int。
               注意 PCL 的窗口是**软上限**：循环条件是 window < max_window，
               所以 cell 0.5 / max_window 6 的实际计划是 [1.5, 2.5, 4.5, 8.5]——
               4 轮，最后一轮 8.5 > 6。别假设最大窗口等于配置值。 */
            int   pmf_max_window     = 6;
            float pmf_slope          = 1.0f;
            float pmf_initial_dist   = 0.15f;
            float pmf_max_dist       = 1.0f;
            float pmf_cell_size      = 0.50f;
            float pmf_base           = 2.0f;
            bool  pmf_exponential    = true;
            /* PMF 失败保护：extract() 在 initCompute() 失败时返回**空**地面点集，
               调用方若天真地把整个输入当非地面，连地板一起喂给 DBSCAN。地面占比
               低于此值就当 PMF 失败，该帧直接跳过（不回退到旧法——静默回退会让
               A/B 数据无法归因）。 */
            float pmf_min_ground_ratio = 0.02f;

            /* 细体素阶段的孤立点滤波（在 PMF **之前**）。这是 ROR 唯一有真实机制
               的位置：地板下方的杂点会把形态学开运算局部拉低，导致周围真实地面
               被误判为非地面。注意 PCL 的 setMinNeighborsInRadius(N) **把查询点
               自己算进邻居数**，所以 N=2 的语义是"半径内至少 2 个别的点"。 */
            int   ror_stage1_min_neighbors = 2;

            float obj_min_z;
            float tolerance; // 50cm tolerance in (x, y, z) coordinate system
            int k_search_num;
            float cuv_angle;
            double eps_angle; // 5degree tolerance in normals
            unsigned int min_cluster_size;
            unsigned int method; // 1。为区域生长算法，2为欧式聚类
            void reset()
            {
                zmax = 2.0;
                zmin = -2.0;
                xmin = -20;
                xmax = 80;
                ymin = -15;
                ymax = 15;
                car_xmin = -5.0;
                car_xmax = 0.1;
                car_ymin = -1.0;
                car_ymax = 1.0;
                cell_size_x = 0.5;
                cell_size_y = 0.5;
                voxel_size = 0.1f;
                in_max_cluster_distance = 0.2;
                tolerance = 0.5f;
                eps_angle = 5 * (M_PI / 180.0); // 区域生长算法的法线夹角， k聚类的
                min_cluster_size = 2;
                method = 1;
                cuv_angle = 3;
                k_search_num = 30;
                obj_min_z = 0.15;
                ground_is = 1;
                threshold_h = 0.1;
                radius_search = 0.8; // 搜索半径
                search_num = 10;
                points_num = 0;
            }
        };
        struct proprecess_config
        {
            // float grid_zmin;
            int points_num;
            float zmax;
            float zmin;
            float xmin;
            float xmax;
            float ymin;
            float ymax;
            float car_xmin;
            float car_xmax;
            float car_ymin;
            float car_ymax;
            float voxel_size;
            float threshold_h;
            int vaild;
            float cell_size_x;
            float cell_size_y;
            float radius_search;
            int search_num;
            void reset()
            {
                // grid_zmin = 2;
                points_num = 0;
                zmax = 1.5;
                zmin = -2.0;
                xmin = -20;
                xmax = 30;
                ymin = -25;
                ymax = 25;
                car_xmin = -5.0;
                car_xmax = 0.1;
                car_ymin = -1.0;
                car_ymax = 1.0;
                cell_size_x = 0.5;
                cell_size_y = 0.5;
                voxel_size = 0.1f;
                vaild = 0;
                threshold_h = 0.1;
                radius_search = 0.8; // 搜索半径
                search_num = 10;
            }
        };
        struct obj_struct
        {
            float xmax;
            float xmin;
            float ymax;
            float ymin;
            float zmax;
            float zmin;
            void reset()
            {
                xmax = -100;
                xmin = 100;
                ymax = -20;
                ymin = 20;
                zmax = -10;
                zmin = 10;
            }
        };
        struct grid_obj
        {
            float x;
            float y;
            int point_num;
            int vaild;
            float zmin;
            float zmax;
            int idx;
            void reset()
            {
                x = 100;
                y = 100;
                point_num = 0;
                vaild = 0;
                zmin = 3;
                zmax = -3;
                idx = -1;
            }
        };
    } //lidar_objs
} //perception

#endif