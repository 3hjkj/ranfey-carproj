#include "../include/dbscan.h"

#include <algorithm>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace perception
{
    DBSCAN::DBSCAN(double eps, int min_points_num) : eps_(eps), min_points_num_(min_points_num), cluster_idx_(DB_NOT_CLASSIFIED) {}

    DBSCAN::~DBSCAN() {}

    void DBSCAN::Init(const vector<points> &data_in)
    {
        points_.clear();
        adj_points_.clear();
        size_ = data_in.size();
        points_.resize(size_);
        adj_points_.resize(size_);
        for (int i = 0; i < size_; i++)
        {
            points_[i].id = i;
            points_[i].x = data_in[i].x;
            points_[i].y = data_in[i].y;
            points_[i].z = data_in[i].z;   // ← 原来漏了这一行，聚类因此退化成二维
        }
    }

    vector<vector<points>> DBSCAN::Clustering(const vector<points> &data_in)
    {
        vector<vector<points>> data_out;
        data_out.clear();
        cluster_.clear();
        cluster_idx_ = DB_NOT_CLASSIFIED;
        if (data_in.empty())
            return data_out;
        Init(data_in);
        // 统计每个点周围的点数，为后面筛选核心点、边缘点及噪点做准备
        rclcpp::Clock clock;                     // ROS 2 取时间
        double time1 = clock.now().seconds();
        CheckNearPoints();
        double time2 = clock.now().seconds();
        // 聚类，将一类的点标记为同一个cluster_idx
        for (int i = 0; i < size_; i++)
        {
            // 未分类的点标记为DB_NOT_CLASSIFIED
            if (points_[i].cluster_idx != DB_NOT_CLASSIFIED)
                continue;
            // 非核心点直接标记为噪点
            if (IsCoreObject(i))
            {
                DepthFirstSearch(i, ++cluster_idx_);
            }
            else
            {
                points_[i].cluster_idx = DB_NOISE;
            }
        }
        double time3 = clock.now().seconds();
        // 将每一类的id取出来
        cluster_.resize(cluster_idx_ + 1);
        for (int i = 0; i < size_; i++)
        {
            if (points_[i].cluster_idx != DB_NOISE)
            {
                cluster_[points_[i].cluster_idx].push_back(i);
            }
        }
        if (verbose_)
        {
            cout << " time check near:" << (time2 - time1) * 1000 << "ms" << endl;
            cout << " cluster:" << (time3 - time2) * 1000 << "ms" << endl;
        }
        data_out = GetResult(data_in, cluster_);
        return data_out;
    }
    vector<vector<points>> DBSCAN::GetResult(const vector<points> &data_in, vector<vector<int>> &indices)
    {
        vector<vector<points>> data_out;
        if (indices.empty())
            return data_out;
        for (int i = 0; i < indices.size(); i++)
        {
            vector<points> obj_indice;
            for (int j = 0; j < indices[i].size(); j++)
            {
                points tmp;
                tmp.x = data_in[indices[i][j]].x;
                tmp.y = data_in[indices[i][j]].y;
                tmp.z = data_in[indices[i][j]].z;
                obj_indice.emplace_back(tmp);
            }
            data_out.emplace_back(obj_indice);
            obj_indice.clear();
        }
        return data_out;
    }
    vector<int> DBSCAN::ClusteringSingle(const vector<points> &data_in)
    {
        vector<int> data_out;
        cluster_.clear();
        if (data_in.empty())
            return data_out;
        size_ = data_in.size();
        points_.resize(size_);
        adj_points_.resize(size_);
        for (int i = 0; i < size_; i++)
        {
            points_[i].id = i;
            points_[i].x = data_in[i].x;
            points_[i].y = data_in[i].y;
            points_[i].z = data_in[i].z;   // ← 原来漏了这一行，聚类因此退化成二维
        }
        // 统计每个点周围的点数，为后面筛选核心点、边缘点及噪点做准备
        CheckNearPoints();
        // 聚类，将一类的点标记为同一个cluster_idx
        for (int i = 0; i < size_; i++)
        {
            // 未分类的点标记为DB_NOT_CLASSIFIED
            if (points_[i].cluster_idx != DB_NOT_CLASSIFIED)
                continue;
            // 非核心点直接标记为噪点
            if (IsCoreObject(i))
            {
                DepthFirstSearch(i, ++cluster_idx_);
            }
            else
            {
                points_[i].cluster_idx = DB_NOISE;
            }
        }
        // 将每一类的id取出来
        cluster_.resize(cluster_idx_ + 1);
        for (int i = 0; i < size_; i++)
        {
            if (points_[i].cluster_idx != DB_NOISE)
            {
                cluster_[points_[i].cluster_idx].push_back(i);
            }
        }
        for (auto p : cluster_)
        {
            if (p.size() > data_out.size())
                data_out = p;
        }
        return data_out;
    }
    void DBSCAN::CheckNearPoints()
    {
        /* KD-tree 半径搜索，取代原来的 O(n²) 双重循环。
           原码每个点都要和**全部** n 个点比一次距离，实测（-O3）：
             2058 点 10.5 ms、6711 点 112 ms、19446 点 928 ms —— 严格二次
             （点数 ×2.9 耗时 ×8.3）。而簇扩展本身只要 0.13~8.4 ms，
             也就是说十几到九百毫秒几乎全花在数邻点上。要往细体素走
             （体素 0.10 时点数 ~13000）就必须先换掉它。

           **下面三条语义必须逐条守住**，否则"纯提速、不改行为"这个说法不成立：

           1. 邻居**不含自己**。原码 `if (i == j) continue;` 让 min_points_num_
              不把自己算进去，与 PCL setMinNeighborsInRadius 的语义**相反**
              （见 dbscan.h:53-59 的注释）。PCL 的 radiusSearch 会把查询点
              自己以距离 0 返回，所以这里要显式跳过。

           2. **返回的下标要自己升序排序**。KdTreeFLANN::radiusSearch 没有
              sorted 参数（那是基类 pcl::search::KdTree 的），返回顺序由 FLANN
              内部决定；原码是按 j 升序 push 的。排序之后两者输出逐位一致 ——
              这是这次改动唯一的验证手段（见 /tmp/dbscan_ref.cpp）。
              别把这行排序当成冗余删掉。

           3. **非有限点要跳过**。DbscanType 存 double，pcl::PointXYZ 是 float，
              正常点往返无损；但 inf/NaN（或超出 float 范围的 double）进 KD-tree
              会破坏树结构，连累**别的**点算错。原码里这种点天然是孤立的
              （GetDist 返回 NaN，`NaN <= eps_` 为假），这里用同样的语义：
              不进树、也没有邻居。

           注意原码只 push 了 j，没有 push i 自己；KD-tree 版同样只 push 别人。 */
        if (size_ <= 0)
            return;

        /* 先把有限点挑出来建树。两张映射表都是 O(1) 查表 —— 反查必须建表，
           不能在邻居循环里线性扫（那会把复杂度做到 O(n²·k)，比原码还糟）。 */
        std::vector<int> tree_of(size_, -1);  // 原下标 → 树内下标
        std::vector<int> orig_of;             // 树内下标 → 原下标
        orig_of.reserve(size_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->reserve(size_);
        for (int i = 0; i < size_; i++)
        {
            pcl::PointXYZ p;
            p.x = static_cast<float>(points_[i].x);
            p.y = static_cast<float>(points_[i].y);
            p.z = static_cast<float>(points_[i].z);
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                continue;
            tree_of[i] = static_cast<int>(orig_of.size());
            orig_of.push_back(i);
            cloud->push_back(p);
        }
        if (cloud->empty())
            return;

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);

        std::vector<int>   idx;
        std::vector<float> dist2;
        for (int i = 0; i < size_; i++)
        {
            if (tree_of[i] < 0)
                continue; // 非有限点：无邻居，等价于原码
            idx.clear();
            dist2.clear();
            kdtree.radiusSearch(cloud->points[tree_of[i]], eps_, idx, dist2); // max_nn=0 → 半径内全部
            std::sort(idx.begin(), idx.end()); // ← 见上面第 2 条，别删
            for (int k : idx)
            {
                const int j = orig_of[k];
                if (j == i)
                    continue; // 自己不算邻居，见上面第 1 条
                points_[i].adj_points_count++; // 统计每个点周围满足距离阈值的点数
                adj_points_[i].push_back(j);   // 将这些点的id统计到adj_points_中
            }
        }
    }
    bool DBSCAN::IsCoreObject(int idx)
    {
        return points_[idx].adj_points_count >= min_points_num_;
    }
    void DBSCAN::DepthFirstSearch(int now, int c)
    {
        points_[now].cluster_idx = c;
        if (!IsCoreObject(now))
            return;

        for (auto &next : adj_points_[now])
        {
            if (points_[next].cluster_idx != DB_NOT_CLASSIFIED)
                continue;
            DepthFirstSearch(next, c);
        }
    }
}