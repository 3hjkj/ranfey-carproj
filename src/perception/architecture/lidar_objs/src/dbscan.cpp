#include "../include/dbscan.h"
namespace perception
{
    DbscanType::DbscanType(/* args */) {}

    DbscanType::~DbscanType() {}
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
        cout << " time check near:" << (time2 - time1) * 1000 << "ms" << endl;
        cout << " cluster:" << (time3 - time2) * 1000 << "ms" << endl;
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
        for (int i = 0; i < size_; i++)
        {
            for (int j = 0; j < size_; j++)
            {
                if (i == j)
                    continue;
                if (points_[i].GetDist(points_[j]) <= eps_)
                {
                    points_[i].adj_points_count++; // 统计每个点周围满足距离阈值的点数
                    adj_points_[i].push_back(j);   // 将这些点的id统计到adj_points_中
                }
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