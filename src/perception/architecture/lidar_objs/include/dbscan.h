#ifndef _DBSCAN_H__
#define _DBSCAN_H__

#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>        // ← 原来是 <ros/ros.h>，仅此一处改动
// #include "glog/include/logging.h"

namespace perception
{
    struct points
    {
        double x;
        double y;
        double z;
    };

    using namespace std;

    const int DB_NOISE          = -2;
    const int DB_NOT_CLASSIFIED = -1;

    class DbscanType
    {
    public:
        DbscanType()  = default;
        ~DbscanType() = default;

        int    id               = 0;
        double x                = 0.0;
        double y                = 0.0;
        int    adj_points_count = 0;
        int    cluster_idx      = DB_NOT_CLASSIFIED;

        double GetDist(const DbscanType& ot)
        {
            if (fabs(x - ot.x) > 0.5 || fabs(y - ot.y) > 0.5)
                return 10;
            else
                return sqrt((x - ot.x) * (x - ot.x) +
                            (y - ot.y) * (y - ot.y));
        }
    };

    class DBSCAN
    {
    private:
        vector<DbscanType> points_;
        vector<vector<int>> cluster_;     // 存储聚类结果
        vector<vector<int>> adj_points_;  // 存储邻接表

        double eps_;
        int    min_points_num_;
        int    size_;
        int    cluster_idx_;

        void ClearBuffer();
        void DepthFirstSearch(int now, int c);
        void CheckNearPoints();
        bool IsCoreObject(int idx);
        vector<vector<points>> GetResult(const vector<points>& data_in,
                                         vector<vector<int>>& indices);

    public:
        DBSCAN(double eps, int min_points_num);
        ~DBSCAN();

        void Init(const vector<points>& data_in);
        vector<vector<points>> Clustering(const vector<points>& data_in);
        vector<int>            ClusteringSingle(const vector<points>& data_in);
    };
}

#endif  // _DBSCAN_H__
