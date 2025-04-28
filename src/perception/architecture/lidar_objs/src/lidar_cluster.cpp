#include "../include/lidar_cluster.h"
namespace perception
{
    PointsCluster::PointsCluster()
    {

        dbscan_ = std::make_shared<DBSCAN>(0.6, 3);
        min_rotate_rect_ = std::make_shared<min_rotate_rect>();
    }
    PointsCluster::~PointsCluster()
    {
    }

    lidar_msgs::msg::Objects PointsCluster::Pub(const pcl::PointCloud<pcl::PointXYZ>::Ptr &data_in)
    {
        std::vector<points> data;
        data = CloudToPoints(data_in);
        // LOG(INFO) << data.size();
        std::vector<std::vector<points>> indices = dbscan_->Clustering(data);
        lidar_msgs::msg::Objects objs;
        int idx = 0;
        if (0)
        {
            fstream outline1;

            outline1.open("/home/wj/code/perception/tool/test_julei/" + std::to_string(0) + ".txt", ios::ate | ios::out);
            for (auto p : data)
            {
                outline1 << std::setprecision(10) << p.x << "," << p.y << "," << p.z << "\n";
            }
        }
        if (indices.empty())
        {
            return objs;
        }
        // 3.取聚类id并进行画框
        // LOG(INFO) << "indices size :" << indices.size() << "\n";
        for (auto p : indices)
        {
            // LOG(INFO) << "p.size():" << p.size();
            lidar_msgs::msg::Object obj;
            std::vector<double> four_points;
            if (p.size() < 4)
                continue;
            // x y length width height rad
            four_points = min_rotate_rect_->MinRotateRect(p);
            // 输出

            if (four_points[4] < 0.2) // z
                continue;
            if (four_points.empty())
            {
                // LOG(ERROR) << "four_points is empty";
                four_points.clear();
            }
            else
            {
                if (0)
                {
                    // LOG(INFO) << "x:" << four_points[0] << ","
                    //           << "y:" << four_points[1] << ","
                    //           << "length:" << four_points[2] << ","
                    //           << "width:" << four_points[3] << ","
                    //           << "height:" << four_points[4] << ","
                    //           << "heading:" << four_points[5] * arc2degree << ",";
                }
                obj.rel_x = four_points[0];
                obj.rel_y = four_points[1];
                obj.rel_z = four_points[4];
                obj.length = four_points[2];
                obj.width = four_points[3];
                obj.height = four_points[4];
                obj.rel_heading = four_points[5] * arc2degree;
                // obj.rel_heading = 0;
                obj.idx = idx;
                idx++;
                if (obj.length < 0.4 && obj.width < 0.4 && obj.height > 0.8)
                {
                    obj.type = 3;
                }
                else if (((obj.length < 3 && obj.length > 0.4 && obj.width < 0.4) ||
                          (obj.length < 0.4 && obj.width > 0.4 && obj.width < 3)) &&
                         obj.height > 0.8)
                {
                    obj.type = 4;
                }
                else if (obj.length > 0.4 && obj.width > 0.4 && obj.height > 0.8)
                {
                    obj.type = 2;
                }
                else
                {
                    obj.type = 0;
                }
                objs.objs.emplace_back(obj);
                four_points.clear();
            }
        }
        indices.clear();

        // LOG(INFO) << "objs:size:" << objs.objs.size();
        INFO("cluster_objs:size:{}", objs.objs.size());
        return objs;
    }
    std::vector<points> PointsCluster::CloudToPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &msg)
    {
        std::vector<points> data_out;
        if (msg->points.empty())
        {
            // LOG(ERROR) << "data in is empty!!!";
            return data_out;
        }
        for (auto &p : msg->points)
        {
            points pp;
            pp.x = p.x;
            pp.y = p.y;
            pp.z = p.z;
            data_out.emplace_back(pp);
        }
        return data_out;
    }
}