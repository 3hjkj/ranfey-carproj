#ifndef __DATA_TYPE_H__
#define __DATA_TYPE_H__

#include <cmath>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <string>
#include <unordered_map>
#include <vector>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include "lidar_msgs/msg/cell.hpp"
#include "lidar_msgs/msg/cells.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "lidar_points_type.h"
#include "lidar_msgs/msg/object.hpp"
#include "lidar_msgs/msg/objects.hpp"
#define MODE_RW_UGO (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH)
namespace perception
{
    // localization
    struct Localization
    {
        bool valid = false;
        double time;
        double lon = 0.0;
        double lat = 0.0;
        double xg = 0.0;
        double yg = 0.0;
        double zg = 0.0;
        double yawrate = 0.0;
        float angle = 0.0;
        float pitch = 0.0;
        float roll = 0.0;
        float vx = 0.0;
        float vy = 0.0;
        float accx = 0.0;
        float accy = 0.0;
        float speed = 0.0;

        double dr_time;
        double dr_x = 0.0;
        double dr_y = 0.0;
        double dr_z = 0.0;
        float dr_roll = 0.0;
        float dr_pitch = 0.0;
        float dr_yaw = 0.0;
    };

    // lidar
    struct LidarPoint
    {
        double x;
        double y;
        double z;
        double i; // intensity
    };
    struct LidarPoints
    {
        std::vector<LidarPoint> points;
    };

    struct LidarDataInType
    {
        double time;
        // std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> point_cloud_ptr_map;
        // LidarDataInType()
        // {
        //     time = 0.0;
        //     point_cloud_ptr_map[0].reset(new pcl::PointCloud<pcl::PointXYZ>);
        //     point_cloud_ptr_map[1].reset(new pcl::PointCloud<pcl::PointXYZ>);
        //     point_cloud_ptr_map[2].reset(new pcl::PointCloud<pcl::PointXYZ>);
        //     point_cloud_ptr_map[3].reset(new pcl::PointCloud<pcl::PointXYZ>);
        //     point_cloud_ptr_map[4].reset(new pcl::PointCloud<pcl::PointXYZ>);
        // }
        // void reset()
        // {
        //     time = 0.0;
        //     point_cloud_ptr_map[0].reset(new pcl::PointCloud<pcl::PointXYZ>);
        //     point_cloud_ptr_map[1].reset(new pcl::PointCloud<pcl::PointXYZ>);
        //     point_cloud_ptr_map[2].reset(new pcl::PointCloud<pcl::PointXYZ>);
        //     point_cloud_ptr_map[3].reset(new pcl::PointCloud<pcl::PointXYZ>);
        //     point_cloud_ptr_map[4].reset(new pcl::PointCloud<pcl::PointXYZ>);
        // }
        std::map<int, pcl_util::PointCloudPtr> point_cloud_ptr_map;
        LidarDataInType()
        {
            time = 0.0;
            point_cloud_ptr_map[0].reset(new pcl_util::PointCloud());
            point_cloud_ptr_map[1].reset(new pcl_util::PointCloud());
            point_cloud_ptr_map[2].reset(new pcl_util::PointCloud());
            point_cloud_ptr_map[3].reset(new pcl_util::PointCloud());
            point_cloud_ptr_map[4].reset(new pcl_util::PointCloud());
        }
        void reset()
        {
            time = 0.0;
            point_cloud_ptr_map[0].reset(new pcl_util::PointCloud());
            point_cloud_ptr_map[1].reset(new pcl_util::PointCloud());
            point_cloud_ptr_map[2].reset(new pcl_util::PointCloud());
            point_cloud_ptr_map[3].reset(new pcl_util::PointCloud());
            point_cloud_ptr_map[4].reset(new pcl_util::PointCloud());
        }
    };
    // radar
    struct RadarObjs
    {
    };

    // fusion
    struct PAndIdx
    {
        int idx;
        Eigen::MatrixXf p;
    };
    struct FusionObjs
    {
        lidar_msgs::msg::Objects lidar_now_objs;
        lidar_msgs::msg::Objects radar_now_objs;
        lidar_msgs::msg::Objects camera_now_objs;
        std::vector<lidar_msgs::msg::Objects> fusion_old_objs; // 用于存储上几帧结果
        std::vector<lidar_msgs::msg::Objects> fusion_tmp_objs; // 用于存储中间的匹配结果，包括匹配，上一帧未匹配，现在未匹配
        lidar_msgs::msg::Objects fusion_output_objects;        // 输出现在的结果
        std::vector<PAndIdx> p_id;                        // 用于存储卡尔曼滤波的p和目标id
        std::vector<PAndIdx> p_id_old;
        std::vector<PAndIdx> p_id_tmp;
    };

    struct MainData
    {
        Localization loc;
        // lidar
        LidarDataInType lidar_points_cells;
        LidarDataInType lidar_points_objs;
        lidar_msgs::msg::Cells lidar_cells;
        lidar_msgs::msg::Objects lidar_objs;
        // radar
        lidar_msgs::msg::Objects radar_objs;
        // camera
        lidar_msgs::msg::Objects camera_objs;
        // fusion
        FusionObjs fusion_objs;
    };

    // } //lidar_cells
} //perception
#endif