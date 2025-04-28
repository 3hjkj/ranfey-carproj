
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include "common/log.h"
// pcl lib
#include <math.h>
#include <vector>
#include <sstream>
#include <string>
#include <fstream>
#include <cstring>
#include <string>
#include <algorithm>
#include <iomanip>
#include "lidar_msgs/msg/object.hpp"
#include "lidar_msgs/msg/objects.hpp"
#include "min_rotate_rect.h"
using namespace std;
#define MODE_RW_UGO (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH)
typedef pcl::PointXYZ xyz;
typedef pcl::PointXYZRGB xyzrgb;
namespace perception
{
    class PointsCluster
    {
    public:
        PointsCluster();
        ~PointsCluster();
        lidar_msgs::msg::Objects Pub(const pcl::PointCloud<pcl::PointXYZ>::Ptr &data_in);

    private:
        const bool debug = true;
        int bfs_counter = 0;
        std::string topic_name1 = "/perception/no_ground_points";

        const double degree2arc = 3.1415926 / 180.0;
        const double arc2degree = 180 / 3.1415926;

        pcl::PointCloud<pcl::PointXYZ>::Ptr point_tmp;
        std::shared_ptr<DBSCAN> dbscan_;
        std::shared_ptr<min_rotate_rect> min_rotate_rect_;
        int counter_txt = 0;
        std::vector<points> CloudToPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &msg);
    };
}