#ifndef LIDAR_OBJECTS_H
#define LIDAR_OBJECTS_H
#include "common/data_pool.h"
#include "common/log.h"
#include "lidar_obj_type.h"
#include <math.h>
// #include "lidar_preprocess.h"
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/region_growing.h>
#include "lidar_preprocess.h"
#include "json/include/json.h"
#include "lidar_cluster.h"
#include "lidar_preprocess2.h"
namespace perception
{
    namespace lidar_objs
    {
        class LidarCluster
        {
        private:
            double pi = 3.1415926;
            objs_config objs_config_;
            // pcl::PointCloud<pcl::PointXYZI>::Ptr clouds_filter(new pcl::PointCloud<pcl::PointXYZI>);
            //debug
            ros::NodeHandle ph;
            ros::Publisher pub_lidar_obj_debug;
            bool debug_ = true;

            perception::ReadConfigCommon readconfig_;
            // std::shared_ptr<LidarPreprocess> lidar_preprocess;
            std::shared_ptr<LidarPreprocess2> lidar_preprocess;
            std::shared_ptr<PointsCluster> lidar_cluster_;
            // LidarPreprocess lidar_preprocess;

            bool ReadObjConfig(const std::map<string, string> &m, objs_config &conf);
            std::vector<pcl::PointIndices> ClusterIndicesEE(const pcl::PointCloud<pcl::PointXYZ>::Ptr &data_in, std::vector<pcl::PointIndices> cluster_indices);
            std::vector<pcl::PointIndices> ClusterIndicesRG(const pcl::PointCloud<pcl::PointXYZ>::Ptr &data_in, std::vector<pcl::PointIndices> cluster_indices);
            template <typename T>
            int Objslist(const T &data_in, std::vector<pcl::PointIndices> cluster_indices, lidar_msgs::msg::Objects &objs_list);
            template <typename T1, typename T2>
            int VoxelFilterTest(const T1 &data_in, T1 &data_out, T2 type);
            bool ReadCellConfigJson(const std::string &path, objs_config &conf);

        public:
            LidarCluster(/* args */);
            ~LidarCluster();
            int Init();
            int Process(const LidarDataInType &lidar_points, lidar_msgs::msg::Objects &lidar_objs);
        };

    }
}
#endif