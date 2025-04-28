#ifndef LIDAR_OBJECTS_H
#define LIDAR_OBJECTS_H

/* ===== ROS 2 头文件 ===== */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

/* ===== 业务公共头，与原文件一致 ===== */
#include "common/data_pool.h"
#include "common/log.h"
#include "lidar_obj_type.h"
#include <math.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/region_growing.h>
#include "lidar_preprocess.h"
#include "lidar_preprocess2.h"
#include "lidar_cluster.h"
#include "json/include/json.h"

namespace perception::lidar_objs
{

class LidarCluster
{
private:
  /* ========= 与算法相关的私有成员（保持不变） ========= */
  double pi = 3.1415926;
  objs_config objs_config_;

  /* 调试发布器：ROS 2 写法改为智能指针 */
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_obj_debug;

  bool debug_ = true;

  perception::ReadConfigCommon readconfig_;
  std::shared_ptr<LidarPreprocess2> lidar_preprocess;
  std::shared_ptr<PointsCluster>   lidar_cluster_;

  /* ========= 私有工具函数（名字完全不变） ========= */
  bool ReadObjConfig(const std::map<std::string, std::string>& m, objs_config& conf);
  std::vector<pcl::PointIndices> ClusterIndicesEE(const pcl::PointCloud<pcl::PointXYZ>::Ptr& data_in,
                                                  std::vector<pcl::PointIndices> cluster_indices);
  std::vector<pcl::PointIndices> ClusterIndicesRG(const pcl::PointCloud<pcl::PointXYZ>::Ptr& data_in,
                                                  std::vector<pcl::PointIndices> cluster_indices);
  template<typename T>
  int Objslist(const T& data_in,
               std::vector<pcl::PointIndices> cluster_indices,
               lidar_msgs::msg::Objects& objs_list);
  template<typename T1, typename T2>
  int VoxelFilterTest(const T1& data_in, T1& data_out, T2 type);
  bool ReadCellConfigJson(const std::string& path, objs_config& conf);

public:
  /* 构造函数现在需要节点句柄 */
  explicit LidarCluster(const rclcpp::Node::SharedPtr& node);
  ~LidarCluster() = default;

  /* 与原接口一致的外部函数 */
  int Init();
  int Process(const LidarDataInType& lidar_points,
              lidar_msgs::msg::Objects& lidar_objs);
};

} // namespace perception::lidar_objs

#endif  // LIDAR_OBJECTS_H
