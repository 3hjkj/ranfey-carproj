#pragma once
/* ─── ROS 2 头文件 ─────────────────────────────────────────── */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

/* ─── PCL & STL 头文件（与原相同） ─────────────────────────── */
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.hpp>          // ROS 2 的 pcl_ros
#include <math.h>
#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <iomanip>
#include "common/log.h"

#include "lidar_msgs/msg/object.hpp"
#include "lidar_msgs/msg/objects.hpp"
#include "min_rotate_rect.h"

namespace perception
{

class PointsCluster
{
public:
  /* 构造函数需传入节点句柄；析构保持默认 */
  explicit PointsCluster(const rclcpp::Node::SharedPtr& node);

  /* 与旧版一致的外部接口：聚类并返回 Objects 消息 */
  lidar_msgs::msg::Objects Pub(const pcl::PointCloud<pcl::PointXYZ>::Ptr& data_in);

private:
  /* ============ ROS 成员 ============ */
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_no_ground_points_;

  /* ============ 业务成员（保持旧名） ============ */
  const bool debug = true;
  int  bfs_counter  = 0;
  std::string topic_name1 = "/perception/no_ground_points";

  const double degree2arc = 3.1415926 / 180.0;
  const double arc2degree = 180.0 / 3.1415926;

  pcl::PointCloud<pcl::PointXYZ>::Ptr point_tmp;
  std::shared_ptr<DBSCAN>           dbscan_;
  std::shared_ptr<min_rotate_rect>  min_rotate_rect_;

  int counter_txt = 0;

  /* 与旧版一致的内部工具函数 */
  std::vector<points> CloudToPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& msg);
};

}  // namespace perception
