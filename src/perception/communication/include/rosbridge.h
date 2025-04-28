#pragma once
/****************************************************************************
 *  ROS 2 版 RosBridge
 *  ◇ 依赖 rclcpp / sensor_msgs / lidar_msgs / localization_msgs
 *  ◇ 支持 5 路 LiDAR 点云（Objs + Cells）和定位数据
 *  ◇ 负责订阅 → 入 DataPool，处理完后 Publish 结果
 ****************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <lidar_msgs/msg/cells.hpp>
#include <lidar_msgs/msg/objects.hpp>
#include <localization_msgs/msg/localization.hpp>
#include "common/data_pool.h"
#include "common/log.h"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <array>
#include <string>
#include <memory>
#include <exception>
namespace perception
{

class RosBridge
{
public:
  /** 构造时注入 Node 句柄，方便创建订阅 / 发布器 / 参数 */
  explicit RosBridge(const rclcpp::Node::SharedPtr & node);
  ~RosBridge()  = default;

  /** 目前 Publish 单独调用即可；如需异步定时，可封装在定时器里 */
  int Publish();

private:
  /* ---------- 内部回调 ---------- */
  void cbObj (const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg, int idx);
  void cbCell(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg, int idx);
  void cbLocalization(const localization_msgs::msg::Localization::ConstSharedPtr & msg);

  /* ---------- ROS 对象 ---------- */
  rclcpp::Node::SharedPtr node_;

  std::array<std::string,5> lidar_topic_names_;   //!< 通过参数读入
  std::array<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr,5> sub_objs_;
  std::array<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr,5> sub_cells_;

  rclcpp::Publisher<lidar_msgs::msg::Cells>::SharedPtr   pub_cells_;
  rclcpp::Publisher<lidar_msgs::msg::Objects>::SharedPtr pub_objs_;
  rclcpp::Publisher<lidar_msgs::msg::Objects>::SharedPtr pub_fusion_;
};

} // namespace perception
