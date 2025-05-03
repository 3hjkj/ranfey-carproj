#pragma once                   // 建议加防重

#include <vector>
#include <string>
#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"                                 // ← ROS2
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lidar_msgs/msg/cells.hpp"
#include "lidar_msgs/msg/cell.hpp"
#include "can_control_msgs/msg/autocontrol.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"           // ← tf2
#include <Eigen/Core>
#include <Eigen/Dense>

class Local_route
{
private:
  rclcpp::Node::SharedPtr node_;
  /* ======= 本地/全局路径可视化 Publisher ======= */
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_local_center_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_local_right_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_local_left_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_global_center_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_global_left_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_global_right_;

  /* ======= 参数 ======= */
  double point_distance     = 0.3;
  int    chang_lane_point   = 50;
  double lane_distance      = 2.0;
  double chang_lane_distance= 0.0;
  double threshold_y        = 1.7;

  int    keep_point         = 10;
  double weight_data        = 0.45;
  double weight_smooth      = 0.4;
  double tolerance          = 0.05;

public:
  Local_route()  = default;
  ~Local_route() = default;

  /* ======= 外部可直接访问的量 ======= */
  int local_path_size = 65;
  std::vector<std::vector<double>> local_path;

  /* ======= 接口函数 ======= */
  void init_pub(const rclcpp::Node::SharedPtr & node);          // ← 参数类型替换
  void ReadTxt(const std::string & trace_path,
               std::vector<std::vector<double>> & paths);
  void split(const std::string & str, const std::string & pattern,
             std::vector<double> & result);

  void visiual_global_trace(const std::vector<std::vector<double>> & paths);
  void route_plan(const std::vector<std::vector<double>> & paths,
                  int trace_id, int local_point_id);

  double azimuthAngle(double x1, double y1, double x2, double y2);

  void smoothPath(std::vector<std::vector<double>> & paths_dect,
                  double weight_data, double weight_smooth,
                  double tolerance);

  bool generator_local_trace(const std::vector<double> & x_orignal,
                             const std::vector<double> & y_orignal,
                             const std::vector<double> & yaw_orignal,
                             const std::vector<double> & x_target,
                             const std::vector<double> & y_target,
                             std::vector<double> & local_x,
                             std::vector<double> & local_y,
                             std::vector<double> & local_yaw,
                             int local_point_id);

  void debug_show_road_cells(std::vector<can_control_msgs::msg::Autocontrol> & vec_at,
                             const lidar_msgs::msg::Cells & cells_,
                             int V_RefPoint,
                             double yaw_vel, double x_vel, double y_vel);

  void calculate_parameter();
};
