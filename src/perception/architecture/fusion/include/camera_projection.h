#pragma once
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <lidar_msgs/msg/vision_boxes.hpp>

namespace perception
{
namespace fusion
{

// 相机粗标定 + 3D→2D 投影关联。
// 约定：相机系 OpenCV（X右/Y下/Z前），车辆系（X前/Y左/Z上）。
// 外参 P_cam = Rv2c * (P_veh - t_v2c)，t_v2c 为相机光心在车辆系坐标。
// Rv2c = Rz(yaw)*Ry(pitch)*Rx(roll)*R0，R0 为名义前置相机旋转。
class CameraProjection
{
public:
  // 读取 JSON 配置（image_width/height、fx/fy/cx/cy、t_x/t_y/t_z、yaw/pitch/roll）
  bool LoadConfig(const std::string & json_path);
  bool valid() const { return valid_; }

  // 车辆系 (xv,yv,zv) → 像素 (u,v)。Zc>0 且在图像内返回 true。
  bool ProjectToImage(double xv, double yv, double zv, double * u, double * v) const;

  // 像素点是否落在 YOLO 框内
  bool InBox(const lidar_msgs::msg::VisionBox & b, double u, double v) const;

  // 返回该 3D 点命中的第一个框索引；未命中返回 -1
  int MatchBox(const lidar_msgs::msg::VisionBoxes & boxes,
               double xv, double yv, double zv) const;

  // COCO 类别 → lidar_msgs::Object.type
  static int CocoToType(int cls_id);

  // 取框内激光点的中位深度（车辆系 x）与横向中位（y）；采样步长降低开销。
  bool BoxDepthFromPoints(const lidar_msgs::msg::VisionBoxes & boxes, int box_idx,
                          const std::vector<Eigen::Vector3f> & points,
                          int sample_step, double * depth_x, double * depth_y) const;

  int width() const { return width_; }
  int height() const { return height_; }

private:
  Eigen::Matrix3d Rv2c_;
  Eigen::Vector3d t_v2c_;
  double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
  int width_ = 0, height_ = 0;
  bool valid_ = false;
};

}  // namespace fusion
}  // namespace perception
