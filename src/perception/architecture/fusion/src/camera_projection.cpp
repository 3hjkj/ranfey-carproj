#include "architecture/fusion/include/camera_projection.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include "json/include/json.h"

namespace perception
{
namespace fusion
{

namespace
{
// 名义前置相机：车辆+X(前)→相机+Z，车辆+Y(左)→相机-X，车辆+Z(上)→相机-Y
Eigen::Matrix3d R0()
{
  Eigen::Matrix3d R;
  R << 0, -1, 0,
       0,  0, -1,
       1,  0,  0;
  return R;
}

Eigen::Matrix3d Rx(double a)
{
  Eigen::Matrix3d R;
  R << 1, 0, 0,
       0, std::cos(a), -std::sin(a),
       0, std::sin(a),  std::cos(a);
  return R;
}
Eigen::Matrix3d Ry(double a)
{
  Eigen::Matrix3d R;
  R << std::cos(a), 0, std::sin(a),
       0, 1, 0,
       -std::sin(a), 0, std::cos(a);
  return R;
}
Eigen::Matrix3d Rz(double a)
{
  Eigen::Matrix3d R;
  R << std::cos(a), -std::sin(a), 0,
       std::sin(a),  std::cos(a), 0,
       0, 0, 1;
  return R;
}
}  // namespace

bool CameraProjection::LoadConfig(const std::string & json_path)
{
  std::ifstream in(json_path, std::ios::binary);
  if (!in.is_open()) return false;
  Json::Reader reader;
  Json::Value v;
  if (!reader.parse(in, v)) return false;

  width_  = v["image_width"].asInt();
  height_ = v["image_height"].asInt();
  fx_     = v["fx"].asDouble();
  fy_     = v["fy"].asDouble();
  cx_     = v["cx"].asDouble();
  cy_     = v["cy"].asDouble();
  t_v2c_ << v["t_x"].asDouble(), v["t_y"].asDouble(), v["t_z"].asDouble();
  const double yaw   = v["yaw"].asDouble();
  const double pitch = v["pitch"].asDouble();
  const double roll  = v["roll"].asDouble();

  Rv2c_ = Rz(yaw) * Ry(pitch) * Rx(roll) * R0();
  valid_ = (width_ > 0 && height_ > 0 && fx_ > 0 && fy_ > 0);
  return valid_;
}

bool CameraProjection::ProjectToImage(double xv, double yv, double zv,
                                      double * u, double * v) const
{
  if (!valid_) return false;
  Eigen::Vector3d pv(xv, yv, zv);
  Eigen::Vector3d pc = Rv2c_ * (pv - t_v2c_);
  if (pc.z() <= 0.01) return false;              // 相机后方
  const double uu = fx_ * pc.x() / pc.z() + cx_;
  const double vv = fy_ * pc.y() / pc.z() + cy_;
  if (uu < 0 || uu >= width_ || vv < 0 || vv >= height_) return false;
  *u = uu;
  *v = vv;
  return true;
}

bool CameraProjection::InBox(const lidar_msgs::msg::VisionBox & b, double u, double v) const
{
  return u >= b.x1 && u <= b.x2 && v >= b.y1 && v <= b.y2;
}

int CameraProjection::MatchBox(const lidar_msgs::msg::VisionBoxes & boxes,
                               double xv, double yv, double zv) const
{
  double u, v;
  if (!ProjectToImage(xv, yv, zv, &u, &v)) return -1;
  for (int i = 0; i < static_cast<int>(boxes.boxes.size()); ++i)
    if (InBox(boxes.boxes[i], u, v)) return i;
  return -1;
}

int CameraProjection::CocoToType(int cls_id)
{
  switch (cls_id) {
    case 0: return 3;   // person
    case 1: return 4;   // bicycle
    case 2: return 2;   // car
    case 3: return 2;   // motorcycle
    case 5: return 1;   // bus
    case 7: return 1;   // truck
    default: return 0;  // 不覆盖
  }
}

bool CameraProjection::BoxDepthFromPoints(
  const lidar_msgs::msg::VisionBoxes & boxes, int box_idx,
  const std::vector<Eigen::Vector3f> & points, int sample_step,
  double * depth_x, double * depth_y) const
{
  if (!valid_ || box_idx < 0 || box_idx >= static_cast<int>(boxes.boxes.size()))
    return false;
  const auto & b = boxes.boxes[box_idx];
  std::vector<float> xs, ys;
  const int step = std::max(1, sample_step);
  for (size_t i = 0; i < points.size(); i += step) {
    const auto & p = points[i];
    double u, v;
    if (!ProjectToImage(p.x(), p.y(), p.z(), &u, &v)) continue;
    if (!InBox(b, u, v)) continue;
    xs.push_back(p.x());
    ys.push_back(p.y());
  }
  if (xs.empty()) return false;
  const auto mid = xs.size() / 2;
  std::nth_element(xs.begin(), xs.begin() + mid, xs.end());
  std::nth_element(ys.begin(), ys.begin() + mid, ys.end());
  *depth_x = xs[mid];
  *depth_y = ys[mid];
  return true;
}

}  // namespace fusion
}  // namespace perception
