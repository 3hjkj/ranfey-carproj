#pragma once
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "common/data_type.h"
#include "fusion_params.h"
#include "tracker.h"
#include "objects2cells.h"
#include "camera_projection.h"

namespace perception
{
namespace fusion
{

// 三传感器融合编排：
//   激光 lidar_objs + 雷达 radar_objs + 相机 VisionBoxes
//   → 关联/跟踪 → fusion_output_objects（全部目标）+ fusion_cells（confirmed 转栅格）
class FusionNode
{
public:
  explicit FusionNode(const rclcpp::Node::SharedPtr & node);
  int Init();

  // 每周期调用（与 perception 主循环同线程）。读 MainData 各源，写 fusion 结果。
  int Process(MainData & data, double now_ts);

private:
  void BuildCameraOnlyTracks(const MainData & data, double now_ts);
  void FillOutputObject(const FusionTrack & t, lidar_msgs::msg::Object & o, double now) const;

  rclcpp::Node::SharedPtr node_;
  FusionParams p_;
  Tracker tracker_;
  Objects2Cells objects2cells_;
  CameraProjection camera_;
  bool camera_valid_ = false;
  bool first_ = true;
  double last_ts_ = 0.0;
};

}  // namespace fusion
}  // namespace perception
