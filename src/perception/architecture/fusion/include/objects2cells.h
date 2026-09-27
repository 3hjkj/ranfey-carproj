#pragma once
#include <vector>
#include <lidar_msgs/msg/cells.hpp>
#include <lidar_msgs/msg/objects.hpp>
#include "fusion_params.h"

namespace perception
{
namespace fusion
{

// 融合目标 → 占用栅格 Cells。
// 栅格几何与 lidar_cell.cpp 的 cell.json 配置对齐：
//   x∈[-80,80], y∈[-15,15], cell=0.3m, col=100,
//   cell.x = gx*0.3 - 80 + 0.15, cell.y = gy*0.3 - 15 + 0.15, idx = gx*100 + gy
class Objects2Cells
{
public:
  explicit Objects2Cells(const FusionParams & p) : p_(p) {}

  // 只把 confirmed 目标转成占用格；objs 为空 → cells 为空（= 无障碍）
  void Convert(const lidar_msgs::msg::Objects & objs, double now,
               lidar_msgs::msg::Cells & cells) const;

private:
  double xmin_ = -80.0, ymin_ = -15.0;
  double cell_size_ = 0.3;
  int row_ = 533, col_ = 100;
  const FusionParams p_;
};

}  // namespace fusion
}  // namespace perception
