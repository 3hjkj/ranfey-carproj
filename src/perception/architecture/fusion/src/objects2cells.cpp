#include "architecture/fusion/include/objects2cells.h"
#include <algorithm>
#include <cmath>

namespace perception
{
namespace fusion
{

void Objects2Cells::Convert(const lidar_msgs::msg::Objects & objs, double now,
                            lidar_msgs::msg::Cells & cells) const
{
  cells.cells.clear();
  if (objs.objs.empty()) return;

  const double half = cell_size_ / 2.0;

  for (const auto & o : objs.objs) {
    // 只输出已确认目标（防止相机/噪声抖动误占位）
    // 确认与否由调用方在填充 objs 时决定（Object.used 或本处过滤不可见）
    const double hx = o.length / 2.0 + p_.cell_inflate;
    const double hy = o.width / 2.0 + p_.cell_inflate;

    const int gx0 = std::max(0, static_cast<int>(std::floor((o.rel_x - hx - xmin_) / cell_size_)));
    const int gx1 = std::min(row_ - 1, static_cast<int>(std::floor((o.rel_x + hx - xmin_) / cell_size_)));
    const int gy0 = std::max(0, static_cast<int>(std::floor((o.rel_y - hy - ymin_) / cell_size_)));
    const int gy1 = std::min(col_ - 1, static_cast<int>(std::floor((o.rel_y + hy - ymin_) / cell_size_)));

    const float h = std::max(o.height, 0.3f);      // 保证 z_intercept 越过 min_h
    const float zmin = 0.0f, zmax = h, zmean = h / 2.0f;
    const float xmin_o = o.rel_x - hx, xmax_o = o.rel_x + hx;
    const float ymin_o = o.rel_y - hy, ymax_o = o.rel_y + hy;

    for (int gx = gx0; gx <= gx1; ++gx) {
      for (int gy = gy0; gy <= gy1; ++gy) {
        const float cx = static_cast<float>(gx * cell_size_ + xmin_ + half);
        const float cy = static_cast<float>(gy * cell_size_ + ymin_ + half);
        // 格心落在目标占地框内才占用
        if (std::fabs(cx - o.rel_x) > hx || std::fabs(cy - o.rel_y) > hy) continue;

        lidar_msgs::msg::Cell cell;
        cell.x = cx;
        cell.y = cy;
        cell.idx = gx * col_ + gy;
        cell.x_min = xmin_o;  cell.x_max = xmax_o;
        cell.y_min = ymin_o;  cell.y_max = ymax_o;
        cell.zmin = zmin;  cell.zmax = zmax;
        cell.z_intercept = h;
        cell.zmean = zmean;
        cell.points_num = 1;
        cell.confidence = static_cast<float>(p_.cell_conf);
        cell.vaild = 1;
        cell.time = now;
        cells.cells.emplace_back(cell);
      }
    }
  }
}

}  // namespace fusion
}  // namespace perception
