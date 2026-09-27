#include "architecture/fusion/include/data_association.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace perception
{
namespace fusion
{

namespace
{

// 贪心分配：把 n_track×n_meas 代价矩阵中的有限元素按代价升序依次分配，
// 每条 track 与每个测量最多用一次。输入输出均为索引向量（-1 = 未分配）。
void GreedyAssign(const std::vector<std::vector<double>> & cost,
                  std::vector<int> & track2meas,
                  std::vector<int> & meas_unmatched)
{
  const int nt = static_cast<int>(cost.size());
  const int nm = nt > 0 ? static_cast<int>(cost[0].size()) : 0;

  track2meas.assign(nt, -1);
  std::vector<bool> meas_used(nm, false);

  // 收集有限代价并排序（代价, track, meas）
  std::vector<std::tuple<double, int, int>> cand;
  cand.reserve(static_cast<size_t>(nt) * nm);
  for (int i = 0; i < nt; ++i) {
    for (int j = 0; j < nm; ++j) {
      if (cost[i][j] < std::numeric_limits<double>::infinity())
        cand.emplace_back(cost[i][j], i, j);
    }
  }
  std::sort(cand.begin(), cand.end(),
            [](const auto & a, const auto & b) { return std::get<0>(a) < std::get<0>(b); });

  for (const auto & [c, i, j] : cand) {
    (void)c;
    if (track2meas[i] != -1 || meas_used[j]) continue;
    track2meas[i] = j;
    meas_used[j] = true;
  }

  meas_unmatched.clear();
  for (int j = 0; j < nm; ++j)
    if (!meas_used[j]) meas_unmatched.push_back(j);
}

}  // namespace

double DataAssociation::LidarCost(const FusionTrack & t,
                                  const lidar_msgs::msg::Object & o, double dt) const
{
  const double spd = std::hypot(t.x[2], t.x[3]);
  const double gate = std::max(p_.gate_xy_m, p_.gate_v_scale + spd * dt);
  const double dx = t.x[0] - o.rel_x;
  const double dy = t.x[1] - o.rel_y;
  const double d = std::hypot(dx, dy);
  return d <= gate ? d : std::numeric_limits<double>::infinity();
}

double DataAssociation::RadarCost(const FusionTrack & t,
                                  const lidar_msgs::msg::Object & o) const
{
  // Mahalanobis：z=[x,y,vx,vy]，S = H P H^T + R（H=I）
  Eigen::Vector4f dz;
  dz << t.x[0] - o.rel_x, t.x[1] - o.rel_y, t.x[2] - o.rel_vx, t.x[3] - o.rel_vy;
  Eigen::Matrix4f S = t.P;
  S(0, 0) += p_.r_radar_xy;  S(1, 1) += p_.r_radar_xy;
  S(2, 2) += p_.r_radar_v;   S(3, 3) += p_.r_radar_v;
  Eigen::Matrix4f Sinv = S.inverse();
  const double d2 = dz.transpose() * Sinv * dz;
  return d2 <= p_.mahalanobis_gate ? d2 : std::numeric_limits<double>::infinity();
}

MatchResult DataAssociation::Match(const std::vector<FusionTrack> & tracks,
                                   const lidar_msgs::msg::Objects & lidar,
                                   const lidar_msgs::msg::Objects & radar,
                                   double dt) const
{
  MatchResult res;
  const int nt = static_cast<int>(tracks.size());
  const int nl = static_cast<int>(lidar.objs.size());
  const int nr = static_cast<int>(radar.objs.size());

  // 激光：纯位置门
  if (nt > 0 && nl > 0) {
    std::vector<std::vector<double>> cost(nt, std::vector<double>(nl));
    for (int i = 0; i < nt; ++i)
      for (int j = 0; j < nl; ++j)
        cost[i][j] = LidarCost(tracks[i], lidar.objs[j], dt);
    GreedyAssign(cost, res.track2lidar, res.lidar_unmatched);
  } else {
    res.track2lidar.assign(nt, -1);
    for (int j = 0; j < nl; ++j) res.lidar_unmatched.push_back(j);
  }

  // 雷达：Mahalanobis（含速度维）
  if (nt > 0 && nr > 0) {
    std::vector<std::vector<double>> cost(nt, std::vector<double>(nr));
    for (int i = 0; i < nt; ++i)
      for (int j = 0; j < nr; ++j)
        cost[i][j] = RadarCost(tracks[i], radar.objs[j]);
    GreedyAssign(cost, res.track2radar, res.radar_unmatched);
  } else {
    res.track2radar.assign(nt, -1);
    for (int j = 0; j < nr; ++j) res.radar_unmatched.push_back(j);
  }

  return res;
}

}  // namespace fusion
}  // namespace perception
