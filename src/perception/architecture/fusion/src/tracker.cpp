#include "architecture/fusion/include/tracker.h"
#include <algorithm>
#include <cmath>

namespace perception
{
namespace fusion
{

namespace
{
constexpr int kSrcLidar = 1;    // Object.msg source: 激光
constexpr int kSrcRadar = 2;    // 雷达
constexpr int kSrcCam   = 3;    // 相机
constexpr int kBitLidar = 0x1;
constexpr int kBitRadar = 0x2;
constexpr int kBitCam   = 0x4;
}  // namespace

void Tracker::Reset()
{
  tracks_.clear();
  next_id_ = 1;
}

void Tracker::KalmanPredict(FusionTrack & t, double dt)
{
  if (dt <= 0.0) return;
  Eigen::Matrix4f F = Eigen::Matrix4f::Identity();
  F(0, 2) = static_cast<float>(dt);
  F(1, 3) = static_cast<float>(dt);
  t.x = F * t.x;
  t.P = F * t.P * F.transpose();
  Eigen::Matrix4f Q = Eigen::Matrix4f::Zero();
  Q(0, 0) = Q(1, 1) = 0.01f;
  Q(2, 2) = Q(3, 3) = static_cast<float>(p_.q_v * dt);
  t.P += Q;
}

void Tracker::KalmanUpdateLidar(FusionTrack & t, const lidar_msgs::msg::Object & o)
{
  // 量测 z=[x,y]，H 为 2×4
  Eigen::Matrix<float, 2, 4> H = Eigen::Matrix<float, 2, 4>::Zero();
  H(0, 0) = H(1, 1) = 1.f;
  Eigen::Vector2f z; z << o.rel_x, o.rel_y;
  Eigen::Vector2f y = z - H * t.x;
  Eigen::Matrix2f S = H * t.P * H.transpose();
  S(0, 0) += static_cast<float>(p_.r_lidar_xy);
  S(1, 1) += static_cast<float>(p_.r_lidar_xy);
  Eigen::Matrix<float, 4, 2> K = t.P * H.transpose() * S.inverse();
  t.x += K * y;
  t.P = (Eigen::Matrix4f::Identity() - K * H) * t.P;

  // 属性合并：激光提供尺寸/类别
  if (o.length > 0) t.length = o.length;
  if (o.width > 0)  t.width  = o.width;
  if (o.height > 0) t.height = o.height;
  if (o.type != 0)  t.type   = o.type;
  t.source = kSrcLidar;
  t.contrib |= kBitLidar;
  if (o.score > 0) t.score = o.score;
  RecomputeConfidence(t);
}

void Tracker::KalmanUpdateRadar(FusionTrack & t, const lidar_msgs::msg::Object & o)
{
  // 量测 z=[x,y,vx,vy]，H=I4
  Eigen::Vector4f z; z << o.rel_x, o.rel_y, o.rel_vx, o.rel_vy;
  Eigen::Vector4f y = z - t.x;
  Eigen::Matrix4f S = t.P;
  S(0, 0) += static_cast<float>(p_.r_radar_xy);
  S(1, 1) += static_cast<float>(p_.r_radar_xy);
  S(2, 2) += static_cast<float>(p_.r_radar_v);
  S(3, 3) += static_cast<float>(p_.r_radar_v);
  Eigen::Matrix4f K = t.P * S.inverse();
  t.x += K * y;
  t.P = (Eigen::Matrix4f::Identity() - K) * t.P;

  if (o.width > 0 && t.width <= 0) t.width = o.width;
  // 来源优先级：保持已有更高优先级的源
  if (t.source == kSrcCam || t.source == 0) t.source = kSrcRadar;
  t.contrib |= kBitRadar;
  if (t.score == 0) t.score = static_cast<float>(p_.c_radar);
  RecomputeConfidence(t);
}

void Tracker::RecomputeConfidence(FusionTrack & t)
{
  double wsum = 0.0, wc = 0.0;
  if (t.contrib & kBitLidar) { wsum += p_.w_lidar;  wc += p_.w_lidar * p_.c_lidar; }
  if (t.contrib & kBitRadar) { wsum += p_.w_radar;  wc += p_.w_radar * p_.c_radar; }
  if (t.contrib & kBitCam)   { wsum += p_.w_camera; wc += p_.w_camera * t.score; }
  t.confidence = wsum > 0 ? static_cast<float>(wc / wsum) : 0.0f;
}

void Tracker::PredictAll(double dt)
{
  for (auto & [id, t] : tracks_)
    if (t.alive) KalmanPredict(t, dt);
}

void Tracker::Update(double dt,
                     const lidar_msgs::msg::Objects & lidar,
                     const lidar_msgs::msg::Objects & radar,
                     double now)
{
  // 1. 预测
  PredictAll(dt);

  // 2. 匹配（把 map 转成 vector 便于索引）
  std::vector<FusionTrack> tv;
  tv.reserve(tracks_.size());
  for (auto & [id, t] : tracks_) tv.push_back(t);
  MatchResult m = da_.Match(tv, lidar, radar, dt);

  // 3. 更新
  int idx = 0;
  for (auto & [id, t] : tracks_) {
    if (!t.alive) { ++idx; continue; }
    const int li = m.track2lidar[idx];
    const int ri = m.track2radar[idx];
    if (li >= 0) KalmanUpdateLidar(t, lidar.objs[li]);
    if (ri >= 0) KalmanUpdateRadar(t, radar.objs[ri]);
    t.last_seen = now;
    t.hits += (li >= 0 || ri >= 0) ? 1 : 0;
    t.misses += (li >= 0 || ri >= 0) ? 0 : 1;
    t.predict_cnt = (li >= 0 || ri >= 0) ? 0 : t.predict_cnt + 1;
    ++idx;
  }

  // 4. 新建未匹配测量
  for (int li : m.lidar_unmatched) {
    const auto & o = lidar.objs[li];
    FusionTrack t;
    t.track_id = next_id_++;
    t.x << o.rel_x, o.rel_y, 0.f, 0.f;
    t.P = Eigen::Matrix4f::Identity() * 2.0f;
    t.length = o.length; t.width = o.width; t.height = o.height;
    t.type = o.type;
    t.last_seen = now;
    t.hits = 1;
    t.contrib = kBitLidar;
    t.source = kSrcLidar;
    t.score = o.score > 0 ? o.score : static_cast<float>(p_.c_lidar);
    RecomputeConfidence(t);
    tracks_[t.track_id] = t;
  }
  for (int ri : m.radar_unmatched) {
    const auto & o = radar.objs[ri];
    FusionTrack t;
    t.track_id = next_id_++;
    t.x << o.rel_x, o.rel_y, o.rel_vx, o.rel_vy;
    t.P = Eigen::Matrix4f::Identity() * 2.0f;
    if (o.width > 0) t.width = o.width;
    t.last_seen = now;
    t.hits = 1;
    t.contrib = kBitRadar;
    t.source = kSrcRadar;
    t.score = static_cast<float>(p_.c_radar);
    RecomputeConfidence(t);
    tracks_[t.track_id] = t;
  }

  // 5. 生命周期管理
  Manage(now);
}

int Tracker::AddCandidate(const Eigen::Vector4f & x, double now, float score, int type)
{
  FusionTrack t;
  t.track_id = next_id_++;
  t.x = x;
  t.P = Eigen::Matrix4f::Identity() * 4.0f;   // 深度不准确 → 大协方差防误关联
  t.type = type;
  t.score = score;
  t.contrib = kBitCam;
  t.source = kSrcCam;
  t.last_seen = now;
  t.hits = 1;
  RecomputeConfidence(t);
  tracks_[t.track_id] = t;
  return t.track_id;
}

void Tracker::MergeTracks(FusionTrack & keep, const FusionTrack & absorb)
{
  keep.contrib |= absorb.contrib;
  if (absorb.length > 0 && keep.length <= 0) keep.length = absorb.length;
  if (absorb.width > 0  && keep.width  <= 0) keep.width  = absorb.width;
  if (absorb.height > 0 && keep.height <= 0) keep.height = absorb.height;
  if (absorb.type != 0 && keep.type == 0)    keep.type   = absorb.type;
  if (absorb.confirmed) keep.confirmed = true;
  keep.hits   = std::max(keep.hits, absorb.hits);
  keep.score  = std::max(keep.score, absorb.score);
}

void Tracker::Manage(double now)
{
  for (auto it = tracks_.begin(); it != tracks_.end();) {
    FusionTrack & t = it->second;
    if (!t.alive) { it = tracks_.erase(it); continue; }

    if (t.hits >= p_.confirm_hits) t.confirmed = true;
    if (t.misses > p_.miss_threshold || t.predict_cnt > p_.predict_max) {
      it = tracks_.erase(it);
      continue;
    }
    ++it;
  }

  // ---- 轨迹间合并：同一目标可能因启动时各源未匹配各自建轨而重复 ----
  // 持续同位（位置差 < merge_gate）merge_sustain_frames 帧 → 合并，
  // 保留"贡献源更多者"（popcount 大者），平局取置信度高者（仍平取命中多者）。
  // 第一遍：本帧是否与任一存活轨迹同位 → 是则计数+1，否则清零（不可按对清零，
  // 否则"远对"会误清掉同目标"近对"的累计）。
  for (auto & [idA, tA] : tracks_) {
    if (!tA.alive) continue;
    bool has_partner = false;
    for (const auto & [idB, tB] : tracks_) {
      if (idB == idA || !tB.alive) continue;
      if (std::hypot(tA.x[0] - tB.x[0], tA.x[1] - tB.x[1]) < p_.merge_gate) {
        has_partner = true;
        break;
      }
    }
    tA.merge_cnt = has_partner ? tA.merge_cnt + 1 : 0;
  }
  // 第二遍：达到持续帧数阈值的同位对 → 合并
  std::vector<int> to_erase;
  for (auto & [idA, tA] : tracks_) {
    if (!tA.alive) continue;
    for (auto & [idB, tB] : tracks_) {
      if (idB <= idA || !tB.alive) continue;   // 每对只处理一次
      if (std::hypot(tA.x[0] - tB.x[0], tA.x[1] - tB.x[1]) >= p_.merge_gate) continue;
      if (tA.merge_cnt < p_.merge_sustain_frames || tB.merge_cnt < p_.merge_sustain_frames) continue;
      const int ca = __builtin_popcount(tA.contrib);
      const int cb = __builtin_popcount(tB.contrib);
      FusionTrack * keep = (ca != cb) ? (ca > cb ? &tA : &tB)
                        : (tA.confidence != tB.confidence)
                          ? (tA.confidence > tB.confidence ? &tA : &tB)
                          : (tA.hits >= tB.hits ? &tA : &tB);
      FusionTrack * absorb = (keep == &tA) ? &tB : &tA;
      MergeTracks(*keep, *absorb);
      RecomputeConfidence(*keep);
      keep->merge_cnt = 0;
      to_erase.push_back(absorb->track_id);
      absorb->alive = false;
    }
  }
  for (int id : to_erase) tracks_.erase(id);
}

}  // namespace fusion
}  // namespace perception
