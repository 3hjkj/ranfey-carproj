#pragma once
#include <map>
#include <lidar_msgs/msg/objects.hpp>
#include "data_association.h"
#include "fusion_params.h"

namespace perception
{
namespace fusion
{

// 跨帧跟踪管理：常速卡尔曼 + 生命周期 + 多源更新
class Tracker
{
public:
  explicit Tracker(const FusionParams & p) : p_(p), da_(p) {}

  void Reset();

  // 主入口：预测 → 匹配 → 更新（激光/雷达）→ 新建未匹配测量 → 管理生命周期
  void Update(double dt,
              const lidar_msgs::msg::Objects & lidar,
              const lidar_msgs::msg::Objects & radar,
              double now);

  // 相机单独检测创建的候选轨迹（unconfirmed）
  int AddCandidate(const Eigen::Vector4f & x, double now, float score, int type);

  const std::map<int, FusionTrack> & Tracks() const { return tracks_; }
  std::map<int, FusionTrack> & Tracks() { return tracks_; }

  // 按贡献源位掩码重算置信度（contrib: bit0=激光 bit1=雷达 bit2=相机）
  void RecomputeConfidence(FusionTrack & t);

private:
  void PredictAll(double dt);
  void KalmanPredict(FusionTrack & t, double dt);
  void KalmanUpdateLidar(FusionTrack & t, const lidar_msgs::msg::Object & o);
  void KalmanUpdateRadar(FusionTrack & t, const lidar_msgs::msg::Object & o);
  void Manage(double now);
  void MergeTracks(FusionTrack & keep, const FusionTrack & absorb);   // 重复轨迹合并：保留者吸收被吸收者属性

  const FusionParams p_;
  DataAssociation da_;
  std::map<int, FusionTrack> tracks_;
  int next_id_ = 1;
};

}  // namespace fusion
}  // namespace perception
