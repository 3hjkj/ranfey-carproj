#pragma once
#include <vector>
#include <Eigen/Dense>
#include <lidar_msgs/msg/object.hpp>
#include <lidar_msgs/msg/objects.hpp>
#include "fusion_params.h"

namespace perception
{
namespace fusion
{

// 单条融合轨迹（跨帧持久化，状态在车辆系：x 前 / y 侧）
struct FusionTrack
{
  int    track_id = 0;
  bool   confirmed = false;
  int    hits = 0, misses = 0, predict_cnt = 0;
  double last_seen = 0.0;                  // 最近一次更新时间戳（s）
  bool   alive = true;
  int    merge_cnt = 0;                    // 与其它轨迹持续同位帧数（用于重复轨迹合并）

  Eigen::Vector4f x = Eigen::Vector4f::Zero();  // [x, y, vx, vy]
  Eigen::Matrix4f P = Eigen::Matrix4f::Identity() * 4.0;  // 初始协方差大

  // 属性（尺寸/类别/来源/置信度）
  float length = 0, width = 0, height = 0;
  int   type = 0;                          // lidar_msgs::Object type
  int   source = 0;                        // lidar_msgs::Object source（取最高优先级源）
  int   contrib = 0;                       // 贡献源位掩码 bit0=激光 bit1=雷达 bit2=相机
  float confidence = 0, score = 0;
};

// 匹配结果：track 索引 ↔ 测量索引（-1 = 未匹配）
struct MatchResult
{
  std::vector<int> track2lidar;            // size == tracks.size()
  std::vector<int> track2radar;
  std::vector<int> lidar_unmatched;
  std::vector<int> radar_unmatched;
};

class DataAssociation
{
public:
  explicit DataAssociation(const FusionParams & p) : p_(p) {}

  // 门限 + 贪心匹配。tracks 须为预测后状态；dt 为当前帧距上帧时长。
  MatchResult Match(const std::vector<FusionTrack> & tracks,
                    const lidar_msgs::msg::Objects & lidar,
                    const lidar_msgs::msg::Objects & radar,
                    double dt) const;

private:
  // 代价：门限内返回距离，否则 +inf
  double LidarCost(const FusionTrack & t, const lidar_msgs::msg::Object & o, double dt) const;
  double RadarCost(const FusionTrack & t, const lidar_msgs::msg::Object & o) const;

  const FusionParams p_;
};

}  // namespace fusion
}  // namespace perception
