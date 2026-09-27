#include "architecture/fusion/include/fusion_node.h"
#include <algorithm>
#include <cmath>

namespace perception
{
namespace fusion
{

namespace
{
constexpr int kBitCam = 0x4;   // 贡献源位掩码 bit2=相机
constexpr double kSampleStep = 8;   // 相机单独检测取激光点深度的采样步长
}  // namespace

FusionNode::FusionNode(const rclcpp::Node::SharedPtr & node)
: node_(node), tracker_(p_), objects2cells_(p_), camera_()
{
}

int FusionNode::Init()
{
  const std::string cam_cfg =
    node_->declare_parameter<std::string>("camera_config", "");
  if (!cam_cfg.empty()) {
    camera_valid_ = camera_.LoadConfig(cam_cfg);
    if (camera_valid_)
      RCLCPP_INFO(node_->get_logger(), "fusion: camera config loaded (%s)", cam_cfg.c_str());
    else
      RCLCPP_WARN(node_->get_logger(), "fusion: camera config load FAILED, 相机增强关闭 (%s)",
                  cam_cfg.c_str());
  } else {
    RCLCPP_WARN(node_->get_logger(), "fusion: 未配置 camera_config, 相机增强关闭");
  }
  tracker_.Reset();
  return 0;
}

int FusionNode::Process(MainData & data, double now_ts)
{
  // 0. dt
  double dt = 0.1;
  if (!first_) dt = std::max(0.001, now_ts - last_ts_);
  last_ts_ = now_ts;
  first_ = false;

  const double stale = p_.stale_s;

  // 1. 雷达可用性（SensorData.timestamp → Object.time）
  lidar_msgs::msg::Objects radar_use;
  if (!data.radar_objs.objs.empty()) {
    double latest = 0.0;
    for (const auto & o : data.radar_objs.objs) latest = std::max(latest, o.time);
    if ((now_ts - latest) < stale) radar_use = data.radar_objs;
  }

  // 2. 相机可用性（VisionBoxes.header.stamp）
  bool camera_ok = false;
  if (camera_valid_ && !data.vision_boxes.boxes.empty()) {
    const auto & st = data.vision_boxes.header.stamp;
    const double box_ts = st.sec + st.nanosec * 1e-9;
    camera_ok = (now_ts - box_ts) < stale;
  }

  // 3. 跟踪（激光 + 雷达）
  tracker_.Update(dt, data.lidar_objs, radar_use, now_ts);

  // 4. 相机投影增强：confirmed track 命中 YOLO 框 → 更新类别/置信度/贡献源
  if (camera_ok) {
    auto & tracks = tracker_.Tracks();
    for (auto & [id, t] : tracks) {
      if (!t.alive || !t.confirmed) continue;
      const int bi = camera_.MatchBox(data.vision_boxes, t.x[0], t.x[1], p_.obj_assume_z);
      if (bi >= 0) {
        const auto & box = data.vision_boxes.boxes[bi];
        if (box.score > p_.camera_type_min_score) {
          const int ctype = camera_.CocoToType(box.cls_id);
          if (ctype != 0) t.type = ctype;      // 相机语义类别优先级最高
        }
        t.score = box.score;
        t.contrib |= kBitCam;
        tracker_.RecomputeConfidence(t);
      }
    }

    // 5. 相机单独检测（未关联任何 confirmed track 的框）→ 用激光点给深度建候选
    BuildCameraOnlyTracks(data, now_ts);
  }

  // 6. 输出：全部活 track → fusion_output_objects；confirmed → fusion_cells
  lidar_msgs::msg::Objects out, confirmed;
  for (const auto & [id, t] : tracker_.Tracks()) {
    if (!t.alive) continue;
    lidar_msgs::msg::Object o;
    FillOutputObject(t, o, now_ts);
    out.objs.push_back(o);
    if (t.confirmed) confirmed.objs.push_back(o);
  }
  data.fusion_objs.fusion_output_objects = out;
  data.fusion_objs.fusion_valid = true;
  objects2cells_.Convert(confirmed, now_ts, data.fusion_objs.fusion_cells);
  return 0;
}

void FusionNode::BuildCameraOnlyTracks(const MainData & data, double now_ts)
{
  const auto & boxes = data.vision_boxes;
  if (boxes.boxes.empty()) return;

  // 已被任一存活 track 命中的框跳过（含未确认相机候选，避免同一目标建两条）
  std::vector<bool> box_used(boxes.boxes.size(), false);
  for (const auto & [id, t] : tracker_.Tracks()) {
    if (!t.alive) continue;
    const int bi = camera_.MatchBox(boxes, t.x[0], t.x[1], p_.obj_assume_z);
    if (bi >= 0) box_used[bi] = true;
  }

  // 收集激光点（车辆系）
  std::vector<Eigen::Vector3f> pts;
  for (const auto & [k, cloud] : data.lidar_points_cells.point_cloud_ptr_map) {
    if (!cloud) continue;
    for (const auto & p : cloud->points)
      pts.emplace_back(p.x, p.y, p.z);
  }

  for (int bi = 0; bi < static_cast<int>(boxes.boxes.size()); ++bi) {
    if (box_used[bi]) continue;
    const auto & b = boxes.boxes[bi];
    if (b.score < p_.camera_type_min_score) continue;
    double dx, dy;
    if (camera_.BoxDepthFromPoints(boxes, bi, pts, static_cast<int>(kSampleStep), &dx, &dy)) {
      Eigen::Vector4f x0;
      x0 << dx, dy, 0.f, 0.f;
      const int ctype = camera_.CocoToType(b.cls_id);
      tracker_.AddCandidate(x0, now_ts, b.score, ctype);
    }
  }
}

void FusionNode::FillOutputObject(const FusionTrack & t, lidar_msgs::msg::Object & o,
                                  double now) const
{
  o.idx = t.track_id;
  o.rel_x = t.x[0];  o.rel_y = t.x[1];  o.rel_z = 0.0f;
  o.abs_x = 0.0f;    o.abs_y = 0.0f;    o.abs_z = 0.0f;
  o.rel_vx = t.x[2]; o.rel_vy = t.x[3]; o.rel_vz = 0.0f;
  o.rel_ax = o.rel_ay = o.rel_az = 0.0f;
  o.abs_vx = o.abs_vy = o.abs_vz = 0.0f;
  o.length = t.length;  o.width = t.width;  o.height = t.height;
  o.confidence = t.confidence;
  o.speed = std::hypot(t.x[2], t.x[3]);
  o.rel_heading = 0.0f;  o.abs_heading = 0.0f;
  o.type = t.type;
  o.source = t.source;
  o.time = now;
  o.score = t.score;
  o.used = t.confirmed;
}

}  // namespace fusion
}  // namespace perception
