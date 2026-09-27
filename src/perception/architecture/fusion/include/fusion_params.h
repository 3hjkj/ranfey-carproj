#pragma once
// 融合可调参数集中定义。默认值经感知验证后可在 launch 中覆盖。
namespace perception
{
namespace fusion
{

struct FusionParams
{
  // ---- 时间同步 ----
  double sync_tol_s   = 0.10;   // 各源最近帧时间容差（s）
  double stale_s      = 0.50;   // 源超过该时长判定缺失（跳过该源，不删 track）。
  //                          // 注意：perception 单节点执行器拥塞导致雷达/yolo 回调实际延迟
  //                          // ~0.2s（main rclcpp::spin 与 perception_thread spin_some 双执行器争抢），
  //                          // 0.30 会把新鲜源误判为 STALE；0.5 仍能可靠探测数据源真正断流。

  // ---- 数据关联 ----
  double gate_xy_m    = 1.5;    // 位置门（米）
  double gate_v_scale = 0.8;    // 位置门随速度外推：gate = max(gate_xy, 0.8 + |v|*dt)
  double mahalanobis_gate = 9.49;  // chi2(4, 0.95)，雷达含速度维

  // ---- 跟踪生命周期 ----
  int    confirm_hits  = 3;     // 连续命中次数达标才 confirmed（才进 Cells）
  int    miss_threshold = 10;   // 连续丢失次数删除（≈1s @10Hz）
  int    predict_max   = 5;     // 连续纯预测帧数超过则降级/删除

  // ---- 轨迹间合并（去重）----
  // 启动时未匹配测量各自建轨，同一目标可能同时有激光轨 + 雷达轨。
  // 持续同位（位置差 < merge_gate）merge_sustain_frames 帧 → 合并为一条，保留信息更丰富者。
  double merge_gate         = 1.5;   // 同位判定门限（米）
  int    merge_sustain_frames = 3;   // 持续同位帧数（0.3s@10Hz）

  // ---- 卡尔曼 ----
  double q_v           = 2.0;   // 速度过程噪声 (m/s^2)^2
  double r_lidar_xy    = 0.04;  // 激光位置方差 (m^2)
  double r_radar_xy    = 0.25;  // 雷达位置方差 (m^2)
  double r_radar_v     = 1.0;   // 雷达速度方差 (m^2/s^2)

  // ---- 相机投影 ----
  double obj_assume_z  = 1.6;   // 3D 目标投影到图像用的高度中点（m）
  double camera_type_min_score = 0.5;  // YOLO 置信度高于此值才覆盖类别

  // ---- 置信度加权（w 权重 / c 单源置信度）----
  double w_lidar = 1.0, c_lidar = 0.9;
  double w_radar = 0.8, c_radar = 0.8;
  double w_camera = 0.4;

  // ---- Cells 输出 ----
  double cell_inflate = 0.3;    // 目标占地框膨胀（m）
  double cell_conf    = 30.0;   // 融合占用格置信度（强占用，>=5 即发布）
};

}  // namespace fusion
}  // namespace perception
