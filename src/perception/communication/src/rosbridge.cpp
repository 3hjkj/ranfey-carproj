#include "communication/include/rosbridge.h"

#include <pcl_conversions/pcl_conversions.h>   // for pcl::fromROSMsg



namespace perception
{

/* ============= 构造函数：完成参数读取、订阅、发布器创建 ============= */
RosBridge::RosBridge(const rclcpp::Node::SharedPtr& node)
: node_(node)
{
  /* ---- 1. 读取 5 路 LiDAR 话题参数 ---- */
  for (int i = 0; i < 5; ++i)
  {
    std::string key = "lidar_topic_name" + std::to_string(i);
    /* 保持与 ROS 1 launch 中相同的缺省值 */
    node_->declare_parameter<std::string>(key, "lidar_point" + std::to_string(i));
    node_->get_parameter(key, lidar_topic_names_[i]);
  }

  /* ---- 2. 创建订阅者（Objs / Cells 各 5 路）---- */
  for (int i = 0; i < 5; ++i)
  {
    /* ↓↓↓  objs  ↓↓↓ */
    sub_objs_[i] = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_names_[i], 10,
      [this,i](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
      { cbObj(msg, i); });

    /* ↓↓↓  cells  ↓↓↓ */
    sub_cells_[i] = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_names_[i], 10,
      [this,i](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
      { cbCell(msg, i); });
  }

  /* ---- 3. 发布器 ---- */
  pub_cells_  = node_->create_publisher<lidar_msgs::msg::Cells>  ("/perception/lidar_cells", 10);
  pub_objs_   = node_->create_publisher<lidar_msgs::msg::Objects>("/perception/lidar_objs", 10);
  pub_fusion_ = node_->create_publisher<lidar_msgs::msg::Objects>("/perception_objs",        10);

  /* ---- 4. 定位订阅 ---- */
  node_->create_subscription<localization_msgs::msg::Localization>(
    "/localization", 10,
    std::bind(&RosBridge::cbLocalization, this, std::placeholders::_1));

  /* ---- 5. 融合使能与毫米波/相机源订阅 ---- */
  node_->declare_parameter<bool>("fusion_enable", false);
  node_->get_parameter("fusion_enable", fusion_enable_);
  node_->declare_parameter<std::string>("radar_topic", "sensorRawData");
  node_->get_parameter("radar_topic", radar_topic_);
  node_->declare_parameter<std::string>("yolo_topic", "/perception/yolo_boxes");
  node_->get_parameter("yolo_topic", yolo_topic_);

  // 队列深度开大：单节点执行器拥塞（回调实际延迟 ~0.3s）下，depth=10 会在
  // 20Hz×2 条/s 的雷达消息下持续丢包（obj_id 偶发缺失）。depth=100 覆盖 2.5s 数据量。
  sub_radar_ = node_->create_subscription<radar_msgs::msg::SensorData>(
    radar_topic_, 100, std::bind(&RosBridge::cbRadar, this, std::placeholders::_1));
  sub_yolo_ = node_->create_subscription<lidar_msgs::msg::VisionBoxes>(
    yolo_topic_, 20, std::bind(&RosBridge::cbYolo, this, std::placeholders::_1));

  if (fusion_enable_)
    INFO("RosBridge: fusion_enable=true, radar=%s yolo=%s",
         radar_topic_.c_str(), yolo_topic_.c_str());
}

/*----------------------------------------  OBJ 回调 5 路共用 ---------------------------------------*/
void RosBridge::cbObj(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg, int idx)
{
  if (msg->data.empty())
  {
    ERROR("lidar %d no obj data !!", idx);
    return;
  }

  auto* DP = DataPool::Instance();
  pcl_util::PointCloudPtr points(new pcl_util::PointCloud);
  pcl::fromROSMsg(*msg, *points);

  switch (idx)
  {
    case 0: DP->SetLidarObjPointsOne  (points); break;
    case 1: DP->SetLidarObjPointsTwo  (points); break;
    case 2: DP->SetLidarObjPointsThree(points); break;
    case 3: DP->SetLidarObjPointsFour (points); break;
    case 4: DP->SetLidarObjPointsFive (points); break;
  }
  INFO("get lidar obj %d points: %zu", idx, msg->data.size());
}

/*----------------------------------------  CELLS 回调 5 路共用 -------------------------------------*/
void RosBridge::cbCell(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg, int idx)
{
  if (msg->data.empty()) return;

  auto* DP = DataPool::Instance();
  pcl_util::PointCloudPtr points(new pcl_util::PointCloud);
  pcl::fromROSMsg(*msg, *points);

  switch (idx)
  {
    case 0: DP->SetLidarCellPointsOne  (points); break;
    case 1: DP->SetLidarCellPointsTwo  (points); break;
    case 2: DP->SetLidarCellPointsThree(points); break;
    case 3: DP->SetLidarCellPointsFour (points); break;
    case 4: DP->SetLidarCellPointsFive (points); break;
  }
}

/*------------------------------------------  定位回调 ---------------------------------------------*/
void RosBridge::cbLocalization(const localization_msgs::msg::Localization::ConstSharedPtr & msg)
{
  auto* DP = DataPool::Instance();
  double now = node_->get_clock()->now().seconds();
  if ((now - msg->system_time) * 1000.0 > 50.0) return;   // 超时

  DP->main_data_.loc.lat  = msg->gps.lat;
  DP->main_data_.loc.lon  = msg->gps.lon;
  DP->main_data_.loc.time = msg->system_time;
}

/*------------------------------------------  毫米波雷达回调 ----------------------------------------*/
void RosBridge::cbRadar(const radar_msgs::msg::SensorData::ConstSharedPtr & msg)
{
  auto* DP = DataPool::Instance();
  lidar_msgs::msg::Object o;
  o.idx = msg->obj_id;
  o.rel_x = msg->x;   o.rel_y = msg->y;
  o.rel_vx = msg->vx; o.rel_vy = msg->vy;
  o.type = 0;                         // 雷达无类别
  o.source = 2;                       // radar
  o.time = msg->timestamp;
  o.used = true;

  // 同 obj_id 覆盖（保留各目标最新一帧），等待 stale 判定
  auto & objs = DP->main_data_.radar_objs.objs;
  for (auto & e : objs) {
    if (e.idx == o.idx) { e = o; return; }
  }
  objs.push_back(o);
}

/*------------------------------------------  YOLO 检测框回调 ----------------------------------------*/
void RosBridge::cbYolo(const lidar_msgs::msg::VisionBoxes::ConstSharedPtr & msg)
{
  auto* DP = DataPool::Instance();
  DP->main_data_.vision_boxes = *msg;   // 整体替换为最新一帧
}

/*------------------------------------------  发布接口 ---------------------------------------------*/
int RosBridge::Publish()
{
  auto* DP = DataPool::Instance();
  try
  {
    if (fusion_enable_)
    {
      /* 融合开启：/perception/lidar_cells 发融合栅格（空也发 = 无障碍清空） */
      pub_cells_->publish(DP->main_data_.fusion_objs.fusion_cells);
      INFO("~~~~pub fusion cells~~~~  size=%zu valid=%d",
           DP->main_data_.fusion_objs.fusion_cells.cells.size(),
           DP->main_data_.fusion_objs.fusion_valid);
    }
    else if (!DP->main_data_.lidar_cells.cells.empty())
    {
      pub_cells_->publish(DP->main_data_.lidar_cells);
      INFO("~~~~pub lidar cells success~~~~  size=%zu",
           DP->main_data_.lidar_cells.cells.size());
    }
    else
      WARN("~~~~pub lidar cells defeat~~~~");

    if (!DP->main_data_.lidar_objs.objs.empty())
    {
      pub_objs_->publish(DP->main_data_.lidar_objs);
      INFO("~~~~pub lidar objs success~~~~  size=%zu",
           DP->main_data_.lidar_objs.objs.size());
    }
    else
      WARN("~~~~pub lidar objs defeat~~~~");

    /* 融合目标（调试/下游扩展用） */
    if (!DP->main_data_.fusion_objs.fusion_output_objects.objs.empty())
      pub_fusion_->publish(DP->main_data_.fusion_objs.fusion_output_objects);
  }
  catch (const std::exception& e)
  {
    ERROR("publish error: %s", e.what());
  }
  return 0;
}

} // namespace perception
