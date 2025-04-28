#include "communication/include/rosbridge.hpp"

#include <pcl_conversions/pcl_conversions.h>   // for pcl::fromROSMsg

/*-----------------------------------------------------------------------------
 *  helper: 统一打印宏（LOG 同级）
 *---------------------------------------------------------------------------*/
#define INFO(fmt, ...)  RCLCPP_INFO (node_->get_logger(), fmt, ##__VA_ARGS__)
#define WARN(fmt, ...)  RCLCPP_WARN (node_->get_logger(), fmt, ##__VA_ARGS__)
#define ERROR(fmt, ...) RCLCPP_ERROR(node_->get_logger(), fmt, ##__VA_ARGS__)

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

/*------------------------------------------  发布接口 ---------------------------------------------*/
int RosBridge::Publish()
{
  auto* DP = DataPool::Instance();
  try
  {
    if (!DP->main_data_.lidar_cells.cells.empty())
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
