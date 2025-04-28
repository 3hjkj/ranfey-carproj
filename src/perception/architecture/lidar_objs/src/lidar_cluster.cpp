#include "../include/lidar_cluster.h"
#include <pcl_conversions/pcl_conversions.h>

/* —— 用 RCLCPP 封装旧版日志宏 —— */
#define INFO(fmt, ...)  RCLCPP_INFO (node_->get_logger(), fmt, ##__VA_ARGS__)
#define WARN(fmt, ...)  RCLCPP_WARN (node_->get_logger(), fmt, ##__VA_ARGS__)
#define ERROR(fmt, ...) RCLCPP_ERROR(node_->get_logger(), fmt, ##__VA_ARGS__)

namespace perception
{

/* ───────── 构造 / 析构 ───────── */
PointsCluster::PointsCluster(const rclcpp::Node::SharedPtr& node)
: node_(node)
{
  /* 若需要发布调试点云，可取消以下两行注释
  pub_no_ground_points_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          topic_name1, 10);
  */

  dbscan_          = std::make_shared<DBSCAN>(0.6, 3);
  min_rotate_rect_ = std::make_shared<min_rotate_rect>();
  point_tmp.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

/* 默认析构函数即可 */
PointsCluster::~PointsCluster() = default;

/* ───────── 对外接口：聚类并返回 Objects ───────── */
lidar_msgs::msg::Objects
PointsCluster::Pub(const pcl::PointCloud<pcl::PointXYZ>::Ptr& data_in)
{
  /* 1. PCL → std::vector<points>（与旧版相同） */
  std::vector<points> data = CloudToPoints(data_in);

  /* 2. DBSCAN 聚类 */
  std::vector<std::vector<points>> indices = dbscan_->Clustering(data);

  lidar_msgs::msg::Objects objs;
  if (indices.empty()) return objs;

  /* 3. 每个聚类画最小外接矩形并生成 Object */
  int idx = 0;
  for (const auto& cluster : indices)
  {
    if (cluster.size() < 4) continue;

    std::vector<double> rect = min_rotate_rect_->MinRotateRect(cluster);
    /* 过滤过低目标 */
    if (rect.empty() || rect[4] < 0.2) continue;

    lidar_msgs::msg::Object obj;
    obj.rel_x  = rect[0];
    obj.rel_y  = rect[1];
    obj.rel_z  = rect[4];
    obj.length = rect[2];
    obj.width  = rect[3];
    obj.height = rect[4];
    obj.rel_heading = rect[5] * arc2degree;
    obj.idx = idx++;

    /* 简单类型判定，与旧版一致 */
    if (obj.length < 0.4 && obj.width < 0.4 && obj.height > 0.8)
      obj.type = 3;
    else if (((obj.length < 3 && obj.length > 0.4 && obj.width < 0.4) ||
              (obj.length < 0.4 && obj.width > 0.4 && obj.width < 3)) &&
             obj.height > 0.8)
      obj.type = 4;
    else if (obj.length > 0.4 && obj.width > 0.4 && obj.height > 0.8)
      obj.type = 2;
    else
      obj.type = 0;

    objs.objs.emplace_back(obj);
  }

  INFO("cluster_objs: size={}", objs.objs.size());
  return objs;
}

/* ───────── PCL 点云 → 自定义 points 数组 ───────── */
std::vector<points>
PointsCluster::CloudToPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  std::vector<points> data_out;
  if (cloud->points.empty()) return data_out;

  data_out.reserve(cloud->points.size());
  for (const auto& p : cloud->points)
    data_out.push_back({p.x, p.y, p.z});

  return data_out;
}

} // namespace perception
