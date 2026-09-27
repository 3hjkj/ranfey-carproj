#pragma once
/* ─── ROS 2 头文件 ─────────────────────────────────────────── */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

/* ─── PCL & STL 头文件（与原相同） ─────────────────────────── */
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.hpp>          // ROS 2 的 pcl_ros
#include <math.h>
#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <iomanip>
#include "common/log.h"

#include "lidar_msgs/msg/object.hpp"
#include "lidar_msgs/msg/objects.hpp"
#include "min_rotate_rect.h"
#include "lidar_obj_type.h"
#include "wall_extract.h"

namespace perception
{

class PointsCluster
{
public:
  /* 构造函数需传入节点句柄；析构保持默认 */
  explicit PointsCluster(const rclcpp::Node::SharedPtr& node);

  /* 注入聚类门槛。构造函数只拿得到 node，拿不到 objs.json 解析出来的配置，
     所以由 LidarCluster::Init() 读完配置后再调一次。
     不调也能跑 —— 成员有与旧字面量一致的默认值。 */
  void Configure(const lidar_objs::objs_config& cfg);

  /* 与旧版一致的外部接口：聚类并返回 Objects 消息 */
  lidar_msgs::msg::Objects Pub(const pcl::PointCloud<pcl::PointXYZ>::Ptr& data_in);

private:
  /* ============ ROS 成员 ============ */
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_no_ground_points_;

  /* 逐点聚类标签（评估用）：把 DBSCAN 的**输入点云**连同每个点的簇号发出来。
     没有它就算不了 ARI / AMI / 均一性 / 完整性 / V-measure / 轮廓系数 ——
     /perception/lidar_objs 里只有矩形框，点与簇的对应关系在那一步已经丢了。
     布局 x,y,z(float32) + label(int32)，point_step=16，label=-1 表示噪点或
     被过滤掉、没成为目标的点。 */
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_labels_;
  std::string topic_labels = "/perception/lidar_cluster_labels";
  void PublishLabels(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                     const std::vector<int32_t>& labels);

  /* ============ 业务成员（保持旧名） ============ */
  const bool debug = true;
  int  bfs_counter  = 0;
  std::string topic_name1 = "/perception/no_ground_points";

  const double degree2arc = 3.1415926 / 180.0;
  const double arc2degree = 180.0 / 3.1415926;

  pcl::PointCloud<pcl::PointXYZ>::Ptr point_tmp;
  std::shared_ptr<DBSCAN>           dbscan_;
  std::shared_ptr<min_rotate_rect>  min_rotate_rect_;

  /* ============ 聚类门槛（默认值与旧字面量一致，见 Configure） ============ */
  double dbscan_eps_         = 0.6;
  int    dbscan_min_pts_     = 3;
  int    min_cluster_points_ = 4;
  double min_obj_height_     = 0.2;
  int    min_pts_near_       = 12;
  double min_pts_rmax_       = 20.0;
  double wall_size_m_        = 6.0;
  double split_eps_          = 0.3;
  double split_size_m_       = 6.0;

  /* 摘墙。判据与实测数据见 wall_extract.h。seg_eps 不在这里给死值 ——
     它在 Configure 里按 cluster_voxel_size * wall_seg_eps_ratio 算出来，
     因为实测「eps 恰好等于体素格距时连通性在浮点边界上断掉」，
     这个门槛和体素是绑死的。 */
  WallConfig wall_cfg_;

  /* 成为目标所需的最少点数，随距离从 min_pts_near_ 线性降到
     min_cluster_points_（r >= min_pts_rmax_ 之后就用远场值）。 */
  int MinPointsAt(double r) const;

  /* 把一个簇落成一个 Object：过门槛、算最小外接矩形、填字段、判型，
     并把该簇各点的标签写成 obj.idx。src_idx 是簇内各点在**原始**点云 data
     里的下标（可能为空 —— 拿不到映射时不写标签，框照出）。
     只有真的产出目标时才 ++idx，所以 idx 与标签始终一一对应。

     force_wall=true 用于摘墙模块分出来的墙片：跳过类型判定直接给 type=5。
     那一片已经被更强的判据（连通片点数 / z 跨度 / XY 薄度）判成墙了，而下面
     那一串只看尺寸的分支会把窄墙、或被切过的墙片判成车。 */
  void EmitObject(const std::vector<points>& cluster,
                  const std::vector<int>& src_idx,
                  int& idx,
                  std::vector<int32_t>& labels,
                  lidar_msgs::msg::Objects& objs,
                  bool force_wall = false);

  int counter_txt = 0;

  /* 与旧版一致的内部工具函数 */
  std::vector<points> CloudToPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& msg);
};

}  // namespace perception
