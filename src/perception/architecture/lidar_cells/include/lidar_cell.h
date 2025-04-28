#ifndef LIDAR_CELLS_H
#define LIDAR_CELLS_H

#include <rclcpp/rclcpp.hpp>                         // ← 新增
#include <sensor_msgs/msg/point_cloud2.hpp>          // ← 发布 PointCloud2
#include "common/data_pool.h"
#include "lidar_cell_type.h"
#include "common/log.h"
#include <math.h>
#include "architecture/lidar_objs/include/lidar_preprocess.h"
#include "json/include/json.h"

namespace perception::lidar_cells
{

class LidarCell
{
private:
  /* ---------- 与算法相关的私有函数，保持不变 ---------- */
  int  Grid(int x, int y, float pz, float px, float py);
  int  GridCell(const pcl::PointCloud<pcl::PointXYZ>::Ptr & data_in);
  int  PushCell(lidar_msgs::msg::Cells & lidar_cells);
  int  PointsCa(int i, int j);
  bool ReadCellConfig(const std::map<std::string, std::string> & m, cell_config & conf);
  bool ReadCellConfigJson(const std::string & path, cell_config & conf);

  /* ---------- ROS 成员（已替换为 rclcpp） ---------- */
  rclcpp::Node::SharedPtr node_;          //!< 节点句柄（代替 ros::NodeHandle）
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pub_no_ground_points;             //!< 发布器改为智能指针

  /* ---------- 业务数据成员：完全保持旧名 ---------- */
  std::vector<std::vector<data_cell>> data_cell_tmp;
  std::vector<std::vector<data_cell>> data_cell_;
  std::shared_ptr<lidar_objs::LidarPreprocess> lidar_preprocess_;
  perception::ReadConfigCommon  readconfig_;
  cell_config cell_config_;

  float  heading_angle   = 0.0;
  float  threshold_cell_z = 0.3;
  int    row   = 0;
  int    col   = 0;
  int    idx   = 0;
  int    counter = 0;
  bool   debug   = false;

public:
  /* ---------- 构造 / 析构 / 对外接口 ---------- */
  explicit LidarCell(const rclcpp::Node::SharedPtr & node);
  ~LidarCell() = default;

  int Init();   //!< 读取参数 + 初始化网格等
  int Process(const LidarDataInType & lidar_points,
              lidar_msgs::msg::Cells & lidar_cells);
};

}   // namespace perception::lidar_cells

#endif  // LIDAR_CELLS_H
