#ifndef RFANS_DRIVER__RFANS_DRIVER_HPP_
#define RFANS_DRIVER__RFANS_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rfans_driver/msg/command.hpp>
#include <fstream>

#include "ioapi.h"
#include "lidar_sdk/sdk_interface.h"
#include "common.h"

// 旧结构体、常量都沿用
struct SdkBuff_S {
  DataBuff_S   rawStream;
  DecStream_S  decStream;
  CalcStream_S calcStream;
  DecStream_S  frame_stream;
};

namespace rfans_driver {

extern std::vector<TransClound_S> buff_original_cloud;   // 仍供别的 C 文件使用

class Rfans_Driver : public rclcpp::Node           // ← 继承 rclcpp::Node
{
public:
  explicit Rfans_Driver(const rclcpp::NodeOptions & opt = rclcpp::NodeOptions());
  ~Rfans_Driver() override;

  /* —— 以下 API / 名字全部保留 —— */
  int  spinOnce();
  int  progSet(lidarAPi::DEB_PROGRM_S &);
  int  dataLevelSet(lidarAPi::DEB_PROGRM_S &);
  rfans_driver::IOAPI* getDevInstance();
  void configDeviceParams();
  int  setSdkPara(bool is_real_time);

private:                       /* 只把 ROS NodeHandle → rclcpp 改动 */
  /* 旧 private 接口不动 ↓↓↓ */
  void setupNodeParams(/*无 ros::NodeHandle*/);
  void InitPointcloud2(sensor_msgs::msg::PointCloud2 &);
  void rosCloundPulish();
  int  realTimeMode();
  int  playBackMode();
  void socketInit();
  void cal2RosClound(LaserPoint_S *, CalcLaserPt_S *);
  void stopDevice();
  void calculation();
  int  cloundTransForm(TransClound_S&, LaserPoint_S*, CalcLaserPt_S*, int);
  int  creatIsf(std::string);
  void sdKobjInit();
  void getTemperFrHeart();
  filterXYZ_S getFilterPara(double, std::vector<crdFilterPara_S>&);

private:
  /* =============== 旧成员变量全保留 =============== */
  InputPara_S m_input_para;
  rfans_driver::IOAPI *m_ctl_socket{nullptr};
  rfans_driver::IOAPI *m_data_socket{nullptr};
  rfans_driver::IOAPI *m_heart_socket{nullptr};

  sensor_msgs::msg::PointCloud2 original_ros_cloud;
  sensor_msgs::msg::PointCloud2 restruct_ros_cloud;
  bool m_is_real_time{true};
  int  m_cur_count{0};

  SDK_OBJ_S   m_sdk_obj{};
  SDK_PARA_S  m_sdk_para{};
  SdkBuff_S   m_sdk_buff{};
  char       *header_buff{nullptr};

  std::vector<std::string> m_file_list;
  std::ofstream            save_xyz_file;
  HEARTBEAT_S              m_heart{};

  std::vector<crdFilterPara_S> m_filter_xyz_vec;
  bool          is_filter_flag{false};
  filterXYZ_S   m_filter{};

  std::ofstream m_isf_fp;
  bool m_is_isf{false};
  int  m_isf_count{0};

  /* —— ROS 2 publisher / subscriber —— */
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Subscription<rfans_driver::msg::Command>::SharedPtr subCommond;
};

}  // namespace rfans_driver
#endif
