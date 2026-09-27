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
  // 必须在 make_shared 之后调用：socketInit() 内部要用 shared_from_this()，
  // 构造函数还没返回时对象尚未被 shared_ptr 接管，会抛 std::bad_weak_ptr。
  void initDevice();
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
  void dumpFrame();          // 整帧落盘，必须在 queue_mutex 之外调用
  int  netInit();            // 建 UDP 转发 socket，失败返回 -1（不影响落盘）
  void netFrame();           // 整帧 UDP 转发，同样必须在 queue_mutex 之外调用
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
  /* —— 落盘缓冲 ——
   * 整帧攒在 m_frame_buf 里，帧末一次性 write，不在逐点循环里做任何 I/O。
   * 逐点写盘（无论 operator<< 还是 snprintf）会让消费线程持 queue_mutex 很久，
   * 解码线程被挡在锁外无法排空 socket，内核丢包，每圈只剩 ~75% 的点。
   * 详见 dumpFrame() 的注释。 */
  std::vector<char>        m_frame_buf;

  /* 二进制落盘：把 TransClound_S 数组原样整块写出，每点 40 字节
   * （#pragma pack(1)），不做任何格式化，可直接跑满速。
   * 文本模式和它差一个数量级，文本转换请挪到宿主机（原生 x86）再做。 */
  std::ofstream            save_bin_file;
  bool                     m_save_bin{false};
  std::string              m_bin_path;

  /* —— UDP 转发（只为绕开 qemu，车机上不需要）——
   * 容器里跑的是 aarch64 二进制、经 qemu-user 模拟，DDS 的组播发现用不了，
   * 外部节点发现不了本节点发布的 /rfans_points。于是把每帧点云在帧末锁外
   * 原样 UDP 单播丢给宿主机（容器是 --network host，共享网络栈），由宿主
   * 原生的 x86-64 节点重新 publish，rviz2 就能看了。
   *
   * **车机上把 net_forward 关掉即可** —— 那里厂商库原生匹配、没有 qemu，
   * 节点直接 publish 就能被发现，多一跳转发反而是累赘。同一份源码靠这个
   * 开关切换，详见 netFrame() 的注释。 */
  bool                     m_net_forward{false};
  std::string              m_net_host{"127.0.0.1"};
  int                      m_net_port{7500};
  int                      m_net_fd{-1};
  uint32_t                 m_net_frame_id{0};
  uint64_t                 m_net_dropped{0};   // 发送缓冲满时被丢掉的包数
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
