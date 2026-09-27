// 雷达适配节点：把雷达目标统一发布到 /sensorRawData (radar_msgs/SensorData)
//
// 两种模式（参数 radar_mode）：
//   - sim（默认）：无真车/无雷达驱动时的模拟目标，保证感知融合与 decision_planning
//     紧急停车逻辑链路不断开。运动目标以 -sim_moving_vx 靠近，静止目标固定不动。
//   - autocontrol：订阅 can_control_msgs/AutocontrolRadardata（rt1~rt6 已解析目标），
//     逐目标转成 SensorData。真雷达驱动若直接发 SensorData 则本节点可整体移除。
//
// 坐标系约定：车辆系，x 前 / y 侧向（米）；vx 纵向速度（远离为正），vy 横向速度。

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "radar_msgs/msg/sensor_data.hpp"
#include "can_control_msgs/msg/autocontrol_radardata.hpp"

namespace radar_adapter
{

class RadarAdapter : public rclcpp::Node
{
public:
  RadarAdapter() : Node("radar_adapter")
  {
    declare_parameter("radar_mode", "sim");
    declare_parameter("radar_topic", "sensorRawData");
    declare_parameter("autocontrol_topic", "/radar_autocontrol");
    declare_parameter("sim_rate_hz", 20.0);
    declare_parameter("sim_moving_x0", 30.0);
    declare_parameter("sim_moving_vx", -5.0);
    declare_parameter("sim_moving_y", -2.0);
    declare_parameter("sim_static_x", 15.0);
    declare_parameter("sim_static_y", 3.0);

    const std::string mode = get_parameter("radar_mode").as_string();
    const std::string topic = get_parameter("radar_topic").as_string();
    pub_ = create_publisher<radar_msgs::msg::SensorData>(topic, 10);

    if (mode == "sim") {
      sim_rate_ = get_parameter("sim_rate_hz").as_double();
      moving_x_ = get_parameter("sim_moving_x0").as_double();
      moving_vx_ = get_parameter("sim_moving_vx").as_double();
      moving_y_ = get_parameter("sim_moving_y").as_double();
      static_x_ = get_parameter("sim_static_x").as_double();
      static_y_ = get_parameter("sim_static_y").as_double();

      const int period_ms = static_cast<int>(1000.0 / sim_rate_);
      sim_timer_ = create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&RadarAdapter::simTick, this));
      RCLCPP_INFO(get_logger(), "radar_adapter sim mode @ %.1fHz -> %s",
                  sim_rate_, topic.c_str());
    } else if (mode == "autocontrol") {
      const std::string ac_topic = get_parameter("autocontrol_topic").as_string();
      sub_ = create_subscription<can_control_msgs::msg::AutocontrolRadardata>(
        ac_topic, 10,
        std::bind(&RadarAdapter::autoCb, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "radar_adapter autocontrol mode, subscribe %s -> %s",
                  ac_topic.c_str(), topic.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "unknown radar_mode=%s (sim|autocontrol)", mode.c_str());
    }
  }

private:
  void simTick()
  {
    const double now = this->now().seconds();

    // 运动目标（靠近中）
    radar_msgs::msg::SensorData m;
    m.sensor_type = 1;
    m.timestamp = now;
    m.obj_id = 1;
    m.x = static_cast<float>(moving_x_);
    m.y = static_cast<float>(moving_y_);
    m.vx = static_cast<float>(moving_vx_);
    m.vy = 0.0f;
    pub_->publish(m);

    // 静止目标
    radar_msgs::msg::SensorData s;
    s.sensor_type = 1;
    s.timestamp = now;
    s.obj_id = 2;
    s.x = static_cast<float>(static_x_);
    s.y = static_cast<float>(static_y_);
    s.vx = 0.0f;
    s.vy = 0.0f;
    pub_->publish(s);

    moving_x_ += moving_vx_ / sim_rate_;
    if (moving_x_ < 4.0) {
      // 与 sim_sensors 激光幕墙夹紧值一致：到达后目标物理停住，雷达速度同步归零。
      // 若继续报 vx=-5，激光(静止)与雷达(运动)互相矛盾，滤波会在错误速度上折中。
      moving_x_ = 4.0;
      moving_vx_ = 0.0;
    }
  }

  // rt1~rt6 -> SensorData（l_long_obj > 0.01 视为有效目标）
  void autoCb(const can_control_msgs::msg::AutocontrolRadardata::ConstSharedPtr & msg)
  {
    const double now = this->now().seconds();
#define PUB_RT(n)                                                       \
    do {                                                                \
      const float l = static_cast<float>(msg->rt##n##_l_long_obj);      \
      if (l > 0.01f) {                                                  \
        radar_msgs::msg::SensorData d;                                  \
        d.sensor_type = 1;                                              \
        d.timestamp = now;                                              \
        d.obj_id = msg->rt##n##_track_id;                               \
        d.x = l;                                                        \
        d.y = static_cast<float>(msg->rt##n##_l_lat_obj);               \
        d.vx = static_cast<float>(msg->rt##n##_v_long_obj);             \
        d.vy = static_cast<float>(msg->rt##n##_v_lat_obj);              \
        pub_->publish(d);                                               \
      }                                                                 \
    } while (0)

    PUB_RT(1);
    PUB_RT(2);
    PUB_RT(3);
    PUB_RT(4);
    PUB_RT(5);
    PUB_RT(6);
#undef PUB_RT
  }

  std::string mode_;
  rclcpp::Publisher<radar_msgs::msg::SensorData>::SharedPtr pub_;
  rclcpp::Subscription<can_control_msgs::msg::AutocontrolRadardata>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr sim_timer_;
  double sim_rate_{20.0};
  double moving_x_{30.0}, moving_vx_{-5.0}, moving_y_{-2.0};
  double static_x_{15.0}, static_y_{3.0};
};

}  // namespace radar_adapter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<radar_adapter::RadarAdapter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
