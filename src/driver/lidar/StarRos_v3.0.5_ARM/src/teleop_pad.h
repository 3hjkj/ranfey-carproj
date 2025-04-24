#ifndef TELEOP_PAD_H
#define TELEOP_PAD_H
#include <geometry_msgs/msg/twist.hpp>          // ★ 新增
#include <rfans_driver/msg/command.hpp>         // ★ 新增
#include <rviz_common/display_context.hpp>      // ★ 新增
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

#ifndef Q_MOC_RUN
#  include <rviz_common/panel.hpp>             // rviz2
#  include <rclcpp/rclcpp.hpp>                 // ROS 2 节点
#endif

// Qt 前向声明
class QLineEdit;
class QComboBox;
class QPushButton;

namespace rviz_teleop_commander
{

class TeleopPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit TeleopPanel(QWidget *parent = nullptr);

  /** rviz2 保存 / 载入配置 */
  void load(const rviz_common::Config &config) override;
  void save(rviz_common::Config config) const override;

public Q_SLOTS:
  void setTopic(const QString &topic);

protected Q_SLOTS:
  void sendVel();
  void update_Linear_Velocity();
  void update_Angular_Velocity();
  void updateTopic();
  void button_clicked();

protected:                       // Qt 控件
  QLineEdit  *output_topic_editor_;
  QString     output_topic_;
  QComboBox  *scan_speed;
  QComboBox  *return_type;
  QPushButton* button_ok;
  QLineEdit  *output_topic_editor_1;
  QString     output_topic_1;
  QLineEdit  *output_topic_editor_2;
  QString     output_topic_2;

protected:                       // ROS 2
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;

  rclcpp::Publisher<rfans_driver::msg::Command>::SharedPtr subComannd;
  rclcpp::Publisher<rfans_driver::msg::Command>::SharedPtr subComannd_ns1;
  rclcpp::Publisher<rfans_driver::msg::Command>::SharedPtr subComannd_ns2;
  rclcpp::Publisher<rfans_driver::msg::Command>::SharedPtr subComannd_ns3;
  rclcpp::Publisher<rfans_driver::msg::Command>::SharedPtr subComannd_ns4;

  float linear_velocity_{0.0};
  float angular_velocity_{0.0};

  std::string model;
  bool  Is_multi{false};
  int   rps{0};
};

}  // namespace rviz_teleop_commander

#endif  // TELEOP_PAD_H
