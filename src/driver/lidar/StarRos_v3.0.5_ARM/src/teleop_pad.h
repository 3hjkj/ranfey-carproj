#ifndef TELEOP_PAD_H
#define TELEOP_PAD_H

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rfans_driver/msg/command.hpp>

class QLineEdit;
class QComboBox;
class QPushButton;

namespace rviz_teleop_commander
{

class TeleopPanel : public rviz_common::Panel
{
Q_OBJECT
public:
  explicit TeleopPanel(QWidget* parent = nullptr);
  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config& config) override;

public Q_SLOTS:
  void setTopic(const QString& topic);

protected Q_SLOTS:
  void sendVel();
  void updateLinearVelocity();
  void updateAngularVelocity();
  void updateTopic();
  void buttonClicked();

protected:
  QLineEdit* output_topic_editor_;
  QString output_topic_;

  QComboBox* scan_speed_;
  QComboBox* return_type_;
  QPushButton* button_ok_;

  QLineEdit* output_topic_editor_1_;
  QString output_topic_1_;
  QLineEdit* output_topic_editor_2_;
  QString output_topic_2_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<rfans_driver::msg::Command>::SharedPtr pub_cmd_;
  rclcpp::Publisher<rfans_driver::msg::Command>::SharedPtr pub_cmd_ns1_;
  rclcpp::Publisher<rfans_driver::msg::Command>::SharedPtr pub_cmd_ns2_;
  rclcpp::Publisher<rfans_driver::msg::Command>::SharedPtr pub_cmd_ns3_;
  rclcpp::Publisher<rfans_driver::msg::Command>::SharedPtr pub_cmd_ns4_;

  rclcpp::Node::SharedPtr raw_node_;

  float linear_velocity_;
  float angular_velocity_;
  std::string model_;
  int rps_;
  bool is_multi_;
};

}  // namespace rviz_teleop_commander

#endif  // TELEOP_PAD_H
