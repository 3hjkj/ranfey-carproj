#include "teleop_pad.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QComboBox>  
#include <QPushButton>
#include <QVBoxLayout>

#include <rviz_common/display_context.hpp>      // ★ 新增
#include <rviz_common/ros_integration/ros_node_abstraction.hpp> // ★ 新增
#include <geometry_msgs/msg/twist.hpp>
#include <rfans_driver/msg/command.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace rviz_teleop_commander
{

/* ---------- 构造函数 ---------- */
TeleopPanel::TeleopPanel(QWidget *parent)
: rviz_common::Panel(parent)
{
  /* 1) 取得 rviz2 自带的 rclcpp::Node */
  node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  /* 2) 读取/声明参数（保持同名） */
  node_->declare_parameter("model", std::string("R-Fans-32"));
  node_->declare_parameter("mult_lidar", false);
  node_->declare_parameter("rfans_driver.rps", 10);

  node_->get_parameter("model", model);
  node_->get_parameter("mult_lidar", Is_multi);
  node_->get_parameter("rfans_driver.rps", rps);

  /* 3) 创建话题发布器 */
  subComannd      = node_->create_publisher<rfans_driver::msg::Command>("contrlComand",        1);
  subComannd_ns1  = node_->create_publisher<rfans_driver::msg::Command>("/ns1/contrlComand",   1);
  subComannd_ns2  = node_->create_publisher<rfans_driver::msg::Command>("/ns2/contrlComand",   1);
  subComannd_ns3  = node_->create_publisher<rfans_driver::msg::Command>("/ns3/contrlComand",   1);
  subComannd_ns4  = node_->create_publisher<rfans_driver::msg::Command>("/ns4/contrlComand",   1);

  /* 4) Qt UI —— 与 ROS1 版基本一致 */
  QVBoxLayout *topic_layout = new QVBoxLayout;

  topic_layout->addWidget(new QLabel("Scan Speed:"));
  scan_speed = new QComboBox;
  scan_speed->clear();

  if (model == "R-Fans-32" || model == "R-Fans-16") {
    scan_speed->addItems({"5", "10", "20"});
    scan_speed->setCurrentIndex(Is_multi ? 1 : (rps == 5 ? 0 : rps == 10 ? 1 : 2));
  } else if (model == "C-Fans-128" || model == "C-Fans-32" ||
             model == "C-Fans-256" || model == "CK-128") {
    scan_speed->addItems({"10", "20", "40", "60"});
    scan_speed->setCurrentIndex(Is_multi ? 0 :
                                (rps == 10 ? 0 : rps == 20 ? 1 : rps == 40 ? 2 : 3));
  } else {
    RCLCPP_WARN(node_->get_logger(), "launch model error");
  }
  topic_layout->addWidget(scan_speed);

  topic_layout->addWidget(new QLabel("Return Type:"));
  return_type = new QComboBox;
  return_type->addItems({tr("Strongest return"), tr("Dual return")});
  bool double_echo = false;
  node_->declare_parameter("rfans_driver.use_double_echo", false);
  node_->get_parameter("rfans_driver.use_double_echo", double_echo);
  return_type->setCurrentIndex(double_echo ? 1 : 0);
  topic_layout->addWidget(return_type);

  button_ok = new QPushButton("OK");
  topic_layout->addWidget(button_ok);

  QHBoxLayout *layout = new QHBoxLayout;
  layout->addLayout(topic_layout);
  setLayout(layout);

  /* 5) Qt 信号槽 */
  connect(button_ok, &QPushButton::clicked, this, &TeleopPanel::button_clicked);
}

/* ---------- 按钮槽：发送指令 ---------- */
void TeleopPanel::button_clicked()
{
  rfans_driver::msg::Command cmd_msg;
  cmd_msg.cmd = 1;
  cmd_msg.speed = scan_speed->currentText().toInt();
  cmd_msg.use_double_echo = (return_type->currentIndex() != 0);

  if (Is_multi) {
    RCLCPP_INFO(node_->get_logger(), "multi_lidar");
    subComannd_ns1->publish(cmd_msg);
    subComannd_ns2->publish(cmd_msg);
    subComannd_ns3->publish(cmd_msg);
    subComannd_ns4->publish(cmd_msg);
  } else {
    RCLCPP_INFO(node_->get_logger(), "single_lidar");
    subComannd->publish(cmd_msg);
  }
}

/* ---------- 以下函数与 ROS1 版保持一致，仅替换消息类型 ---------- */
void TeleopPanel::update_Linear_Velocity()
{
  linear_velocity_ = output_topic_editor_1->text().toFloat();
}
void TeleopPanel::update_Angular_Velocity()
{
  angular_velocity_ = output_topic_editor_2->text().toFloat();
}
void TeleopPanel::updateTopic()
{
  setTopic(output_topic_editor_->text());
}
void TeleopPanel::setTopic(const QString &new_topic)
{
  if (new_topic == output_topic_) return;
  output_topic_ = new_topic;

  if (output_topic_.isEmpty()) {
    velocity_publisher_.reset();
  } else {
    velocity_publisher_ =
      node_->create_publisher<geometry_msgs::msg::Twist>(output_topic_.toStdString(), 1);
  }
  Q_EMIT configChanged();
}
void TeleopPanel::sendVel()
{
  if (!velocity_publisher_) return;

  geometry_msgs::msg::Twist msg;
  msg.linear.x  = linear_velocity_;
  msg.angular.z = angular_velocity_;
  velocity_publisher_->publish(msg);
}
void TeleopPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Topic", output_topic_);
}
void TeleopPanel::load(const rviz_common::Config &config)
{
  rviz_common::Panel::load(config);
  QString topic;
  if (config.mapGetString("Topic", &topic)) {
    output_topic_editor_->setText(topic);
    updateTopic();
  }
}

}  // namespace rviz_teleop_commander

/* ---------- 插件导出宏 ---------- */
PLUGINLIB_EXPORT_CLASS(rviz_teleop_commander::TeleopPanel, rviz_common::Panel)
