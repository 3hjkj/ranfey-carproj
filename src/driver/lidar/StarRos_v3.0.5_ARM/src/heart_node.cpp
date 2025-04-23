// #include "ros/ros.h"
// #include "std_msgs/msg/string.hpp"
// #include<rfans_driver/msg/heart_msg.hpp>
// #include"lidar_sdk/CLidarIO.h"
// #include "lidar_sdk/configPara.h"
// #include "lidar_sdk/CLidarDecoder.h"
// #include "lidar_sdk/CLidarCfg.h"
// #include "lidar_sdk/CLidarCalc.h"
// #include"lidar_sdk/sdk_interface.h"
// #include "ioapi.h"
// #include"filter.h"
// #include<thread>

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "heart_node");
//   ros::NodeHandle nh;
//   ros::Publisher chatter_pub = nh.advertise<rfans_driver::msg::HeartMsg>("chatter", 1000);

//   ros::Rate loop_rate(10);
//   HEARTBEAT_S heart_para;
//   rfans_driver::msg::HeartMsg heart_pub;
//   rfans_driver::IOAPI* ctl_socket=new rfans_driver::IOSocketAPI("192.168.0.8", 2030, 2030);
//   while (ros::ok())
//   {
//     if(ctl_socket->read((unsigned char*)&heart_para,sizeof(heart_para))==256)
//     {
//       heart_pub.temper=heart_para.temperature/100.0;
//       heart_pub.device_id=heart_para.device_id;
//     }
//     chatter_pub.publish(heart_pub);
//     ros::spinOnce();
//     loop_rate.sleep();
//   }
// //  while (ros::ok())
// //  {
// //    std_msgs::String msg;
// //    msg.data = "hello world";
// //    chatter_pub.publish(msg);
// //    ros::spinOnce();
// //    loop_rate.sleep();
// //  }

//   return 0;
// }
#include <rclcpp/rclcpp.hpp>
#include <rfans_driver/msg/heart_msg.hpp>
#include "lidar_sdk/CLidarIO.h"
#include "lidar_sdk/configPara.h"
#include "lidar_sdk/CLidarDecoder.h"
#include "lidar_sdk/CLidarCfg.h"
#include "lidar_sdk/CLidarCalc.h"
#include "lidar_sdk/sdk_interface.h"
#include "ioapi.h"
#include <thread>
#include <memory>
#include <string>
#include <chrono>

class HeartNode : public rclcpp::Node
{
public:
  HeartNode()
  : Node("heart_node")
  {
    this->declare_parameter<std::string>("device_ip", "192.168.0.8");
    this->declare_parameter<int>("device_port", 2030);
    this->declare_parameter<int>("local_port", 2030);

    std::string ip = this->get_parameter("device_ip").as_string();
    int dev_port = this->get_parameter("device_port").as_int();
    int local_port = this->get_parameter("local_port").as_int();

    publisher_ = this->create_publisher<rfans_driver::msg::HeartMsg>("chatter", 10);

    ctl_socket_ = std::make_unique<rfans_driver::IOSocketAPI>(
      shared_from_this(), ip, static_cast<uint16_t>(dev_port), static_cast<int16_t>(local_port));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&HeartNode::read_and_publish, this)
    );
  }

private:
  void read_and_publish()
  {
    rfans_driver::msg::HeartMsg msg;
    HEARTBEAT_S heart_para{};
    int len = ctl_socket_->read(reinterpret_cast<unsigned char*>(&heart_para), sizeof(heart_para));

    if (len == static_cast<int>(sizeof(HEARTBEAT_S))) {
      msg.temper = static_cast<float>(heart_para.temperature) / 100.0f;
      msg.device_id = heart_para.device_id;
      publisher_->publish(msg);
    } else {
      RCLCPP_DEBUG(this->get_logger(), "未发现有效数据 (连续 %d bytes)", len);
    }
  }

  std::unique_ptr<rfans_driver::IOSocketAPI> ctl_socket_;
  rclcpp::Publisher<rfans_driver::msg::HeartMsg>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeartNode>());
  rclcpp::shutdown();
  return 0;
}