#include <rclcpp/rclcpp.hpp>
#include "rfans_driver/srv/rfans_command.hpp"  // 注意 ROS 2 服务是 srv，不是 .h
#include <iostream>
#include <cstdlib>

int main(int argc, char **argv)
{
  if (argc != 3) {
    std::cout << "Usage: rfans_subscriber [cmd] [speed]\n"
              << "     Parameter:\n"
              << "       cmd,   0: stop RFans; 1:start RFans\n"
              << "     speed,   5: 5hz; 10: 10hz; 20: 20hz\n";
    return 1;
  }

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rfans_subscriber");

  auto client = node->create_client<rfans_driver::srv::RfansCommand>("rfans_driver/rfans_control");

  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "服务未启动！");
    rclcpp::shutdown();
    return 1;
  }

  auto request = std::make_shared<rfans_driver::srv::RfansCommand::Request>();
  request->cmd = std::stoll(argv[1]);
  request->speed = std::stoll(argv[2]);

  auto result_future = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result_future) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result_future.get();
    RCLCPP_INFO(node->get_logger(), "Service call successful, status: %ld", (long int)response->status);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed.");
  }

  rclcpp::shutdown();
  return 0;
}
