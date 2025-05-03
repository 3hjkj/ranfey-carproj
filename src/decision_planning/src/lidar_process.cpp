#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "lidar_msgs/msg/objects.hpp"

using std::placeholders::_1;   // 占位符，便于绑定回调

class LidarProcessNode : public rclcpp::Node
{
public:
  LidarProcessNode() : Node("lidar_process_node")
  {
    /*
     * QoS 这里用 10 条历史缓存，等价于 ROS1 中 queue_size=10。
     * 如果话题频率较高或需要可靠传输，可改成 rclcpp::SensorDataQoS()
     */
    sub_ = this->create_subscription<lidar_msgs::msg::Objects>(
      "/perception/lidar_objs",
      10,
      std::bind(&LidarProcessNode::lidarCallback, this, _1));
  }

private:
  // === 回调函数 ===
  void lidarCallback(const lidar_msgs::msg::Objects::ConstSharedPtr msg) const
  {
    /* 下面演示打印 objects 数量。若要遍历，可根据自定义消息的字段修改
       例如：for (const auto & obj : msg->objects) { … }             */
    RCLCPP_INFO(this->get_logger(), "接收到 %zu 个物体", msg->cells.size());
  }

  rclcpp::Subscription<lidar_msgs::msg::Objects>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);                    // 初始化 RCL
  rclcpp::spin(std::make_shared<LidarProcessNode>());  // 进入事件循环
  rclcpp::shutdown();
  return 0;
}
