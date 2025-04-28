#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "communication/include/perception_thread.h"

int main(int argc, char **argv)
{
    // ── 初始化 ───────────────────────────────────────────────
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("lidar_cell");

    // ── 感知线程对象 ─────────────────────────────────────────
    auto perc_thread = std::make_shared<perception::PerceptionThread>(node); 
    // ↑ 在 PerceptionThread 构造函数里保存 node，为后续发布 / 订阅 / 参数使用

    if (perc_thread->Init() != 0)
    {
        RCLCPP_ERROR(node->get_logger(), "Perception lidar_cell Init() failed!");
        rclcpp::shutdown();
        return -1;
    }
    RCLCPP_INFO(node->get_logger(), "========== Perception lidar_cell inited, Start ==========");

    if (perc_thread->Start(20.0) != 0)
    {
        RCLCPP_ERROR(node->get_logger(), "Perception lidar_cell Start() failed!");
        rclcpp::shutdown();
        return -1;
    }

    // ── ROS 事件循环 ────────────────────────────────────────
    // 如果 PerceptionThread 内部自己跑线程，可以选 spin_some()；这里用 spin() 阻塞
    rclcpp::spin(node);

    // ── 退出清理 ────────────────────────────────────────────
    rclcpp::shutdown();
    return 0;
}
