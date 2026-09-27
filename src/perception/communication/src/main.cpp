#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <pcl/console/print.h>
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
    // 回调由本线程 spin() 独占处理（感知线程只跑流水线，不再 spin_some）。
    // PCL 转换缺字段警告（angle/ring/timestamp）每秒刷屏数百条，压沉执行器，置 L_ERROR 静音。
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    rclcpp::spin(node);

    // ── 退出清理 ────────────────────────────────────────────
    rclcpp::shutdown();
    return 0;
}
