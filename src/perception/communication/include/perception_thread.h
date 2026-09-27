#pragma once
#include <rclcpp/rclcpp.hpp>

#include "communication/include/rosbridge.h"
#include "architecture/lidar_cells/include/lidar_cell.h"
#include "architecture/lidar_objs/include/grid_cluster.h"
#include "architecture/fusion/include/fusion_node.h"

#include "common/threadpool.h"
#include "common/log.h"

#include <thread>
#include <future>
#include <memory>

namespace perception
{

class PerceptionThread
{
public:
  /** 构造时传入 ROS 2 节点句柄，便于内部创建定时器 / 参数 */
  explicit PerceptionThread(const rclcpp::Node::SharedPtr & node);
  ~PerceptionThread() = default;

  /** 初始化资源（参数读取、模块 init 等）*/
  int Init();

  /**
   * 启动主循环  
   * @param hz 处理频率（Hz）。若为 0，可由外部 Executor 驱动；否则内部用定时器。
   */
  int Start(double hz);

private:
  /* ============ ROS ============ */
  rclcpp::Node::SharedPtr   node_;
  rclcpp::TimerBase::SharedPtr timer_;        //!< 可选：定时触发执行一次

  std::shared_ptr<RosBridge> ros_bridge_;

  /* ============ 感知流水线 ============ */
  std::shared_ptr<lidar_cells::LidarCell>   lidar_cell_;
  std::shared_ptr<lidar_objs::LidarCluster> lidar_cluster_;
  std::shared_ptr<fusion::FusionNode>       fusion_;

  /* ============ 线程池 ============ */
  static constexpr unsigned THREAD_NUM  = 5;
  std::shared_ptr<threadpool> executor_;     //!< 你已有的线程池封装

  /* ============ 运行模式 ============ */
  int  method_ = 1;           //!< 1 = 单目流；2 = 多流…  按原项目逻辑
  std::string log_path_;      //!< 日志目录，由参数读取

  /* ============ 内部执行函数 ============ */
  void runOnce();             //!< 每帧 / 每周期执行一次
  void RunLoop();             //!< 流水线主循环（后台线程执行；回调由 main 线程 spin 独占处理）
};

} // namespace perception
