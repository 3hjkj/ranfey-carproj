#include "communication/include/perception_thread.h"


namespace perception
{

/* ───────────────────────────────────────────────────────────────
 *  构造：读参数 / 创建子模块 / 线程池 / 日志
 * ───────────────────────────────────────────────────────────── */
PerceptionThread::PerceptionThread(const rclcpp::Node::SharedPtr& node)
: node_(node)
{
  /* 1. 读取参数（保持与旧 launch 参数名一致） */
  node_->declare_parameter<int>("method", 1);
  node_->declare_parameter<std::string>("log_path", "/tmp/perception.log");
  node_->get_parameter("method",    method_);
  node_->get_parameter("log_path",  log_path_);

  std::cout << "log_path: " << log_path_ << std::endl;

  /* 2. 若旧日志存在则删除（与原逻辑一致） */
  if (access(log_path_.c_str(), 0) != -1)
    ::remove(log_path_.c_str());

  /* 3. 初始化 spdlog  */
  spdlog::cfg::load_env_levels();
  spdlog::rotating_logger_mt(SPDLOG_NAME, log_path_, 5 * 1024 * 1024, 1);

  /* 4. 创建各功能模块 */
  ros_bridge_    = std::make_shared<RosBridge>(node_);
  lidar_cell_    = std::make_shared<lidar_cells::LidarCell>(node_);
  lidar_cluster_ = std::make_shared<lidar_objs::LidarCluster>(node_);
  executor_      = std::make_shared<threadpool>(THREAD_NUM);
}

/* ─────────────────────── Init：各子模块初始化 ─────────────────────── */
int PerceptionThread::Init()
{
  ros_bridge_->Publish();         // 初始化时可先发布一次空结果
  lidar_cell_->Init();
  lidar_cluster_->Init();
  return 0;
}

/* ─────────────────────── Start：主循环 ───────────────────────────── */
int PerceptionThread::Start(double /*hz*/)
{
  DataPool* DP = DataPool::Instance();
  INFO("~~~~~~~~~~ perception start ~~~~~~~~~~~~~~~");

  rclcpp::Rate rate(10.0);                       // == ros::Rate(10)
  unsigned int a = 0;

  while (rclcpp::ok())
  {
    if (++a > 10)
    {
      const double ts_start = node_->get_clock()->now().seconds();

      /* ----- 1. 线程任务封装 ----- */
      auto lidar_cell_job = [&]() {
        return lidar_cell_->Process(DP->main_data_.lidar_points_cells,
                                    DP->main_data_.lidar_cells);
      };
      auto lidar_obj_job  = [&]() {
        return lidar_cluster_->Process(DP->main_data_.lidar_points_objs,
                                       DP->main_data_.lidar_objs);
      };

      /* ----- 2. 提交到线程池 ----- */
      std::future<int> fut_cell;
      std::future<int> fut_obj;
      if (executor_->idlCount() > 0)
      {
        fut_cell = executor_->commit(lidar_cell_job);
        fut_obj  = executor_->commit(lidar_obj_job);
      }
      else
        WARN("no idle thread!!!");

      /* ----- 3. 等待结果、打印耗时 ----- */
      const int cell_ret = fut_cell.get();
      INFO("lidar_cell ret={}", cell_ret);
      const double ts_mid = node_->get_clock()->now().seconds();
      INFO("lidar cell time = {} ms", (ts_mid - ts_start) * 1000.0);

      const int obj_ret  = fut_obj.get();
      INFO("lidar_obj  ret={}", obj_ret);
      INFO("lidar objs time = {} ms",
           (node_->get_clock()->now().seconds() - ts_start) * 1000.0);

      /* ----- 4. Publish 并清理缓存 ----- */
      ros_bridge_->Publish();
      DP->main_data_.lidar_objs.objs.clear();
      DP->main_data_.lidar_cells.cells.clear();
      DP->main_data_.fusion_objs.fusion_output_objects.objs.clear();
      DP->main_data_.lidar_points_objs.reset();
      DP->main_data_.lidar_points_cells.reset();
    }

    /* spin_some 等价于原来的 ros::spinOnce() */
    rclcpp::spin_some(node_);
    rate.sleep();
  }

  spdlog::shutdown();
  return 0;
}

} // namespace perception
