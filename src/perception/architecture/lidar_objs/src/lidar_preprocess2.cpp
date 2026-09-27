#include "architecture/lidar_objs/include/lidar_preprocess2.h"
#include <pcl_conversions/pcl_conversions.h>          // 调试发布

namespace perception
{
    namespace lidar_objs
    {
        LidarPreprocess2::LidarPreprocess2(const rclcpp::Node::SharedPtr& node)
        : node_(node) {}       

        LidarPreprocess2::~LidarPreprocess2()
        {
        }
        int LidarPreprocess2::Init()
        {
        INFO("LidarPreprocess2 Init");

        /* 调试发布器 */
        if (!node_->has_parameter("debug_pre"))
            node_->declare_parameter<bool>("debug_pre", false);
        node_->get_parameter("debug_pre", debug_);
        if (debug_) {
            pub_lidar_voxel      = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/lidar_voxel",       10);
            pub_lidar_radius     = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/lidar_radius",      10);
            pub_ground_points    = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/ground_points",     10);
            pub_no_ground_points = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/no_ground_points_", 10);
        }

        /* 读取参数 */
        std::string cfg_path;
        if (!node_->has_parameter("preprocess_config"))
            node_->declare_parameter<std::string>(
                "preprocess_config", "config_json/preprocess.json");
        node_->get_parameter("preprocess_config", cfg_path);
        ReadCellConfigJson(cfg_path, proprecess_config_);

        row = static_cast<int>((proprecess_config_.xmax - proprecess_config_.xmin) /
                                proprecess_config_.cell_size_x);
        col = static_cast<int>((proprecess_config_.ymax - proprecess_config_.ymin) /
                                proprecess_config_.cell_size_y);
        return 0;
        }

        void LidarPreprocess2::Configure(const objs_config &cfg)
        {
            objs_config_ = cfg;
            /* 只读 ground_* / pmf_* / cluster_voxel_size / ror_stage1_min_neighbors。
               不要在这里读 xmin/xmax/car_x_xxx/cell_size_xxx——ROI 与车体框的门限走
               proprecess_config_（preprocess.json），见成员声明处的注释。 */
            INFO("preprocess cfg: ground_method={} ground_voxel={} cluster_voxel={} "
                 "pmf(cell={} win={} slope={} init={} max={} base={} exp={}) "
                 "min_ground_ratio={} ror_stage1={}",
                 objs_config_.ground_method, objs_config_.ground_voxel_size,
                 objs_config_.cluster_voxel_size, objs_config_.pmf_cell_size,
                 objs_config_.pmf_max_window, objs_config_.pmf_slope,
                 objs_config_.pmf_initial_dist, objs_config_.pmf_max_dist,
                 objs_config_.pmf_base, objs_config_.pmf_exponential,
                 objs_config_.pmf_min_ground_ratio, objs_config_.ror_stage1_min_neighbors);
        }

        // int LidarPreprocess2::VoxelFilter(const LidarDataInType &lidar_points, pcl::PointCloud<pcl::PointXYZI>::Ptr &data_out,
        //                                  float &voxel_size)
        // {
        //     pcl::PointCloud<pcl::PointXYZI>::Ptr clouds(new pcl::PointCloud<pcl::PointXYZI>);
        //     // 1.1 jump dont need points
        //     for (auto &pair : lidar_points.point_cloud_ptr_map)
        //     {
        //         if (!pair.second->empty())
        //         {
        //             for (auto &p : pair.second->points)
        //             {
        //                 if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z))
        //                 {
        //                     continue; //Jump those points.
        //                 }

        //                 if (p.x < proprecess_config_.xmin || p.x > proprecess_config_.xmax || p.y > proprecess_config_.ymax || p.y < proprecess_config_.ymin || p.z < proprecess_config_.zmin || p.z > proprecess_config_.zmax)
        //                 {
        //                     continue;
        //                 }
        //                 if (p.x > proprecess_config_.car_xmin && p.x < proprecess_config_.car_xmax && p.y > proprecess_config_.car_ymin && p.y < proprecess_config_.car_ymax)
        //                 {
        //                     continue;
        //                 }
        //                 pcl::PointXYZI p_xyz;
        //                 p_xyz.x = p.x;
        //                 p_xyz.y = p.y;
        //                 p_xyz.z = p.z;
        //                 p_xyz.intensity = p.intensity;
        //                 clouds->points.emplace_back(p_xyz);
        //             }
        //         }
        //     }
        //     // LOG(INFO) << "clouds->points:" << clouds->points.size();
        //     // 1.2 voxel filter
        //     static pcl::VoxelGrid<pcl::PointXYZI> sor;
        //     sor.setLeafSize(voxel_size, voxel_size, voxel_size);
        //     sor.setInputCloud(clouds);
        //     sor.filter(*data_out);
        //     // LOG(INFO) << "filtered points:" << data_out->points.size();
        //     if (debug_)
        //     {
        //         sensor_msgs::msg::PointCloud2 output; //声明的输出的点云的格式
        //         pcl::toROSMsg(*data_out, output);
        //         output.header.frame_id = "world";
        //         pub_lidar_voxel->publish(output);
        //     }
        //     return 0;
        // }
        int LidarPreprocess2::VoxelFilter(const LidarDataInType &lidar_points,
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr &data_out, float &voxel_size)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr clouds(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr clouds2(new pcl::PointCloud<pcl::PointXYZ>);
            // 1.1 jump dont need points
            for (auto &pair : lidar_points.point_cloud_ptr_map)
            {
                if (!pair.second->empty())
                {
                    for (auto &p : pair.second->points)
                    {
                        if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z))
                        {
                            continue; //Jump those points.
                        }

                        if (p.x < proprecess_config_.xmin || p.x > proprecess_config_.xmax || p.y > proprecess_config_.ymax || p.y < proprecess_config_.ymin || p.z < proprecess_config_.zmin || p.z > proprecess_config_.zmax)
                        {
                            continue;
                        }
                        if (p.x > proprecess_config_.car_xmin && p.x < proprecess_config_.car_xmax && p.y > proprecess_config_.car_ymin && p.y < proprecess_config_.car_ymax)
                        {
                            continue;
                        }
                        pcl::PointXYZ p_xyz;
                        p_xyz.x = p.x;
                        p_xyz.y = p.y;
                        p_xyz.z = p.z;
                        try
                        {
                            clouds->points.emplace_back(p_xyz);
                        }
                        catch (std::exception &e)
                        {
                            // LOG(ERROR) << " lidar preprocess:" << e.what();
                        }
                    }
                }
            }
            // 1.2 voxel filter
            try
            {
                std::vector<int> mapping;
                // pcl::PointCloud<pcl::PointXYZ>::Ptr data_in_tmp(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::removeNaNFromPointCloud(*clouds, *clouds, mapping);
                pcl::VoxelGrid<pcl::PointXYZ> sor;
                sor.setLeafSize(voxel_size, voxel_size, voxel_size);
                sor.setInputCloud(clouds);
                sor.filter(*clouds2);
                data_out = clouds2;
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
            }

            // LOG(INFO) << "filtered points:" << data_out->points.size();

            // debug
            if (debug_)
            {
                sensor_msgs::msg::PointCloud2 output; //声明的输出的点云的格式
                pcl::toROSMsg(*data_out, output);
                output.header.frame_id = "world";
                pub_lidar_voxel->publish(output);
            }

            return 0;
        }
        int LidarPreprocess2::VoxelFilter(const pcl_util::PointCloudPtr &lidar_points,
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr &data_out, float &voxel_size)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr clouds(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr clouds2(new pcl::PointCloud<pcl::PointXYZ>);
            // 1.1 jump dont need points
            for (auto &p : lidar_points->points)
            {
                if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z))
                {
                    continue; //Jump those points.
                }

                if (p.x < proprecess_config_.xmin || p.x > proprecess_config_.xmax || p.y > proprecess_config_.ymax || p.y < proprecess_config_.ymin || p.z < proprecess_config_.zmin || p.z > proprecess_config_.zmax)
                {
                    continue;
                }
                if (p.x > proprecess_config_.car_xmin && p.x < proprecess_config_.car_xmax && p.y > proprecess_config_.car_ymin && p.y < proprecess_config_.car_ymax)
                {
                    continue;
                }
                // std::cout << std::setprecision(20) << "time:" << p.timestamp << "\n";
                pcl::PointXYZ p_xyz;
                p_xyz.x = p.x;
                p_xyz.y = p.y;
                p_xyz.z = p.z;
                try
                {
                    clouds->points.emplace_back(p_xyz);
                    // clouds.points.emplace_back(p_xyz);
                }
                catch (std::exception &e)
                {
                    // LOG(ERROR) << " lidar preprocess:" << e.what();
                }
            }
            // 1.2 voxel filter
            // LOG(INFO) << "voxel_size:" << voxel_size;
            // LOG(INFO) << "clouds:" << clouds->points.size();
            try
            {
                // std::vector<int> mapping;
                pcl::PointCloud<pcl::PointXYZ> clouds3;
                pcl::PointCloud<pcl::PointXYZ>::Ptr data_in_tmp(new pcl::PointCloud<pcl::PointXYZ>);
                // pcl::removeNaNFromPointCloud(*data_in, *data_in_tmp, mapping);
                DeleteNanPoints(clouds, data_in_tmp);
                static pcl::VoxelGrid<pcl::PointXYZ> sor;
                sor.setLeafSize(voxel_size, voxel_size, voxel_size);
                sor.setInputCloud(data_in_tmp);
                // DeleteNanPoints(clouds2, clouds3);
                /* 这行原来是 std::cout，10 Hz 下会把控制台刷爆。改成限流日志。 */
                RCLCPP_DEBUG_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                    "voxel(lidar map): in=" << data_in_tmp->points.size());
                sor.filter(clouds3);
                data_out = clouds3.makeShared();
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
            }

            // LOG(INFO) << "filtered points:" << data_out->points.size();
            // debug
            if (debug_)
            {
                sensor_msgs::msg::PointCloud2 output; //声明的输出的点云的格式
                pcl::toROSMsg(*data_out, output);
                output.header.frame_id = "world";
                pub_lidar_voxel->publish(output);
            }

            return 0;
        }
        int LidarPreprocess2::VoxelFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &lidar_points,
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr &data_out,
                                          float &voxel_size)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr clouds(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ> clouds2;
            // 1.1 jump dont need points
            for (auto &p : lidar_points->points)
            {
                if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z))
                {
                    continue; //Jump those points.
                }

                if (p.x < proprecess_config_.xmin || p.x > proprecess_config_.xmax || p.y > proprecess_config_.ymax || p.y < proprecess_config_.ymin || p.z < proprecess_config_.zmin || p.z > proprecess_config_.zmax)
                {
                    continue;
                }
                if (p.x > proprecess_config_.car_xmin && p.x < proprecess_config_.car_xmax && p.y > proprecess_config_.car_ymin && p.y < proprecess_config_.car_ymax)
                {
                    continue;
                }
                pcl::PointXYZ p_xyz;
                p_xyz.x = p.x;
                p_xyz.y = p.y;
                p_xyz.z = p.z;
                try
                {
                    clouds->points.emplace_back(p_xyz);
                    // clouds.points.emplace_back(p_xyz);
                }
                catch (std::exception &e)
                {
                    // LOG(ERROR) << " lidar preprocess:" << e.what();
                }
            }
            // 1.2 voxel filter
            // LOG(INFO) << "voxel_size:" << voxel_size;
            // LOG(INFO) << "clouds:" << clouds->points.size();
            RCLCPP_DEBUG_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "voxel(cloud): in=" << clouds->points.size());
            static pcl::VoxelGrid<pcl::PointXYZ> sor;
            sor.setLeafSize(voxel_size, voxel_size, voxel_size);
            sor.setInputCloud(clouds);
            sor.filter(clouds2);
            data_out = clouds2.makeShared();
            // sor.filter(*clouds2);
            // data_out = clouds2;
            // LOG(INFO) << "filtered points:" << data_out->points.size();
            // debug
            if (debug_)
            {
                sensor_msgs::msg::PointCloud2 output; //声明的输出的点云的格式
                pcl::toROSMsg(*data_out, output);
                output.header.frame_id = "world";
                pub_lidar_voxel->publish(output);
            }
            return 0;
        }
        int LidarPreprocess2::GroundPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &data_in,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_poins,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_points,
                                           float &threshold_h,
                                           int &points_num)
        {
            // 1.grid
            grid_preprocess = std::vector<std::vector<grid_obj>>(row, std::vector<grid_obj>(col));
            // 2.得到每个栅格的最低值
            try
            {
                GetGridZmin(data_in, grid_preprocess, points_num);
            }
            catch (const std::exception &e)
            {
                // LOG(ERROR) << e.what() << '\n';
            }

            // 3.根据最低值进行过滤地面点
            // LOG(INFO) << "groud points data in size :" << data_in->points.size();
            GetGroundPoints(data_in, grid_preprocess, ground_poins, no_ground_points, threshold_h);
            // LOG(INFO) << "no_ground_points size :" << no_ground_points->points.size();
            if (debug_)
            {
                sensor_msgs::msg::PointCloud2 output_groud_points;    //声明的输出的点云的格式
                sensor_msgs::msg::PointCloud2 output_no_groud_points; //声明的输出的点云的格式
                pcl::toROSMsg(*ground_poins, output_groud_points);
                pcl::toROSMsg(*no_ground_points, output_no_groud_points);
                output_groud_points.header.frame_id = "world";
                output_no_groud_points.header.frame_id = "world";
                pub_ground_points->publish(output_groud_points);
                pub_no_ground_points->publish(output_no_groud_points);
            }
            return 0;
        }
        int LidarPreprocess2::GroundSegmentationPMF(const pcl::PointCloud<pcl::PointXYZ>::Ptr &data_in,
                                                    pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_points,
                                                    pcl::PointCloud<pcl::PointXYZ>::Ptr &no_ground_points)
        {
            /* 两堆都要先清空：调用方复用同一批 Ptr，残留会把上一帧的点混进来。 */
            ground_points->points.clear();
            no_ground_points->points.clear();
            ground_points->width = ground_points->height = 0;
            no_ground_points->width = no_ground_points->height = 0;

            if (!data_in || data_in->points.empty())
            {
                return 1;
            }

            /* 去 NaN。PMF 内部用点坐标建 XY 包围盒再做 getMinMax3D，NaN 会污染结果。 */
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            DeleteNanPoints(data_in, cloud);
            if (cloud->points.empty())
            {
                return 1;
            }

            /* PMF 至少要能撑起一次开运算，点太少直接判失败——比返回一堆
               垃圾分类强，调用方（grid_cluster.cpp）会据此跳过这一帧。 */
            if (cloud->points.size() < 4)
            {
                return 1;
            }

            pcl::PointIndices ground_idx;
            try
            {
                pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
                pmf.setInputCloud(cloud);
                /* max_window 单位是米（PCL 内部把窗口按 cell_size 乘出来再和它比），
                   虽然 setter 收 int。窗口是**软上限**：循环条件 window < max_window，
                   所以 cell 0.5 / max_window 6 实际跑 [1.5, 2.5, 4.5, 8.5] 四轮，
                   最后一轮超出配置值。 */
                pmf.setMaxWindowSize(objs_config_.pmf_max_window);
                pmf.setSlope(objs_config_.pmf_slope);
                pmf.setInitialDistance(objs_config_.pmf_initial_dist);
                pmf.setMaxDistance(objs_config_.pmf_max_dist);
                pmf.setCellSize(objs_config_.pmf_cell_size);
                pmf.setBase(objs_config_.pmf_base);
                pmf.setExponential(objs_config_.pmf_exponential);
                pmf.extract(ground_idx.indices);
            }
            catch (const std::exception &e)
            {
                /* initCompute() 失败时 extract() 提前返回**空**地面点集。
                   这里必须显式判失败，否则下面会把整个输入（含地板）当非地面
                   喂给 DBSCAN —— 那正是这一轮要修的失败模式。 */
                ERROR("pmf extract failed: %s", e.what());
                return 1;
            }

            if (ground_idx.indices.empty())
            {
                ERROR("pmf returned empty ground (in=%zu)", cloud->points.size());
                return 1;
            }

            /* 按位图分堆，保证 ground ∪ no_ground == 输入且互斥。旧法用两个
               独立的 push 分支，容易出现某个点两边都没进（被静默吞掉）。 */
            std::vector<char> is_ground(cloud->points.size(), 0);
            for (int i : ground_idx.indices)
            {
                if (i >= 0 && static_cast<size_t>(i) < is_ground.size())
                    is_ground[i] = 1;
            }
            for (size_t i = 0; i < cloud->points.size(); ++i)
            {
                if (is_ground[i])
                    ground_points->points.emplace_back(cloud->points[i]);
                else
                    no_ground_points->points.emplace_back(cloud->points[i]);
            }
            ground_points->width    = ground_points->points.size();
            ground_points->height   = 1;
            no_ground_points->width = no_ground_points->points.size();
            no_ground_points->height = 1;

            if (debug_)
            {
                sensor_msgs::msg::PointCloud2 output_groud_points;
                sensor_msgs::msg::PointCloud2 output_no_groud_points;
                pcl::toROSMsg(*ground_points, output_groud_points);
                pcl::toROSMsg(*no_ground_points, output_no_groud_points);
                output_groud_points.header.frame_id = "world";
                output_no_groud_points.header.frame_id = "world";
                pub_ground_points->publish(output_groud_points);
                pub_no_ground_points->publish(output_no_groud_points);
            }

            RCLCPP_DEBUG_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "pmf ground: in=" << cloud->points.size()
                << " ground=" << ground_points->points.size()
                << " (" << std::fixed << std::setprecision(1)
                << (100.0 * ground_points->points.size() / cloud->points.size()) << "%)");
            return 0;
        }
        int LidarPreprocess2::GetGridZmin(const pcl::PointCloud<pcl::PointXYZ>::Ptr &data_in,
                                          std::vector<std::vector<grid_obj>> &grid_preprocess,
                                          int &points_num)
        {
            if (data_in == nullptr)
            {
                return 1;
            }
            if (data_in->empty())
            {
                // LOG(ERROR) << "no lidar data grond";
                return 1;
            }
            for (auto &p : data_in->points)
            {
                // jump these points
                if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z))
                {
                    continue;
                }
                if ((p.x < proprecess_config_.xmin || p.x > proprecess_config_.xmax ||
                     p.y < proprecess_config_.ymin || p.y > proprecess_config_.ymax ||
                     p.z < proprecess_config_.zmin || p.z > proprecess_config_.zmax) ||
                    (p.x < proprecess_config_.car_xmax && p.x > proprecess_config_.car_xmin &&
                     p.y < proprecess_config_.car_ymax && p.y > proprecess_config_.car_ymin))
                {
                    continue;
                }
                int x = int(std::floor(p.x - proprecess_config_.xmin) / proprecess_config_.cell_size_x);
                int y = int(std::floor(p.y - proprecess_config_.ymin) / proprecess_config_.cell_size_y);
                if (x > row - 1 || y > col - 1 || x < 0 || y < 0)
                {
                    continue;
                }
                // start jisuan
                try
                {
                    Grid(x, y, p.z, points_num);
                }
                catch (std::exception &e)
                {
                    // LOG(ERROR) << e.what();
                }
            }
            return 0;
        }
        int LidarPreprocess2::Grid(int x, int y, float z, int &points_num)
        {
            grid_preprocess[x][y].x = x * proprecess_config_.cell_size_x + proprecess_config_.xmin + proprecess_config_.cell_size_x / 2;
            grid_preprocess[x][y].y = y * proprecess_config_.cell_size_y + proprecess_config_.ymin + proprecess_config_.cell_size_y / 2;
            grid_preprocess[x][y].idx = x * row + y;
            if (grid_preprocess[x][y].point_num > 0)
            {
                grid_preprocess[x][y].zmax = z > grid_preprocess[x][y].zmax ? z : grid_preprocess[x][y].zmax;
                grid_preprocess[x][y].zmin = z < grid_preprocess[x][y].zmin ? z : grid_preprocess[x][y].zmin;
            }
            else
            {
                grid_preprocess[x][y].zmax = z;
                grid_preprocess[x][y].zmin = z;
            }
            grid_preprocess[x][y].point_num++;

            if (grid_preprocess[x][y].point_num > points_num) // 最小有效点云数
            {
                grid_preprocess[x][y].vaild = 1;
            }
            return 0;
        }
        int LidarPreprocess2::GetGroundPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &data_in,
                                              std::vector<std::vector<grid_obj>> &grid_preprocess,
                                              pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_points,
                                              pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_points,
                                              float &threshold_h)
        {
            for (auto &p : data_in->points)
            {
                // jump the points
                if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z))
                {
                    continue;
                }
                if ((p.x < proprecess_config_.xmin || p.x > proprecess_config_.xmax ||
                     p.y < proprecess_config_.ymin || p.y > proprecess_config_.ymax ||
                     p.z < proprecess_config_.zmin || p.z > proprecess_config_.zmax) ||
                    (p.x < proprecess_config_.car_xmax && p.x > proprecess_config_.car_xmin &&
                     p.y < proprecess_config_.car_ymax && p.y > proprecess_config_.car_ymin))
                {
                    continue;
                }
                int x = int(std::floor(p.x - proprecess_config_.xmin) / proprecess_config_.cell_size_x);
                int y = int(std::floor(p.y - proprecess_config_.ymin) / proprecess_config_.cell_size_y);
                if (x > row - 1 || y > col - 1 || x < 0 || y < 0)
                {
                    continue;
                }
                if (grid_preprocess[x][y].vaild == 1)
                {
                    if (p.z - grid_preprocess[x][y].zmin > threshold_h)
                    {
                        pcl::PointXYZ p_xyz;
                        p_xyz.x = p.x;
                        p_xyz.y = p.y;
                        p_xyz.z = p.z;
                        no_ground_points->points.emplace_back(p_xyz);
                    }
                    else
                    {
                        pcl::PointXYZ p_xyz;
                        p_xyz.x = p.x;
                        p_xyz.y = p.y;
                        p_xyz.z = p.z;
                        ground_points->points.emplace_back(p_xyz);
                    }
                }
                else
                {
                    pcl::PointXYZ p_xyz;
                    p_xyz.x = p.x;
                    p_xyz.y = p.y;
                    p_xyz.z = p.z;
                    ground_points->points.emplace_back(p_xyz);
                }
            }

            return 0;
        }
        int LidarPreprocess2::RadiusFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &data_in,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr &data_out,
                                           float &radius_search,
                                           int &search_num)
        {
            data_out->points.clear();
            data_out->width = data_out->height = 0;

            /* 极小云陷阱：RadiusOutlierRemoval 对每个点做 k 近邻查询，点数不足
               search_num 时 k != mean_k，于是**每个点都被删**——3 个点的云配
               search_num=5 会被清成空。这里直接原样返回，别让滤波器吃掉它。 */
            if (!data_in || static_cast<int>(data_in->points.size()) <= search_num)
            {
                if (data_in)
                    for (const auto &p : data_in->points)
                        if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                            data_out->points.emplace_back(p);
                data_out->width  = data_out->points.size();
                data_out->height = 1;
                RCLCPP_DEBUG_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                    "radius filter skipped (too few points): " << (data_in ? data_in->points.size() : 0));
                return 0;
            }

            try
            {

                // std::vector<int> mapping;
                pcl::PointCloud<pcl::PointXYZ>::Ptr data_in_tmp(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ> data_out_tmp;
                // pcl::removeNaNFromPointCloud(*data_in, *data_in_tmp, mapping);
                DeleteNanPoints(data_in, data_in_tmp);
                pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
                outrem.setInputCloud(data_in_tmp);
                outrem.setRadiusSearch(radius_search);      // 搜索半径
                /* ⚠ PCL 把**查询点自己**算进邻居数（radius_outlier_removal.hpp:142），
                   所以 search_num=N 的真实语义是"半径内至少 N 个**别的**点"，
                   不是 N-1。实测本场景 19228 个原始点里 0.5 m 内少于 1 个邻居的
                   有 0 个——N=1 时这个滤波器可证明是空操作。 */
                outrem.setMinNeighborsInRadius(search_num); // 搜索 最少点数(含自己)
                // apply filter
                //
                outrem.filter(data_out_tmp);
                data_out = data_out_tmp.makeShared();
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
            }

            // debug
            if (debug_)
            {
                sensor_msgs::msg::PointCloud2 output; //声明的输出的点云的格式
                pcl::toROSMsg(*data_out, output);
                // pcl_conversions::fromPCL(*data_out, output);
                output.header.frame_id = "world";
                pub_lidar_radius->publish(output);
            }
            return 0;
        }
        int LidarPreprocess2::RadiusFilter(const LidarDataInType &lidar_points,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr &data_out,
                                           float &radius_search,
                                           int &search_num)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr clouds(new pcl::PointCloud<pcl::PointXYZ>);
            // 1.1 jump dont need points
            for (auto &pair : lidar_points.point_cloud_ptr_map)
            {
                if (!pair.second->empty())
                {
                    for (auto &p : pair.second->points)
                    {
                        if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z))
                        {
                            continue; //Jump those points.
                        }

                        if (p.x < proprecess_config_.xmin || p.x > proprecess_config_.xmax || p.y > proprecess_config_.ymax || p.y < proprecess_config_.ymin || p.z < proprecess_config_.zmin || p.z > proprecess_config_.zmax)
                        {
                            continue;
                        }
                        if (p.x > proprecess_config_.car_xmin && p.x < proprecess_config_.car_xmax && p.y > proprecess_config_.car_ymin && p.y < proprecess_config_.car_ymax)
                        {
                            continue;
                        }
                        pcl::PointXYZ p_xyz;
                        p_xyz.x = p.x;
                        p_xyz.y = p.y;
                        p_xyz.z = p.z;
                        try
                        {
                            clouds->points.emplace_back(p_xyz);
                            // clouds.points.emplace_back(p_xyz);
                        }
                        catch (std::exception &e)
                        {
                            // LOG(ERROR) << " lidar preprocess:" << e.what();
                        }
                    }
                }
            }
            // 1.2 Radius filter
            pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
            outrem.setInputCloud(clouds);
            outrem.setRadiusSearch(radius_search);      // 搜索半径
            outrem.setMinNeighborsInRadius(search_num); // 搜索 最少点数
            // apply filter
            outrem.filter(*data_out);
            // debug
            if (debug_)
            {
                sensor_msgs::msg::PointCloud2 output; //声明的输出的点云的格式
                pcl::toROSMsg(*data_out, output);
                output.header.frame_id = "world";
                pub_lidar_radius->publish(output);
            }

            return 0;
        }
        bool LidarPreprocess2::ReadProConfig(const std::map<string, string> &m, proprecess_config &conf)
        {
            map<string, string>::const_iterator mite = m.begin();
            for (; mite != m.end(); ++mite)
            {
                if (mite->first == "zmax")
                {
                    conf.zmax = std::atof(mite->second.c_str());
                }
                else if (mite->first == "zmin")
                {
                    conf.zmin = std::atof(mite->second.c_str());
                }
                else if (mite->first == "xmin")
                {
                    conf.xmin = std::atof(mite->second.c_str());
                }
                else if (mite->first == "xmax")
                {
                    conf.xmax = std::atof(mite->second.c_str());
                }
                else if (mite->first == "ymin")
                {
                    conf.ymin = std::atof(mite->second.c_str());
                }
                else if (mite->first == "ymax")
                {
                    conf.ymax = std::atof(mite->second.c_str());
                }
                else if (mite->first == "car_xmin")
                {
                    conf.car_xmin = std::atof(mite->second.c_str());
                }
                else if (mite->first == "car_xmax")
                {
                    conf.car_xmax = std::atof(mite->second.c_str());
                }
                else if (mite->first == "car_ymin")
                {
                    conf.car_ymin = std::atof(mite->second.c_str());
                }
                else if (mite->first == "car_ymax")
                {
                    conf.car_ymax = std::atof(mite->second.c_str());
                }
                else if (mite->first == "cell_size_x")
                {
                    conf.cell_size_x = std::atof(mite->second.c_str());
                }
                else if (mite->first == "cell_size_y")
                {
                    conf.cell_size_y = std::atof(mite->second.c_str());
                }
                else if (mite->first == "vaild")
                {
                    conf.vaild = std::atoi(mite->second.c_str());
                }
                else if (mite->first == "voxel_size")
                {
                    conf.voxel_size = std::atof(mite->second.c_str());
                }
                else if (mite->first == "points_num")
                {
                    conf.points_num = std::atoi(mite->second.c_str());
                }
                else if (mite->first == "radius_search")
                {
                    conf.radius_search = std::atof(mite->second.c_str());
                }
                else if (mite->first == "search_num")
                {
                    conf.search_num = std::atoi(mite->second.c_str());
                }
                else if (mite->first == "threshold_h")
                {
                    conf.threshold_h = std::atof(mite->second.c_str());
                }
                cout << mite->first << "=" << mite->second << endl;
            }
            return true;
        }
        bool LidarPreprocess2::ReadCellConfigJson(const std::string &path, proprecess_config &conf)
        {
            ifstream in(path, ios::binary);
            if (!in.is_open())
            {
                // LOG(ERROR) << "Error opening preprocess file";
                return false;
            }
            Json::Reader reader;
            Json::Value value;
            if (reader.parse(in, value))
            {
                conf.xmax = value["xmax"].asFloat();
                conf.xmin = value["xmin"].asFloat();
                conf.ymax = value["ymax"].asFloat();
                conf.ymin = value["ymin"].asFloat();
                conf.zmax = value["zmax"].asFloat();
                conf.zmin = value["zmin"].asFloat();
                conf.car_xmax = value["car_xmax"].asFloat();
                conf.car_xmin = value["car_xmin"].asFloat();
                conf.car_ymax = value["car_ymax"].asFloat();
                conf.car_ymin = value["car_ymin"].asFloat();
                conf.cell_size_x = value["cell_size_x"].asFloat();
                conf.cell_size_y = value["cell_size_y"].asFloat();
                // conf.points_num = value["points_num"].asInt();
                // conf.radius_search = value["radius_search"].asFloat();
                // conf.search_num = value["search_num"].asInt();
                // conf.threshold_h = value["threshold_h"].asFloat();
                // conf.vaild = value["vaild"].asBool();
                // conf.voxel_size = value["voxel_size"].asFloat();
            }
            in.close();
            return true;
        }
        int LidarPreprocess2::DeleteNanPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &lidar_points,
                                              pcl::PointCloud<pcl::PointXYZ>::Ptr &data_out)
        {
            for (auto p : lidar_points->points)
            {
                if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z))
                    continue;
                data_out->points.emplace_back(p);
            }
            return 0;
        }

    } // lidar_objs
} // perception
