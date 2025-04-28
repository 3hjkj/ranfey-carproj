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
        debug_ = node_->declare_parameter<bool>("debug_pre", false);
        if (debug_) {
            pub_lidar_voxel      = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/lidar_voxel",       10);
            pub_lidar_radius     = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/lidar_radius",      10);
            pub_ground_points    = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/ground_points",     10);
            pub_no_ground_points = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/no_ground_points_", 10);
        }

        /* 读取参数 */
        std::string cfg_path = node_->declare_parameter<std::string>(
            "preprocess_config", "config_json/preprocess.json");
        ReadCellConfigJson(cfg_path, proprecess_config_);

        row = static_cast<int>((proprecess_config_.xmax - proprecess_config_.xmin) /
                                proprecess_config_.cell_size_x);
        col = static_cast<int>((proprecess_config_.ymax - proprecess_config_.ymin) /
                                proprecess_config_.cell_size_y);
        return 0;
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
                std::cout << "data_in_tmp:" << data_in_tmp->points.size() << "\n";
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
            std::cout << "clouds size:" << clouds->points.size() << "\n";
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
            // LOG(INFO) << "radius filter size:" << data_in->points.size();
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
                outrem.setMinNeighborsInRadius(search_num); // 搜索 最少点数
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
