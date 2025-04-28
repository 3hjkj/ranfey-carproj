#ifndef LIDAR_PERPROCESS_H
#define LIDAR_PERPROCESS_H
#include "common/data_pool.h"
#include "lidar_obj_type.h"
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <math.h>
#include "json/include/json.h"
#include <iomanip>
#include "common/log.h"
#include <rclcpp/rclcpp.hpp>
namespace perception
{
    namespace lidar_objs
    {
        class LidarPreprocess
        {
        private:
            // objs_config objs_config_;
            proprecess_config proprecess_config_;
            // pcl::PointCloud<pcl::PointXYZI>::Ptr clouds_filter(new pcl::PointCloud<pcl::PointXYZI>);
            //debug

            perception::ReadConfigCommon readconfig_;
            rclcpp::Node::SharedPtr node_;   //!< 代替 ros::NodeHandle
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_radius;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_voxel;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_points;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_no_ground_points;
            bool debug_ = false;
            int row;
            int col;
            std::vector<std::vector<grid_obj>> grid_preprocess;

            bool ReadProConfig(const std::map<string, string> &m, proprecess_config &conf);
            bool ReadCellConfigJson(const std::string &path, proprecess_config &conf);
            // get groud points
            template <typename T1>
            int GetGridZminTest(const T1 &data_in, std::vector<std::vector<grid_obj>> &grid_preprocess)
            {
                if (data_in == nullptr)
                {
                    return 1;
                }
                if (data_in->empty())
                {
                    RCLCPP_ERROR_STREAM(node_->get_logger(), "no lidar data ground");
                    return 1;
                }
                for (auto &p : data_in->points)
                {
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
                        Grid(x, y, p.z);
                    }
                    catch (std::exception &e)
                    {
                        RCLCPP_DEBUG_STREAM(node_->get_logger(), e.what());
                    }
                }
                return 0;
            }
            int GetGridZmin(const pcl::PointCloud<pcl::PointXYZ>::Ptr &data_in,
                            std::vector<std::vector<grid_obj>> &grid_preprocess, int &points_num);
            int Grid(int x, int y, float z, int &points_num);
            int GetGroundPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &data_in,
                                std::vector<std::vector<grid_obj>> &grid_preprocess,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_poins,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_points,
                                float &threshold_h);
            template <typename T1, typename T2>
            int GetGroundPointsTest(const T1 &data_in, std::vector<std::vector<grid_obj>> &grid_preprocess,
                                    T1 &ground_poins, T1 &no_ground_points, T2 type)
            {
                int point_type = GetRighStructPoints(type);
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
                        if (p.z - grid_preprocess[x][y].zmin > proprecess_config_.threshold_h)
                        {
                            T2 p_xyz;
                            // p_xyz.x = p.x;
                            // p_xyz.y = p.y;
                            // p_xyz.z = p.z;
                            p_xyz = GetNeedPoints(p, point_type);
                            no_ground_points->points.emplace_back(p_xyz);
                        }
                        else
                        {
                            T2 p_xyz;
                            // p_xyz.x = p.x;
                            // p_xyz.y = p.y;
                            // p_xyz.z = p.z;
                            p_xyz = GetNeedPoints(p, point_type);
                            ground_poins->points.emplace_back(p_xyz);
                        }
                    }
                    else
                    {
                        T2 p_xyz;
                        // p_xyz.x = p.x;
                        // p_xyz.y = p.y;
                        // p_xyz.z = p.z;
                        p_xyz = GetNeedPoints(p, point_type);
                        ground_poins->points.emplace_back(p_xyz);
                    }
                }
                return 0;
            }

        public:
            explicit LidarPreprocess(const rclcpp::Node::SharedPtr& node);
            ~LidarPreprocess();
            int Init();
            int DeleteNanPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &lidar_points,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr &data_out);
            int VoxelFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &lidar_points,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &data_out,
                            float &voxel_size);
            int VoxelFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_points,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &data_out,
                            float &voxel_size);
            int VoxelFilter(const pcl_util::PointCloudPtr &lidar_points,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &data_out,
                            float &voxel_size);
            int VoxelFilter(const LidarDataInType &lidar_points,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr &data_out,
                            float &voxel_size);
            int VoxelFilter(const LidarDataInType &lidar_points,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &data_out,
                            float &voxel_size);
            int GroundPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &data_in,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_poins,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_points,
                             float &threshold_h,
                             int &points_num);
            int GroundPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr &data_in,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr &ground_poins,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_points,
                             float &threshold_h,
                             int &points_num);
            int GroundPoints(const LidarDataInType &data_in,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_poins,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_points,
                             float &threshold_h,
                             int &points_num);
            int GroundPoints(const LidarDataInType &data_in,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr &ground_poins,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_points,
                             float &threshold_h,
                             int &points_num);
            int RadiusFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &data_in,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr &data_out,
                             float &radius_search,
                             int &search_num);
            int RadiusFilter(const LidarDataInType &lidar_points,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr &data_out, float &radius_search,
                             int &search_num);
            template <typename T1, typename T2>
            int VoxelFilterTest(const T1 &data_in, T1 &data_out, T2 type)
            {
                static pcl::VoxelGrid<T2> sor;
                sor.setLeafSize(proprecess_config_.voxel_size, proprecess_config_.voxel_size, proprecess_config_.voxel_size);
                sor.setInputCloud(data_in);
                sor.filter(*data_out);
                if (debug_)
                {
                    sensor_msgs::msg::PointCloud2 output; //声明的输出的点云的格式
                    pcl::toROSMsg(*data_out, output);
                    // pcl_conversions::fromPCL(*data_out, output);
                    output.header.frame_id = "world";
                    pub_lidar_voxel->publish(output);
                }
                return 0;
            }
            template <typename T1, typename T2>
            int RadiusFilterTest(const T1 &data_in, T1 &data_out, T2 type)
            {
                pcl::RadiusOutlierRemoval<T2> outrem;
                outrem.setInputCloud(data_in);
                outrem.setRadiusSearch(proprecess_config_.radius_search);      // 搜索半径
                outrem.setMinNeighborsInRadius(proprecess_config_.search_num); // 搜索 最少点数
                // apply filter
                outrem.filter(*data_out);
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
            template <typename T1, typename T2>
            int GroundPointsTest(const T1 &data_in, T1 &ground_poins, T1 &no_ground_points, T2 type)
            {
                grid_preprocess = std::vector<std::vector<grid_obj>>(row, std::vector<grid_obj>(col));
                // 2.得到每个栅格的最低值
                try
                {
                    GetGridZminTest(data_in, grid_preprocess);
                }
                catch (const std::exception &e)
                {
                    std::cerr << e.what() << '\n';
                }

                // 3.根据最低值进行过滤地面点
                GetGroundPointsTest(data_in, grid_preprocess, ground_poins, no_ground_points, type);
                if (debug_)
                {
                    sensor_msgs::msg::PointCloud2 output_groud_points;    //声明的输出的点云的格式
                    sensor_msgs::msg::PointCloud2 output_no_groud_points; //声明的输出的点云的格式
                    pcl::toROSMsg(*ground_poins, output_groud_points);
                    pcl::toROSMsg(*no_ground_points, output_no_groud_points);
                    // pcl_conversions::fromPCL(*data_out, output);
                    // output_groud_points.header.frame_id = "/rslidar";
                    output_groud_points.header.frame_id = "world";
                    // output_no_groud_points.header.frame_id = "/rslidar";
                    output_no_groud_points.header.frame_id = "world";
                    pub_ground_points->publish(output_groud_points);
                    pub_no_ground_points->publish(output_no_groud_points);
                }
                return 0;
            }
            template <typename T>
            int GetRighStructPoints(const T &points)
            {
                // xy-0 xyz-1 XYZIH-2 XYZIRT-3 XYZIART-4 XYZIT-5 XYZIRTd-6
                // XYZIRTd-7 XYZITd -8 XYZRGBRCG-9 xyz-10 xyzi-11 xyzrgb-12
                pcl::PointXYZ xyz;              // 1
                pcl_util::PointXYZIART xyziart; // 4
                pcl::PointXYZI xyzi;            // 11
                pcl::PointXYZRGB xyzrgb;        // 12
                T type;
                int p1 = std::memcmp(&xyz, &type, sizeof(struct pcl::PointXYZ));
                int p4 = std::memcmp(&xyziart, &type, sizeof(struct pcl_util::PointXYZIART));
                int p11 = std::memcmp(&xyzi, &type, sizeof(struct pcl::PointXYZI));
                int p12 = std::memcmp(&xyzrgb, &type, sizeof(struct pcl::PointXYZRGB));
                if (p1 == 0)
                    return 1;
                else if (p4 == 0)
                    return 4;
                else if (p11 == 0)
                    return 11;
                else if (p12 == 0)
                    return 12;
                return -1;
            }
            template <typename T>
            T GetNeedPoints(const T &point, int type)
            {
                T point_ret;
                switch (type)
                {
                case 1:
                    point_ret.x = point.x;
                    point_ret.y = point.y;
                    point_ret.z = point.z;
                    break;
                case 4:
                    // point_ret = GetARTpoint(point);
                    break;
                    // case 11:
                    //     point_ret.x = point.x;
                    //     point_ret.y = point.y;
                    //     point_ret.z = point.z;
                    //     point_ret.intensity = point.intensity;
                    //     break;
                    // case 12:
                    //     point_ret.x = point.x;
                    //     point_ret.y = point.y;
                    //     point_ret.z = point.z;
                    //     point_ret.r = point.r;
                    //     point_ret.g = point.g;
                    //     point_ret.b = point.b;
                    break;
                default:
                    break;
                }
                return point_ret;
            }
            template <typename T1, typename T2, typename T3, typename T4>
            T3 GetNeedPoints2(const T1 &points, T2 &point_type, T3 &data_out_type_ptr, T4 &data_out_type)
            {
                T3 point_ret(new T4);
                int type = GetRighStructPoints(point_type);
                for (auto &point : points->points)
                {
                    T2 point_tmp;
                    switch (type)
                    {
                    case 1:
                        point_tmp.x = point.x;
                        point_tmp.y = point.y;
                        point_tmp.z = point.z;
                        break;
                    // case 4:
                    //     point_tmp = GetARTpoint(point);
                    //     break;
                    // case 11:
                    //     point_tmp.x = point.x;
                    //     point_tmp.y = point.y;
                    //     point_tmp.z = point.z;
                    //     point_tmp.intensity = point.intensity;
                    //     break;
                    // case 12:
                    //     point_tmp.x = point.x;
                    //     point_tmp.y = point.y;
                    //     point_tmp.z = point.z;
                    //     point_tmp.r = point.r;
                    //     point_tmp.g = point.g;
                    //     point_tmp.b = point.b;
                    //     break;
                    default:
                        break;
                    }
                    point_ret->points.emplace_back(point_tmp);
                }
                return point_ret;
            }
            template <typename T>
            T GetARTpoint(const T &point)
            {
                T point_tmp;
                point_tmp.x = point.x;
                point_tmp.y = point.y;
                point_tmp.z = point.z;
                point_tmp.intensity = point.intensity;
                point_tmp.angle = point.angle;
                point_tmp.ring = point.ring;
                point_tmp.timestamp = point.timestamp;
                return point_tmp;
            }
            // pcl_util::PointXYZIART GetARTpoint(const pcl_util::PointXYZIART &point)
            // {
            //     pcl_util::PointXYZIART point_tmp;
            //     point_tmp.x = point.x;
            //     point_tmp.y = point.y;
            //     point_tmp.z = point.z;
            //     point_tmp.intensity = point.intensity;
            //     point_tmp.angle = point.angle;
            //     point_tmp.ring = point.ring;
            //     point_tmp.timestamp = point.timestamp;
            //     return point_tmp;
            // }
        };
    }
}
#endif