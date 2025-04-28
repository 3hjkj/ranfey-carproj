#include "architecture/lidar_cells/include/lidar_cell.h"
#include <pcl_conversions/pcl_conversions.h>

namespace perception
{
    namespace lidar_cells
    {

                /* ───────── 构造：仅保存 node_，算法成员保持原来 ───────── */
        LidarCell::LidarCell(const rclcpp::Node::SharedPtr& node)
        : node_(node)
        {
        lidar_preprocess_ = std::make_shared<lidar_objs::LidarPreprocess>(node_);
        }

        /* ───────── Init：参数读取、发布器创建、表格初始化 ───────── */
        int LidarCell::Init()
        {
        INFO("~~~~~init lidar cell~~~~~");

        /* ---------- 1. 读取 launch 中传入的参数 ---------- */
        /*   先 declare，再 get_parameter，可覆盖默认值 */
        std::string topic_name =
            node_->declare_parameter<std::string>("topic_name", "fusion");
        std::string cell_cfg =
            node_->declare_parameter<std::string>("cell_config",
                "config_json/cell.json");

        /* ---------- 2. 读取 Cell 配置 JSON ---------- */
        if (!ReadCellConfigJson(cell_cfg, cell_config_))
        {
            ERROR("Failed to read cell config json: %s", cell_cfg.c_str());
            return -1;
        }

        /* ---------- 3. 初始化预处理模块 ---------- */
        lidar_preprocess_->Init();

        /* ---------- 4. 分配二维表格 ---------- */
        row = static_cast<int>((cell_config_.xmax - cell_config_.xmin) /
                                cell_config_.cell_size_x);
        col = static_cast<int>((cell_config_.ymax - cell_config_.ymin) /
                                cell_config_.cell_size_y);
        data_cell_.assign(row, std::vector<data_cell>(col));
        data_cell_tmp.assign(row, std::vector<data_cell>(col));

        /* ---------- 5. 创建发布器 ---------- */
        pub_no_ground_points =
            node_->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/perception/no_ground_points", 10);

        return 0;
        }
        int LidarCell::Process(const LidarDataInType &lidar_points, lidar_msgs::msg::Cells &lidar_cells)
        {
            // algorithm process
            INFO("~~~~start lidar cell process~~~~~");
            lidar_cells.cells.clear();
            int counter_l = 0;
            // 方法1 不滤波不分割地面
            if (cell_config_.method == 1)
            {
                for (auto &points : lidar_points.point_cloud_ptr_map)
                {
                    if (points.second->empty())
                    {

                        ++counter_l;
                    }
                    if (counter_l == 5)
                    {
                        RCLCPP_ERROR(node_->get_logger(), "no lidar data");
                        return 1;
                    }
                    for (auto &p : points.second->points)
                    {
                        // jump the points
                        if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z))
                        {
                            continue;
                        }
                        if ((p.x < cell_config_.xmin || p.x > cell_config_.xmax ||
                             p.y < cell_config_.ymin || p.y > cell_config_.ymax ||
                             p.z < cell_config_.zmin || p.z > cell_config_.zmax) ||
                            (p.x < cell_config_.car_xmax && p.x > cell_config_.car_xmin &&
                             p.y < cell_config_.car_ymax && p.y > cell_config_.car_ymin))
                        {
                            continue;
                        }
                        int x = int(std::round(p.x - cell_config_.xmin) / cell_config_.cell_size_x);
                        int y = int(std::round(p.y - cell_config_.ymin) / cell_config_.cell_size_y);
                        if (x > row - 1 || y > col - 1 || x < 0 || y < 0)
                        {
                            continue;
                        }
                        // start jisuan
                        try
                        {
                            // leiji z
                            Grid(x, y, p.z, p.x, p.y);
                        }
                        catch (std::exception &e)
                        {
                            RCLCPP_DEBUG_STREAM(node_->get_logger(), e.what());
                        }
                    }
                }
            }
            // 方法2 2-1为三种都用,2-2为半径加地面分割,2-3为只半径滤波,三种方式都是所有点云全部一起计算,对标定要求严格
            else if (cell_config_.method == 2)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_filter_v(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_filter(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_points(new pcl::PointCloud<pcl::PointXYZ>);
                switch (cell_config_.preprocess_method)
                {
                case 1:
                    lidar_preprocess_->VoxelFilter(lidar_points, clouds_filter_v, cell_config_.voxel_size);
                    lidar_preprocess_->RadiusFilter(clouds_filter_v, clouds_filter, cell_config_.radius_search,
                                                    cell_config_.search_num);
                    lidar_preprocess_->GroundPoints(clouds_filter, ground_points, no_ground_points,
                                                    cell_config_.threshold_h, cell_config_.points_num);
                    GridCell(no_ground_points);
                    break;
                case 2:
                    lidar_preprocess_->RadiusFilter(lidar_points, clouds_filter, cell_config_.radius_search,
                                                    cell_config_.search_num);
                    lidar_preprocess_->GroundPoints(clouds_filter, ground_points, no_ground_points,
                                                    cell_config_.threshold_h, cell_config_.points_num);
                    GridCell(no_ground_points);
                    break;
                case 3:
                    lidar_preprocess_->RadiusFilter(lidar_points, clouds_filter, cell_config_.radius_search,
                                                    cell_config_.search_num);

                    GridCell(clouds_filter);
                    break;
                default:
                    break;
                }
            }
            // 方法3 体素+半径+地面,但是每个雷达分开计算,对标定要求不高,但是会损失点云
            else if (cell_config_.method == 3)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_filter_r(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_filter_v(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_no_ground(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_ground(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_no_ground_all(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr clouds(new pcl::PointCloud<pcl::PointXYZ>);

                for (auto &points : lidar_points.point_cloud_ptr_map)
                {
                    // counter++;
                    if (points.second->empty())
                    {

                        counter_l++;
                        WARN("counter_l :{},counter:{}", counter_l, counter);
                        continue;
                    }
                    if (counter_l == 5)
                    {
                        // LOG(ERROR) << "no lidar data :" << counter_l;
                        counter_l = 0;
                        return 1;
                    }
                    lidar_preprocess_->VoxelFilter(points.second, clouds_filter_v,
                                                   cell_config_.voxel_size);
                    lidar_preprocess_->RadiusFilter(clouds_filter_v, clouds_filter_r,
                                                    cell_config_.radius_search, cell_config_.search_num);
                    // LOG(WARNING) << "clouds_filter_r->points.size:" << clouds_filter_r->points.size();
                    lidar_preprocess_->GroundPoints(clouds_filter_r, clouds_ground, clouds_no_ground,
                                                    cell_config_.threshold_h, cell_config_.points_num);
                    // 注释部分为使用模板函数进行函数调用,但是实际测试发现,使用模板函数的运行速度下降了大概10-20毫秒,故不使用
                    // pcl::PointXYZ tmp;
                    // pcl::PointCloud<pcl::PointXYZ> tmp_;
                    // clouds = lidar_preprocess_->GetNeedPoints2(points.second, tmp, clouds, tmp_);
                    // lidar_preprocess_->VoxelFilterTest(clouds, clouds_filter_v, tmp);
                    // lidar_preprocess_->RadiusFilterTest(clouds_filter_v, clouds_filter_r, tmp);
                    // lidar_preprocess_->GroundPointsTest(clouds_filter_r, clouds_ground, clouds_no_ground, tmp);
                    INFO("clouds_no_ground->points.size:{}", clouds_no_ground->points.size());
                    for (auto &p : clouds_no_ground->points)
                    {
                        clouds_no_ground_all->points.emplace_back(p);
                    }
                    if (1)
                    {
                        sensor_msgs::msg::PointCloud2 output_no_groud_points; //声明的输出的点云的格式
                        pcl::toROSMsg(*clouds_no_ground_all, output_no_groud_points);
                        output_no_groud_points.header.frame_id = "world";
                        pub_no_ground_points->publish(output_no_groud_points);
                    }
                }
                if (clouds_no_ground_all->points.empty())
                {
                    ERROR("no clouds_no_ground_all points");
                    return 1;
                }
                GridCell(clouds_no_ground_all);
            }
            PushCell(lidar_cells);
            data_cell_tmp = data_cell_;
            counter_l = 0;
            counter = 0;
            return 0;
        } //process
        int LidarCell::PointsCa(int i, int j)
        {

            // if (data_cell_[i][j])
            if (data_cell_tmp[i][j].point_num > data_cell_[i][j].point_num)
            {
                data_cell_[i][j].point_num = data_cell_tmp[i][j].point_num;
            }
            else
            {
                data_cell_tmp[i][j].point_num = data_cell_[i][j].point_num;
            }
            return 0;
        }
        int LidarCell::GridCell(const pcl::PointCloud<pcl::PointXYZ>::Ptr &data_in)
        {
            for (auto &p : data_in->points)
            {
                // 跳过无效点
                if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z))
                    continue;

                // 跳过车体
                if ((p.x < cell_config_.xmin || p.x > cell_config_.xmax ||
                     p.y < cell_config_.ymin || p.y > cell_config_.ymax ||
                     p.z < cell_config_.zmin || p.z > cell_config_.zmax) ||
                    (p.x < cell_config_.car_xmax && p.x > cell_config_.car_xmin &&
                     p.y < cell_config_.car_ymax && p.y > cell_config_.car_ymin))
                    continue;
                if (debug)
                {

                    if (p.x >= 1 && p.x <= 18 && p.y <= 1 && p.y > -1)
                    {
                        std::cout << "~~~~~~~~~:::~~~~~~~~~~~~~~" << std::endl;
                        std::cout << " x:" << p.x << " y:" << p.y << " z:" << p.z << std::endl;
                        std::cout << " x_min:" << std::floor(12 - cell_config_.xmin) / cell_config_.cell_size_x
                                  << " x_max:" << std::floor(30 - cell_config_.xmin) / cell_config_.cell_size_x
                                  << " y_min:" << std::floor(-2 - cell_config_.ymin) / cell_config_.cell_size_y
                                  << " y_max:" << std::floor(2 - cell_config_.ymin) / cell_config_.cell_size_y
                                  << " grid_x:" << int(std::floor(p.x - cell_config_.xmin) / cell_config_.cell_size_x)
                                  << " grid_y:" << int(std::floor(p.y - cell_config_.ymin) / cell_config_.cell_size_y)
                                  << std::endl;
                    }
                }
                // int x = int(std::round(p.x - cell_config_.xmin) / cell_config_.cell_size_x);
                // int y = int(std::round(p.y - cell_config_.ymin) / cell_config_.cell_size_y);
                int x = int(std::round((p.x - cell_config_.xmin) / cell_config_.cell_size_x));
                int y = int(std::round((p.y - cell_config_.ymin) / cell_config_.cell_size_y));
                // 跳过检测范围外的
                if (x > row - 1 || y > col - 1 || x < 0 || y < 0)
                    continue;

                // 计算
                try
                {
                    // 累计高度
                    Grid(x, y, p.z, p.x, p.y);
                }
                catch (std::exception &e)
                {
                    // LOG(ERROR) << e.what();
                }
            }
            return 0;
        }
        int LidarCell::Grid(int x, int y, float pz, float px, float py)
        { 
            data_cell_[x][y].x = x * cell_config_.cell_size_x + cell_config_.xmin + cell_config_.cell_size_x / 2;
            data_cell_[x][y].y = y * cell_config_.cell_size_y + cell_config_.ymin + cell_config_.cell_size_y / 2;
            data_cell_[x][y].idx = x * col + y;
            // 根据点云数量初始化xyz最大最小值
            if (data_cell_[x][y].point_num > 0)
            {
                data_cell_[x][y].z_max = pz > data_cell_[x][y].z_max ? pz : data_cell_[x][y].z_max;
                data_cell_[x][y].z_min = pz < data_cell_[x][y].z_min ? pz : data_cell_[x][y].z_min;
                data_cell_[x][y].x_max = px > data_cell_[x][y].x_max ? px : data_cell_[x][y].x_max;
                data_cell_[x][y].x_min = px < data_cell_[x][y].x_min ? px : data_cell_[x][y].x_min;
                data_cell_[x][y].y_max = py > data_cell_[x][y].y_max ? py : data_cell_[x][y].y_max;
                data_cell_[x][y].y_min = py < data_cell_[x][y].y_min ? py : data_cell_[x][y].y_min;
            }
            else
            {
                data_cell_[x][y].z_max = pz;
                data_cell_[x][y].z_min = pz;
                data_cell_[x][y].y_max = py;
                data_cell_[x][y].y_min = py;
                data_cell_[x][y].x_max = px;
                data_cell_[x][y].x_min = px;
            }
            data_cell_[x][y].sum += pz;
            data_cell_[x][y].point_num++;
            data_cell_[x][y].z_intercept = data_cell_[x][y].z_max - data_cell_[x][y].z_min;
            data_cell_[x][y].z_mean = data_cell_[x][y].sum / data_cell_[x][y].point_num;
            data_cell_[x][y].x_sum += px;
            data_cell_[x][y].x = data_cell_[x][y].x_sum / data_cell_[x][y].point_num;
            data_cell_[x][y].y_sum += py;
            data_cell_[x][y].y = data_cell_[x][y].y_sum / data_cell_[x][y].point_num;
            // 只对十米以内的障碍物进行筛选,十米以上的障碍物不关注高度,都做为障碍物,十米以内必须要大于0.3
            if (x >= std::abs(int(std::round(cell_config_.dist10 - cell_config_.xmin) /
                                  cell_config_.cell_size_x)))
            {
                if (data_cell_[x][y].z_intercept < 0.1)
                {
                    data_cell_[x][y].z_intercept = 0.21;
                    data_cell_[x][y].vaild = 1;
                    data_cell_[x][y].time = node_->get_clock()->now().seconds();
                    if (debug)
                    {
                        if (x >= int(std::round(16 - cell_config_.xmin) / cell_config_.cell_size_x) &&
                            x <= int(std::round(18 - cell_config_.xmin) / cell_config_.cell_size_x) &&
                            y <= int(std::round(1 - cell_config_.ymin) / cell_config_.cell_size_y) &&
                            y >= int(std::round(-1 - cell_config_.ymin) / cell_config_.cell_size_y))
                        {
                            std::cout << "~~~~~~~~~xxxxx~~~~~~~~~~~~~" << std::endl;
                            std::cout << "x:" << x * cell_config_.cell_size_x + cell_config_.xmin
                                      << " y:" << y * cell_config_.cell_size_y + cell_config_.ymin
                                      << " z_intercept:" << data_cell_[x][y].z_intercept
                                      << " zmax:" << data_cell_[x][y].z_max
                                      << " zmin:" << data_cell_[x][y].z_min
                                      << " zmean:" << data_cell_[x][y].z_mean
                                      << " points_num:" << data_cell_[x][y].point_num
                                      << " vaild:" << data_cell_[x][y].vaild
                                      << " confidence:" << data_cell_[x][y].confidence
                                      << " time:" << data_cell_[x][y].time
                                      << std::endl;
                        }
                    }
                }
                else
                {
                    data_cell_[x][y].vaild = 1;
                    data_cell_[x][y].time = node_->get_clock()->now().seconds();
                }
            }
            else if (x < std::abs(int(std::round(cell_config_.dist10 - cell_config_.xmin) /
                                      cell_config_.cell_size_x)))
            {
                if (data_cell_[x][y].z_intercept > 0.3)
                {
                    data_cell_[x][y].vaild = 1;
                    data_cell_[x][y].time = node_->get_clock()->now().seconds();
                }
            }
            if (0)
            {
                if (x >= int(std::round(-5 - cell_config_.xmin) / cell_config_.cell_size_x) &&
                    x <= int(std::round(5 - cell_config_.xmin) / cell_config_.cell_size_x) &&
                    y <= int(std::round(5 - cell_config_.ymin) / cell_config_.cell_size_y) &&
                    y >= int(std::round(-5 - cell_config_.ymin) / cell_config_.cell_size_y) &&
                    data_cell_[x][y].z_intercept > 0.3)
                {
                    std::cout << "~~~~~~~~~12-20~~~~~~~~~~~~~" << std::endl;
                    std::cout << "x:" << x * cell_config_.cell_size_x + cell_config_.xmin
                              << " y:" << y * cell_config_.cell_size_y + cell_config_.ymin
                              << " z_intercept:" << data_cell_[x][y].z_intercept
                              << " zmax:" << data_cell_[x][y].z_max
                              << " zmin:" << data_cell_[x][y].z_min
                              << " zmean:" << data_cell_[x][y].z_mean
                              << " points_num:" << data_cell_[x][y].point_num
                              << " vaild:" << data_cell_[x][y].vaild
                              << " confidence:" << data_cell_[x][y].confidence
                              << " time:" << data_cell_[x][y].time
                              << std::endl;
                }
            }
            return 0;
        }
        int LidarCell::PushCell(lidar_msgs::msg::Cells &lidar_cells)
        {
            for (int i = 0; i < row; i++)
            {
                for (int j = 0; j < col; j++)
                {
                    lidar_msgs::msg::Cell cell;
                    if (0)
                    {

                        if (i >= int(std::round(-5 - cell_config_.xmin) / cell_config_.cell_size_x) &&
                            i <= int(std::round(5 - cell_config_.xmin) / cell_config_.cell_size_x) &&
                            j <= int(std::round(5 - cell_config_.ymin) / cell_config_.cell_size_y) &&
                            j >= int(std::round(-5 - cell_config_.ymin) / cell_config_.cell_size_y))
                        {
                            std::cout << "~~~~~~~~~sss~~~~~~~~~~~~" << std::endl;
                            std::cout << "x:" << data_cell_[i][j].x << " y:" << data_cell_[i][j].y
                                      << " zmin:" << data_cell_[i][j].z_min
                                      << " zmax:" << data_cell_[i][j].z_max
                                      << " z_int:" << data_cell_[i][j].z_intercept
                                      << " confidence:" << data_cell_[i][j].confidence
                                      << " point_num:" << data_cell_[i][j].point_num
                                      << std::endl;
                        }
                    }
                    double time = node_->get_clock()->now().seconds();
                    // 判断是否超时，若超时，则当前帧没有得到此cell的点云，减置信度，否则加置信度
                    if ((time - data_cell_[i][j].time) * 1000 > 100)
                    {
                        if (0)
                        {
                            if (i >= int(std::round(-5 - cell_config_.xmin) / cell_config_.cell_size_x) &&
                                i <= int(std::round(5 - cell_config_.xmin) / cell_config_.cell_size_x) &&
                                j <= int(std::round(5 - cell_config_.ymin) / cell_config_.cell_size_y) &&
                                j >= int(std::round(-5 - cell_config_.ymin) / cell_config_.cell_size_y) &&
                                data_cell_[i][j].z_intercept > 0.3)
                            {
                                std::cout << "~~~~~~~~~12-20~~~~~~~~~~~~~" << std::endl;
                                std::cout << "x:" << i * cell_config_.cell_size_x + cell_config_.xmin
                                          << " y:" << j * cell_config_.cell_size_y + cell_config_.ymin
                                          << " z_intercept:" << data_cell_[i][j].z_intercept
                                          << " zmax:" << data_cell_[i][j].z_max
                                          << " zmin:" << data_cell_[i][j].z_min
                                          << " zmean:" << data_cell_[i][j].z_mean
                                          << " points_num:" << data_cell_[i][j].point_num
                                          << " vaild:" << data_cell_[i][j].vaild
                                          << " confidence:" << data_cell_[i][j].confidence
                                          << " time:" << (time - data_cell_[i][j].time) * 1000
                                          << std::endl;
                            }
                        }
                        data_cell_[i][j].time = time;
                        data_cell_[i][j].confidence -= cell_config_.confidence_;
                        PointsCa(i, j);
                    }
                    else
                    {
                        if (data_cell_[i][j].confidence < 30 && data_cell_[i][j].confidence > 4)
                            data_cell_[i][j].confidence += 1;
                        else if (data_cell_[i][j].confidence > 30)
                            data_cell_[i][j].confidence = 30;
                        else if (data_cell_[i][j].confidence < 5 && data_cell_[i][j].confidence > 0)
                            data_cell_[i][j].confidence += 5;
                        else if (data_cell_[i][j].confidence < 1)
                            data_cell_[i][j].confidence = 6;
                    }
                    // 筛除不满足置信度的
                    if (data_cell_[i][j].confidence < 5)
                    {
                        data_cell_[i][j].reset();
                        continue;
                    }
                    // 筛除不满足最高度的
                    if (data_cell_[i][j].z_intercept < cell_config_.min_h)
                        continue;

                    // 筛除车体
                    if (i < 0.1 && i > -4.1 && j < 1.1 && j > -1.1)
                        continue;

                    // 根据距离筛选cell 分10 20 30 m，根据密度及点数筛选
                    if (std::fabs(data_cell_[i][j].x) < cell_config_.dist10)
                    {
                        if (data_cell_[i][j].point_num / (cell_config_.cell_size_x * cell_config_.cell_size_y * data_cell_[i][j].z_intercept) < cell_config_.density_10 ||
                            data_cell_[i][j].point_num < cell_config_.min_point_num_10 ||
                            data_cell_[i][j].z_intercept < 0.3)
                            continue;
                    }
                    else if (std::fabs(data_cell_[i][j].x) >= cell_config_.dist10 &&
                             std::fabs(data_cell_[i][j].x) <= cell_config_.dist20)
                    {
                        if (data_cell_[i][j].point_num / (cell_config_.cell_size_x * cell_config_.cell_size_y * data_cell_[i][j].z_intercept) < cell_config_.density_20 ||
                            data_cell_[i][j].point_num < cell_config_.min_point_num_20)
                            continue;
                    }
                    else if (std::fabs(data_cell_[i][j].x) > cell_config_.dist30)
                    {
                        if (data_cell_[i][j].point_num / (cell_config_.cell_size_x * cell_config_.cell_size_y * data_cell_[i][j].z_intercept) < cell_config_.density_30 ||
                            data_cell_[i][j].point_num < cell_config_.min_point_num_30)
                            continue;
                    }
                    cell.x = data_cell_[i][j].x;
                    cell.y = data_cell_[i][j].y;
                    cell.x_min = data_cell_[i][j].x_min;
                    cell.y_min = data_cell_[i][j].y_min;
                    cell.x_max = data_cell_[i][j].x_max;
                    cell.y_max = data_cell_[i][j].y_max;
                    cell.zmin = data_cell_[i][j].z_min;
                    cell.zmax = data_cell_[i][j].z_max;
                    cell.z_intercept = data_cell_[i][j].z_intercept;
                    cell.zmean = data_cell_[i][j].z_mean;
                    cell.vaild = data_cell_[i][j].vaild;
                    cell.idx = data_cell_[i][j].idx;
                    cell.points_num = data_cell_[i][j].point_num;
                    cell.confidence = data_cell_[i][j].confidence;
                    cell.time = data_cell_[i][j].time;
                    data_cell_[i][j].point_num = 0;
                    data_cell_[i][j].sum = 0;
                    data_cell_[i][j].x_sum = 0;
                    data_cell_[i][j].y_sum = 0;
                    lidar_cells.cells.push_back(cell);
                    if (debug)
                    {
                        if (cell.x > 1 && cell.x < 18 && cell.y > -1 && cell.y < 1)
                        {
                            std::cout << "~~~~~~~~~~push~~~~~~~~~" << std::endl;
                            std::cout << "x:" << cell.x << " y:" << cell.y << " z_intercept:" << cell.z_intercept << " zmax:" << cell.zmax << " zmin:" << cell.zmin << " zmean:" << cell.zmean << " points_num:" << cell.points_num << std::endl;
                        }
                    }
                }
            }
            return 0;
        }
        bool LidarCell::ReadCellConfig(const std::map<string, string> &m, cell_config &conf)
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
                else if (mite->first == "method")
                {
                    conf.method = std::atoi(mite->second.c_str());
                }
                else if (mite->first == "min_point_num_10")
                {
                    conf.min_point_num_10 = std::atoi(mite->second.c_str());
                }
                else if (mite->first == "min_h")
                {
                    conf.min_h = std::atoi(mite->second.c_str());
                }
                else if (mite->first == "density_10")
                {
                    conf.density_10 = std::atof(mite->second.c_str());
                }
                else if (mite->first == "dist10")
                {
                    conf.dist10 = std::atoi(mite->second.c_str());
                }
                else if (mite->first == "min_point_num_20")
                {
                    conf.min_point_num_20 = std::atoi(mite->second.c_str());
                }
                else if (mite->first == "density_20")
                {
                    conf.density_10 = std::atof(mite->second.c_str());
                }
                else if (mite->first == "dist20")
                {
                    conf.dist20 = std::atoi(mite->second.c_str());
                }
                else if (mite->first == "min_point_num_30")
                {
                    conf.min_point_num_20 = std::atoi(mite->second.c_str());
                }
                else if (mite->first == "density_30")
                {
                    conf.density_10 = std::atof(mite->second.c_str());
                }
                else if (mite->first == "dist30")
                {
                    conf.dist30 = std::atoi(mite->second.c_str());
                }
                else if (mite->first == "preprocess_method")
                {
                    conf.preprocess_method = std::atoi(mite->second.c_str());
                }
                else if (mite->first == "confidence_")
                {
                    conf.confidence_ = std::atoi(mite->second.c_str());
                }

                cout << mite->first << "=" << mite->second << endl;
            }
            return true;
        }
        bool LidarCell::ReadCellConfigJson(const std::string &path, cell_config &conf)
        {
            ifstream in(path, ios::binary);
            if (!in.is_open())
            {
                // LOG(ERROR) << "Error opening lidar cell file";
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
                conf.method = value["method"].asInt();                       // 1 为老方法 2 为引用lidar_obj的预处理 所有点云一起 3 为引用lidar_obj的预处理 单个点云进行
                conf.preprocess_method = value["preprocess_method"].asInt(); // 1-体素栅格滤波+radius+地面分割 2-radius+地面  3-radius
                conf.min_h = value["min_h"].asFloat();                       // 最小的障碍物高度
                conf.confidence_ = value["confidence_"].asInt();             // confidence下降速度
                conf.min_point_num_10 = value["min_point_num_10"].asInt();   // 十米的点数
                conf.min_point_num_20 = value["min_point_num_20"].asInt();   // 20m的点数
                conf.min_point_num_30 = value["min_point_num_30"].asInt();   // 30m的点数
                conf.density_10 = value["density_10"].asFloat();             // density = points_num / 体积
                conf.density_20 = value["density_20"].asFloat();
                conf.density_30 = value["density_30"].asFloat();
                conf.dist10 = value["dist10"].asInt();
                conf.dist20 = value["dist20"].asInt();
                conf.dist30 = value["dist30"].asInt();

                conf.voxel_size = value["voxel_size"].asFloat();
                conf.points_num = value["points_num"].asInt();
                conf.radius_search = value["radius_search"].asFloat();
                conf.search_num = value["search_num"].asInt();
                conf.threshold_h = value["threshold_h"].asFloat();
            }
            in.close();
            return true;
        }
    } //lidar_cells
} //perception
