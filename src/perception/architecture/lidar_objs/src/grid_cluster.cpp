#include "architecture/lidar_objs/include/grid_cluster.h"
#include <pcl_conversions/pcl_conversions.h>          // pcl↔ROS2 消息



namespace perception
{
    namespace lidar_objs
    {
        LidarCluster::LidarCluster(const rclcpp::Node::SharedPtr& node)
        : node_(node)
        {
        }



        int LidarCluster::Init()
        {
        INFO("grid lidar cluster init");

        lidar_preprocess = std::make_shared<LidarPreprocess2>();
        lidar_cluster_   = std::make_shared<PointsCluster>();

        /* ---------- 1. 调试开关与调试话题 ---------- */
        debug_ = node_->declare_parameter<bool>("debug_objs", true);
        if (debug_)
        pub_objs_ =
                node_->create_publisher<lidar_msgs::msg::Objects>(
                    "/perception/lidar_objs_debug", 10);

        /* ---------- 2. obj_config 路径 ---------- */
        std::string filename =
            node_->declare_parameter<std::string>("obj_config",
                "config_json/objs.json");

        /* ---------- 3. 读取 JSON 配置 ---------- */
        if (!ReadCellConfigJson(filename, objs_config_))
        {
            ERROR("read obj config json failed: %s", filename.c_str());
            return -1;
        }

        /* ---------- 4. 子模块初始化 ---------- */
        lidar_preprocess->Init();
        return 0;
        }
        /* =============================================================
        *  Process：点云预处理 + 聚类 + 发布
        * ===========================================================*/
        int LidarCluster::Process(const LidarDataInType& lidar_points,
                                lidar_msgs::msg::Objects& lidar_objs)
        {
        INFO("start grid lidar cluster");

        pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_filter_r(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_filter_v(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_no_ground(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_ground(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr clouds_no_ground_all(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr clouds(new pcl::PointCloud<pcl::PointXYZ>);

        /* ---------- A. 逐雷达预处理 ---------- */
        for (auto& pair : lidar_points.point_cloud_ptr_map)
        {
            if (pair.second->empty()) continue;

            lidar_preprocess->VoxelFilter (pair.second,  clouds_filter_v, objs_config_.voxel_size);
            lidar_preprocess->RadiusFilter(clouds_filter_v, clouds_filter_r,
                                        objs_config_.radius_search, objs_config_.search_num);
            lidar_preprocess->GroundPoints(clouds_filter_r, clouds_ground, clouds_no_ground,
                                        objs_config_.threshold_h, objs_config_.points_num);

            for (auto& p : clouds_no_ground->points)
            if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
                clouds_no_ground_all->points.emplace_back(p);
        }

        /* ---------- B. 如果没有地面外点则返回 ---------- */
        if (clouds_no_ground_all->points.empty())
        {
            ERROR("no noground points");
            return 1;
        }

        /* ---------- C. 二次体素、半径滤波，加速聚类 ---------- */
        clouds_filter_v->points.clear();
        float voxel_size = 0.3f;
        lidar_preprocess->VoxelFilter(clouds_no_ground_all, clouds_filter_v, voxel_size);
        lidar_preprocess->RadiusFilter(clouds_filter_v, clouds,
                                        objs_config_.radius_search, objs_config_.search_num);

        /* ---------- D. 调用聚类模块并发布 ---------- */
        lidar_objs = lidar_cluster_->Pub(clouds);

        if (debug_)
        pub_objs_->publish(lidar_objs);

        return 0;
        }
        std::vector<pcl::PointIndices> LidarCluster::ClusterIndicesEE(const pcl::PointCloud<pcl::PointXYZ>::Ptr &data_in, std::vector<pcl::PointIndices> cluster_indices)
        {
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
            // Normal estimation
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
            ne.setInputCloud(data_in);

            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>());
            ne.setSearchMethod(tree_n);
            ne.setRadiusSearch(objs_config_.in_max_cluster_distance);
            ne.compute(*cloud_normals);

            // Creating the kdtree object for the search method of the extraction
            pcl::KdTree<pcl::PointXYZ>::Ptr tree_ec(new pcl::KdTreeFLANN<pcl::PointXYZ>());
            tree_ec->setInputCloud(data_in);

            // Extracting Euclidean clusters using cloud and its normals
            // std::vector<pcl::PointIndices> cluster_indices;
            // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;   //创建欧式聚类对象
            // ec.setClusterTolerance(objs_config_.tolerance);      // 设置距离阈值为13cm。点与点之间小于这个距离阈值视为一类
            // ec.setMinClusterSize(objs_config_.min_cluster_size); //设置聚类最少点数
            // ec.setMaxClusterSize(5000);                          //设置聚类最大点数
            // ec.setSearchMethod(tree);                            //输入点云搜索方法
            // ec.setInputCloud(data_in);
            // ec.extract(cluster_indices);

            pcl::extractEuclideanClusters(*data_in, *cloud_normals, objs_config_.tolerance, tree_ec, cluster_indices, objs_config_.eps_angle, objs_config_.min_cluster_size);

            // LOG(INFO) << "No of clusters formed are ee: " << cluster_indices.size();
            return cluster_indices;
        }
        std::vector<pcl::PointIndices> LidarCluster::ClusterIndicesRG(const pcl::PointCloud<pcl::PointXYZ>::Ptr &data_in, std::vector<pcl::PointIndices> cluster_indices)
        {
            // Normal estimation
            pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
            normal_estimator.setSearchMethod(tree);
            normal_estimator.setInputCloud(data_in);
            normal_estimator.setKSearch(50);
            normal_estimator.compute(*normals);

            // Extracting Euclidean clusters using data_in and its normals
            pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
            reg.setMinClusterSize(objs_config_.min_cluster_size);
            reg.setMaxClusterSize(1000000);
            //算法需要k近邻搜索，搜索近邻点的数量限制为30
            reg.setSearchMethod(tree);
            reg.setNumberOfNeighbours(objs_config_.k_search_num);
            reg.setInputCloud(data_in);
            //reg.setIndices (indices);
            reg.setInputNormals(normals);
            //此处两个初始化的值是最关键的，直接影响分割结果的好坏；
            // setSmoothnessThreshold是法线的夹角门限
            // setCurvatureThreshold是曲率门限，对于满足发现夹角的点，如果也满足曲率门限
            //则将会被添加进种子列表内；
            reg.setSmoothnessThreshold(objs_config_.eps_angle);
            reg.setCurvatureThreshold(objs_config_.cuv_angle);
            //下面执行区域生长算法，返回一个簇的数组
            reg.extract(cluster_indices);

            // LOG(INFO) << "No of clusters formed are rg: " << cluster_indices.size();
            return cluster_indices;
        }
        template <typename T>
        int LidarCluster::Objslist(const T& data_in,
                                std::vector<pcl::PointIndices> cluster_indices,
                                lidar_msgs::msg::Objects& objs_list)
        {
        int idx = 0;
        if (cluster_indices.empty()) return 0;

        for (auto& obj : cluster_indices)
        {
            obj_struct obj_tmp; obj_tmp.reset();

            for (int id : obj.indices)
            {
            obj_tmp.zmax = std::max(obj_tmp.zmax, data_in->points[id].z);
            obj_tmp.zmin = std::min(obj_tmp.zmin, data_in->points[id].z);
            obj_tmp.xmax = std::max(obj_tmp.xmax, data_in->points[id].x);
            obj_tmp.xmin = std::min(obj_tmp.xmin, data_in->points[id].x);
            obj_tmp.ymax = std::max(obj_tmp.ymax, data_in->points[id].y);
            obj_tmp.ymin = std::min(obj_tmp.ymin, data_in->points[id].y);
            }

            if ((obj_tmp.zmax + obj_tmp.zmin) / 2 < objs_config_.obj_min_z) continue;

            lidar_msgs::msg::Object o;
            o.idx   = idx++;
            o.rel_x = (obj_tmp.xmax + obj_tmp.xmin) / 2;
            o.rel_y = (obj_tmp.ymax + obj_tmp.ymin) / 2;
            o.rel_z = (obj_tmp.zmax + obj_tmp.zmin) / 2;
            o.length = (obj_tmp.xmax - obj_tmp.xmin);
            o.width  = (obj_tmp.ymax - obj_tmp.ymin);
            o.height = (obj_tmp.zmax - obj_tmp.zmin);
            o.rel_heading = std::atan2(o.rel_y, o.rel_x) * 180 / pi;
            o.time = node_->get_clock()->now().seconds();     // ← 时间戳

            objs_list.objs.push_back(o);
        }
        return 0;
        }
        bool LidarCluster::ReadObjConfig(const std::map<string, string> &m, objs_config &conf)
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
                else if (mite->first == "voxel_size")
                {
                    conf.voxel_size = std::atof(mite->second.c_str());
                }
                else if (mite->first == "in_max_cluster_distance")
                {
                    conf.in_max_cluster_distance = std::atof(mite->second.c_str());
                }
                else if (mite->first == "tolerance")
                {
                    conf.tolerance = std::atof(mite->second.c_str());
                }
                else if (mite->first == "eps_angle")
                {
                    conf.eps_angle = std::atof(mite->second.c_str()) * (M_PI / 180.0);
                }
                else if (mite->first == "min_cluster_size")
                {
                    conf.min_cluster_size = std::atoi(mite->second.c_str());
                }
                else if (mite->first == "k_search_num")
                {
                    conf.k_search_num = std::atoi(mite->second.c_str());
                }
                else if (mite->first == "cuv_angle")
                {
                    conf.cuv_angle = std::atof(mite->second.c_str());
                }
                else if (mite->first == "obj_min_z")
                {
                    conf.obj_min_z = std::atof(mite->second.c_str());
                }
                else if (mite->first == "ground_is")
                {
                    conf.ground_is = std::atoi(mite->second.c_str());
                }
                cout << mite->first << "=" << mite->second << endl;
            }
            return true;
        }
        bool LidarCluster::ReadCellConfigJson(const std::string &path, objs_config &conf)
        {
            ifstream in(path, ios::binary);
            if (!in.is_open())
            {
                // LOG(ERROR) << "Error opening  lidar objs file";
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
                conf.cuv_angle = value["cuv_angle"].asFloat();
                conf.eps_angle = value["eps_angle"].asFloat();
                conf.ground_is = value["ground_is"].asInt();
                conf.in_max_cluster_distance = value["in_max_cluster_distance"].asFloat();
                conf.k_search_num = value["k_search_num"].asInt();
                conf.method = value["method"].asInt();
                conf.min_cluster_size = value["min_cluster_size"].asInt();
                conf.obj_min_z = value["obj_min_z"].asInt();
                conf.tolerance = value["tolerance"].asFloat();
                conf.voxel_size = value["voxel_size"].asFloat();

                conf.points_num = value["points_num"].asInt();
                conf.radius_search = value["radius_search"].asFloat();
                conf.search_num = value["search_num"].asInt();
                conf.threshold_h = value["threshold_h"].asFloat();
            }
            in.close();
            return true;
        }

    }
}
