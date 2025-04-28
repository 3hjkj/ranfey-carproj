#ifndef PERCEPTION_LIDAR_DATA_POOL_H_
#define PERCEPTION_LIDAR_DATA_POOL_H_

#include <condition_variable>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <vector>
#include "base_data_pool.h"
#include "data_type.h"
#include "lidar_points_type.h"
// #include "glog/include/logging.h"
namespace perception
{
    // namespace lidar_cells
    // {
    class DataPool
    {
    public:
        MainData *GetMainDataPtr() { return &main_data_; }
        MainData &GetMainDataRef() { return main_data_; }

        // one
        // get lidar points
        void GetLidarCellPointsOne(const pcl_util::PointCloudPtr &points){
            std::unique_lock<std::mutex> locker(points_cell_mutex_one_);
            // main_data_.lidar_points_cells.point_cloud_ptr_map[0] = points;
        }
        // set lidar points
        void SetLidarCellPointsOne(pcl_util::PointCloudPtr &points){
            std::unique_lock<std::mutex> locker(points_cell_mutex_one_);
            main_data_.lidar_points_cells.point_cloud_ptr_map[0] = points;
        }
        // two
        void GetLidarCellPointsTwo(const pcl_util::PointCloudPtr &points){
            std::unique_lock<std::mutex> locker(points_cell_mutex_two_);
            // main_data_.lidar_points_cells.point_cloud_ptr_map[0] = points;
        }
        // set lidar points
        void SetLidarCellPointsTwo(pcl_util::PointCloudPtr &points){
            std::unique_lock<std::mutex> locker(points_cell_mutex_two_);
            main_data_.lidar_points_cells.point_cloud_ptr_map[1] = points;
        }
        // three
        void GetLidarCellPointsThree(const pcl_util::PointCloudPtr &points){
            std::unique_lock<std::mutex> locker(points_cell_mutex_three_);
            // main_data_.lidar_points_cells.point_cloud_ptr_map[0] = points;
        }
        // set lidar points
        void SetLidarCellPointsThree(pcl_util::PointCloudPtr &points){
            std::unique_lock<std::mutex> locker(points_cell_mutex_three_);
            main_data_.lidar_points_cells.point_cloud_ptr_map[2] = points;
        }
        // four
        void GetLidarCellPointsFour(const pcl_util::PointCloudPtr &points){
            std::unique_lock<std::mutex> locker(points_cell_mutex_four_);
            // main_data_.lidar_points_cells.point_cloud_ptr_map[0] = points;
        }
        // set lidar points
        void SetLidarCellPointsFour(pcl_util::PointCloudPtr &points){
            std::unique_lock<std::mutex> locker(points_cell_mutex_four_);
            main_data_.lidar_points_cells.point_cloud_ptr_map[3] = points;
        }
        // five
        void GetLidarCellPointsFive(const pcl_util::PointCloudPtr &points){
            std::unique_lock<std::mutex> locker(points_cell_mutex_five_);
            // main_data_.lidar_points_cells.point_cloud_ptr_map[0] = points;
        }
        // set lidar points
        void SetLidarCellPointsFive(pcl_util::PointCloudPtr &points){
            std::unique_lock<std::mutex> locker(points_cell_mutex_five_);
            main_data_.lidar_points_cells.point_cloud_ptr_map[4] = points;
        }

        // lidar obj

        void GetLidarObjPointsOne(const pcl_util::PointCloudPtr &points){
            std::unique_lock<std::mutex> locker(points_Obj_mutex_one_);
            // main_data_.lidar_points_objs.point_cloud_ptr_map[0] = points;
        }
        // set lidar points
        void SetLidarObjPointsOne(pcl_util::PointCloudPtr &points){
            std::unique_lock<std::mutex> locker(points_Obj_mutex_one_);
            main_data_.lidar_points_objs.point_cloud_ptr_map[0] = points;
        }
        // two
        void GetLidarObjPointsTwo(const pcl_util::PointCloudPtr &points){
            std::unique_lock<std::mutex> locker(points_Obj_mutex_two_);
            // main_data_.lidar_points_objs.point_cloud_ptr_map[0] = points;
        }
        // set lidar points
        void SetLidarObjPointsTwo(pcl_util::PointCloudPtr &points){
            std::unique_lock<std::mutex> locker(points_Obj_mutex_two_);
            main_data_.lidar_points_objs.point_cloud_ptr_map[1] = points;
        }
        // three
        void GetLidarObjPointsThree(const pcl_util::PointCloudPtr &points){
            std::unique_lock<std::mutex> locker(points_Obj_mutex_three_);
            // main_data_.lidar_points_objs.point_cloud_ptr_map[0] = points;
        }
        // set lidar points
        void SetLidarObjPointsThree(pcl_util::PointCloudPtr &points){
            std::unique_lock<std::mutex> locker(points_Obj_mutex_three_);
            main_data_.lidar_points_objs.point_cloud_ptr_map[2] = points;
        }
        // four
        void GetLidarObjPointsFour(const pcl_util::PointCloudPtr &points){
            std::unique_lock<std::mutex> locker(points_Obj_mutex_four_);
            // main_data_.lidar_points_objs.point_cloud_ptr_map[0] = points;
        }
        // set lidar points
        void SetLidarObjPointsFour(pcl_util::PointCloudPtr &points){
            std::unique_lock<std::mutex> locker(points_Obj_mutex_four_);
            main_data_.lidar_points_objs.point_cloud_ptr_map[3] = points;
        }
        // five
        void GetLidarObjPointsFive(const pcl_util::PointCloudPtr &points){
            std::unique_lock<std::mutex> locker(points_Obj_mutex_five_);
            // main_data_.lidar_points_objs.point_cloud_ptr_map[0] = points;
        }
        // set lidar points
        void SetLidarObjPointsFive(pcl_util::PointCloudPtr &points){
            std::unique_lock<std::mutex> locker(points_Obj_mutex_five_);
            main_data_.lidar_points_objs.point_cloud_ptr_map[4] = points;
        }



    private:
        std::mutex points_cell_mutex_one_;
        std::mutex points_cell_mutex_two_;
        std::mutex points_cell_mutex_three_;
        std::mutex points_cell_mutex_four_;
        std::mutex points_cell_mutex_five_;

        std::mutex points_Obj_mutex_one_;
        std::mutex points_Obj_mutex_two_;
        std::mutex points_Obj_mutex_three_;
        std::mutex points_Obj_mutex_four_;
        std::mutex points_Obj_mutex_five_;
    public:
        MainData main_data_; //

        DataPool() {}

        BASE_DECLARE_SINGLETON(DataPool)
    };
    // } // lidar_cell
} //perception
#endif // AVOS_MAPENGINE_DATA_POOL_H_
