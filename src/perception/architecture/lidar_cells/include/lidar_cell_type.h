#ifndef LIDAR_CELLS_TYPE_H
#define LIDAR_CELLS_TYPE_H
#include "data_pool.h"
#include "common/tool/read_config.h"
namespace perception
{
    namespace lidar_cells
    {

        struct cell_config
        {
            float zmax;
            float zmin;
            float xmin;
            float xmax;
            float ymin;
            float ymax;
            float car_xmin;
            float car_xmax;
            float car_ymin;
            float car_ymax;
            float cell_size_x;
            float cell_size_y;
            int method;
            int min_point_num_10;
            int min_point_num_20;
            int min_point_num_30;
            float min_h;
            float density_10;
            float density_20;
            float density_30;
            int dist10;
            int dist20;
            int dist30;
            int preprocess_method;
            int confidence_;
            float voxel_size;
            float radius_search;
            int search_num;
            float threshold_h;
            int points_num;
            void reset()
            {
                zmax = 2.0;
                zmin = -2.0;
                xmin = -20;
                xmax = 80;
                ymin = -15;
                ymax = 15;
                car_xmin = -5.0;
                car_xmax = 0.1;
                car_ymin = -1.5;
                car_ymax = 1.5;
                cell_size_x = 0.5;
                cell_size_y = 0.5;

                method = 2;            // 1 为老方法 2 为引用lidar_obj的预处理 所有点云一起 3 为引用lidar_obj的预处理 单个点云进行
                preprocess_method = 1; // 1-体素栅格滤波+radius+地面分割 2-radius+地面  3-radius
                min_h = 0.1;           // 最小的障碍物高度

                confidence_ = 3;       // confidence下降速度
                min_point_num_10 = 10; // 十米的点数
                min_h = 0.0;
                density_10 = 20;      // density = points_num / 体积
                min_point_num_20 = 5; // 20m的点数
                min_point_num_30 = 0; // 30m的点数
                density_20 = 10;
                density_30 = 5;
                dist10 = 10;
                dist20 = 20;
                dist30 = 30;
                threshold_h = 0.1;
                radius_search = 0.8; // 搜索半径
                search_num = 10;
                points_num = 0;
                voxel_size = 0.1;
            }
        };
        struct data_cell
        {
            int idx = -1;
            float x = -1;
            float y = -1;
            float z_intercept = 0;
            float z_min = 2;
            float z_max = -2;
            float z_mean = 0;
            float x_min = 2;
            float x_max = -2;
            float x_mean = 0;
            float y_min = 2;
            float y_max = -2;
            float y_mean = 0;
            float sum = 0;
            float x_sum = 0;
            float y_sum = 0;
            int vaild = 0;
            int point_num = 0;
            int confidence = 0;
            double time;
            void reset()
            {
                idx = -1;
                x = -1;
                y = -1;
                z_intercept = 0;
                z_min = 2;
                z_max = -2;
                z_mean = 0;
                sum = 0;
                x_sum = 0;
                y_sum = 0;
                vaild = 0;
                point_num = 0;
                confidence = -1;
                time = 0;
                x_min = 2;
                x_max = -2;
                x_mean = 0;
                y_min = 2;
                y_max = -2;
                y_mean = 0;
            }
        };

    } //lidar_cells
} //perception

#endif