#ifndef LIDAR_OBJS_TYPE_H
#define LIDAR_OBJS_TYPE_H
#include "data_pool.h"
#include "common/tool/read_config.h"
// #include "common/read_config.h"
namespace perception
{
    namespace lidar_objs
    {

        struct objs_config
        {
            int ground_is;
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
            float voxel_size;
            float radius_search;
            int search_num;
            float threshold_h;
            int points_num;

            double in_max_cluster_distance; // setRadiusSearch

            float obj_min_z;
            float tolerance; // 50cm tolerance in (x, y, z) coordinate system
            int k_search_num;
            float cuv_angle;
            double eps_angle; // 5degree tolerance in normals
            unsigned int min_cluster_size;
            unsigned int method; // 1。为区域生长算法，2为欧式聚类
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
                car_ymin = -1.0;
                car_ymax = 1.0;
                cell_size_x = 0.5;
                cell_size_y = 0.5;
                voxel_size = 0.1f;
                in_max_cluster_distance = 0.2;
                tolerance = 0.5f;
                eps_angle = 5 * (M_PI / 180.0); // 区域生长算法的法线夹角， k聚类的
                min_cluster_size = 2;
                method = 1;
                cuv_angle = 3;
                k_search_num = 30;
                obj_min_z = 0.15;
                ground_is = 1;
                threshold_h = 0.1;
                radius_search = 0.8; // 搜索半径
                search_num = 10;
                points_num = 0;
            }
        };
        struct proprecess_config
        {
            // float grid_zmin;
            int points_num;
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
            float voxel_size;
            float threshold_h;
            int vaild;
            float cell_size_x;
            float cell_size_y;
            float radius_search;
            int search_num;
            void reset()
            {
                // grid_zmin = 2;
                points_num = 0;
                zmax = 1.5;
                zmin = -2.0;
                xmin = -20;
                xmax = 30;
                ymin = -25;
                ymax = 25;
                car_xmin = -5.0;
                car_xmax = 0.1;
                car_ymin = -1.0;
                car_ymax = 1.0;
                cell_size_x = 0.5;
                cell_size_y = 0.5;
                voxel_size = 0.1f;
                vaild = 0;
                threshold_h = 0.1;
                radius_search = 0.8; // 搜索半径
                search_num = 10;
            }
        };
        struct obj_struct
        {
            float xmax;
            float xmin;
            float ymax;
            float ymin;
            float zmax;
            float zmin;
            void reset()
            {
                xmax = -100;
                xmin = 100;
                ymax = -20;
                ymin = 20;
                zmax = -10;
                zmin = 10;
            }
        };
        struct grid_obj
        {
            float x;
            float y;
            int point_num;
            int vaild;
            float zmin;
            float zmax;
            int idx;
            void reset()
            {
                x = 100;
                y = 100;
                point_num = 0;
                vaild = 0;
                zmin = 3;
                zmax = -3;
                idx = -1;
            }
        };
    } //lidar_objs
} //perception

#endif