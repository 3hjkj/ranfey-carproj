#include "../include/min_rotate_rect.h"
namespace perception
{
    min_rotate_rect::min_rotate_rect(/* args */)
    {
    }

    min_rotate_rect::~min_rotate_rect()
    {
    }
    vector<double> min_rotate_rect::GetRad(const vector<points> &data_in)
    {
        vector<double> tmp;

        if (data_in.size() < 4)
        {
            std::cout << "points is too low:" << data_in.size() << std::endl;
            return tmp;
        }
        // tmp.emplace_back(0);
        for (int i = 0; i < data_in.size(); i++)
        {
            if (i == 0)
            {
                double x = data_in[data_in.size() - 1].x - data_in[i].x;
                double y = data_in[data_in.size() - 1].y - data_in[i].y;
                double deta = atan(y / (x + EPS)); // +EPS防止x为0
                tmp.emplace_back(deta);
            }
            else
            {
                double x = data_in[i - 1].x - data_in[i].x;
                double y = data_in[i - 1].y - data_in[i].y;
                double deta = atan(y / (x + EPS)); // +EPS防止x为0
                tmp.emplace_back(deta);
            }
        }
        return tmp;
    }
    vector<double> min_rotate_rect::MinRotateRect(const vector<points> &data_in)
    {
        vector<double> final_points;
        vector<double> data_out;
        final_points.clear();
        vector<double> rad = GetRad(data_in);
        // std::cout << "counter:" << counter << std::endl;
        if (rad.empty())
        {
            std::cout << "rad is empty" << std::endl;
            return final_points;
        }
        // 获取最小外接矩形的x_min, x_max,y_min,y_max,rad存在final_points
        // std::cout << "rad size:" << rad.size() << ","
        //           << "data in size:" << data_in.points.size() << std::endl;
        double init_rad = 0;
        final_points = GetArea(data_in, init_rad);
        for (auto r : rad)
        {
            vector<points> rotate_data;
            double rr = -r;
            rotate_data = RotateRect(data_in, rr);
            vector<double> final_points_tmp = GetArea(rotate_data, r);
            if (final_points_tmp[5] < final_points[5])
                final_points = final_points_tmp;
        }
        vector<points> four_points = GetFourPoints(final_points);
        vector<points> final_four_points = RotateRect(four_points, final_points[4]);
        if (0)
        {
            for (int i = 0; i < final_four_points.size(); i++)
            {
                cout << "~~~~~~~" << i << "~~~~~~~~"
                     << "\n"
                     << "x:" << final_four_points[i].x << "\n"
                     << "y:" << final_four_points[i].y << "\n";
            }
        }

        data_out = GetFinalShape(final_four_points);
        counter++;
        return data_out;
    }
    vector<double> min_rotate_rect::GetFinalShape(vector<points> &data_in)
    {
        vector<double> data_out;
        double length = sqrt((data_in[0].x - data_in[2].x) * (data_in[0].x - data_in[2].x) +
                             (data_in[0].y - data_in[2].y) * (data_in[0].y - data_in[2].y));
        double width = sqrt((data_in[0].x - data_in[1].x) * (data_in[0].x - data_in[1].x) +
                            (data_in[0].y - data_in[1].y) * (data_in[0].y - data_in[1].y));
        double height = data_in[0].z;
        double x = (data_in[0].x + data_in[1].x) / 2;
        double y = (data_in[0].y + data_in[2].y) / 2;
        double heading = atan2((data_in[0].y - data_in[2].y), (data_in[0].x - data_in[2].x) + EPS);
        data_out.emplace_back(x);
        data_out.emplace_back(y);
        data_out.emplace_back(length);
        data_out.emplace_back(width);
        data_out.emplace_back(height);
        data_out.emplace_back(heading);
        return data_out;
    }
    // 旋转所有的点
    vector<points> min_rotate_rect::RotateRect(
        const vector<points> &data_in, double &rad)
    {
        vector<points> tmp_out;
        for (auto p : data_in)
        {
            points tmp;
            tmp.x = p.x * cos(rad) - p.y * sin(rad);
            tmp.y = p.x * sin(rad) + p.y * cos(rad);
            tmp.z = p.z;
            tmp_out.emplace_back(tmp);
        }
        if (0)
        {

            fstream outline;
            outline.open(std::to_string(counter) + ".txt", ios::out);
            for (auto pp : tmp_out)
            {
                outline << pp.x << "," << pp.y << "\n";
            }
        }
        // counter++;
        return tmp_out;
    }
    // 得到当前最大最小及面积
    vector<double> min_rotate_rect::GetArea(const vector<points> &data_in, double &rad)
    {
        double x_min = 100;
        double x_max = -100;
        double y_min = 100;
        double y_max = -100;
        double z_min = 100;
        double z_max = -100;
        vector<double> data_out;
        for (auto p : data_in)
        {
            x_min = x_min > p.x ? p.x : x_min;
            x_max = x_max < p.x ? p.x : x_max;
            y_min = y_min > p.y ? p.y : y_min;
            y_max = y_max < p.y ? p.y : y_max;
            z_min = z_min > p.z ? p.z : z_min;
            z_max = z_max < p.z ? p.z : z_max;
        }
        double area_tmp = (x_max - x_min) * (y_max - y_min);
        data_out.emplace_back(x_min);
        data_out.emplace_back(x_max);
        data_out.emplace_back(y_min);
        data_out.emplace_back(y_max);
        data_out.emplace_back(rad);
        data_out.emplace_back(area_tmp);
        data_out.emplace_back(z_min);
        data_out.emplace_back(z_max);
        if (0)
        {
            std::cout << "x_max:" << x_max << ","
                      << "x_min:" << x_min << ","
                      << "y_min:" << y_min << ","
                      << "y_max:" << y_max << ","
                      << "area_tmp:" << area_tmp << ","
                      << std::endl;
        }
        return data_out;
    }
    vector<points> min_rotate_rect::GetFourPoints(const vector<double> final_points)
    {
        //  3    4
        //  ------
        //  |    |
        //  |    |
        //  ------
        //  1    2
        vector<points> final_four_points;
        points xmin_ymin;
        xmin_ymin.x = final_points[0];
        xmin_ymin.y = final_points[2];
        xmin_ymin.z = final_points[7] - final_points[6];
        final_four_points.emplace_back(xmin_ymin);
        points xmax_ymin;
        xmax_ymin.x = final_points[1];
        xmax_ymin.y = final_points[2];
        xmax_ymin.z = final_points[7] - final_points[6];
        final_four_points.emplace_back(xmax_ymin);
        points xmin_ymax;
        xmin_ymax.x = final_points[0];
        xmin_ymax.y = final_points[3];
        xmin_ymax.z = final_points[7] - final_points[6];
        final_four_points.emplace_back(xmin_ymax);
        points xmax_ymax;
        xmax_ymax.x = final_points[1];
        xmax_ymax.y = final_points[3];
        xmax_ymax.z = final_points[7] - final_points[6];
        final_four_points.emplace_back(xmax_ymax);
        return final_four_points;
    }
}