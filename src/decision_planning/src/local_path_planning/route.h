#include<vector>
#include <fstream> 
#include<iostream>
#include<string>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <lidar_msgs/msg/cells.hpp>
#include <lidar_msgs/msg/cell.hpp>
#include<tf/transform_datatypes.h>
#include "ros/ros.h"
#include <can_control_msgs/msg/autocontrol.hpp>
#include <visualization_msgs/MarkerArray.h>
#include<Eigen/Core>
#include <Eigen/Dense>
using namespace std; 
class Local_route
{
private:
    ros::Publisher pub_local_center;
    ros::Publisher pub_local_right;
    ros::Publisher pub_local_left;

    ros::Publisher pub_global_center;
    ros::Publisher pub_global_left;
    ros::Publisher pub_global_right;

	double point_distance=0.3;
	int chang_lane_point=50;
	double lane_distance=2;
	double chang_lane_distance;
    double threshold_y=1.7;//

    int keep_point=10;
    double weight_data=0.45;
	double weight_smooth=0.4;
	double tolerance=0.05;
public:
    Local_route();
    ~Local_route();
        int local_path_size=65;
    void init_pub(ros::NodeHandle n);
    vector<vector<double>> local_path;
    void ReadTxt(string trace_path,vector<vector<double>>&paths);
    void split(string str, string pattern,vector<double>& result);
    void visiual_global_trace(vector<vector<double>>paths);
    void route_plan(vector<vector<double>>paths,   int trace_id,int local_point_id);
    double  azimuthAngle( double  x1,double  y1,double  x2,double  y2);
    void  smoothPath(std::vector<vector<double>>& paths_dect, double weight_data,double weight_smooth, double tolerance);
    bool   generator_local_trace(vector<double>x_orignal,vector<double>y_orignal,vector<double>yaw_orignal,
    vector<double>x_target,vector<double>y_target,vector<double>&local_x,
    vector<double>&local_y,vector<double> &local_yaw,int local_point_id);
    void   debug_show_road_cells(vector<can_control_msgs::msg::Autocontrol>& vec_at,
    lidar_msgs::msg::Cells cells_,int V_RefPoint,double yaw_vel,double x_vel,double y_vel);
    void calculate_parameter();
    //所有路径点

};


