// #include <chrono>
// #include<math.h>
// #include <sstream>
// #include <string>
// // #include <ros/ros.h>
// #include<vector>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <iostream>
// #include <fstream>
// #include <string>
// // #include<tf/transform_datatypes.h>
// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "std_msgs/msg/u_int16_multi_array.hpp"
// /**定位**/
// #include "localization_msgs/msg/gps.hpp"
// #include "localization_msgs/msg/localization.hpp"
// #include "localization_msgs/msg/pose_angle.hpp"
// #include "localization_msgs/msg/time.hpp"
// #include "localization_msgs/msg/xyz.hpp"
// #include <sensor_msgs/NavSatFix.h>
// #include<can_control_msgs/msg/vehicle_status.hpp>
// using namespace std;

// static vector<double> X_;
// static vector<double> Y_;
// static vector<double> speed_;
// static vector<double> yaw_;
// static vector<double> yaw_local;
// static vector<double> X_left;
// static vector<double> Y_left;
// static vector<double> speed_left;
// static vector<double> yaw_left;

// static vector<double> X_right;
// static vector<double> Y_right;
// static vector<double> yaw_right;
// static double x_last=0;
// static double y_last=0;
// static double lane_dist=3.0;
// static double vehicle_speed=0;

// // 计算方位角函数
// double  azimuthAngle( double x1,double y1,double x2, double y2)
// {
//     double angle = 0.0;
//     double dx = x2 - x1;
//     double dy = y2 - y1;
//     if(x2 == x1)
//     {
//         angle = M_PI_2 ;
//         if( y2 == y1 )
//             angle = 0.0;
//         else// (y2 < y1 )
//             angle = 3.0 *M_PI_2 ;
//     }
//     else if (x2 > x1 && y2 > y1)  //一象限
//         angle = atan(dy / dx);
//     else if (x2 > x1 && y2 < y1)  //四象限
//         angle = 2*M_PI + atan(dy / dx);
//     else if (x2 < x1 && y2 < y1 )  //三象限
//         angle = M_PI+ atan(dy / dx);
//     else //(x2 < x1 && y2 > y1 )  //二象限
//         angle = M_PI + atan(dy / dx);
//     return angle;
// }
// static int allow_rewrite=1;
// void callback_route_record(const localization_msgs::msg::Localization &loc)
// {
// 	double yaw_vel;

// 	yaw_vel=loc.xy.z/180*M_PI;
// 	// if(X_.size()==0  )
// 	// {
// 	// 	X_.push_back(loc.plane_xyz.x);
// 	// 	Y_.push_back(loc.plane_xyz.y);
		
// 	// 	// speed_.push_back(loc.abs_speed);
// 	// 	// yaw_.push_back(yaw_vel);
// 	// 	// X_left.push_back(x_last+lane_dist*cos(yaw_vel+M_PI_2));//+是Y轴正方向  -Y轴负方向
// 	// 	// Y_left.push_back(y_last+lane_dist*sin(yaw_vel+M_PI_2));
// 	// 	// speed_left.push_back(loc.abs_speed);
// 	// 	// yaw_left.push_back(yaw_vel);
// 	// 	// X_right.push_back(x_last-lane_dist*cos(yaw_vel+M_PI_2));//+是Y轴正方向  -Y轴负方向
// 	// 	// Y_right.push_back(y_last-lane_dist*sin(yaw_vel+M_PI_2));
// 	// 	// yaw_right.push_back(yaw_vel);
// 	// 	ROS_INFO("X_: [%f],Y_:[%f],yaw:[%f]",loc.plane_xyz.x,loc.plane_xyz.y,yaw_vel);
// 	// }
// 	if(X_.size()==0)
// 	{
// 		X_.push_back(loc.xy.x);
// 		Y_.push_back(loc.xy.y);
// 		yaw_local.push_back(yaw_vel);
// 		speed_.push_back(vehicle_speed);
// 		speed_left.push_back(vehicle_speed);
// 		ROS_INFO("X_: [%f],Y_:[%f]",loc.xy.x,loc.xy.y);
// 	}
// 	else if(sqrt(pow(X_[X_.size()-1]-loc.xy.x,2)+pow(Y_[Y_.size()-1]-loc.xy.y,2))>0.3 )//&& sqrt(pow(loc.plane_xyz.x-x_last,2)+pow(loc.plane_xyz.y-y_last,2))>0.3  && x_last!=0)
// 	{
// 		double yaw=azimuthAngle(X_[X_.size()-1],Y_[Y_.size()-1],loc.xy.x,loc.xy.y);
// 		X_.push_back(loc.xy.x);
// 		Y_.push_back(loc.xy.y);
// 		speed_.push_back(vehicle_speed);
// 		yaw_.push_back(yaw);
// 		// X_left.push_back(x_last+lane_dist*cos(yaw+M_PI_2));
// 		// Y_left.push_back(y_last+lane_dist*sin(yaw+M_PI_2));
// 		speed_left.push_back(vehicle_speed);
// 		yaw_left.push_back(yaw);
// 		yaw_local.push_back(yaw_vel);
// 		// X_right.push_back(x_last-lane_dist*cos(yaw+M_PI_2));
// 		// Y_right.push_back(y_last-lane_dist*sin(yaw+M_PI_2));
// 		// yaw_right.push_back(yaw);
// 		// x_last=loc.plane_xyz.x;
// 		// y_last=loc.plane_xyz.y;
// 		// //yaw.push_back(yaw);
// 		ROS_INFO("X_: [%f],Y_:[%f],yaw:[%f]",loc.xy.x,loc.xy.y,yaw);
// 		// allow_rewrite=1;
// 	}

// 	// if(sqrt(pow(loc.plane_xyz.x-X_[X_.size()-1],2)+pow(loc.plane_xyz.y-Y_[Y_.size()-1],2))>0.3 && allow_rewrite==1)
// 	// {	
// 	// 	x_last=loc.plane_xyz.x;
// 	// 	y_last=loc.plane_xyz.y;
// 	// 	allow_rewrite=0;
// 	// }

// }

// void smoothPath(std::vector<vector<double>>& paths_dect, double weight_data,
//     double weight_smooth, double tolerance)
// {
// 	for(int ix=0;ix<int(paths_dect.size()/2);ix++)
// 	{
// 		std::vector<vector<double>> path_in;
// 		path_in.assign(paths_dect.begin()+(ix*2),paths_dect.begin()+(ix*2)+2);
// 		cout<<__LINE__<<";"<<path_in.size();
// 		std::vector<vector<double>> smoothPath_out = path_in;

// 		double change = tolerance;
// 		double xtemp, ytemp;
// 		int nIterations = 0;

// 		int size = paths_dect[ix].size();

// 		while (change >= tolerance) {
// 			change = 0.0;
// 			for (int i = 1; i < size - 1; i++) {
// 				xtemp = smoothPath_out[0][i];
// 				ytemp = smoothPath_out[1][i];

// 				smoothPath_out[0][i] += weight_data * (path_in[0][i] - smoothPath_out[0][i]);
// 				smoothPath_out[1][i] += weight_data * (path_in[1][i]  - smoothPath_out[1][i]);

// 				smoothPath_out[0][i] += weight_smooth * (smoothPath_out[0][i - 1] + smoothPath_out[0][i + 1] - (2.0 * smoothPath_out[0][i]));
// 				smoothPath_out[1][i] += weight_smooth * (smoothPath_out[1][i - 1] + smoothPath_out[1][i + 1] - (2.0 * smoothPath_out[1][i]));

// 				change += fabs(xtemp - smoothPath_out[0][i]);
// 				change += fabs(ytemp - smoothPath_out[1][i]);
// 			}
// 			nIterations++;
// 		}
// 		paths_dect[ix] = smoothPath_out[0];
// 		paths_dect[ix+1] = smoothPath_out[0+1];
// 		// smoothPath_out.clear();
// 		// path_in.clear();
// 	}
// }
// void callback_vehicle_status(const can_control_msgs::msg::VehicleStatus &msg)
// {
// 	vehicle_speed=msg.vehicle_spd;

// }
// int main(int argc,char** argv){
//     // X_.push_back(0);
//     // Y_.push_back(0);

//     ros::init(argc, argv, "route_record_node");
//     ros::NodeHandle nh;
//     ros::Subscriber Hdsub = nh.subscribe("/localization", 10, callback_route_record);
// 	ros::Subscriber vehicle_sub = nh.subscribe("/vehicle_status", 10, callback_vehicle_status);
//     vector<vector<double>>path_center;
// 	vector<vector<double>>path_left;
// 	vector<vector<double>>path_right;
//     while (ros::ok())
//     {
//         // ROS_INFO("X_.size() :[%d]",X_.size());
//     	// ROS_INFO("Y_.size() :[%d]",Y_.size());
// 		// ROS_INFO("X_left.size() :[%d]",X_left.size());
//     	// ROS_INFO("Y_left.size() :[%d]",Y_left.size());
//        ros::spinOnce();
//     }

// 	X_.pop_back();
// 	Y_.pop_back();
// 	speed_.pop_back();
// 	speed_left.pop_back();
// 	path_center.push_back(X_);
// 	path_center.push_back(Y_);
// 	for(int i=0;i<X_.size();i++)
// 	{
// 		X_left.push_back(X_[i]+lane_dist*cos(yaw_[i]+M_PI_2));
// 		Y_left.push_back(Y_[i]+lane_dist*sin(yaw_[i]+M_PI_2));

// 	}
// 	path_left.push_back(X_left);
// 	path_left.push_back(Y_left);
// 	//path_right.push_back(X_right);
// 	//path_right.push_back(Y_right);
	
// // 	double weight_data=0.45
// // double weight_smooth=0.4
// // double tolerance=0.05
// 	// smoothPath(path_center,0.45,0.4,0.05);
// 	// smoothPath(path_left,0.45,0.4,0.05);

// 	//smoothPath(path_right,0.45,0.4,0.05);
// 	ofstream outfile("global_trace.txt", ios::trunc);


//     //outfile<<"real_T C_RoadTraj1Xtable_f32s23__left["<<path_left[0].size()<<"]={";
// 	for (int i = 1; i < path_left[0].size(); i++)
// 	{

// 	if(i<(path_left[0].size()-1))
// 	{
// 	    outfile << path_left[0][i] <<",";
// 	}
// 	else
// 	{
	    
//             outfile << path_left[0][i] <<endl;//"};"<<endl;
// 	}
// 	}
// 	//outfile<<"real_T C_RoadTraj1Ytable_f32s23__left["<<path_left[1].size()<<"]={";
// 	for (int it = 1; it < path_left[1].size(); it++)
// 	{
// 	if(it<(path_left[1].size()-1))
// 	{
// 		outfile << path_left[1][it] <<",";
// 	}

// 	else
// 	{
//             outfile << path_left[1][it] <<endl;//"};"<<endl;
// 	}

// 	}


// 	for (int it = 1; it < yaw_left.size(); it++)
// 	{
// 	if(it<(yaw_left.size()-1))
// 	{
// 		outfile << yaw_left[it] <<",";
// 	}
// 	else
// 	{
//             outfile << yaw_left[it] <<endl;//"};"<<endl;
// 	}

// 	}

// 	// for (int it = 0; it < speed_left.size(); it++)
// 	// {
// 	// if(it<(speed_left.size()-1))
// 	// {
// 	// 	outfile << speed_left[it] <<",";
// 	// }
// 	// else
// 	// {
//     //         outfile << speed_left[it] <<endl;//"};"<<endl;
// 	// }

// 	// }
// 	//outfile<<"real_T C_RoadTraj1Xtable_f32s23["<<X_.size()<<"]={";
// 	for (int i = 1; i < path_center[0].size(); i++)
// 	{

// 	if(i<(path_center[0].size()-1))
// 	{
// 	    outfile << path_center[0][i] <<",";
// 	}
// 	else
// 	{
//             outfile << path_center[0][i] <<endl;//"};"<<endl;
// 	}
// 	}
// 	//outfile<<"real_T C_RoadTraj1Ytable_f32s23["<<path_center[1].size()<<"]={";
// 	for (int it = 1; it < path_center[1].size(); it++)
// 	{
// 	if(it<(path_center[1].size()-1))
// 	{
// 		outfile << path_center[1][it] <<",";
// 	}
// 	else
// 	{
//             outfile << path_center[1][it] <<endl;//"};"<<endl;
// 	}
// 	}
// 	// for (int it = 0; it < yaw_.size(); it++)
// 	// {
// 	// if(it<(yaw_.size()-1))
// 	// {
// 	// 	outfile << yaw_[it] <<",";
// 	// }
// 	// else
// 	// {
//     //         outfile << yaw_[it] <<endl;//"};"<<endl;
// 	// }

// 	// }
//     for (int it = 1; it < speed_.size(); it++)
// 	{
// 	if(it<(speed_.size()-1))
// 	{
// 		outfile << speed_[it] <<",";
// 	}
// 	else
// 	{
//             outfile << speed_[it] <<endl;//"};"<<endl;
// 	}

// 	}

// 	for (int it = 1; it < yaw_local.size(); it++)
// 	{
// 	if(it<(yaw_local.size()-1))
// 	{
// 		outfile << yaw_local[it] <<",";
// 	}
// 	else
// 	{
//             outfile << yaw_local[it] <<endl;//"};"<<endl;
// 	}

// 	}
// /*
// 	    //outfile<<"real_T C_RoadTraj1Xtable_f32s23__right["<<path_right[0].size()<<"]={";
// 	for (int i = 0; i < path_right[0].size(); i++)
// 	{

// 	if(i<(path_right[0].size()-1))
// 	{
// 	    outfile << path_right[0][i] <<",";
// 	}
// 	else
// 	{
	    
//             outfile << path_right[0][i] <<endl;//"};"<<endl;
// 	}
// 	}
// 	//outfile<<"real_T C_RoadTraj1Ytable_f32s23__right["<<path_right[1].size()<<"]={";
// 	for (int it = 0; it < path_right[1].size(); it++)
// 	{
// 	if(it<(path_right[1].size()-1))
// 	{
// 		outfile << path_right[1][it] <<",";
// 	}

// 	else
// 	{
//             outfile << path_right[1][it] <<endl;//"};"<<endl;
// 	}

// 	}

// 	for (int it = 0; it < yaw_right.size(); it++)
// 	{
// 	if(it<(yaw_right.size()-1))
// 	{
// 		outfile << yaw_right[it] <<",";
// 	}
// 	else
// 	{
//             outfile << yaw_right[it] <<endl;//"};"<<endl;
// 	}

// 	}
// 	*/

// 	outfile.close();
    
//     return 0;
// }
// gps_route_record_trans.cpp  ── ROS2 Humble 版
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "localization_msgs/msg/localization.hpp"
#include "can_control_msgs/msg/vehicle_status.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RouteRecordNode : public rclcpp::Node
{
public:
  RouteRecordNode()
  : rclcpp::Node("route_record_node")
  {
    // 参数（如有需要可暴露为 ROS2 参数）
    lane_dist_ = this->declare_parameter<double>("lane_distance", 3.0);

    // 订阅定位与车辆状态
    loc_sub_ = this->create_subscription<localization_msgs::msg::Localization>(
      "/localization", 10, std::bind(&RouteRecordNode::locCallback, this, _1));

    status_sub_ = this->create_subscription<can_control_msgs::msg::VehicleStatus>(
      "/vehicle_status", 10, std::bind(&RouteRecordNode::statusCallback, this, _1));

    RCLCPP_INFO(get_logger(), "gps_route_record_trans node started (ROS2 Humble)");
  }

  ~RouteRecordNode() override
  {
    // 节点析构时写出结果文件
    writeFile();
    RCLCPP_INFO(get_logger(), "Trace file saved, node exiting.");
  }

private:
  // === 数据结构 ===
  std::vector<double> X_, Y_, speed_, yaw_;
  std::vector<double> yaw_local;
  std::vector<double> X_left, Y_left, speed_left, yaw_left;
  std::vector<double> X_right, Y_right, yaw_right;
  double x_last_ {0.0};
  double y_last_ {0.0};
  double lane_dist_;
  double vehicle_speed_ {0.0};

  // === 订阅者 ===
  rclcpp::Subscription<localization_msgs::msg::Localization>::SharedPtr loc_sub_;
  rclcpp::Subscription<can_control_msgs::msg::VehicleStatus>::SharedPtr status_sub_;

  // === 回调 ===
  void statusCallback(const can_control_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    vehicle_speed_ = msg->vehicle_spd;
  }

  void locCallback(const localization_msgs::msg::Localization::SharedPtr loc)
  {
    double yaw_vel = loc->xy.z * M_PI / 180.0;                // deg → rad

    if (X_.empty()) {
      // 第一个点
      X_.push_back(loc->xy.x);
      Y_.push_back(loc->xy.y);
      yaw_local.push_back(yaw_vel);
      speed_.push_back(vehicle_speed_);
      speed_left.push_back(vehicle_speed_);
      RCLCPP_INFO(get_logger(), "Start point: (%.3f, %.3f)",
                  loc->xy.x, loc->xy.y);
      return;
    }

    double dx = X_.back() - loc->xy.x;
    double dy = Y_.back() - loc->xy.y;
    if (std::sqrt(dx * dx + dy * dy) > 0.3) {                  // 超过 0.3 m 记录
      double yaw = azimuthAngle(X_.back(), Y_.back(), loc->xy.x, loc->xy.y);
      X_.push_back(loc->xy.x);
      Y_.push_back(loc->xy.y);
      speed_.push_back(vehicle_speed_);
      yaw_.push_back(yaw);
      speed_left.push_back(vehicle_speed_);
      yaw_left.push_back(yaw);
      yaw_local.push_back(yaw_vel);
      RCLCPP_INFO(get_logger(), "Record (%.3f, %.3f) yaw=%.3f°",
                  loc->xy.x, loc->xy.y, yaw * 180.0 / M_PI);
    }
  }

  // === 工具函数 ===
  static double azimuthAngle(double x1, double y1, double x2, double y2)
  {
    double dx = x2 - x1;
    double dy = y2 - y1;
    if (dx == 0.0) {
      return (dy >= 0.0) ? M_PI_2 : 3.0 * M_PI_2;
    }
    double ang = std::atan2(dy, dx);
    return (ang >= 0.0) ? ang : ang + 2 * M_PI;                // 0 ~ 2π
  }

  void writeFile()
  {
    if (X_.size() < 2) {
      RCLCPP_WARN(get_logger(), "No data collected, skip writing file.");
      return;
    }

    // 生成中心/左车道坐标
    std::vector<std::vector<double>> path_center { X_, Y_ };
    for (size_t i = 0; i < X_.size(); ++i) {
      X_left.push_back(X_[i] + lane_dist_ * std::cos(yaw_[i] + M_PI_2));
      Y_left.push_back(Y_[i] + lane_dist_ * std::sin(yaw_[i] + M_PI_2));
    }
    std::vector<std::vector<double>> path_left { X_left, Y_left };

    std::ofstream outfile("global_trace.txt", std::ios::out | std::ios::trunc);
    auto dump_vec = [&outfile](const std::vector<double>& v) {
      for (size_t i = 0; i < v.size(); ++i) {
        outfile << v[i];
        if (i + 1 < v.size()) outfile << ",";
        else outfile << "\n";
      }
    };

    dump_vec(path_left[0]);
    dump_vec(path_left[1]);
    dump_vec(yaw_left);
    dump_vec(path_center[0]);
    dump_vec(path_center[1]);
    dump_vec(speed_);
    dump_vec(yaw_local);
    outfile.close();
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RouteRecordNode>());
  rclcpp::shutdown();
  return 0;
}
