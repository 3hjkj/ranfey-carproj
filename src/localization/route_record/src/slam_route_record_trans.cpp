// #include <chrono>
// #include<math.h>
// #include <sstream>
// #include <string>
// #include <rclcpp/rclcpp.hpp>

// #include<vector>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <iostream>
// #include <fstream>
// #include <string>
// #include "localization_msgs/msg/localization.hpp"
// #include "localization_msgs/msg/pose_angle.hpp"
// #include "localization_msgs/msg/time.hpp"
// #include "localization_msgs/msg/xyz.hpp"
// #include<tf/transform_datatypes.h>
// #include<can_control_msgs/msg/vehicle_status.hpp>

// using namespace std;

// static vector<double> X_;
// static vector<double> Y_;
// static vector<double> yaw_;
// static vector<double> X_left;
// static vector<double> Y_left;
// static vector<double> speed;
// static vector<double> X_right;
// static vector<double> Y_right;
// static vector<double> yaw_right;
// static double x_last=0;
// static double y_last=0;
// static double lane_dist=2;
// static int id_x=0;
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

// void callback_route_record(const localization_msgs::msg::Localization &msg)
// {
// 	geometry_msgs::PoseStamped pose_msg=msg.salm_pose;
// 	double yaw_vel;
// 	tf::Quaternion quat;
// 	yaw_vel=tf::getYaw(pose_msg.pose.orientation);
// 	if(X_.size()==0 && x_last!=0 && y_last!=0)
// 	{
// 		X_.push_back(x_last);
// 		Y_.push_back(y_last);
// 		yaw_.push_back(yaw_vel);
// 		X_left.push_back(x_last+lane_dist*cos(yaw_vel+M_PI_2));//+是Y轴正方向  -Y轴负方向
// 		Y_left.push_back(y_last+lane_dist*sin(yaw_vel+M_PI_2));
// 		speed.push_back(vehicle_speed);
// 		// X_right.push_back(x_last-lane_dist*cos(yaw_vel+M_PI_2));//+是Y轴正方向  -Y轴负方向
// 		// Y_right.push_back(y_last-lane_dist*sin(yaw_vel+M_PI_2));
// 		// yaw_right.push_back(yaw_vel);
// 		id_x++;
// 		ROS_INFO("ID:[%d],X_: [%f],Y_:[%f],yaw:[%f]",id_x,pose_msg.pose.position.x,pose_msg.pose.position.y,yaw_vel);
// 	}
// 	if(!X_.empty() && sqrt(pow(X_[X_.size()-1]-x_last,2)+pow(Y_[Y_.size()-1]-y_last,2))>0.15 && sqrt(pow(pose_msg.pose.position.x-x_last,2)+pow(pose_msg.pose.position.y-y_last,2))>0.15 )
// 	{
// 		double yaw=azimuthAngle(x_last,y_last,pose_msg.pose.position.x,pose_msg.pose.position.y);
// 		X_.push_back(x_last);
// 		Y_.push_back(y_last);
// 		yaw_.push_back(yaw_vel);
// 		X_left.push_back(x_last+lane_dist*cos(yaw+M_PI_2));
// 		Y_left.push_back(y_last+lane_dist*sin(yaw+M_PI_2));
// 		speed.push_back(vehicle_speed);
// 		// X_right.push_back(x_last-lane_dist*cos(yaw+M_PI_2));
// 		// Y_right.push_back(y_last-lane_dist*sin(yaw+M_PI_2));
// 		// yaw_right.push_back(yaw_vel);
// 		//yaw.push_back(yaw);
// 		id_x++;
// 		ROS_INFO("ID:[%d],X_: [%f],Y_:[%f],yaw:[%f]",id_x,pose_msg.pose.position.x,pose_msg.pose.position.y,yaw);
// 	}
// 	if(sqrt(pow(pose_msg.pose.position.x-x_last,2)+pow(pose_msg.pose.position.y-y_last,2))>0.15)
//     //ROS_INFO("X_: [%f],Y_:[%f],yaw:[%f]",pose_msg.pose.position.x,pose_msg.pose.position.y,yaw);
// 	{	x_last=pose_msg.pose.position.x;
// 		y_last=pose_msg.pose.position.y;
// 	}

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
//     ros::Subscriber Hdsub = nh.subscribe("ndt_pose", 10, callback_route_record);
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
// 	path_center.push_back(X_);
// 	path_center.push_back(Y_);
// 	path_left.push_back(X_left);
// 	path_left.push_back(Y_left);
// 	path_right.push_back(X_right);
// 	path_right.push_back(Y_right);
// // 	double weight_data=0.45
// // double weight_smooth=0.4
// // double tolerance=0.05
// 	smoothPath(path_center,0.45,0.4,0.05);
// 	smoothPath(path_left,0.45,0.4,0.05);
// 	//smoothPath(path_right,0.45,0.4,0.05);
// 	ofstream outfile("global_trace.txt", ios::trunc);
// 	// ofstream outfile("trace.txt", ios::trunc);
//     //outfile<<"real_T C_RoadTraj1Xtable_f32s23__left["<<path_left[0].size()<<"]={";
// 	for (int i = 0; i < path_left[0].size(); i++)
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
// 	for (int it = 0; it < path_left[1].size(); it++)
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
// 	for (int it = 0; it < yaw_.size(); it++)
// 	{
// 	if(it<(yaw_.size()-1))
// 	{
// 		outfile << yaw_[it] <<",";
// 	}
// 	else
// 	{
//             outfile << yaw_[it] <<endl;//"};"<<endl;
// 	}

// 	}
// 	//outfile<<"real_T C_RoadTraj1Xtable_f32s23["<<X_.size()<<"]={";
// 	for (int i = 0; i < path_center[0].size(); i++)
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
// 	for (int it = 0; it < path_center[1].size(); it++)
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
// 	for (int it = 0; it < speed.size(); it++)
// 	{
// 	if(it<(speed.size()-1))
// 	{
// 		outfile << speed[it] <<",";
// 	}
// 	else
// 	{
//             outfile << speed[it] <<endl;//"};"<<endl;
// 	}
// 	}
// 	outfile.close();
//     return 0;
// }

// slam_route_record_trans.cpp ── ROS 2 Humble 版本
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

// TF2：用于计算 yaw
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h> 

class SlamRouteRecordNode : public rclcpp::Node
{
public:
  SlamRouteRecordNode()
  : rclcpp::Node("slam_route_record_node")
  {
    lane_dist_ = declare_parameter<double>("lane_distance", 2.0);

    loc_sub_ = create_subscription<localization_msgs::msg::Localization>(
      "ndt_pose", 10,
      std::bind(&SlamRouteRecordNode::locCallback, this, std::placeholders::_1));

    status_sub_ = create_subscription<can_control_msgs::msg::VehicleStatus>(
      "/vehicle_status", 10,
      std::bind(&SlamRouteRecordNode::statusCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "slam_route_record_trans node started (ROS2 Humble)");
  }

  ~SlamRouteRecordNode() override
  {
    saveFile();                                 // 节点析构时写文件
    RCLCPP_INFO(get_logger(), "trace saved, node exit.");
  }

private:
  /*――――――――――  回调  ――――――――――*/
  void statusCallback(const can_control_msgs::msg::VehicleStatus::SharedPtr msg)
  { vehicle_speed_ = msg->vehicle_spd; }

  void locCallback(const localization_msgs::msg::Localization::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped pose = msg->salm_pose;
    double yaw = tf2::getYaw(pose.pose.orientation);          // 由四元数求 yaw(rad)

    /* 记录第一帧：要求 x_last_/y_last_ 已有值 */
    if (X_.empty() && x_last_ != 0.0 && y_last_ != 0.0) {
      addPoint(x_last_, y_last_, yaw);
      RCLCPP_INFO(get_logger(), "ID:%d  (%.3f, %.3f)", ++id_x_, x_last_, y_last_);
    }

    /* 判断路程增量 */
    double dx = pose.pose.position.x - x_last_;
    double dy = pose.pose.position.y - y_last_;
    if (std::hypot(dx, dy) > 0.15) {
      /* 记录当前 last 点 */
      if (!X_.empty()) {
        double yaw_seg = azimuthAngle(x_last_, y_last_,
                                      pose.pose.position.x, pose.pose.position.y);
        addPoint(x_last_, y_last_, yaw_seg);
        RCLCPP_INFO(get_logger(), "ID:%d  (%.3f, %.3f)", ++id_x_, x_last_, y_last_);
      }
      /* 刷新 last */
      x_last_ = pose.pose.position.x;
      y_last_ = pose.pose.position.y;
    }
  }

  /*―――――――――― 工具函数 ――――――――――*/
  static double azimuthAngle(double x1, double y1, double x2, double y2)
  {
    return std::atan2(y2 - y1, x2 - x1);                      // (-π, π]
  }

  void addPoint(double x, double y, double yaw)
  {
    X_.push_back(x);
    Y_.push_back(y);
    yaw_.push_back(yaw);

    /* 左车道点 */
    X_left_.push_back(x + lane_dist_ * std::cos(yaw + M_PI_2));
    Y_left_.push_back(y + lane_dist_ * std::sin(yaw + M_PI_2));
    speed_.push_back(vehicle_speed_);
  }

  void saveFile()
  {
    if (X_.empty()) {
      RCLCPP_WARN(get_logger(), "no data collected, skip save.");
      return;
    }

    std::ofstream out("global_trace.txt",std::ios::out | std::ios::trunc);
    auto dump = [&out](const std::vector<double>& v) {
      for (size_t i = 0; i < v.size(); ++i) {
        out << v[i];
        if (i + 1 < v.size()) out << ",";
        else out << "\n";
      }
    };

    dump(X_left_);    dump(Y_left_);  dump(yaw_);        // 左侧 / yaw
    dump(X_);         dump(Y_);       dump(speed_);      // 中心 / 速度
  }

  /*―――――――――― 数据成员 ――――――――――*/
  double lane_dist_;
  double vehicle_speed_ {0.0};
  double x_last_ {0.0}, y_last_ {0.0};
  int    id_x_ {0};

  std::vector<double> X_, Y_, yaw_;
  std::vector<double> X_left_, Y_left_, speed_;

  rclcpp::Subscription<localization_msgs::msg::Localization>::SharedPtr loc_sub_;
  rclcpp::Subscription<can_control_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
};

/*================================== main ==================================*/
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamRouteRecordNode>());
  rclcpp::shutdown();
  return 0;
}
