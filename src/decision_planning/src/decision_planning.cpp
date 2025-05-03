#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <sys/time.h>
#include <math.h>
#include <vector>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"c
#include "decision_planning_msgs/msg/decision_planning.hpp"
#include "can_control_msgs/msg/autocontrol.hpp"
/**定位**/
#include "localization_msgs/msg/gps.hpp"
#include "localization_msgs/msg/localization.hpp"
#include "localization_msgs/msg/pose_angle.hpp"
#include "localization_msgs/msg/time.hpp"
#include "localization_msgs/msg/xyz.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "mutex"
#include "route.h"
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>
#include<math.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_cloud.h>
// #include <pcl/common/transforms.h>
using namespace std;
//radar
#include"radar_msgs/msg/sensor_data.hpp"
static bool radar_driving=false;

// 所有路径点
static vector<vector<double>> paths;
static Local_route local_route;
int trace_id = 1;
static int change_lane_vfpoint = 0;
static int biandaochushi = 0;
std::mutex m_trace;
std::mutex m_local;

// gps和slam定位标志
static bool gps_loc = false;
static bool slam_loc = false;
// 定位点
static double gps_loc_x; // gps 定位
static double gps_loc_y;
static double gps_loc_yaw;
static double gps_loc_pitch;
static double slam_loc_x; // slam 定位
static double slam_loc_y;
static double slam_loc_yaw;
static double slam_loc_pitch;
static double loc_x; // 车辆定位
static double loc_y;
static double loc_yaw;

static int V_RefPoint;
static int old_respoint = 1;
static bool traffic_ligt_driving = false; // 走，停

static double slam_start_x = 0;
static double slam_start_y = 0;
static double slam_start_yaw = 0;
static bool forbiden_change_lane = true;

// 话题发布
rclcpp::Publisher<decision_planning_msgs::msg::DecisionPlanning>::SharedPtr planning_pub;
rclcpp::Publisher<can_control_msgs::msg::Autocontrol>::SharedPtr            fusion_pub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr               local_path_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                         traffic_obu;
// 路径读取
vector<double> C_RoadTraj1Xtable_f32s23;
vector<double> C_RoadTraj1Ytable_f32s23;
vector<double> C_Road_yaw;
vector<double> C_Local_yaw;
vector<double> C_TrajeSpd;
vector<int> road_label;









void radar_callback(const radar_msgs::msg::SensorData& radar_msg)
{
	//ROS_ERROR("+++++++===");
	cout<<"x="<<radar_msg.x<<"y="<<radar_msg.y<<endl;
	cout<<"x_v"<<radar_msg.vx<<"y_v"<<radar_msg.vy<<endl;
	
	if(radar_msg.x<12&&abs(radar_msg.vy)>0.3)
	{
	
		
		radar_driving=true;
		cout<<"vy="<<radar_msg.vy<<endl;
	
	}
	else
	{
		radar_driving=false;
	}

}
void lidar_callback(const lidar_msgs::msg::Cells::ConstPtr &msg)
{
	cout << "lidar callback info" << endl;
	vector<double> x;
	vector<double> y;
	// cout<<"xxxxxx"<<endl;
	vector<can_control_msgs::msg::Autocontrol> msg_Lidar_self;
	vector<double> x_temporary;
	vector<double> y_temporary;
	vector<double> yaw_;
	vector<vector<double>> l_path;
	vector<vector<double>> back_path;
	vector<double> back_x;
	vector<double> back_y;
	vector<double> back_yaw;
	cout<<"dian:"<<paths[5].size();
	cout << "V_RefPoint:" << V_RefPoint << endl;
	local_route.route_plan(paths, trace_id, V_RefPoint);
	l_path = local_route.local_path;
	// if(trace_id==0  ){
	// 	if(V_RefPoint>30){
	// 		  for(int i=0;i<30;i++){
	// 			back_x.push_back(paths[3][10+V_RefPoint-i]);
	// 			back_y.push_back(paths[4][10+V_RefPoint-i]);
	// 			back_yaw.push_back(paths[2][10+V_RefPoint-i]);
	// 		}
	// 		}
	// 	else
	// 	{
	// 		 for(int i=0;i<30;i++){
	// 			back_x.push_back(paths[3][10+V_RefPoint-i]);
	// 			back_y.push_back(paths[4][10+V_RefPoint-i]);
	// 			back_yaw.push_back(paths[2][10+V_RefPoint-i]);
	// 		}
	// 	}
	// }
	// else if(trace_id==1)
	// {
	// 	if(V_RefPoint>30){
	// 		  for(int i=0;i<30;i++){
	// 			back_x.push_back(paths[0][V_RefPoint-i]);
	// 			back_y.push_back(paths[1][V_RefPoint-i]);
	// 			back_yaw.push_back(paths[2][V_RefPoint-i]);
	// 		}
	// 		}
	// 	else
	// 	{
	// 		 for(int i=0;i<30;i++){
	// 			back_x.push_back(paths[0][V_RefPoint-i]);
	// 			back_y.push_back(paths[1][V_RefPoint-i]);
	// 			back_yaw.push_back(paths[2][V_RefPoint-i]);
	// 		}
	// 	}
	// }
	// local_route.local_path.push_back(back_x);
	// local_route.local_path.push_back(back_y);
	// local_route.local_path.push_back(back_yaw);

	// 大地坐标西下  车辆航向，坐标X，坐标Y
	//  GPS坐标系下航向角度与Y(正北)夹角，角度方向顺势针旋转与车辆坐标系相反。
	double yaw_vel = loc_yaw;
	double x_vel = loc_x;
	double y_vel = loc_y;
	cout << "V_RefPoint:" << V_RefPoint << endl;
	local_route.debug_show_road_cells(msg_Lidar_self, *msg, V_RefPoint, yaw_vel, x_vel, y_vel);
	cout << msg_Lidar_self[trace_id].front_l_long_obj << endl;
	cout << "change_lane_vfpoint:" << change_lane_vfpoint << endl;
	cout << "V_RefPoint:" << V_RefPoint << endl;
	// ROS_ERROR("======");
	// cout << "路径:" << msg_Lidar_self[1].front_l_long_obj << "米内有障碍物" << endl;
	// cout << "路径:" << msg_Lidar_self.size()<<endl;
	// ROS_ERROR("======");

	// cout<<"target_back_obj_distance:"<<msg_Lidar_self[2].target_back_obj_distance<<endl;
	if ((3 < msg_Lidar_self[trace_id].front_l_long_obj && msg_Lidar_self[trace_id].front_l_long_obj < 10))
	{
		cout << "路径:" << msg_Lidar_self[trace_id].front_l_long_obj << "米内有障碍物" << endl;
		cout << "路径:" << msg_Lidar_self[trace_id].front_l_long_obj << "米内有障碍物" << endl;
		cout << "路径:" << msg_Lidar_self[trace_id].front_l_long_obj << "米内有障碍物" << endl;
		cout << "路径:" << msg_Lidar_self[trace_id].front_l_long_obj << "米内有障碍物" << endl;
		cout << "路径:" << msg_Lidar_self[trace_id].front_l_long_obj << "米内有障碍物" << endl;
	}
	// else
	// {
	// 	// if((3400<V_RefPoint && V_RefPoint<3700)||((1170<V_RefPoint && V_RefPoint<1500)))
	// 	if (20 < V_RefPoint && V_RefPoint < 500)
	// 	{
	// 		cout << "路径---------------------:" << msg_Lidar_self[trace_id].front_l_long_obj << " 米内有障碍物" << endl;
	// 		for (int i = 0; i < msg_Lidar_self.size(); i++)
	// 		{
	// 			cout << "track_id：" << trace_id << endl;

	// 			if (V_RefPoint + l_path[i * 3].size() < C_RoadTraj1Xtable_f32s23.size() &&  ((msg_Lidar_self[trace_id].front_l_long_obj > 3) && msg_Lidar_self[trace_id].front_l_long_obj < 12)
	// 			&& (old_respoint != V_RefPoint))
	// 			{
	// 				if  (msg_Lidar_self[i].front_l_long_obj < 0.0001)
	// 				{	//if(msg_Lidar_self[0].front_l_long_obj > 0.001)
	// 						//break;

	// 					cout<<"mmmmm"<<endl;
	// 					//ROS_ERROR("CHANGE_LANE_START---------------");
	// 					cout << "msg_Lidar_self[i].front_l_long_obj : " << msg_Lidar_self[i].front_l_long_obj <<endl;
	// 					if (abs(trace_id - i) < 2)
	// 					{
	// 						cout << "start .............................................." << endl;
	// 						cout << "V_RefPoint:" << V_RefPoint << endl;
	// 						x_temporary.insert(x_temporary.end(), C_RoadTraj1Xtable_f32s23.begin(), C_RoadTraj1Xtable_f32s23.begin() + V_RefPoint);
	// 						cout << "l_path[i*3].size():" << l_path[i * 3].size() << endl;
	// 						x_temporary.insert(x_temporary.end(), l_path[i * 3].begin(), l_path[i * 3].end());
	// 						x_temporary.insert(x_temporary.end(), paths[i * 3].begin() + local_route.local_path_size + V_RefPoint, paths[i * 3].end());
	// 						y_temporary.insert(y_temporary.end(), C_RoadTraj1Ytable_f32s23.begin(), C_RoadTraj1Ytable_f32s23.begin() + V_RefPoint);
	// 						y_temporary.insert(y_temporary.end(), l_path[i * 3 + 1].begin(), l_path[i * 3 + 1].end());
	// 						y_temporary.insert(y_temporary.end(), paths[i * 3 + 1].begin() + local_route.local_path_size + V_RefPoint, paths[i * 3 + 1].end());
	// 						yaw_.insert(yaw_.end(), C_Road_yaw.begin(), C_Road_yaw.begin() + V_RefPoint);
	// 						yaw_.insert(yaw_.end(), l_path[i * 3 + 2].begin(), l_path[i * 3 + 2].end());
	// 						yaw_.insert(yaw_.end(), paths[i * 3 + 2].begin() + local_route.local_path_size + V_RefPoint, paths[i * 3 + 2].end());
	// 						m_trace.lock();
	// 						C_RoadTraj1Xtable_f32s23 = x_temporary;
	// 						C_RoadTraj1Ytable_f32s23 = y_temporary;
	// 						C_Road_yaw = yaw_;
	// 						x_temporary.clear();
	// 						y_temporary.clear();
	// 						yaw_.clear();
	// 						int old_trace_id = trace_id;
	// 						trace_id = i;
	// 						m_trace.unlock();
	// 						// if (biandaochushi == 0)
	// 						// {
	// 						// 	change_lane_vfpoint = V_RefPoint;
	// 						// 	biandaochushi += 1;
	// 						// }
	// 						change_lane_vfpoint = V_RefPoint;
	// 						break;
	// 					}
	// 				}
	// 			}
	// 			else if (//V_RefPoint + l_path[i * 3].size() < C_RoadTraj1Xtable_f32s23.size() && 0 == msg_Lidar_self[1].front_l_long_obj &&
	// 			!((msg_Lidar_self[1].front_l_long_obj > 7) && msg_Lidar_self[1].front_l_long_obj < 12) &&
	// 			V_RefPoint - change_lane_vfpoint > 18 &&
	// 			change_lane_vfpoint > 0)
	// 			{ //切换的路径走完
	// 				cout<<"nnnnnn"<<endl;
	// 				ROS_ERROR("LANE_RETURN_BEGIM~~~~~~~~~~~~~~~");
	// 				x.insert(x.end(), C_RoadTraj1Xtable_f32s23.begin(), C_RoadTraj1Xtable_f32s23.begin() + V_RefPoint);
	// 				x.insert(x.end(), l_path[3].begin(), l_path[3].end());
	// 				x.insert(x.end(), paths[3].begin() + local_route.local_path_size + V_RefPoint, paths[3].end());
	// 				y.insert(y.end(), C_RoadTraj1Ytable_f32s23.begin(), C_RoadTraj1Ytable_f32s23.begin() + V_RefPoint);
	// 				y.insert(y.end(), l_path[3 + 1].begin(), l_path[3 + 1].end());
	// 				y.insert(y.end(), paths[3 + 1].begin() + local_route.local_path_size + V_RefPoint, paths[3 + 1].end());
	// 				yaw_.insert(yaw_.end(), C_Road_yaw.begin(), C_Road_yaw.begin() + V_RefPoint);
	// 				yaw_.insert(yaw_.end(), l_path[3 + 2].begin(), l_path[3 + 2].end());
	// 				yaw_.insert(yaw_.end(), paths[3 + 2].begin() + local_route.local_path_size + V_RefPoint, paths[3 + 2].end());
	// 				m_trace.lock();
	// 				C_RoadTraj1Xtable_f32s23 = x;
	// 				C_RoadTraj1Ytable_f32s23 = y;
	// 				C_Road_yaw = yaw_;
	// 				x.clear();
	// 				y.clear();
	// 				trace_id = 1;
	// 				m_trace.unlock();
	// 				change_lane_vfpoint = 0;
	// 				break;
	// 			}
	// 		}
	// 	}
	// }

	old_respoint = V_RefPoint;
	if(true)
	{
		std_msgs::String obu_msg;
		obu_msg.data="traffic_obu";
		traffic_obu->publish(obu_msg);

	}

	fusion_pub->publish(msg_Lidar_self[trace_id]);
}

void fusion_callback(const can_control_msgs::msg::Autocontrol &msg)
{
	cout << "ok" << endl;
}
static int light_cont = 0;
void camera_callback(const can_control_msgs::msg::Autocontrol &msg)
{
	if (road_label[V_RefPoint] == 0 && (msg.left_traffic_light == 1 || msg.left_traffic_light == 2))
	{ // 左转 红灯或在黄灯
		traffic_ligt_driving = true;
		light_cont = 30;
	}
	else if (road_label[V_RefPoint] == 1 && (msg.middle_traffic_light == 1 || msg.middle_traffic_light == 2))
	{ // 左转 红灯或在黄灯
		traffic_ligt_driving = true;
		light_cont = 30;
	}
	else if (road_label[V_RefPoint] == 2 && (msg.right_traffic_light == 1 || msg.right_traffic_light == 2))
	{ // 左转 红灯或在黄灯
		traffic_ligt_driving = true;
		light_cont = 30;
	}
	else if (light_cont == 0)
	{
		traffic_ligt_driving = false;
	}
	light_cont--;
}
void loc_callback(const localization_msgs::msg::Localization &msg)
{

	m_local.lock();
	if (msg.satellite_status == 4 || msg.satellite_status == 2)
	{
		gps_loc = true;
		// cout<<msg.xy.x<<"  " <<msg.xy.y<<endl;
		gps_loc_x = msg.xy.x;
		gps_loc_y = msg.xy.y;
		gps_loc_yaw = msg.xy.z / 180 * M_PI; // 航向角
		gps_loc_pitch = msg.pose.pitch;
	}
	else
	{
		gps_loc = false;
	}
	m_local.unlock();
}
void loc_slam_callback(const localization_msgs::msg::Localization &msg)
{
	m_local.lock();
	if (msg.slam_loc_ok)
	{
		tf2::Quaternion quat;
        tf2::fromMsg(msg.salm_pose.pose.orientation, quat);   // ROS2 tf2 用法
		Eigen::Matrix3d own_utm_rotation;
		double roll, pitch, yaw;					  // 定义存储r\p\y的容器
		tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		own_utm_rotation = Eigen::AngleAxisd(-slam_start_yaw + M_PI_2, Eigen::Vector3d::UnitZ()) *
						   Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
						   Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
		Eigen::Vector3d cell_pos(msg.salm_pose.pose.position.x, msg.salm_pose.pose.position.y, 0);
		Eigen::Vector3d vhicle_pos(slam_start_x, slam_start_y, 0);
		auto cell_utm = own_utm_rotation * cell_pos + vhicle_pos;
		slam_loc = true;
        slam_loc_x    = cell_utm.x();
        slam_loc_y    = cell_utm.y();
		// slam_loc_x = msg.salm_pose.pose.position.x;
		// slam_loc_y = msg.salm_pose.pose.position.y;
		slam_loc_yaw = yaw;
		slam_loc_pitch = pitch;
	}
	else
	{
		slam_loc = false;
	}
	m_local.unlock();
}
void *switch_gps_slam(void *param)
{
	int gps_wait_time = 0;
	int gps_error_time = 5;
	int loc_type = -1; // 0 gps , 1 slam ,-1 None
	double control_yaw = 0;
	bool vehicle_driving = true;
	bool calculate_flag;
	double pitch = 0;
	ofstream ofs;
	ofs.open("decision_planning.txt", ios::out);
	int point_id;
	std::this_thread::sleep_for(std::chrono::seconds(3));
	while (rclcpp::ok())
	{
		double distance_loc_min = 5;
		m_local.lock();
		// gps_loc_x=-69.8303;
		// gps_loc_y=361.585;
		// gps_loc_yaw=4.78714;
		// gps_loc=true;
		// gps_loc=true;
		if (gps_loc)
		{
			loc_x = gps_loc_x;
			loc_y = gps_loc_y;
			loc_yaw = -gps_loc_yaw + M_PI_2;
			if (loc_yaw < 0)
				loc_yaw = loc_yaw + 2 * M_PI;
			control_yaw = gps_loc_yaw;
			pitch = gps_loc_pitch;
			gps_wait_time = 300; // 等待gps  1000个循环
			gps_error_time = 5;
			calculate_flag = true;
			loc_type = 0;
		}
		else if ((!gps_loc) && slam_loc && gps_wait_time == 0)
		{
			loc_x = slam_loc_x;
			loc_y = slam_loc_y;
			loc_yaw = slam_loc_yaw;
			calculate_flag = true;
			// cout<<"gps_wait_ti
			loc_type = 1;
			pitch = slam_loc_pitch;
			if (M_PI_2 - slam_loc_yaw < 0)
				control_yaw = M_PI_2 - slam_loc_yaw + 2 * M_PI + slam_start_yaw;
			else
				control_yaw = M_PI_2 - slam_loc_yaw + slam_start_yaw;
		}
		m_local.unlock();
		if (calculate_flag)
		{
			calculate_flag = false;
			point_id = 0;
			for (int i = V_RefPoint; i < C_RoadTraj1Xtable_f32s23.size() - 10; i++)
			{
				double distance_loc_1 = sqrt(pow((C_RoadTraj1Xtable_f32s23[i] - loc_x), 2) + pow((C_RoadTraj1Ytable_f32s23[i] - loc_y), 2));
				// cout<<"distance_loc_1:"<<distance_loc_1<<endl;
				// std::cout << "gps_loc_yaw:-----------------------------------------------" << gps_loc_yaw << std::endl;
				// std::cout << "C_Road_yaw[i]----------------------------------------------:" << C_Road_yaw[i] << std::endl;
				// std::cout << "delta_C_Road_yaw[i]---------------------------------------------:" << (C_Local_yaw[i] - gps_loc_yaw) << std::endl;
				// std::cout << "i:--------------------------------------------------------------" << i << std::endl;
				// if(distance_loc_1<distance_loc_min) // old
				double det_yaw = C_Local_yaw[i] - gps_loc_yaw;
				if (det_yaw > M_PI + M_PI_2)
					det_yaw = det_yaw - M_PI * 2;
				else if (det_yaw < -M_PI - M_PI_2)
					det_yaw = det_yaw + M_PI * 2;

				if (distance_loc_1 < distance_loc_min && std::fabs(det_yaw) < M_PI_2)
				{
					// std::cout << "loc_yaw:" << loc_yaw << std::endl;
					// std::cout << "gps_loc_yaw:-----------------------------------------------" << gps_loc_yaw << std::endl;
					// std::cout << "C_Local_yaw[i]----------------------------------------------:" << C_Local_yaw[i] << std::endl;
					// // std::cout << "delta_C_Road_yaw[i]---------------------------------------------:" << (C_Local_yaw[i] - gps_loc_yaw) << std::endl;
					// std::cout << "i:--------------------------------------------------------------" << i << std::endl;
					distance_loc_min = distance_loc_1;
					point_id = i;
				}
			}
			if (point_id > 0)
			{
				V_RefPoint = point_id;
				// cout<<"distance_loc_min:"<<distance_loc_min<<endl;
				// cout << "V_RefPoint:" << V_RefPoint << endl;
				vehicle_driving = false;
			}
			else
			{
				cout << "没找到点" << endl;
				V_RefPoint = 0;
			}
		}
		else
		{
			gps_wait_time--;
			// cout<<"gps_wait_time"<<gps_wait_time<<endl;
			// cout<<"gps_error_time"<<gps_error_time<<endl;
			if (gps_error_time < 0)
			{
				vehicle_driving = true;
				loc_type = -1;
			}
		}
		decision_planning_msgs::msg::DecisionPlanning planning_msg;
		if (V_RefPoint > C_RoadTraj1Xtable_f32s23.size())
		{
			planning_msg.vfpoint = V_RefPoint;
			planning_msg.aim_x = C_RoadTraj1Xtable_f32s23[V_RefPoint];
			planning_msg.aim_y = C_RoadTraj1Ytable_f32s23[V_RefPoint];
			planning_msg.speed = C_TrajeSpd[V_RefPoint];
			planning_msg.trace_driving = true;
			planning_msg.traffic_light_driving = traffic_ligt_driving;
			planning_msg.loc_type = loc_type;
			planning_msg.loca_x = loc_x;
			planning_msg.loca_y = loc_y;
			planning_msg.loca_yaw = control_yaw;
			planning_msg.loca_pitch = pitch;
		}
		else if (V_RefPoint > (C_RoadTraj1Xtable_f32s23.size() - 10))
		{
			planning_msg.vfpoint = V_RefPoint;
			planning_msg.aim_x = C_RoadTraj1Xtable_f32s23[V_RefPoint];
			planning_msg.aim_y = C_RoadTraj1Ytable_f32s23[V_RefPoint];
			planning_msg.speed = 1;
			planning_msg.trace_driving = true;
			planning_msg.traffic_light_driving = traffic_ligt_driving;
			planning_msg.loc_type = loc_type;
			planning_msg.loca_x = loc_x;
			planning_msg.loca_y = loc_y;
			planning_msg.loca_yaw = control_yaw;
			planning_msg.loca_pitch = pitch;
		}
		else
		{
			planning_msg.vfpoint = V_RefPoint;
			planning_msg.aim_x = C_RoadTraj1Xtable_f32s23[V_RefPoint + 20];
			planning_msg.aim_y = C_RoadTraj1Ytable_f32s23[V_RefPoint + 20];
			//planning_msg.speed = C_TrajeSpd[V_RefPoint + 20];
			




			if(radar_driving)
			{
				planning_msg.speed =-1;
				RCLCPP_ERROR(rclcpp::get_logger("decision_planning"),"===============");
			}
			else
			{
				if(C_TrajeSpd[V_RefPoint + 20]*3.6>15)
				{
					planning_msg.speed =4.16;
				}
				else
				{
					planning_msg.speed = C_TrajeSpd[V_RefPoint + 20];
				}
			}
			// if(V_RefPoint>450&&V_RefPoint<900)
			// {18,
			// 	planning_msg.speed =2;
			// }
			planning_msg.trace_driving = vehicle_driving;
		
			planning_msg.traffic_light_driving = traffic_ligt_driving;
			planning_msg.loc_type = loc_type;
			planning_msg.loca_x = loc_x;
			planning_msg.loca_y = loc_y;
			planning_msg.loca_yaw = control_yaw;
			planning_msg.loca_pitch = pitch;
		}
		if (V_RefPoint > 0)
		{
			std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
			long start = ms.count();
			planning_pub->publish(planning_msg);
			ofs << dec << start << "\t"
				<< planning_msg.vfpoint << "\t"
				<< planning_msg.aim_x << "\t"
				<< planning_msg.aim_y << "\t"
				<< planning_msg.loca_x << "\t"
				<< planning_msg.loca_y << "\t"
				<< planning_msg.loca_yaw << "\t"
				<< trace_id << "\t" << endl;
		}
		// if(V_RefPoint>160)
		// 	V_RefPoint=true;
		slam_loc = false;
		gps_loc = false;
		if(radar_driving)
		{
			rclcpp::Rate r(2);
			r.sleep();
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
	}
	ofs.close();
	return nullptr;   
}
int main(int argc, char * argv[])
{
  /* ---------- ROS 2 初始化 ---------- */
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("decision_planning");

  /* ---------- 读取/声明参数 ---------- */
  node->declare_parameter<std::string>(
      "global_path", "/home/nvidia/ZHITAI/qingling_ros2/global_trace.txt");
  node->declare_parameter<int>("change_trace_point", 10);
  node->declare_parameter<double>("trace_distance", 0.8);

  std::string trace_path   = node->get_parameter("global_path").as_string();
  int         change_trace_point =
      node->get_parameter("change_trace_point").as_int();
  double      trace_distance =
      node->get_parameter("trace_distance").as_double();

  /* ---------- 业务初始化 ---------- */
  local_route.init_pub(node);              // ← 若编译报类型不匹配，仅重载一行即可
  local_route.ReadTxt(trace_path, paths);

  trace_id = 1;                            // 1 = 右车道
  std::cout << "trace_id: " << paths.size() << std::endl;

  C_RoadTraj1Xtable_f32s23.assign(paths[trace_id * 3].begin(),
                                  paths[trace_id * 3].end());
  C_RoadTraj1Ytable_f32s23.assign(paths[trace_id * 3 + 1].begin(),
                                  paths[trace_id * 3 + 1].end());
  C_Road_yaw .assign(paths[2].begin(),  paths[2].end());
  C_Local_yaw.assign(paths[6].begin(), paths[6].end());
  C_TrajeSpd .assign(paths[5].begin(), paths[5].end());

  /* ---------- 订阅 ---------- */
  auto sub_lidar = node->create_subscription<lidar_msgs::msg::Cells>(
      "/perception/lidar_cells", 10, lidar_callback);

  auto sub_fusion = node->create_subscription<can_control_msgs::msg::Autocontrol>(
      "/perception/fusion", 10, fusion_callback);

  auto sub_camera = node->create_subscription<can_control_msgs::msg::Autocontrol>(
      "/perception/camera", 10, camera_callback);

  auto sub_location = node->create_subscription<localization_msgs::msg::Localization>(
      "/localization", 10, loc_callback);

  auto sub_radar = node->create_subscription<radar_msgs::msg::SensorData>(
      "sensorRawData", 10, radar_callback);

  auto loc_slam_sub = node->create_subscription<localization_msgs::msg::Localization>(
      "ndt_pose", 10, loc_slam_callback);

  /* ---------- 发布 ---------- */
  planning_pub = node->create_publisher<decision_planning_msgs::msg::DecisionPlanning>(
      "decision_planning/aim", 100);

  fusion_pub   = node->create_publisher<can_control_msgs::msg::Autocontrol>(
      "route_filter/fusion", 1000);

  traffic_obu  = node->create_publisher<std_msgs::msg::String>("traffic_obu", 1000);
  local_path_pub = node->create_publisher<visualization_msgs::msg::Marker>("path_local", 10);

  /* ---------- 后台线程保持原 pthread 逻辑 ---------- */
  pthread_t switch_;
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, 1);
  if (pthread_create(&switch_, &attr, switch_gps_slam, nullptr) != 0) {
    std::cout << "fail... send thread error" << std::endl;
    return 1;
  }

  /* ---------- 可视化 Marker 初始化 ---------- */
  visualization_msgs::msg::Marker sloacl_path;
  sloacl_path.header.frame_id = "world";
  sloacl_path.ns              = "";
  sloacl_path.action          = visualization_msgs::msg::Marker::ADD;
  sloacl_path.type            = visualization_msgs::msg::Marker::LINE_STRIP;
  sloacl_path.scale.x         = 0.3;
  sloacl_path.color.set__g(1.0f).set__a(1.0f);

  geometry_msgs::msg::Point wp;
  rclcpp::Rate loop_rate(50);              // 50 Hz 与原来一致

  /* ---------- 主循环 ---------- */
  while (rclcpp::ok()) {
    sloacl_path.points.clear();
    for (int i = 0; i < 30; ++i) {
      wp.x = C_RoadTraj1Xtable_f32s23[V_RefPoint + i];
      wp.y = C_RoadTraj1Ytable_f32s23[V_RefPoint + i];
      wp.z = 0.0;
      sloacl_path.points.push_back(wp);
    }
    sloacl_path.header.stamp = node->now();
    local_path_pub->publish(sloacl_path);

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}