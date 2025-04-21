#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>
#include <ctype.h>
#include <arpa/inet.h>
#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <sys/time.h>
#include <vector>
#include "../include/can_com/maindata.h"
#include "../include/can_com/int.h"
#include <can_com/msg/autocontrol_radardata.hpp>
#include <can_com/msg/autocontrol.hpp>
#include <can_com/msg/mqtt_data.hpp>
#include "can_com/RoutePlanning.h"
#include "can_com/AutoVehicleControl.h"
#include <can_com/msg/radardate.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <lidar_msgs/msg/cells.hpp>
#include <lidar_msgs/msg/cell.hpp>
/**高精度地图读取数据* */
#include"string.h"
#include "can_com/msg/getmap.hpp"
#include "OdrManager.h"
#include "mutex"
#include<tf/transform_datatypes.h>
static vector<vector<double>>paths;
static int local_path_size=40;

static vector<vector<double>> local_path;
static double threshold_y=0.5;
static int trace_id=0;
mutex m_trace;
static geometry_msgs::PoseStamped vhicle_pose;
mutex m_vhicle_pose;
mutex local_trace_mtx;
double  azimuthAngle( double  x1,double  y1,double  x2,double  y2)
{
    double angle = 0.0;
    double dx = x2 - x1;
    double dy = y2 - y1;
    if(x2 == x1)
    {
        angle = M_PI_2 ;
        if( y2 == y1 )
            angle = 0.0;
        else// (y2 < y1 )
            angle = 3.0 *M_PI_2 ;
    }
    else if (x2 > x1 && y2 > y1)  //一象限
        angle = atan(dy / dx);
    else if (x2 > x1 && y2 < y1)  //四象限
        angle = 2*M_PI + atan(dy / dx);
    else if (x2 < x1 && y2 < y1 )  //三象限
        angle = M_PI+ atan(dy / dx);
    else if(x2 < x1 && y2 > y1 )  //二象限
        angle = M_PI + atan(dy / dx);
    return angle;

}
void smoothPath(std::vector<vector<double>>& paths_dect, double weight_data,
    double weight_smooth, double tolerance)
{
	for(int ix=0;ix<int(paths_dect.size()/2);ix++)
	{
		std::vector<vector<double>> path_in;
		path_in.assign(paths_dect.begin()+(ix*2),paths_dect.begin()+(ix*2)+2);
		std::vector<vector<double>> smoothPath_out = path_in;

		double change = tolerance;
		double xtemp, ytemp;
		int nIterations = 0;

		int size = paths_dect[ix].size();

		while (change >= tolerance) {
			change = 0.0;
			for (int i = 1; i < size - 1; i++) {
				xtemp = smoothPath_out[0][i];
				ytemp = smoothPath_out[1][i];

				smoothPath_out[0][i] += weight_data * (path_in[0][i] - smoothPath_out[0][i]);
				smoothPath_out[1][i] += weight_data * (path_in[1][i]  - smoothPath_out[1][i]);

				smoothPath_out[0][i] += weight_smooth * (smoothPath_out[0][i - 1] + smoothPath_out[0][i + 1] - (2.0 * smoothPath_out[0][i]));
				smoothPath_out[1][i] += weight_smooth * (smoothPath_out[1][i - 1] + smoothPath_out[1][i + 1] - (2.0 * smoothPath_out[1][i]));

				change += fabs(xtemp - smoothPath_out[0][i]);
				change += fabs(ytemp - smoothPath_out[1][i]);
			}
			nIterations++;
		}
		
		paths_dect[ix] = smoothPath_out[0];
		paths_dect[ix+1] = smoothPath_out[0+1];
		// smoothPath_out.clear();
		// path_in.clear();
	}
}
bool generator_local_trace(vector<double>x_orignal,vector<double>y_orignal,vector<double>yaw_orignal,vector<double>x_target,vector<double>y_target,
vector<double>&local_x,vector<double>&local_y,vector<double> &local_yaw,int local_point_id,int keep_point,int chang_lane_point,
double point_distance,double lane_distance,int local_path_size,int id_interval)
{
    double yaw_change=azimuthAngle(x_orignal[local_point_id+keep_point],y_orignal[local_point_id+keep_point],
	x_target[local_point_id+keep_point+chang_lane_point],y_target[local_point_id+keep_point+chang_lane_point]);
	double x_dis=x_orignal[local_point_id+keep_point]-x_target[local_point_id+keep_point+chang_lane_point];
	double y_dis=y_orignal[local_point_id+keep_point]-y_target[local_point_id+keep_point+chang_lane_point];
	double distance_yaw=sqrt(pow(x_dis,2)+pow(y_dis,2))/(chang_lane_point+1);//*id_interval/abs(id_interval);
    //double distance_yaw=sqrt(pow(point_distance*chang_lane_point,2)+pow(lane_distance*id_interval,2))/(chang_lane_point+1)*id_interval/abs(id_interval);
    if(local_point_id+keep_point+chang_lane_point<x_orignal.size())
    {
		local_x.assign(x_orignal.begin()+local_point_id,x_orignal.begin()+local_point_id+keep_point+1);
		local_y.assign(y_orignal.begin()+local_point_id,y_orignal.begin()+local_point_id+keep_point+1);
		local_yaw.assign(yaw_orignal.begin()+local_point_id,yaw_orignal.begin()+local_point_id+keep_point);
        for (unsigned int i = 0; i < chang_lane_point; i++) {
			local_x.push_back(x_orignal[local_point_id+keep_point] + (i+1)*distance_yaw * cos(yaw_change ));
			local_y.push_back(y_orignal[local_point_id+keep_point] + (i+1)*distance_yaw * sin(yaw_change ));
			local_yaw.push_back(yaw_change);
        }

		if(keep_point+chang_lane_point<local_path_size)
		{
			local_x.insert(local_x.end(),x_target.begin()+local_point_id+keep_point+chang_lane_point,x_target.begin()+(local_path_size+local_point_id-1));
			local_y.insert(local_y.end(),y_target.begin()+local_point_id+keep_point+chang_lane_point,y_target.begin()+(local_path_size+local_point_id-1));
			local_yaw.insert(local_yaw.end(),yaw_orignal.begin()+local_point_id+keep_point+chang_lane_point,yaw_orignal.begin()+(local_path_size+local_point_id-1));
		}
		else
		{
			cout<<"keep_point + chang_lane_point 大于"<<local_path_size<<endl;
		}
		return 1;
    }
	else 
	{
		return 0;
	}
	
}

void* route_plan(void* lp)
{
    
	int detection_distance;
	double point_distance=0.3;
	int chang_lane_point=int(3/point_distance);
	double lane_distance=2;
	double chang_lane_distance;
	double aim_distance=3; 
	
	double weight_data=0.45;
	double weight_smooth=0.4;
	double tolerance=0.05;
	
	vector<visualization_msgs::Marker> sloacl_path;
	visualization_msgs::Marker sloacl_path_;
    geometry_msgs::Point wp;
	vector<vector<double>> local_path_1;
	for(int i=0;i<paths.size()/3;i++)
	{
		sloacl_path_.header.frame_id = "/world";
		sloacl_path_.header.stamp = ros::Time();
		sloacl_path_.ns = "";
		sloacl_path_.action = visualization_msgs::Marker::ADD;
		sloacl_path_.frame_locked = false;
		sloacl_path_.scale.x = 0.3;
		sloacl_path_.frame_locked = false;
		//sloacl_path[i].points.clear();
		sloacl_path.push_back(sloacl_path_);
	}
	//local_path.clear();
	ros::Rate loop_rate(20);
	while(ros::ok())
	{
		if(local_path_size+V_RefPoint<paths[0].size())
		{

		vector<double> local_x;
		vector<double> local_y;
		vector<double> local_yaw;
		for( int idd=0;idd<paths.size()/3;idd++)
		{
			if(idd==trace_id)
			{	
				if(V_RefPoint+local_path_size<paths[trace_id*3].size())
				{
					// local_x.assign(paths[trace_id*3].begin()+V_RefPoint,paths[trace_id*3].begin()+V_RefPoint+local_path_size);
					// local_y.assign(paths[trace_id*3+1].begin()+V_RefPoint,paths[trace_id*3+1].begin()+V_RefPoint+local_path_size);
					// local_yaw.assign(paths[trace_id*3+2].begin()+V_RefPoint,paths[trace_id*3+2].begin()+V_RefPoint+local_path_size);
					local_x.assign(C_RoadTraj1Xtable_f32s23.begin()+V_RefPoint,C_RoadTraj1Xtable_f32s23.begin()+V_RefPoint+local_path_size);
					local_y.assign(C_RoadTraj1Ytable_f32s23.begin()+V_RefPoint,C_RoadTraj1Ytable_f32s23.begin()+V_RefPoint+local_path_size);
					local_yaw.assign(C_Road_yaw.begin()+V_RefPoint,C_Road_yaw.begin()+V_RefPoint+local_path_size);
					local_path_1.push_back(local_x);
					local_path_1.push_back(local_y);
					local_path_1.push_back(local_yaw);
					local_x.clear();
					local_y.clear();
					local_yaw.clear();
				}
			}
			else
			{       
				
				if(generator_local_trace(paths[trace_id*3],paths[trace_id*3+1],paths[trace_id*3+2],paths[idd*3],paths[idd*3+1],local_x,local_y,local_yaw,V_RefPoint,
				int(aim_distance/point_distance),chang_lane_point,point_distance,lane_distance,local_path_size,1)){
					local_path_1.push_back(local_x);
					local_path_1.push_back(local_y);
					local_path_1.push_back(local_yaw);
					local_x.clear();
					local_y.clear();
					local_yaw.clear();
				}
				
			}
		}
		vector<vector<double>> local_path_;
		for(int ix=0;ix<int(local_path_1.size()/3);ix++)
		{
			local_path_.push_back(local_path_1[ix*3]);
			local_path_.push_back(local_path_1[ix*3+1]);
			smoothPath(local_path_,weight_data,weight_smooth,tolerance);
			for(int i=0;i<local_path_[0].size();i++)
			{
				wp.x=local_path_[0][i];
				wp.y=local_path_[1][i];
				wp.z=0;
				sloacl_path[ix].points.push_back(wp);
			}
			local_path_1[ix*3]=local_path_[0];
			local_path_1[ix*3+1]=local_path_[1];
			local_path_.clear();
		}
		local_trace_mtx.lock();
		local_path.clear();
		local_path=local_path_1;
		local_trace_mtx.unlock();
		local_path_1.clear();
		for(int ii=0;ii<3;ii++)
		{
			if(ii==trace_id)
			{
				sloacl_path[ii].type = visualization_msgs::Marker::LINE_STRIP;
				sloacl_path[ii].color.b = 0;
				sloacl_path[ii].color.g = 0;
				sloacl_path[ii].color.r = 1;
				sloacl_path[ii].color.a = 1;
			}
			else
			{
				sloacl_path[ii].type = visualization_msgs::Marker::LINE_STRIP;
				sloacl_path[ii].color.b = 0;
				sloacl_path[ii].color.g = 1;
				sloacl_path[ii].color.r = 0;
				sloacl_path[ii].color.a = 1;
			}
			
		}
		// if(sloacl_path[0].points.size()>0)
		// {
		cout<<__LINE__<<":"<<sloacl_path[1].points.size()<<endl;
		pub_local_center.publish(sloacl_path[1]);
		pub_local_left.publish(sloacl_path[0]);
		pub_local_right.publish(sloacl_path[2]);
		sloacl_path[0].points.clear();
		sloacl_path[1].points.clear();
		sloacl_path[2].points.clear();
		//}
			//}

		}
		loop_rate.sleep();
	}
}

void split(string str, string pattern,vector<double>& result)
{
    string::size_type pos;
    str += pattern;//扩展字符串以方便操作
    int size = str.size();
    for (int i = 0; i < size; i++)
    {
        pos = str.find(pattern, i);
        if (pos < size )
        {

			result.push_back(std::atof(str.substr(i, pos - i).c_str()));
            i = pos + pattern.size() - 1;
        }
    }
}


void ReadTxt(string trace_path,vector<vector<double>>&paths)
{
	vector<double> path;
    // 只适用于逗号分分隔

    string line;
    std::ifstream input;
    input.open(trace_path);
    // ofstream output;
    // output.open("/home/wj/code/perception/test/x.txt");
	int num_lines=0;
    while (getline(input, line))
    {   num_lines++;
		split(line, ",",path);
		// if(num_lines%3=0)
		// {
		// 	split(line, ",",path,0);
		// }
		// else if(num_lines%3=1)
		// {
		// 	split(line, ",",path,1);
		// }
		// else if (num_lines%3=2)
		// {
		// 	split(line, ",",path,2);
		// }
		paths.push_back(path);
		path.clear();
    }
    input.close();
}
void visiual_global_trace()
{
	vector<visualization_msgs::Marker> sloacl_path;
	for(int ii=0;ii<paths.size()/3;ii++)
	{
	visualization_msgs::Marker sloacl_path_;
	geometry_msgs::Point wp;
	sloacl_path_.header.frame_id = "/world";
	sloacl_path_.header.stamp = ros::Time();
	sloacl_path_.ns = "";
	sloacl_path_.action = visualization_msgs::Marker::ADD;
	sloacl_path_.scale.x = 0.1;
	sloacl_path_.frame_locked = false;

	for(int i=0;i<paths[ii].size();i++)
	{
		wp.x=paths[ii*3][i];
		wp.y=paths[ii*3+1][i];
		wp.z=0;
		sloacl_path_.points.push_back(wp);
	}
	sloacl_path_.type = visualization_msgs::Marker::LINE_STRIP;
	sloacl_path_.color.b = 1;
	sloacl_path_.color.g = 1;
	sloacl_path_.color.r = 1;
	sloacl_path_.color.a = 1;
	sloacl_path.push_back(sloacl_path_);

	}
	pub_global_center.publish(sloacl_path[1]);
	pub_global_left.publish(sloacl_path[0]);
	pub_global_right.publish(sloacl_path[2]);

	
}
main()
{
    cout<<__LINE__<<endl;
	vector<can_com::msg::Autocontrol> msg_Lidar_self;
	vector<double> x;
	vector<double> y;
	vector<double> yaw_;
	int old_trace_id=trace_id;
	vector<vector<double>> l_path=local_path;
	debug_show_road_cells(l_path, msg_Lidar_self,*msg, threshold_y,l_path[0].size(),trace_id);
    for(int i=0;i<msg_Lidar_self.size();i++){
		if(4<msg_Lidar_self[trace_id].front_l_long_obj && msg_Lidar_self[trace_id].front_l_long_obj<8 )
		{
			if( msg_Lidar_self[i].front_l_long_obj<0.001)
			{
				if(abs(trace_id-i)<2)
				{
					x.insert(x.end(),C_RoadTraj1Xtable_f32s23.begin(),C_RoadTraj1Xtable_f32s23.begin()+V_RefPoint);
					x.insert(x.end(),l_path[i*3].begin(),l_path[i*3].end());
					x.insert(x.end(),paths[i*3].begin()+local_path_size+V_RefPoint,paths[i*3].end());
					y.insert(y.end(),C_RoadTraj1Ytable_f32s23.begin(),C_RoadTraj1Ytable_f32s23.begin()+V_RefPoint);
					y.insert(y.end(),l_path[i*3+1].begin(),l_path[i*3+1].end());
					y.insert(y.end(),paths[i*3+1].begin()+local_path_size+V_RefPoint,paths[i*3+1].end());
					yaw_.insert(yaw_.end(),C_Road_yaw.begin(),C_Road_yaw.begin()+V_RefPoint);
					yaw_.insert(yaw_.end(),l_path[i*3+2].begin(),l_path[i*3+2].end());
					yaw_.insert(yaw_.end(),paths[i*3+2].begin()+local_path_size+V_RefPoint,paths[i*3+2].end());
					m_trace.lock();
					C_RoadTraj1Xtable_f32s23=x;
					C_RoadTraj1Ytable_f32s23=y;
					C_Road_yaw=yaw_;
					x.clear();
					y.clear();
					trace_id=i;
					m_trace.unlock();
					cout<<__LINE__<<trace_id<<endl;
					break;
				}
			}
		}
		else{
			break;
		}
	}}

