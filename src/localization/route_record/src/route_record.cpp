#include <chrono>
#include<math.h>
#include <sstream>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include<vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include<tf/transform_datatypes.h>
using namespace std;

static vector<double> X_;
static vector<double> Y_;
static vector<double> yaw_;
static vector<double> X_1;
static vector<double> Y_1;


void callback_route_record(const geometry_msgs::PoseStamped &pose_msg)
{
	tf::Quaternion quat;
	double yaw=tf::getYaw(pose_msg.pose.orientation);
	if(X_.size()==0)
	{
		X_.push_back(pose_msg.pose.position.x);
        Y_.push_back(pose_msg.pose.position.y);
		X_1.push_back(pose_msg.pose.position.x-2*cos(yaw+M_PI_2));
        Y_1.push_back(pose_msg.pose.position.y-2*sin(yaw+M_PI_2));
		//yaw.push_back(yaw);
        ROS_INFO("X_: [%f],Y_:[%f],yaw:[%f]",pose_msg.pose.position.x,pose_msg.pose.position.y,yaw);
	}
    if(sqrt(pow(X_[X_.size()-1]-pose_msg.pose.position.x,2)+pow(Y_[Y_.size()-1]-pose_msg.pose.position.y,2))>0.1)
    {
        X_.push_back(pose_msg.pose.position.x);
        Y_.push_back(pose_msg.pose.position.y);
		X_1.push_back(pose_msg.pose.position.x-2*cos(yaw+M_PI_2));
        Y_1.push_back(pose_msg.pose.position.y-2*sin(yaw+M_PI_2));
        //yaw.push_back(yaw);
        ROS_INFO("X_: [%f],Y_:[%f],yaw:[%f]",pose_msg.pose.position.x,pose_msg.pose.position.y,yaw);
    }
}
int main(int argc,char** argv){
    // X_.push_back(0);
    // Y_.push_back(0);

    ros::init(argc, argv, "route_record_node");
    ros::NodeHandle nh;
    ros::Subscriber Hdsub = nh.subscribe("ndt_pose", 10, callback_route_record);

    while (ros::ok())
    {
        // ROS_INFO("X_.size() :[%d]",X_.size());
    	// ROS_INFO("Y_.size() :[%d]",Y_.size());
		// ROS_INFO("X_1.size() :[%d]",X_1.size());
    	// ROS_INFO("Y_1.size() :[%d]",Y_1.size());
       ros::spinOnce();
    }
	ofstream outfile("out.txt", ios::trunc);
	outfile<<"real_T C_RoadTraj1Xtable_f32s23["<<X_.size()<<"]={";
	for (int i = 0; i < X_.size(); i++)
	{

	if(i<(X_.size()-1))
	{
	    outfile << X_[i] <<",";
	}
	else
	{
	    
            outfile << X_[i] <<"};"<<endl;
	}
	}
	outfile<<"real_T C_RoadTraj1Ytable_f32s23["<<Y_.size()<<"]={";
	for (int it = 0; it < Y_.size(); it++)
	{
	if(it<(Y_.size()-1))
	{
		outfile << Y_[it] <<",";
	}
	else
	{
            outfile << Y_[it] <<"};"<<endl;
	}

	}

    outfile<<"real_T C_RoadTraj1Xtable_f32s23__1["<<X_.size()<<"]={";
	for (int i = 0; i < X_1.size(); i++)
	{

	if(i<(X_1.size()-1))
	{
	    outfile << X_1[i] <<",";
	}
	else
	{
	    
            outfile << X_1[i] <<"};"<<endl;
	}
	}
	outfile<<"real_T C_RoadTraj1Ytable_f32s23__1["<<Y_1.size()<<"]={";
	for (int it = 0; it < Y_1.size(); it++)
	{
	if(it<(Y_1.size()-1))
	{
		outfile << Y_1[it] <<",";
	}

	else
	{
            outfile << Y_1[it] <<"};"<<endl;
	}

	}
	outfile.close();
    


    
    return 0;
}
