#include <chrono>
#include<math.h>
#include <sstream>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include<vector>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <iostream>
#include <fstream>
#include <string>


static vector<double> X_;
static vector<double> Y_;

void callback_route_record(const geometry_msgs::PoseStamped &pose_msg)
{

    // if(X_[X_.size()]-pose_msg.pose.position.x>0.2)
    // {
        X_.push_back(pose_msg.pose.position.x);
        Y_.push_back(pose_msg.pose.position.y);
        ROS_INFO("X_: [%f],Y_:[%f]",pose_msg.pose.position.x,pose_msg.pose.position.y);
    //}
}
int main(int argc,char** argv){
    X_.push_back(0);
    Y_.push_back(0);
    ros::init(argc, argv, "rotue_records");
    ros::NodeHandle nh;
    ros::Subscriber Hdsub = nh.subscribe("ndt_pose", 10, callback_route_record);

    while (ros::ok())
	{

        ros::spinOnce();
    }
    ofstream outfile("out.txt", std::ios::out |ios::trunc);
    outfile<<"real_T C_RoadTraj1Xtable_f32s23["<<X_.size()<<"]={"
	for (int i = 0; i < X_.size(); i++)
	{
        if(i=(X_.size()-1))
            outfile << X_[i] <<"}"<<endl;
        else
		    outfile << X_[i] <<","
        
	};
    outfile<<"real_T C_RoadTraj1Ytable_f32s23["<<Y_.size()<<"]={"
	for (int it = 0; it < Y_.size(); it++)
	{
        if(it=(Y_.size()-1))
            outfile << Y_[it] <<"}"<<endl;
        else
		    outfile << Y_[it] <<","
        
	};
    outfile.close();
    return 0;
}