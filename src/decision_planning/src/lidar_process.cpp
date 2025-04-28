#include"lidar_msgs/msg/objects.hpp"
#include "ros/ros.h"
using namespace std;
void lidar_process_callback(const lidar_msgs::msg::Objects::ConstPtr& msg)
{
//     lidar_msgs::msg::Cells ms=*msg;
//    for(auto m: ms)
//    {
//     cout<<"m="<<m<<endl;
//    }
}
int main(int argc,char*argv[])
{
    ros::init(argc,argv,"lidar_process_node");
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe("/perception/lidar_objs",1000,lidar_process_callback);
    ros::spin();

}