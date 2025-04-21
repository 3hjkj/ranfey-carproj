// #include "ros/ros.h"
// #include "std_msgs/msg/string.hpp"
// #include <sstream>
// #include <unistd.h>
// #include <sys/types.h>
// #include <ctype.h>
// #include <arpa/inet.h>
// #include "std_msgs/msg/u_int16_multi_array.hpp"
// #include <fstream>
// #include <chrono>
// #include <sys/time.h>
// #include <vector>
// // 定位msg
// #include <localization_msgs/msg/localization.hpp>
// #include <localization_msgs/msg/g_p_s.hpp>
// #include <localization_msgs/msg/pose_angle.hpp>
// #include <localization_msgs/msg/time.hpp>
// #include <localization_msgs/msg/xyz.hpp>
// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <net/if.h>
// #include <sys/ioctl.h>
// #include <sys/socket.h>
// #include <linux/can.h>
// #include <linux/can/raw.h>
// #include <stdint.h>
// #include <iostream>
// #include <gps_rtk.h>
// #include <pthread.h>
// #include<iostream>
// #include "geotool.h"	
// #include <chrono>										 //调用 pthread_create() 函数
// #define CAN_SFF_MASK 0x000007FFU								 /* 接收标准帧格式 (SFF) */
// #define CAN_EFF_MASK 0x1FFFFFFFU								 /* 接受扩展帧格式 (EFF) */
// #define CAN_ERR_MASK 0x1FFFFFFFU								 /* 忽略 EFF, RTR, ERR 标志 */
// #define RTK_1 0x323
// #define RTK_2 0x324
// #define RTK_3 0x32A
// #define RTK_4 0x320
// #define RTK_5 0x329
// #define QL_Car 0x0C0228D1
// using namespace std;
// Radar_CANDATA Radar_CANDATA_1 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// ros::Publisher RtkMsgPub;
// // double X0=652636.046660;
// // double Y0=3266039.300042;
// double X0=537854.690940;
// double Y0=4337387.744176;
// // int GetLongZone(double longitude)
// // {
// // 	double longZone = 0.0;
// // 	if (longitude < 0.0)
// // 	{
// // 		longZone = ((180.0 + longitude) / 6.0) + 1;
// // 	}
// // 	else
// // 	{
// // 		longZone = (longitude / 6.0) + 31;
// // 	}
// // 	return static_cast<int>(longZone);
// // }
// static localization_msgs::msg::Localization loc;
// void Rtk_Data_Input(can_frame Rtk_Data_Input[], ros::Publisher pub)
// {

// 	GeoTool geo;
// 	if ((Rtk_Data_Input[0].can_id & CAN_SFF_MASK) == RTK_1)
// 	{
// 		loc.satellite_status = Rtk_Data_Input[0].data[2];
// 		loc.system_state = Rtk_Data_Input[0].data[0];
// 		loc.GpsNumSatsUsed = Rtk_Data_Input[0].data[1];
// 	}
// 	if ((Rtk_Data_Input[1].can_id & CAN_SFF_MASK)== RTK_2)
// 	{
// 		loc.gps.lat = ((Rtk_Data_Input[1].data[0] << 24) + (Rtk_Data_Input[1].data[1] << 16) +
// 					   (Rtk_Data_Input[1].data[2] << 8) + (Rtk_Data_Input[1].data[3])) *
// 					  0.0000001;
// 		loc.gps.lon = ((Rtk_Data_Input[1].data[4] << 24) + (Rtk_Data_Input[1].data[5] << 16) +
// 					   (Rtk_Data_Input[1].data[6] << 8) + (Rtk_Data_Input[1].data[7])) *
// 					  0.0000001;
// 	}
// 	if ((Rtk_Data_Input[2].can_id & CAN_SFF_MASK) == RTK_3)
// 	{
// 		loc.pose.yaw = ((Rtk_Data_Input[2].data[0] << 8) + Rtk_Data_Input[2].data[1]) * 0.01;
// 		loc.pose.pitch = ((Rtk_Data_Input[2].data[2] << 8) + Rtk_Data_Input[2].data[3]) * 0.01;
// 		loc.pose.roll = ((Rtk_Data_Input[2].data[4] << 8) + Rtk_Data_Input[2].data[5]) * 0.01;
// 	}
// 	if ((Rtk_Data_Input[3].can_id & CAN_SFF_MASK) == RTK_4)
// 	{
// 		int week_time = (Rtk_Data_Input[3].data[0] << 8) + (Rtk_Data_Input[3].data[1]);
// 		int gps_time = (Rtk_Data_Input[3].data[2] << 24) + (Rtk_Data_Input[3].data[3] << 16) + (Rtk_Data_Input[3].data[4] << 8) + (Rtk_Data_Input[3].data[5]) * 0.001;
// 	}
// 	// if ((Rtk_Data_Input[4].can_id & CAN_SFF_MASK) == RTK_5)
// 	// {
// 	// }
// 	// if ((Rtk_Data_Input[5].can_id & CAN_EFF_MASK) == QL_Car)
// 	// {
// 	// 	loc.car_speed = Rtk_Data_Input[5].data[5] / 3.6;
// 	// }

// 	PointGPS gps_;
// 	PointGCCS gccs_;
// 	int z = geo.GetLongZone(loc.gps.lon);
// 	geo.SetUtmZone(z);
// 	gps_.lat = loc.gps.lat;
// 	gps_.lon = loc.gps.lon;
// 	gps_.heading = loc.pose.yaw;
// 	geo.GPS2GCCS(gps_, gccs_);
// 	loc.xy.x = gccs_.xg-X0;
// 	loc.xy.y = gccs_.yg-Y0;
// 	loc.xy.z= loc.pose.yaw;
// 	cout<<loc.xy.x<<endl;
// 	cout<<loc.xy.y<<endl;
// 	cout<<loc.xy.z<<endl;
// 	loc.system_time = ros::Time::now().toSec();
// 	pub.publish(loc);
// }

// void *rtk_can(void *lp)
// {
// 	pid_t status; 
// 	status =system("echo 'nvidia' | sudo -S  /home/nvidia/ZHITAI/qingling_ros2/src/localization/gps_localization/can_start.sh"); 
// 	if (WIFEXITED(status)){
// 		if (0 == WEXITSTATUS(status)){  
// 			printf("run shell script successfully.\n");  
// 		}
// 		else if( WEXITSTATUS(status)==2){
// 			printf("run shell script fail, script exit code: %d\n", WEXITSTATUS(status));  
// 			status =system("echo 'nvidia' | sudo -S  /home/nvidia/ZHITAI/qingling_ros2/src/localization/gps_localization/can_down.sh");
// 			if(0 == WEXITSTATUS(status)){
// 				status =system("echo 'nvidia' | sudo -S  /home/nvidia/ZHITAI/qingling_ros2/src/localization/gps_localization/can_start.sh");
// 				if (0 == WEXITSTATUS(status)){  
// 				printf("run shell script successfully.\n");  
// 				}
// 			}
// 			else{exit(0);}
// 		}
// 	}
// 	else
// 	{  
// 		printf("exit status = [%d]\n", WEXITSTATUS(status));  
// 		exit(0);
// 	}  
// 	int can_skt;
// 	int bytes;
// 	struct sockaddr_can addr;
// 	struct ifreq ifr;

// 	can_skt= socket(PF_CAN, SOCK_RAW, CAN_RAW); //创建套接字
// 	strcpy(ifr.ifr_name, "can1");
// 	ioctl(can_skt, SIOCGIFINDEX, &ifr); //指定 can1 设备
// 	addr.can_family = AF_CAN;
// 	addr.can_ifindex = ifr.ifr_ifindex;
// 	bind(can_skt, (struct sockaddr *)&addr, sizeof(addr)); //将套接字与 can1 绑定
// 	//过滤
// 	struct can_filter rfilter[3];
// 	rfilter[0].can_id = RTK_1;
// 	rfilter[0].can_mask = CAN_SFF_MASK; //扩展帧
// 	rfilter[1].can_id = RTK_2;
// 	rfilter[1].can_mask = CAN_SFF_MASK; //扩展帧
// 	rfilter[2].can_id = RTK_3;
// 	rfilter[2].can_mask = CAN_SFF_MASK; //扩展帧
// 	// rfilter[3].can_id = RTK_4;
// 	// rfilter[3].can_mask = CAN_SFF_MASK;
// 	// rfilter[4].can_id = RTK_5;
// 	// rfilter[4].can_mask = CAN_SFF_MASK;
// 	// rfilter[5].can_id = QL_Car;
// 	// rfilter[5].can_mask = CAN_EFF_MASK;											 //扩展帧
// 	setsockopt(can_skt, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)); //设置过滤规则，只接受 扩展帧0x123 

// 	ros::Rate loop_rate(50);
// 	ofstream ofs;
// 	ofs.open("gps.txt",ios::out );
// 	while (ros::ok())
// 	{
// 		can_frame  frames[5];
// 		bool RTK1=false;
// 		bool RTK2=false;
// 		bool RTK3=false;
// 		while(true){
// 			struct can_frame  frame;
// 			std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch());
// 			long start= ms.count();
// 			bytes = read(can_skt, &frame, sizeof(frame)); //接收总线上的报文保存在frame[1]中
// 			if (bytes != sizeof(frame))
// 			{
// 				printf("rec  Error\n!");
// 				break;
// 			}
// 			ofs <<dec<<start<< "\t" 
// 			<< hex<<int(frame.can_id & CAN_EFF_MASK)  << "\t" 
// 			<< hex<< int(frame.data[0])  << "\t" 
// 			<< hex<< int(frame.data[1])  << "\t"
// 			<< hex<< int(frame.data[2])  << "\t"
// 			<< hex<< int(frame.data[3] ) << "\t"
// 			<< hex<< int(frame.data[4])  << "\t"
// 			<< hex<< int(frame.data[5])  << "\t"
// 			<< hex<< int(frame.data[6])  << "\t"
// 			<< hex<< int(frame.data[7])  << "\t"<< endl;
// 			if((frame.can_id & CAN_EFF_MASK)==RTK_1){
// 				RTK1=true;
// 				frames[0]=frame;}
// 			else if((frame.can_id & CAN_EFF_MASK)==RTK_2){
// 				RTK2=true;
// 				frames[1]=frame;}
// 			else if((frame.can_id & CAN_EFF_MASK)==RTK_3){
// 				RTK3=true;
// 				frames[2]=frame;}
// 			// else if((frame.can_id & CAN_EFF_MASK)==RTK_4){
// 			// 	RTK4=true;
// 			// 	frames[3]=frame;}
// 			// else if((frame.can_id & CAN_EFF_MASK)==RTK_5){
// 			// 	RTK5=true;
// 			// 	frames[4]=frame;}
// 				if(RTK1  && RTK2 && RTK3 )
// 					break;
// 		}
// 		Rtk_Data_Input(frames, RtkMsgPub);
// 		loop_rate.sleep();
// 	}
// 	//for(int i=0;i<6;i++)
// 		close(can_skt);
// 	pthread_exit(NULL);
// }

// int main(int argc, char *argv[])
// {
// 	int res;
// 	ros::init(argc, argv, "localization");
// 	ros::NodeHandle n;

// 	RtkMsgPub = n.advertise<localization_msgs::msg::Localization>("localization", 10);
// 	//ros::Publisher pub = n.advertise<can_com::msg::AutocontrolRadardata>("msg_RadarData", 10); //pub ifv info
// 	pthread_t threads;
// 	pthread_attr_t attr;
// 	pthread_attr_init(&attr);
// 	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
// 	res = pthread_create(&threads, &attr, rtk_can, NULL);
// 	if (res != 0)
// 	{
// 		printf("线程创建失败");
// 		return 0;
// 	}
// 	ros::Rate loop_rate(50);
// 	while (ros::ok())
// 	{
// 		loop_rate.sleep();
// 		//usleep(10000);
// 		//cout << "ok" << endl;
// 	}
// 	return 0;
// }
// gps_localization/src/localization_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "localization_msgs/msg/localization.hpp"
#include "localization_msgs/msg/gps.hpp"
#include "localization_msgs/msg/pose_angle.hpp"
#include "localization_msgs/msg/time.hpp"
#include "localization_msgs/msg/xyz.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <pthread.h>
#include <sstream>
#include <string>
#include <vector>

#include "geotool.hpp"
#include "gps_rtk.h"

#define CAN_SFF_MASK 0x000007FFU
#define CAN_EFF_MASK 0x1FFFFFFFU

#define RTK_1 0x323
#define RTK_2 0x324
#define RTK_3 0x32A
#define RTK_4 0x320
#define RTK_5 0x329
#define QL_Car 0x0C0228D1

using std::placeholders::_1;
using namespace std::chrono_literals;

// struct Radar_CANDATA
// {
//   uint8_t id;
//   uint8_t dis1;
//   uint8_t v1;
//   uint8_t angle1;
//   uint8_t dis2;
//   uint8_t v2;
//   uint8_t angle2;
//   uint8_t dis3;
//   uint8_t v3;
//   uint8_t angle3;
// };

class LocalizationNode : public rclcpp::Node
{
public:
  LocalizationNode()
  : Node("localization")
  {
    publisher_ = this->create_publisher<localization_msgs::msg::Localization>("localization", 10);

    // 启动接收线程
    int ret = pthread_create(&can_thread_, nullptr,
                             &LocalizationNode::canThreadHelper, this);
    if (ret != 0) {
      RCLCPP_FATAL(this->get_logger(), "创建 CAN 线程失败，错误码：%d", ret);
      rclcpp::shutdown();
    }
  }

  ~LocalizationNode() override
  {
    pthread_cancel(can_thread_);
    pthread_join(can_thread_, nullptr);
  }

private:
  /** ---------------- 业务逻辑 ---------------- **/

  void rtkDataInput(struct can_frame frames[5])
  {
    static localization_msgs::msg::Localization loc;

    /* ---------------- 解析不同 ID ---------------- */
    if ((frames[0].can_id & CAN_SFF_MASK) == RTK_1) {
      loc.satellite_status   = frames[0].data[2];
      loc.system_state       = frames[0].data[0];
      loc.gps_num_sats_used  = frames[0].data[1];
    }
    if ((frames[1].can_id & CAN_SFF_MASK) == RTK_2) {
      loc.gps.lat = ((frames[1].data[0] << 24) | (frames[1].data[1] << 16) |
                     (frames[1].data[2] <<  8) |  frames[1].data[3]) * 0.0000001;
      loc.gps.lon = ((frames[1].data[4] << 24) | (frames[1].data[5] << 16) |
                     (frames[1].data[6] <<  8) |  frames[1].data[7]) * 0.0000001;
    }
    if ((frames[2].can_id & CAN_SFF_MASK) == RTK_3) {
      loc.pose.yaw   = ((frames[2].data[0] << 8) | frames[2].data[1]) * 0.01;
      loc.pose.pitch = ((frames[2].data[2] << 8) | frames[2].data[3]) * 0.01;
      loc.pose.roll  = ((frames[2].data[4] << 8) | frames[2].data[5]) * 0.01;
    }

    /* ---------------- WGS‑84 → UTM / GCCS ---------------- */
    GeoTool geo;
    PointGPS gps_;
    PointGCCS gccs_;
    gps_.lat    = loc.gps.lat;
    gps_.lon    = loc.gps.lon;
    gps_.heading= loc.pose.yaw;
    int zone    = geo.GetLongZone(gps_.lon);
    geo.SetUtmZone(zone);
    geo.GPS2GCCS(gps_, gccs_);

    constexpr double X0 = 537854.690940;
    constexpr double Y0 = 4337387.744176;

    loc.xy.x = gccs_.xg - X0;
    loc.xy.y = gccs_.yg - Y0;
    loc.xy.z = loc.pose.yaw;
    loc.system_time = this->get_clock()->now().seconds();

    publisher_->publish(loc);
  }

  /** ---------------- CAN 线程 ---------------- **/

  static void * canThreadHelper(void * context)
  {
    auto * self = static_cast<LocalizationNode *>(context);
    self->canLoop();
    return nullptr;
  }

  void canLoop()
  {
    // 确保 CAN 口已开启（调用外部脚本）
    system("echo 'nvidia' | sudo -S /home/nvidia/ZHITAI/qingling_ros2/src/localization/gps_localization/can_start.sh");

    /* --------- 初始化 SocketCAN --------- */
    int can_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_sock < 0) {
      RCLCPP_FATAL(this->get_logger(), "Socket 创建失败");
      rclcpp::shutdown();
      return;
    }

    struct ifreq ifr {};
    strcpy(ifr.ifr_name, "can1");
    ioctl(can_sock, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr {};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(can_sock, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr));

    /* --------- 过滤器 --------- */
    struct can_filter filters[3];
    filters[0].can_id   = RTK_1; filters[0].can_mask = CAN_SFF_MASK;
    filters[1].can_id   = RTK_2; filters[1].can_mask = CAN_SFF_MASK;
    filters[2].can_id   = RTK_3; filters[2].can_mask = CAN_SFF_MASK;
    setsockopt(can_sock, SOL_CAN_RAW, CAN_RAW_FILTER, &filters, sizeof(filters));

    std::ofstream ofs("gps.txt", std::ios::out);

    rclcpp::Rate rate(50);
    while (rclcpp::ok()) {
      struct can_frame frame_set[5];
      bool got1 = false, got2 = false, got3 = false;

      /* -- 轮询直到收齐三帧 -- */
      while (rclcpp::ok()) {
        struct can_frame frame {};
        ssize_t nbytes = read(can_sock, &frame, sizeof(frame));
        if (nbytes != sizeof(frame)) {
          RCLCPP_WARN(this->get_logger(), "CAN 帧读取异常");
          continue;
        }

        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count();
        ofs << std::dec << ms << '\t'
            << std::hex << int(frame.can_id & CAN_EFF_MASK) << '\t';
        for (int i = 0; i < 8; ++i) ofs << int(frame.data[i]) << '\t';
        ofs << std::endl;

        uint32_t id = frame.can_id & CAN_EFF_MASK;
        if (id == RTK_1) { got1 = true; frame_set[0] = frame; }
        else if (id == RTK_2) { got2 = true; frame_set[1] = frame; }
        else if (id == RTK_3) { got3 = true; frame_set[2] = frame; }

        if (got1 && got2 && got3)
          break;
      }

      if (got1 && got2 && got3)
        rtkDataInput(frame_set);

      rate.sleep();
    }

    close(can_sock);
  }

  /** ---------------- 成员 ---------------- **/
  rclcpp::Publisher<localization_msgs::msg::Localization>::SharedPtr publisher_;
  pthread_t can_thread_;
};

/** ---------------- main ---------------- **/
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LocalizationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
