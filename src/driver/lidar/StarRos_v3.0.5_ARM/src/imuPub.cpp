// #include "ros/ros.h"
// #include "std_msgs/msg/string.hpp"
// #include <unistd.h>
// #include <netinet/in.h>
// #include <sys/socket.h>
// #include <arpa/inet.h>
// #include <poll.h>
// #include <string>
// #include <sstream>
// #include <iostream>
// #include <sdk-decoder/Socket.h>
// #include <thread>
// #include <stdlib.h>
// #include <cstdlib>
// #include<rfans_driver/msg/imu_data.hpp>
// #include <fstream>
// using namespace std;
// using namespace ss;
// #define IMU_PACKET_NUM 50
// #define pi 3.1415926
// //byte alignment
// #pragma pack(1)
// typedef struct
// {
//   uint8_t year;
//   uint8_t month;
//   uint8_t day ;
//   uint8_t hour;
// }TIME_PACKET;
// typedef struct
// {
//   uint16_t flag;
//   uint16_t UDPcnt;
//   TIME_PACKET UTCtime;
//   uint8_t IMUID;
//   uint16_t IMUTemperature;
//   unsigned char reserve[21];
// }IMU_HEADER;//32 byte
// typedef struct
// { uint32_t utctime_data;
//   int16_t accel_x;
//   int16_t accel_y;
//   int16_t accel_z;
//   int16_t gyro_x;
//   int16_t gyro_y;
//   int16_t gyro_z;
//   int16_t ROLL;
//   int16_t pitch;
//   int16_t yaw;
// }IMU_DATE;//22 bytes

// typedef struct
// {
//   char DataTail[2];
// }IMU_TRAILE;//2 bytes
// typedef struct
// {
//   IMU_HEADER header;
//   IMU_DATE data[50];
//   IMU_TRAILE trail;
// }IMU_PACKET;//32+22*50+2=1134byte
// #pragma pack()



// int ssWeekDay(int yy, int mm, int dd)//year,month,day
// {
//   int weekDay = 0;

//   int i , days = 0 ,s ;
//   int mont[13]={0,31,28,31,30,31,30,31,31,30,31,30,31} ;//month has days

//   yy = 2000 + yy ;
//   //liyp 4年一润，百年不润，400年一润
//   if ( ( (yy % 4 == 0) && (yy % 100 !=0) ) || (yy % 400 == 0) ) {//判断闰年，是闰年二月为29天。
//     mont[2] = 29 ;
//   } else {
//     mont[2] = 28 ;
//   }
//   for( i=0 ; i < mm ; i++ ) { //计算是一年中的第几天
//     days += mont[i] ;
//   }
//   days += dd ;

//   s= yy-1+(int)((yy-1)/4)-(int)((yy-1)/100)+(int)((yy-1)/400)+days ;
//   weekDay = s % 7 ;
//   return weekDay;
// }


// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "imuPub");
//   ros::NodeHandle nh;
//   TIME_PACKET time;
//   rfans_driver::msg::ImuData IMUPUB;
//   double utcStamp;
//   int weekDay_;
//   float AccelCoff;
//   float GyroCoff;
//   float angleCoff;

//   ros::Publisher chatter_pub = nh.advertise<rfans_driver::msg::ImuData>("ImuTopic", 1000);
//   ros::Rate loop_rate(10);

//   char buffer[sizeof (IMU_PACKET)];
//   memset(buffer,sizeof (IMU_PACKET),1);
//   IMU_PACKET* pt;
//   IMU_HEADER header_;
//   ofstream file;
//   file.open("/home/bkth/lc/imuraw_sroll-y.txt");
//   Socket udp = Socket::udp();
//   if (!udp.bind(InternetEndpoint(2025)) )
//   {
//     std::cerr << "connect failed" << std::endl;
//   }
//   bool flag=true;
//   while (ros::ok())
//   {
//     const ssize_t ret = udp.read(buffer, sizeof (buffer));
//     if(ret < 0)
//     {
//       std::cout << "read error:" << std::endl;
//       break;
//     }
//     pt=(IMU_PACKET*)buffer;
//     if(flag)
//     {

//       if(file.is_open())// save utctime
//       {
//         file<<"year:"<<(int)pt->header.UTCtime.year<<"\n";
//         file<<"month:"<<(int)pt->header.UTCtime.month<<"\n";
//         file<<"day:"<<(int)pt->header.UTCtime.day<<"\n";
//         file<<"hour:"<<(int)pt->header.UTCtime.hour<<"\n";
//       }

//       /*find imuID,otherwise there is no return value,going to around*/
//       if(pt->header.IMUID==0X01)
//       {
//         AccelCoff=0.061*0.0098;
//         GyroCoff=4.37*pi/180*0.001;
//         angleCoff=1.0;
//         flag=false;
//       }else if(pt->header.IMUID==0X02)
//       {
//         AccelCoff=1.0/2048;
//         GyroCoff=125.0/2048;
//         angleCoff=45.0/8196;
//         flag=false;
//       }

//     }
//     weekDay_=ssWeekDay(pt->header.UTCtime.year,pt->header.UTCtime.month,pt->header.UTCtime.day);
//     utcStamp = (double)(weekDay_*86400 + pt->header.UTCtime.hour*3600);
//     for (int i=0;i<IMU_PACKET_NUM;i++)
//     {
//       IMUPUB.time=pt->data[i].utctime_data*0.000001+utcStamp;
//       IMUPUB.accel_x=pt->data[i].accel_x*AccelCoff;
//       IMUPUB.accel_y=pt->data[i].accel_y*AccelCoff;
//       IMUPUB.accel_z=pt->data[i].accel_z*AccelCoff;
//       IMUPUB.gyro_x=pt->data[i].gyro_x*GyroCoff;
//       IMUPUB.gyro_y=pt->data[i].gyro_y*GyroCoff;
//       IMUPUB.gyro_z=pt->data[i].gyro_z*GyroCoff;
//       IMUPUB.yaw=pt->data[i].yaw*angleCoff;
//       IMUPUB.roll=pt->data[i].ROLL*angleCoff;
//       IMUPUB.pitch=pt->data[i].pitch*angleCoff;
//       chatter_pub.publish(IMUPUB);
//     }
//   }
//   file.close();
//   return 0;
// }
#include "rclcpp/rclcpp.hpp"
#include "rfans_driver/msg/imu_data.hpp"
#include <unistd.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sdk-decoder/Socket.h>
#include <fstream>
#include <cmath>
#include <cstdlib>
#include <cstring>

#define IMU_PACKET_NUM 50
#define PI 3.1415926

using std::placeholders::_1;
using namespace std;
using namespace ss;

#pragma pack(1)
typedef struct { uint8_t year, month, day, hour; } TIME_PACKET;
typedef struct {
  uint16_t flag;
  uint16_t UDPcnt;
  TIME_PACKET UTCtime;
  uint8_t IMUID;
  uint16_t IMUTemperature;
  unsigned char reserve[21];
} IMU_HEADER;

typedef struct {
  uint32_t utctime_data;
  int16_t accel_x, accel_y, accel_z;
  int16_t gyro_x, gyro_y, gyro_z;
  int16_t ROLL, pitch, yaw;
} IMU_DATE;

typedef struct { char DataTail[2]; } IMU_TRAILE;

typedef struct {
  IMU_HEADER header;
  IMU_DATE data[IMU_PACKET_NUM];
  IMU_TRAILE trail;
} IMU_PACKET;
#pragma pack()

int ssWeekDay(int yy, int mm, int dd)
{
  int mont[13] = {0,31,28,31,30,31,30,31,31,30,31,30,31};
  yy += 2000;
  if ((yy % 4 == 0 && yy % 100 != 0) || (yy % 400 == 0)) mont[2] = 29;
  int days = dd;
  for (int i = 0; i < mm; ++i) days += mont[i];
  int s = yy - 1 + (yy - 1)/4 - (yy - 1)/100 + (yy - 1)/400 + days;
  return s % 7;
}

class ImuPublisher : public rclcpp::Node
{
public:
  ImuPublisher() : Node("imu_pub_node")
  {
    imu_pub_ = this->create_publisher<rfans_driver::msg::ImuData>("ImuTopic", 1000);
    file_.open("/home/bkth/lc/imuraw_sroll-y.txt");

    udp_ = ss::Socket::udp();
    if (!udp_.bind(ss::InternetEndpoint(2025))) {
      RCLCPP_ERROR(this->get_logger(), "UDP bind failed");
      return;
    }

    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
      std::bind(&ImuPublisher::read_udp_loop, this));
  }

private:
  void read_udp_loop()
  {
    char buffer[sizeof(IMU_PACKET)] = {0};
    ssize_t ret = udp_.read(buffer, sizeof(buffer));
    if (ret < 0) {
      RCLCPP_WARN(this->get_logger(), "Read error");
      return;
    }

    auto pt = reinterpret_cast<IMU_PACKET*>(buffer);
    if (!pt) return;

    if (first_) {
      file_ << "year:" << (int)pt->header.UTCtime.year << "\n"
            << "month:" << (int)pt->header.UTCtime.month << "\n"
            << "day:" << (int)pt->header.UTCtime.day << "\n"
            << "hour:" << (int)pt->header.UTCtime.hour << "\n";
      first_ = false;

      if (pt->header.IMUID == 0x01) {
        accel_coeff_ = 0.061 * 0.0098;
        gyro_coeff_ = 4.37 * PI / 180 * 0.001;
        angle_coeff_ = 1.0;
      } else if (pt->header.IMUID == 0x02) {
        accel_coeff_ = 1.0 / 2048;
        gyro_coeff_ = 125.0 / 2048;
        angle_coeff_ = 45.0 / 8196;
      }
    }

    int week_day = ssWeekDay(pt->header.UTCtime.year, pt->header.UTCtime.month, pt->header.UTCtime.day);
    double utc_base = week_day * 86400 + pt->header.UTCtime.hour * 3600;

    for (int i = 0; i < IMU_PACKET_NUM; ++i) {
      auto msg = rfans_driver::msg::ImuData();
      msg.time = pt->data[i].utctime_data * 1e-6 + utc_base;
      msg.accel_x = pt->data[i].accel_x * accel_coeff_;
      msg.accel_y = pt->data[i].accel_y * accel_coeff_;
      msg.accel_z = pt->data[i].accel_z * accel_coeff_;
      msg.gyro_x  = pt->data[i].gyro_x  * gyro_coeff_;
      msg.gyro_y  = pt->data[i].gyro_y  * gyro_coeff_;
      msg.gyro_z  = pt->data[i].gyro_z  * gyro_coeff_;
      msg.yaw     = pt->data[i].yaw     * angle_coeff_;
      msg.roll    = pt->data[i].ROLL    * angle_coeff_;
      msg.pitch   = pt->data[i].pitch   * angle_coeff_;
      imu_pub_->publish(msg);
    }
  }

  rclcpp::Publisher<rfans_driver::msg::ImuData>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::ofstream file_;
  ss::Socket udp_;
  bool first_ = true;
  float accel_coeff_ = 1.0;
  float gyro_coeff_ = 1.0;
  float angle_coeff_ = 1.0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPublisher>());
  rclcpp::shutdown();
  return 0;
}
