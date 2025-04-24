// /* -*- mode: C++ -*-
//  *  All right reserved, Sure_star Coop.
//  *  @Technic Support: <sdk@isurestar.com>
//  *  $Id$
//  */
// #include <ros/ros.h>
// #include "rfans_driver.h"
// #include <dlfcn.h>
// ros::Publisher  pub_cloud;
// double min_range=0;
// double max_range=500;
// double min_angle=0;
// double max_angle=361;
// int ringID;
// bool is_lidarId_slc;
// bool distortion_flag;
// float Angle_resolution;
// bool is_Rfans;
// Matrix3d R;
// double RTX[4];
// double RTY[4];
// double RTZ[4];
// bool transflag;
// #ifndef PI
// #define PI 3.14159
// #endif

// void callback(rfans_driver::FilterParamsConfig &config, uint32_t level){
//   //ROS_INFO("callback");
//   min_range = config.min_range;
//   max_range = config.max_range;
//   if(is_Rfans)
//   {
//     config.rfans_angleSelection=true;
//     min_angle = config.min_angle;
//     max_angle = config.max_angle;
//     config.cfans_angleSelection =false;
//   }
//   else
//   {
//     config.cfans_angleSelection =true;
//     min_angle = config.cfans_min_angle;
//     max_angle = config.cfans_max_angle;
//     config.rfans_angleSelection = false;
//   }

//   is_lidarId_slc = config.use_laserSelection;
//   ringID = config.laserID;
// }

// void string_date(string& str,vector<float>& num)
// {
//   char*s_input=(char*) str.c_str();
//   const char * split=",";
//   char*p=strtok(s_input,split);
//   float a;
//   while(p!=NULL)
//   {
//     a=atof(p);
//     num.push_back(a);
//     p=strtok(NULL,split);//如果第一个参数为空，则函数保存的指针SAVE_ptr再下一次调用作位置；

//   };

// }

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "rfans_driver");
//   ros::NodeHandle node;
//   ros::NodeHandle nh("~");
//   rfans_driver::Rfans_Driver* driver = new rfans_driver::Rfans_Driver(node, nh);
//   pub_cloud = node.advertise<sensor_msgs::PointCloud2>("points_raw", 3);
//   dynamic_reconfigure::Server<rfans_driver::FilterParamsConfig> server;
//   dynamic_reconfigure::Server<rfans_driver::FilterParamsConfig>::CallbackType f;
//   f = boost::bind(&callback,_1,_2);
//   server.setCallback(f);
//   string str;
//   vector<float> NUM;
//   node.param<string>("RT",str,"0.0,0.0,0.0,0.0,0.0,0.0,0");
//   string_date(str,NUM);
//   AngleAxisd Rx(NUM[0]*PI/180,Eigen::Vector3d::UnitX());
//   AngleAxisd Ry(NUM[1]*PI/180,Eigen::Vector3d::UnitY());
//   AngleAxisd Rz(NUM[2]*PI/180,Eigen::Vector3d::UnitZ());
//   R=Rz*Ry*Rx;
//   transflag=NUM[6];
//   //Vector3d TXYZ1(NUM[3],NUM[4],NUM[5]);
//   RTX[0]=R.coeffRef(0,0);
//   RTX[1]=R.coeffRef(0,1);
//   RTX[2]=R.coeffRef(0,2);
//   RTX[3]=NUM[3];


//   RTY[0]=R.coeffRef(1,0);
//   RTY[1]=R.coeffRef(1,1);
//   RTY[2]=R.coeffRef(1,2);
//   RTY[3]=NUM[4];

//   RTZ[0]=R.coeffRef(2,0);
//   RTZ[1]=R.coeffRef(2,1);
//   RTZ[2]=R.coeffRef(2,2);
//   RTZ[3]=NUM[5];


//   while (ros::ok())
//   {
//     driver->spinOnce();
//   }


//   return 0;
// }

/* -*- mode: C++ -*-
 *  Converted to ROS 2 (Foxy+ / Galactic+)
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Dense>
#include "rfans_driver.h"

using Eigen::Matrix3d;
using Eigen::AngleAxisd;
using std::placeholders::_1;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud;

double min_range = 0.0;
double max_range = 500.0;
double min_angle = 0.0;
double max_angle = 361.0;
int ringID = 0;
bool is_lidarId_slc = false;
bool distortion_flag = false;
float Angle_resolution = 0.0f;
bool is_Rfans = false;
Matrix3d R;
double RTX[4], RTY[4], RTZ[4];
bool transflag = false;

// 把“0.0,1.0,2.0…”这种逗号分隔字符串转成 float vector
void string_date(const std::string &str, std::vector<float> &num)
{
  char *s = strdup(str.c_str());
  const char *delim = ",";
  for (char *p = strtok(s, delim); p; p = strtok(NULL, delim)) {
    num.push_back(static_cast<float>(atof(p)));
  }
  free(s);
}

// 参数修改回调
rcl_interfaces::msg::SetParametersResult parameters_callback(
  const std::vector<rclcpp::Parameter> &params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto &p : params) {
    const auto &name = p.get_name();
    if (name == "min_range") {
      min_range = p.as_double();
    } else if (name == "max_range") {
      max_range = p.as_double();
    } else if (name == "min_angle") {
      min_angle = p.as_double();
    } else if (name == "max_angle") {
      max_angle = p.as_double();
    } else if (name == "use_laserSelection") {
      is_lidarId_slc = p.as_bool();
    } else if (name == "laserID") {
      ringID = p.as_int();
    }
    // 如果需要，还可以在这里处理更多参数…
  }
  return result;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rfans_driver");

  // 1. 声明参数并给出默认值
  node->declare_parameter<double>("min_range", 0.0);
  node->declare_parameter<double>("max_range", 500.0);
  node->declare_parameter<double>("min_angle", 0.0);
  node->declare_parameter<double>("max_angle", 361.0);
  node->declare_parameter<bool>("use_laserSelection", false);
  node->declare_parameter<int>("laserID", 0);
  node->declare_parameter<std::string>("RT", "0.0,0.0,0.0,0.0,0.0,0.0,0");

  // 2. 创建发布者
  pub_cloud = node->create_publisher<sensor_msgs::msg::PointCloud2>("points_raw", 3);

  // 3. 注册参数修改回调
  node->add_on_set_parameters_callback(
    std::bind(parameters_callback, _1)
  );

  // 4. 读取并解析 RT 转换参数
  std::string rt_str;
  node->get_parameter("RT", rt_str);
  std::vector<float> NUM;
  string_date(rt_str, NUM);
  // 按原来顺序计算旋转矩阵 R 和平移向量
  AngleAxisd Rx(NUM[0] * M_PI/180.0, Eigen::Vector3d::UnitX());
  AngleAxisd Ry(NUM[1] * M_PI/180.0, Eigen::Vector3d::UnitY());
  AngleAxisd Rz(NUM[2] * M_PI/180.0, Eigen::Vector3d::UnitZ());
  R = Rz * Ry * Rx;
  transflag = (NUM.size() > 6 ? NUM[6] : false);

  RTX[0]=R(0,0); RTX[1]=R(0,1); RTX[2]=R(0,2); RTX[3]=NUM[3];
  RTY[0]=R(1,0); RTY[1]=R(1,1); RTY[2]=R(1,2); RTY[3]=NUM[4];
  RTZ[0]=R(2,0); RTZ[1]=R(2,1); RTZ[2]=R(2,2); RTZ[3]=NUM[5];

  // 5. 实例化驱动
  auto driver_node = std::make_shared<rfans_driver::Rfans_Driver>(
    rclcpp::NodeOptions().allow_undeclared_parameters(true) );


  // 6. 用定时器周期性调用 spinOnce()
  rclcpp::TimerBase::SharedPtr timer = node->create_wall_timer(
    10ms,
    [driver_node]() { driver_node->spinOnce(); }   // <-- 加 value 捕获
    // 或者 [&] { driver->spinOnce(); }  // 整个外部引用都捕获
  );

  // 7. 进入 ROS 2 事件循环
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
