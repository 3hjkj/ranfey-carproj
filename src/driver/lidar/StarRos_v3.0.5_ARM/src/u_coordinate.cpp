/*******************************************************
 * fusion_node.cpp — Multi-LiDAR fusion node, ROS 2 Humble
 *******************************************************/
#include <memory>
#include <fstream>
#include <vector>
#include <array>
#include <iostream>
#include <functional>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "ssFrameLib.h"     // 含 TransClound_S 定义
#include "ioapi.h"
#include "rfans_driver.h"

#include <Eigen/Eigen>
#include <unistd.h>

using namespace std::placeholders;
namespace mf = message_filters;
using  PointCloud2 = sensor_msgs::msg::PointCloud2;
using  PointCloud2Ptr = PointCloud2::ConstSharedPtr;

/* ------------ 全局变量 ------------ */
static rclcpp::Publisher<PointCloud2>::SharedPtr fusion_pub;
static PointCloud2  msg_pub;
static std::ofstream file;
static std::vector<int> num_Pub;

/* ------------ PointCloud2 元数据 ------------ */
void InitPointcloud2(PointCloud2 &c)
{
  c.data.clear();
  c.is_bigendian = false;
  c.is_dense     = false;
  c.fields.resize(10);            // 7+3 (最后 3 个按你需要可删)
  uint32_t off = 0;
  auto set_f=[&](int i,const std::string &n,uint8_t dt,uint32_t sz){
     c.fields[i].name=n; c.fields[i].datatype=dt;
     c.fields[i].offset=off; c.fields[i].count=1; off+=sz; };
  set_f(0,"x",         7u,4);
  set_f(1,"y",         7u,4);
  set_f(2,"z",         7u,4);
  set_f(3,"intensity", 7u,4);
  set_f(4,"laserid",   5u,4);
  set_f(5,"timeflag",  7u,4);
  set_f(6,"hangle",    7u,4);
  /* 如确有 pulseWidth/range/rol/mirrorid，请自行打开
  set_f(7,"pulseWidth",7u,4);
  set_f(8,"range",     7u,4);
  set_f(9,"rol",       5u,4);
  */
  c.height      = 1;
  c.point_step  = sizeof(TransClound_S);
  c.row_step    = 0;
  c.width       = 0;
  c.header.frame_id = "world";
}

/* ------------ 帮助函数 ------------ */
inline void fill_and_pub(PointCloud2 &out,
                         const std::vector<const PointCloud2*> &ins)
{
  out.header.stamp = rclcpp::Clock().now();
  out.height       = 1;
  out.point_step   = sizeof(TransClound_S);
  size_t total = 0; for(auto p:ins) total+=p->width;
  out.width = total;
  out.data.resize(total * out.point_step);
  out.row_step = out.data.size();
  size_t cursor=0;
  for(auto p:ins){
    std::memcpy(out.data.data()+cursor, p->data.data(), p->data.size());
    cursor += p->data.size();
  }
  fusion_pub->publish(out);
}
inline void dump_cloud(const PointCloud2Ptr &pc)
{
  const auto *ptr = reinterpret_cast<const TransClound_S*>(pc->data.data());
  for(uint32_t i=0;i<pc->width;++i){
    file<<ptr[i].x<<','<<ptr[i].y<<','<<ptr[i].z<<','
        <<ptr[i].intent<<','<<ptr[i].timeflag<<','
        <<ptr[i].laserid<<','<<ptr[i].hangle
        /* 若真有 mirrorid 字段 → “<<','<<ptr[i].mirrorid” */
        <<'\n';
  }
}

/* ---------- pub 回调 ---------- */
void pub1 (const PointCloud2Ptr &a){ fill_and_pub(msg_pub,{a.get()}); }
void pub2 (const PointCloud2Ptr &a,const PointCloud2Ptr &b){
  fill_and_pub(msg_pub,{a.get(),b.get()}); }
void pub3 (const PointCloud2Ptr &a,const PointCloud2Ptr &b,
           const PointCloud2Ptr &c){
  fill_and_pub(msg_pub,{a.get(),b.get(),c.get()}); }
void pub4 (const PointCloud2Ptr &a,const PointCloud2Ptr &b,
           const PointCloud2Ptr &c,const PointCloud2Ptr &d){
  fill_and_pub(msg_pub,{a.get(),b.get(),c.get(),d.get()}); }

/* ---------- save 回调 ---------- */
void save1(const PointCloud2Ptr &a){ dump_cloud(a); }
void save2(const PointCloud2Ptr &a,const PointCloud2Ptr &b){
  dump_cloud(a); dump_cloud(b); }
void save3(const PointCloud2Ptr &a,const PointCloud2Ptr &b,
           const PointCloud2Ptr &c){
  dump_cloud(a); dump_cloud(b); dump_cloud(c); }
void save4(const PointCloud2Ptr &a,const PointCloud2Ptr &b,
           const PointCloud2Ptr &c,const PointCloud2Ptr &d){
  dump_cloud(a); dump_cloud(b); dump_cloud(c); dump_cloud(d); }

/* ---------- 指针别名，方便 static_cast ---------- */
using CB1 = void(*)(const PointCloud2Ptr &);
using CB2 = void(*)(const PointCloud2Ptr &,const PointCloud2Ptr &);
using CB3 = void(*)(const PointCloud2Ptr &,const PointCloud2Ptr &,
                   const PointCloud2Ptr &);
using CB4 = void(*)(const PointCloud2Ptr &,const PointCloud2Ptr &,
                   const PointCloud2Ptr &,const PointCloud2Ptr &);

/* =================================================== */
int main(int argc,char **argv)
{
  rclcpp::init(argc,argv);
  auto node = rclcpp::Node::make_shared("fusion_node");

  InitPointcloud2(msg_pub);
  fusion_pub = node->create_publisher<PointCloud2>("fusion_point",10);

  const std::vector<std::string> topics={
      "/ns1/lidar_points","/ns2/lidar_points",
      "/ns3/lidar_points","/ns4/lidar_points"};

  /* 读取“是否启用”参数 */
  const std::array<std::string,4> keys={
      "ns1.rfans_driver.Is_Start","ns2.rfans_driver.Is_Start",
      "ns3.rfans_driver.Is_Start","ns4.rfans_driver.Is_Start"};
  bool start[4]{};
  for(int i=0;i<4;++i){
    node->declare_parameter<bool>(keys[i],false);
    node->get_parameter(keys[i],start[i]);
    if(start[i]) num_Pub.push_back(i);
  }

  bool save=node->declare_parameter<bool>("save_xyz",false);
  std::string path=node->declare_parameter<std::string>(
      "OutExport_path","/tmp/out.txt");
  if(save){
    file.open(path);
    file<<"x,y,z,intensity,timeflag,laserid,hangle\n";  //7列
  }

  auto qos = rclcpp::SensorDataQoS();

  /* =========== 不同雷达数量分支 =========== */
  switch(num_Pub.size()){
    case 1:{
      node->create_subscription<PointCloud2>(
        topics[num_Pub[0]], qos,
        static_cast<CB1>(&pub1));
      if(save){
        node->create_subscription<PointCloud2>(
          topics[num_Pub[0]], qos,
          static_cast<CB1>(&save1));
      }
      break;
    }
    case 2:{
      using Sync = mf::sync_policies::ApproximateTime<PointCloud2,PointCloud2>;
      mf::Subscriber<PointCloud2> s1(node,topics[num_Pub[0]],qos.get_rmw_qos_profile());
      mf::Subscriber<PointCloud2> s2(node,topics[num_Pub[1]],qos.get_rmw_qos_profile());
      auto sync = std::make_shared<mf::Synchronizer<Sync>>(Sync(10),s1,s2);
      sync->registerCallback(static_cast<CB2>(&pub2));
      if(save){
        auto sync_s = std::make_shared<mf::Synchronizer<Sync>>(Sync(10),s1,s2);
        sync_s->registerCallback(static_cast<CB2>(&save2));
      }
      break;
    }
    case 3:{
      using Sync = mf::sync_policies::ApproximateTime<PointCloud2,PointCloud2,PointCloud2>;
      mf::Subscriber<PointCloud2> s1(node,topics[num_Pub[0]],qos.get_rmw_qos_profile());
      mf::Subscriber<PointCloud2> s2(node,topics[num_Pub[1]],qos.get_rmw_qos_profile());
      mf::Subscriber<PointCloud2> s3(node,topics[num_Pub[2]],qos.get_rmw_qos_profile());
      auto sync = std::make_shared<mf::Synchronizer<Sync>>(Sync(10),s1,s2,s3);
      sync->registerCallback(static_cast<CB3>(&pub3));
      if(save){
        auto sync_s = std::make_shared<mf::Synchronizer<Sync>>(Sync(10),s1,s2,s3);
        sync_s->registerCallback(static_cast<CB3>(&save3));
      }
      break;
    }
    case 4:{
      using Sync = mf::sync_policies::ApproximateTime<PointCloud2,PointCloud2,PointCloud2,PointCloud2>;
      mf::Subscriber<PointCloud2> s1(node,topics[num_Pub[0]],qos.get_rmw_qos_profile());
      mf::Subscriber<PointCloud2> s2(node,topics[num_Pub[1]],qos.get_rmw_qos_profile());
      mf::Subscriber<PointCloud2> s3(node,topics[num_Pub[2]],qos.get_rmw_qos_profile());
      mf::Subscriber<PointCloud2> s4(node,topics[num_Pub[3]],qos.get_rmw_qos_profile());
      auto sync = std::make_shared<mf::Synchronizer<Sync>>(Sync(10),s1,s2,s3,s4);
      sync->registerCallback(static_cast<CB4>(&pub4));
      if(save){
        auto sync_s = std::make_shared<mf::Synchronizer<Sync>>(Sync(10),s1,s2,s3,s4);
        sync_s->registerCallback(static_cast<CB4>(&save4));
      }
      break;
    }
    default:
      RCLCPP_ERROR(node->get_logger(),"No lidar enabled, node exits.");
      rclcpp::shutdown();
      return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
