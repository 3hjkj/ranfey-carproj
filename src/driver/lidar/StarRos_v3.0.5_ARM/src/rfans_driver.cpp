/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */
#include <unistd.h>
#include <string>
#include <vector>
#include <cstring>
#include <cstdio>
#include <sstream>
#include <sys/socket.h>
#include <sys/uio.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <Eigen/Eigen>
#include <queue>
#include<condition_variable>
#include<unistd.h>
#include "rfans_driver.h"
#include "rfans_driver/srv/rfans_command.hpp"
#include "rfans_driver/msg/rfans_scan.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>
#include <chrono>
#include <sys/time.h>
#include "lidar_sdk/config.h"

static rclcpp::Subscription<rfans_driver::msg::Command>::SharedPtr subCommond;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud;
const float timeVal =0.01;
const float xyVal =2 ;
const float zVal =1 ;
extern double min_range;
extern double max_range;
extern double min_angle;
extern double max_angle;
extern int ringID;
extern bool is_lidarId_slc;
extern bool is_Rfans;
extern double RTX[4];
extern double RTY[4];
extern double RTZ[4];
extern bool transflag;
static int numMirror[4]={0};
vector<TransClound_S> buff_mirror_cloud[4];
vector<TransClound_S> buff_restruct_cloud[4];
static const int MAX_ISF_SIZE=0X8000000;
namespace rfans_driver {
vector<TransClound_S> buff_original_cloud;
static Rfans_Driver *s_this = NULL;
static std::queue<DecStream_S> packets_queue;//队列储存解码
static std::mutex queue_mutex;
static std::condition_variable cond;
vector<char> temp_buf;

void creatBuf(SDK_PARA_S sdk_para_,char* header_buff_,  DataBuff_S& rawStream_, DecStream_S& decStream_, CalcStream_S&  calcStream_)
{
  //（1）配置buff的大小
  int blockPtN = 0;
  header_buff_ = NULL;
  header_buff_ = new  char[ISF_HEADER];
  memset(header_buff_, 0, ISF_HEADER);
  blockPtN = BLOCK_SIZE / PACKET0_SIZE * PACKET0_POINT_NUM*DEC_BLOCK_COFF;//固定的udp计数（1412字节192点）
  //（2）解码输入流buff，存储文件流
  rawStream_._ptr = new char[IMP_ISF_SIZE];
  rawStream_.buff_capacity = IMP_ISF_SIZE;
  //（3）解码buff
  decStream_._lpoints = (LaserPoint_S*) new char[blockPtN * sizeof(LaserPoint_S)];
  decStream_.buff_boundary = blockPtN * sizeof(LaserPoint_S);
  decStream_.las_num = 0;
  memset(decStream_._lpoints, 0, decStream_.buff_boundary);
  // (4)解码错误日志输出buff
  decStream_.error_log._ptr = new char[ERORRLOG_BUFFSIZE];
  decStream_.error_log.buff_capacity = ERORRLOG_BUFFSIZE;
  decStream_.error_log.cur_size = 0;
  // (5)解算buff
  calcStream_._calcPt = (CalcLaserPt_S*)new char[blockPtN * sizeof(CalcLaserPt_S)];
  calcStream_.buff_size = blockPtN * sizeof(CalcLaserPt_S);
  calcStream_.error_log._ptr = new char[ERORRLOG_BUFFSIZE];
  memset(calcStream_.error_log._ptr, 0, ERORRLOG_BUFFSIZE);
  calcStream_.error_log.buff_capacity = ERORRLOG_BUFFSIZE;
  calcStream_.error_log.cur_size = 0;
  // (6)慢数据buff
  decStream_.slow_stream.imu_data._ptr = new char[0X5000000];//5M buff 给慢数据IMU创建buff
  decStream_.slow_stream.imu_data.buff_capacity = 0X5000000;
  decStream_.slow_stream.imu_data.cur_size = 0;
  decStream_.slow_stream.imu_txt._ptr = new char[0X5000000];
  decStream_.slow_stream.imu_txt.buff_capacity = 0X5000000;
  decStream_.slow_stream.imu_txt.cur_size = 0;
  return;
}

/*
*控制转速和单双回波
*/
void CommandHandle(const std::shared_ptr<rfans_driver::msg::Command> req)
{
  RCLCPP_INFO(rclcpp::get_logger("rfans_driver"),
              "request: cmd = %d, speed = %d Hz",
              static_cast<int>(req->use_double_echo),
              static_cast<int>(req->speed));

  lidarAPi::DEB_PROGRM_S tmpProg;
  unsigned int tmpData = 0;

  tmpProg.cmdstat    = static_cast<lidarAPi::DEB_CMD_E>(req->cmd);  // 控制设备是否待机
  tmpProg.dataFormat = lidarAPi::eFormatCalcData;
  tmpProg.scnSpeed   = req->speed;
  tmpProg.dataLevel  = static_cast<int>(req->use_double_echo);

  tmpData = (tmpProg.dataLevel == 0) ? CFANS_ECHO : CFANS_DUAL;

  if (s_this) {
    s_this->progSet(tmpProg);
    s_this->getDevInstance()->HW_WRREG(0, REG_DATA_LEVEL_OLD, tmpData);
    s_this->getDevInstance()->HW_WRREG(0, REG_DATA_LEVEL,     tmpData);
  }
}




void Rfans_Driver::stopDevice()
{
  lidarAPi::DEB_PROGRM_S params;
  params.cmdstat = lidarAPi::eDevCmdWork;
  params.dataFormat = lidarAPi::eFormatCalcData;
  params.scnSpeed =ANGLE_SPEED_0HZ_cfan;//stop
  progSet(params);
  RCLCPP_INFO(rclcpp::get_logger("rfans_driver"), "%s device stop", m_input_para.device_ip.c_str());
}


void Rfans_Driver::sdKobjInit()
{
  m_sdk_obj._io_ptr = getLidarIO(ISF_DEC);
  m_sdk_obj._cal_ptr= getCalcPtr();
  m_sdk_obj._dec_ptr= createDecObj(ISF_DEC);
  m_sdk_obj._cfg_ptr= creatLidarCfg(ISF_DEC);

}

int Rfans_Driver::setSdkPara(bool is_real_time)
{

  m_sdk_para.dec_opt.angelSel=true;
  if(m_input_para.device_name=="CK-128")
   m_sdk_para.dec_para.isf_ver=1;
  m_sdk_para.dec_para.angel_seg=0.0;
  m_sdk_para.io_para.data_type=FAST_DATA;
  m_sdk_para.cfg_opt.cfg_Itg  =No_Sel;    	//  系统集成参数
  /*配置文件的buff*/
  header_buff = new  char[ISF_HEADER];//配置文件revise.ini的buff
  memset(header_buff, 0, ISF_HEADER);

  int blockPtN=0;
  if(!is_real_time)//回放模式
  {
   /*ISF文件流的buff*/
    blockPtN=BLOCK_SIZE / PACKET0_SIZE * PACKET0_POINT_NUM*DEC_BLOCK_COFF;  
    m_sdk_buff.rawStream._ptr = new char[BLOCK_SIZE];//文件ISF流的buff
    m_sdk_buff.rawStream.buff_capacity = UDP_MAX_SIZE;
  }else
  {
    blockPtN = DEC_POINT_SIZE;
    /* 实时流的原始缓冲必须能容下【多个】UDP 包：
     * decFastRun() 是流式解析，返回值是本次消耗掉的字节数，调用方要滑动窗口
     * （见 playBackMode 的用法）。原来只分配 UDP_MAX_SIZE(1500)，而一个雷达包
     * 就是 PACKET0_SIZE(1412) 字节，连两个包都放不下，更不可能跨包拼出完整扫描帧。 */
    const int RT_STREAM_SIZE = UDP_MAX_SIZE * 64;   // 96K
    m_sdk_buff.rawStream._ptr  = new char[RT_STREAM_SIZE]();
    m_sdk_buff.rawStream.buff_capacity = RT_STREAM_SIZE;
    m_sdk_buff.rawStream.cur_size = 0;   // 从"空"开始：原来是满 buff，等于告诉解码器有 1500 字节垃圾数据
    m_sdk_buff.frame_stream._lpoints=(LaserPoint_S*) new char[250000 * sizeof(LaserPoint_S)];
  }

  m_sdk_buff.decStream._lpoints = (LaserPoint_S*) new char[blockPtN * sizeof(LaserPoint_S)];
  m_sdk_buff.decStream.buff_boundary = blockPtN * sizeof(LaserPoint_S);
  m_sdk_buff.decStream.las_num = 0;
  memset(m_sdk_buff.decStream._lpoints, 0, m_sdk_buff.decStream.buff_boundary);
  m_sdk_buff.decStream.error_log._ptr = new char[ERORRLOG_BUFFSIZE];
  m_sdk_buff.decStream.error_log.buff_capacity = ERORRLOG_BUFFSIZE;
  m_sdk_buff.decStream.error_log.cur_size = 0;
  /*解算输出buff的创建*/
  m_sdk_buff.calcStream._calcPt = (CalcLaserPt_S*)new char[blockPtN * sizeof(CalcLaserPt_S)];
  m_sdk_buff.calcStream.buff_size = blockPtN * sizeof(CalcLaserPt_S);
  m_sdk_buff.calcStream.error_log._ptr = new char[ERORRLOG_BUFFSIZE];
  memset(m_sdk_buff.calcStream.error_log._ptr, 0, ERORRLOG_BUFFSIZE);
  m_sdk_buff.calcStream.error_log.buff_capacity = ERORRLOG_BUFFSIZE;
  m_sdk_buff.calcStream.error_log.cur_size = 0;
  memset(&m_sdk_para.cal_iniPara,0,sizeof(CalcInitPara_S));
  if(m_input_para.alg_flag)//解算是否选择默认
  {
    m_sdk_para.cal_algMode=Nav_DevCalc_ALG;
  }else {                 //手动配置算法
    m_sdk_para.cal_algMode=Config_ALG;
    assignAlg(ISF_DEC, m_sdk_para.cal_algOpt);
  }

  return 0;
}


Rfans_Driver::Rfans_Driver(const rclcpp::NodeOptions & opt)
: rclcpp::Node("rfans_driver", opt), m_is_isf(0), m_isf_count(0)
{
  setupNodeParams();//从launch文件中获取参数
  /* 2. ROS2 通信对象 */
  pub_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "rfans_points", 10);
  subCommond = this->create_subscription<rfans_driver::msg::Command>(
                "contrlComand", 10,
                [&](const rfans_driver::msg::Command::SharedPtr msg){
                  CommandHandle(msg);
                });

  InitPointcloud2(original_ros_cloud);
  InitPointcloud2(restruct_ros_cloud);
  m_filter_xyz_vec=getFilterXYZ(m_input_para.filter_path);
  if(m_filter_xyz_vec.size()>0)
    is_filter_flag=true;
  else
    is_filter_flag=false;
  //构建sdk对象
  sdKobjInit();
  /*判断是回放模式还是实时模式*/
  if (m_input_para.simu_filepath != "")
  {
    //如果多台雷达启动的时候，从multi_lidar.launch获取该参数
    //如果启动单雷达设备的launch文件，默认设置为ture
    bool device_start;
    this->declare_parameter<bool>("Is_Start", true);
    this->get_parameter("Is_Start", device_start);
    if(device_start)//normal start
    {
      //设置isf文件列表
      m_sdk_obj._io_ptr->multFileFrPath(m_input_para.simu_filepath,m_file_list);
      //设置配置文件参数
      m_sdk_para.cfg_para.cfg_path=m_input_para.cfg_path;
      //从文件名获取温度
      float temperature=getTemperFrStr(m_input_para.simu_filepath);
      m_is_real_time=0;
    }
  }
  else {//实时解算
    m_is_real_time=1;
    // 设备通迅的初始化要等对象被 shared_ptr 接管之后再做，
    // 见下方 initDevice()，这里只决定模式。
  }
  m_sdk_para.cfg_para.cfg_path=m_input_para.cfg_path;
  //配置参数及buff的创建
  setSdkPara(m_is_real_time);
  //是否保存解算坐标值
  if(m_input_para.save_xyz)
  {
    save_xyz_file.open(m_input_para.export_path);
    if(!save_xyz_file.is_open())
    {
      m_input_para.save_xyz=false;
      RCLCPP_WARN(this->get_logger(), "save_xyz is failure");
    }
    else
      /* 文本模式才用 m_frame_buf：每点最多 48 字节（%.6g + 逗号 + 换行）。
       * 二进制模式整块写 buff_original_cloud，根本不经过这个缓冲。 */
      m_frame_buf.reserve(45056 * 48);
  }
  /* 二进制落盘：每点 6 个 float（x,y,z,hangle,range,intent）。
   * 这里不做任何格式化，只 memcpy，所以能跑满速。 */
  if(m_save_bin)
  {
    save_bin_file.open(m_bin_path, std::ios::out | std::ios::binary | std::ios::trunc);
    if(!save_bin_file.is_open())
    {
      m_save_bin=false;
      RCLCPP_WARN(this->get_logger(), "save_bin open failure: %s", m_bin_path.c_str());
    }
    else
      RCLCPP_INFO(this->get_logger(), "save_bin -> %s (每点 %zu 字节)",
                  m_bin_path.c_str(), sizeof(TransClound_S));
  }
  /* UDP 转发：建 socket 失败不致命，只是转发这条路不通，落盘和发布照常。 */
  if(m_net_forward && netInit() < 0)
    m_net_forward = false;
  if(m_input_para.save_isf)
  {
    if(creatIsf(m_input_para.isf_path)<0)
      RCLCPP_INFO(this->get_logger(), "ISF open failure");
    else
      m_is_isf=true;
  }
  m_cur_count=0;

}


int Rfans_Driver::creatIsf(string path)
{
  char file_Name[512] = { 0 };//生成文件的名称
  time_t nowtime;
  time(&nowtime);
  localtime(&nowtime);
  sprintf(file_Name, "%s/data-%04d%02d%02d-%02d%02d%02d.isf", path.c_str(), \
          localtime(&nowtime)->tm_year + 1900, \
          localtime(&nowtime)->tm_mon + 1, \
          localtime(&nowtime)->tm_mday, \
          localtime(&nowtime)->tm_hour, \
          localtime(&nowtime)->tm_min, \
          localtime(&nowtime)->tm_sec);

  m_isf_fp.open(file_Name,ios::binary);
  if(m_isf_fp.is_open())
    return 0;
  else
    return -1;
}

void Rfans_Driver::getTemperFrHeart()
{
  float temper = 0.0;
  while (rclcpp::ok())
  {
    if(m_heart_socket->read((unsigned char*)&m_heart, sizeof(m_heart)) == 256)
    {
      temper = m_heart.temperature / 100.0;
    }
    // 使用RCLCPP日志宏并传入logger
    RCLCPP_WARN(get_logger(), "temperature=%f", temper);
    RCLCPP_INFO(get_logger(), "DEVICE_id=%d", m_heart.device_id);
    // 确保chrono使用标准命名空间
    std::this_thread::sleep_for(std::chrono::seconds(10));
  }
}


void Rfans_Driver::initDevice()
{
  if (!m_is_real_time)
    return;//回放模式不需要和设备通迅

  this->socketInit();//初始化socket

  bool mutli_Start;
  if (this->get_parameter("/mult_lidar", mutli_Start)) // 启动多台雷达
  {
    // 如果多台雷达启动的时候，multi_lidar.launch 含有这个参数，默认值设置为 false，由配置参数中获取真值
    rclcpp::Parameter device_start_param;
    bool device_start = false;
    if (this->get_parameter("Is_Start", device_start_param)) {
      device_start = device_start_param.as_bool();
    }

    if (device_start) // normal start
    {
      configDeviceParams();
      RCLCPP_INFO(this->get_logger(), "%s normal start", m_input_para.device_ip.c_str());
    }
    else {
      stopDevice(); // stop device
    }
  }
  else // 启动单台雷达
  {
    configDeviceParams(); // speed control command writing
  }

  std::thread temper_thd=thread(&Rfans_Driver::getTemperFrHeart,this);
  temper_thd.detach();
  s_this = this ;//实时解算才会有设备通讯
}

void Rfans_Driver::socketInit()
{
  m_ctl_socket   =new rfans_driver::IOSocketAPI(this->shared_from_this(), m_input_para.device_ip, m_input_para.msgport, m_input_para.msgport);
  m_data_socket  =new rfans_driver::IOSocketAPI(this->shared_from_this(), m_input_para.device_ip, m_input_para.dataport, m_input_para.dataport);
  m_heart_socket =new rfans_driver::IOSocketAPI(this->shared_from_this(), m_input_para.device_ip, m_input_para.heart_port, m_input_para.heart_port);
}
Rfans_Driver::~Rfans_Driver()
{
  /* 落盘改成每帧一次 write()，已经没有跨帧残留的缓冲了，这里只负责关文件和 socket。
   * 注意：被 SIGTERM 杀掉时析构不会执行（qemu 下会直接 abort 并 core dump），
   * 所以最后一帧可能没写完，分析前按 sizeof(TransClound_S) 即 40 字节对齐截断即可。 */
  if(save_xyz_file.is_open())
    save_xyz_file.close();
  if(save_bin_file.is_open())
    save_bin_file.close();
  if(m_net_fd >= 0)
  {
    ::close(m_net_fd);
    m_net_fd = -1;
  }
}
int Rfans_Driver::cloundTransForm(TransClound_S& out_ros,LaserPoint_S * dec_input,CalcLaserPt_S * cal_input ,int idx)
{

  if(transflag)
  {
    out_ros.x       =cal_input->pulse[idx].x*RTX[0]+cal_input->pulse[idx].y*RTX[1]+cal_input->pulse[idx].Z*RTX[2]+RTX[3];
    out_ros.y       =cal_input->pulse[idx].x*RTY[0]+cal_input->pulse[idx].y*RTY[1]+cal_input->pulse[idx].Z*RTY[2]+RTY[3];
    out_ros.z       =cal_input->pulse[idx].x*RTZ[0]+cal_input->pulse[idx].y*RTZ[1]+cal_input->pulse[idx].Z*RTZ[2]+RTZ[3];

  }

  else
  {
    out_ros.x       =cal_input->pulse[idx].x;
    out_ros.y       =cal_input->pulse[idx].y;
    out_ros.z       =cal_input->pulse[idx].Z;
  }

  out_ros.range   =cal_input->pulse[idx].range;
  out_ros.intent  =cal_input->pulse[idx].intensity;
  out_ros.laserid =cal_input->lmac[0];
  out_ros.hangle  =cal_input->scanAngle;
  out_ros.timeflag=cal_input->stamp_time;
  out_ros.vangle  =dec_input->turn_angle;//????? isf是否需要包含转台角度

  if(m_filter.frameFlag && is_filter_flag)
  {
    for(int ptNum=0;ptNum<m_filter.ptNum;ptNum++)
    {
      bool flagX=out_ros.x >m_filter.ptMin[ptNum].x && out_ros.x  <m_filter.ptMax[ptNum].x ;
      bool flagY=out_ros.y >m_filter.ptMin[ptNum].y && out_ros.y  <m_filter.ptMax[ptNum].y ;
      bool flagZ=out_ros.z >m_filter.ptMin[ptNum].z && out_ros.z  <m_filter.ptMax[ptNum].z ;
      if(flagX &&flagY&&flagZ)
      {
        out_ros.x=0;
        out_ros.y=0;
        out_ros.z=0;
      }
    }
  }
  /* 这里不再写盘。本函数是在 queue_mutex 保护区内逐点调用的，
   * 在这里做任何 I/O 都会拖长临界区、挡住解码线程排空 socket。
   * 落盘统一挪到帧末、锁外的 dumpFrame()。 */
  return 1;
}

/* 整帧落盘。**必须在 queue_mutex 之外调用** —— 这正是它存在的理由。
 *
 * 原先写盘写在 cloundTransForm() 里，而那个函数是在 calculation() 持锁的
 * 逐点循环中调的。消费线程每帧要持锁做完 4.5 万点的转换 + 写盘，解码线程
 * 就只能干等，socket 缓冲在此期间不再被排空，内核按到达顺序丢包。
 * 实测后果：收包速率看着是满的（2381 包/秒，缓冲余量全程 0），但每一圈
 * 只剩 75.6% 的点 —— 丢在 socket 上，不在解码器里，所以从驱动内部看不出来。
 *
 * 现在逐点循环里只做纯计算，帧末解锁后再整帧一次性写出，临界区回到原来的
 * 长度，解码线程不再被挡。代价是极端情况下消费跟不上会**整帧被覆盖丢弃**
 * （frame_stream 是单槽），但那总好过每帧都残缺。 */
void Rfans_Driver::dumpFrame()
{
  const size_t n = m_cur_count;
  if(n == 0 || m_input_para.display_mode != "overlay")
    return;

  if(m_save_bin)
  {
    /* 把 TransClound_S 数组原样整块写出：一次 write()，逐点循环彻底省掉。
     * buff_original_cloud 本来就是本帧点的连续数组，没有任何重排的需要 ——
     * 之前逐点拼 6 个 float 是白花 45k 次循环，qemu 下就是 ~30 毫秒/帧。
     *
     * 结构体是 #pragma pack(1) 的 40 字节：
     *   float x,y,z,intent,vangle,hangle,range (7×4=28)
     *   double timeflag (8)  +  int laserid (4)          = 40
     * 宿主机按同样布局用 numpy 解析即可。比只存 6 个 float 多带
     * vangle/timeflag/laserid —— laserid 就是激光环号，做聚类时用得上。 */
    save_bin_file.write(reinterpret_cast<const char*>(buff_original_cloud.data()),
                        (std::streamsize)(n * sizeof(TransClound_S)));
  }
  else if(m_input_para.save_xyz)
  {
    /* 文本模式：%.6g 与原来 iostream 的 6 位有效数字输出一致，格式不变。
     * 每点一次 snprintf，仍然很贵，但已经挪出锁外，不会再造成残缺帧，
     * 只是整体帧率会掉。想要满速请用 save_bin。 */
    if(m_frame_buf.size() < n * 48)
      m_frame_buf.resize(n * 48);
    char *d = m_frame_buf.data();
    const TransClound_S *p = buff_original_cloud.data();
    for(size_t i = 0; i < n; ++i)
    {
      int k = snprintf(d, 48, "%.6g,%.6g,%.6g,%.6g,%.6g,%.6g\n",
                       p[i].x, p[i].y, p[i].z,
                       p[i].hangle, p[i].range, p[i].intent);
      if(k > 0)
        d += (k < 48) ? k : 47;
    }
    save_xyz_file.write(m_frame_buf.data(), (size_t)(d - m_frame_buf.data()));
  }
}

/* 建立 UDP 转发 socket。
 *
 * connect() 之后就能直接 send()，不必每次填 sockaddr；副作用是内核会把这个
 * socket 绑定到对端，若宿主 relay 没起来会收到 ICMP 端口不可达，send 返回
 * ECONNREFUSED —— 正好是个现成的"对面没在听"信号，日志里报一次就够。 */
int Rfans_Driver::netInit()
{
  m_net_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if(m_net_fd < 0)
  {
    RCLCPP_WARN(this->get_logger(), "net_forward: socket() 失败: %s", strerror(errno));
    return -1;
  }

  struct sockaddr_in dst;
  memset(&dst, 0, sizeof(dst));
  dst.sin_family = AF_INET;
  dst.sin_port   = htons((uint16_t)m_net_port);
  if(::inet_pton(AF_INET, m_net_host.c_str(), &dst.sin_addr) != 1)
  {
    RCLCPP_WARN(this->get_logger(), "net_forward: 目标地址不合法 '%s'", m_net_host.c_str());
    ::close(m_net_fd); m_net_fd = -1;
    return -1;
  }
  if(::connect(m_net_fd, (struct sockaddr *)&dst, sizeof(dst)) < 0)
  {
    RCLCPP_WARN(this->get_logger(), "net_forward: connect %s:%d 失败: %s",
                m_net_host.c_str(), m_net_port, strerror(errno));
    ::close(m_net_fd); m_net_fd = -1;
    return -1;
  }

  /* 发送缓冲给大一点，单帧 1.36 MB、7.5 帧/秒，默认 208 KB 会频繁 send 失败。
   * 注意内核同样会把 SO_SNDBUF 折半记账，给 4 MB 实际约 2 MB。 */
  int sndbuf = 4 * 1024 * 1024;
  ::setsockopt(m_net_fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

  RCLCPP_INFO(this->get_logger(), "net_forward -> udp://%s:%d", m_net_host.c_str(), m_net_port);
  return 0;
}

/* 整帧 UDP 转发。和 dumpFrame() 一样 **必须在 queue_mutex 之外调用** ——
 * 帧末锁外发，消费线程不因为网络阻塞多持一次锁，解码线程照常排空 socket。
 *
 * 报文格式（全部小端）：
 *   magic(4) frame_id(4) pkt_idx(2) pkt_total(2) n_points(4) reserved(4) = 20 字节
 *   后面跟本片数据，最多 PAYLOAD 字节
 * 整帧 34000 点 × 40 字节 = 1.36 MB，切成 937 片；7.5 帧/秒下约 7000 包/秒。
 * 实测（容器内 qemu 下）这套发送循环能跑 182 帧/秒，余量二十四倍，够用。
 *
 * 宿主 relay 按 frame_id 收齐所有片后，把整块拼回来直接 memcpy 进
 * PointCloud2.data —— TransClound_S 是 40 字节的紧凑结构，和 PointCloud2 的
 * point_step 一一对应，所以一个点都不用转换。字段顺序见 point_types.h。
 *
 * 发送用 MSG_DONTWAIT：宁可丢这一片，也不能让 UDP 把消费线程堵住。 */
void Rfans_Driver::netFrame()
{
  const size_t n = m_cur_count;
  if(n == 0 || m_net_fd < 0 || m_input_para.display_mode != "overlay")
    return;

  /* 每包净荷 —— 这个 1452 是有来历的，别想当然往大改。
   *
   * 20 字节头 + 1452 = 1472 = 1500(MTU) − 20(IP 头) − 8(UDP 头)。
   *
   * 在 WSL2 镜像网络模式下，发往 127.0.0.1 的包并不走真正的 lo：
   *   $ ip route get 127.0.0.1
   *   127.0.0.1 via 169.254.73.152 dev loopback0 ...
   * 走的是一块按 1500 处理的 loopback0，而且**超过 MTU 的 IP 分片会被丢掉**。
   * 实测边界正好卡在 1472：1472 通，1473 就没了。
   *
   * 最坑的是它**静默**：sendmsg 返回成功、不报错、errno 也是 0，包就是不
   * 到。所以如果你把这里改大，表现是"驱动一切正常、relay 一个包都收不到"。
   *
   * 发往宿主真实 IP（192.168.x.x 等）没有这个限制，60000 都能过；但这个
   * 默认值不依赖宿主 IP 会变，更稳。 */
  constexpr size_t PAYLOAD = 1452;
  /* pkt_total 是 uint16，真要把 PAYLOAD 改小到包数超过 65535，这里直接编译不过 */
  static_assert((45056 * sizeof(TransClound_S) + PAYLOAD - 1) / PAYLOAD < 65536,
                "PAYLOAD 太小，每帧包数会撑爆 uint16 的 pkt_total");
  const char *base = reinterpret_cast<const char *>(buff_original_cloud.data());
  const size_t total = n * sizeof(TransClound_S);
  const uint16_t pkt_total = (uint16_t)((total + PAYLOAD - 1) / PAYLOAD);

  /* 先把所有片塞进内核发送队列，再回来查一次错误。
   * 逐片检查会多 20 多次系统调用，这里图省事，代价是错误定位粗一点。 */
  for(uint16_t i = 0; i < pkt_total; ++i)
  {
    const size_t off = (size_t)i * PAYLOAD;
    const uint32_t len = (uint32_t)((total - off < PAYLOAD) ? (total - off) : PAYLOAD);

    char hdr[20];
    uint32_t magic = 0x314E4652u;        // 'RFN1'
    uint32_t fid   = m_net_frame_id;
    uint32_t npts  = (uint32_t)n;
    uint32_t rsv   = 0;
    memcpy(hdr +  0, &magic, 4);
    memcpy(hdr +  4, &fid,   4);
    memcpy(hdr +  8, &i,     2);
    memcpy(hdr + 10, &pkt_total, 2);
    memcpy(hdr + 12, &npts,  4);
    memcpy(hdr + 16, &rsv,   4);

    struct iovec iov[2];
    iov[0].iov_base = hdr;
    iov[0].iov_len  = sizeof(hdr);
    iov[1].iov_base = const_cast<char *>(base + off);
    iov[1].iov_len  = len;

    struct msghdr msg;
    memset(&msg, 0, sizeof(msg));
    msg.msg_iov    = iov;
    msg.msg_iovlen = 2;

    if(::sendmsg(m_net_fd, &msg, MSG_DONTWAIT) < 0)
      ++m_net_dropped;
  }

  ++m_net_frame_id;

  if(m_net_dropped)
  {
    /* 每 100 帧报一次，别刷屏。持续增长说明宿主 relay 没跟上。 */
    static int cnt = 0;
    if(++cnt % 100 == 0)
      RCLCPP_WARN(this->get_logger(), "net_forward: 累计丢弃 %lu 个包（宿主 relay 是否在跑？）",
                  (unsigned long)m_net_dropped);
  }
}

void Rfans_Driver::cal2RosClound(LaserPoint_S * dec_input,CalcLaserPt_S * cal_input)
{
  if(m_input_para.display_mode=="overlay")
  {
    for (std::size_t idx = 0; idx < 2; ++idx) {
      if (cal_input->pulse[idx].flag)//判断当前点是否有效值
      {
        //判断当前点的角度和距离是否在区域范围内
        if(cal_input->pulse[idx].range > max_range || cal_input->pulse[idx].range < min_range|| cal_input->scanAngle>max_angle || cal_input->scanAngle<min_angle)
          continue;
        if(buff_original_cloud.size()<=m_cur_count)
          buff_original_cloud.resize(m_cur_count+10000);
        if(is_lidarId_slc)//是否选择激光通道过滤
        {
          if(cal_input->lmac[0]==ringID)
          {
            cloundTransForm(buff_original_cloud[m_cur_count],dec_input,cal_input,idx);
            ++m_cur_count;
          }
          continue;
        }else {
          cloundTransForm(buff_original_cloud[m_cur_count],dec_input,cal_input,idx);
          ++m_cur_count;
        }
      }
    }
  }
  else if (m_input_para.display_mode=="pipeline" || m_input_para.display_mode=="snapshot")
  {
    for (std::size_t idx = 0; idx < 2; ++idx)
    {
      if (cal_input->pulse[idx].flag)
      {
        auto mirrorid_=cal_input->lmac[1];
        if(mirrorid_>=4 || mirrorid_<0) return;//判断激光器通道号有没有越界
        if(buff_mirror_cloud[mirrorid_].size()<=numMirror[mirrorid_])
        {
          buff_mirror_cloud[mirrorid_].resize(buff_mirror_cloud[mirrorid_].size()+10000);
        }
        if(cloundTransForm(buff_mirror_cloud[mirrorid_][numMirror[mirrorid_]],dec_input,cal_input,idx)<0)
          return ;
        numMirror[mirrorid_]++;
      }
    }
  }
}

void Rfans_Driver::rosCloundPulish()
{
  if(m_input_para.display_mode=="overlay")
  {
    original_ros_cloud.header.stamp = this->now();                 // ①
    original_ros_cloud.width = m_cur_count;
    int data_size=original_ros_cloud.point_step*original_ros_cloud.width;
    original_ros_cloud.data.resize(data_size);
    original_ros_cloud.row_step = original_ros_cloud.data.size();
    memcpy(&original_ros_cloud.data[0] , &buff_original_cloud[0], data_size );
    pub_cloud->publish(original_ros_cloud);
    rclcpp::spin_some(this->get_node_base_interface());
  }else if (m_input_para.display_mode=="pipeline")
  {
    static bool firstFlag=true;
    if(firstFlag)//第一次输出的四个镜面不参与发布
    {
      for (int i=0;i<4;i++)
      {
        buff_restruct_cloud[i].resize(numMirror[i]);
        memcpy(buff_restruct_cloud[i].data() , buff_mirror_cloud[i].data(), numMirror[i]*sizeof (TransClound_S));
      }
      firstFlag=false;
    }
    else
    {
      for (int i=0;i<4;i++)
      {
        buff_restruct_cloud[i].resize(numMirror[i]);//replace a mirror
        memcpy(buff_restruct_cloud[i].data() , buff_mirror_cloud[i].data(), numMirror[i]*sizeof (TransClound_S));
        auto length1=buff_restruct_cloud[0].size();
        auto length2=buff_restruct_cloud[1].size();
        auto length3=buff_restruct_cloud[2].size();
        auto length4=buff_restruct_cloud[3].size();
        restruct_ros_cloud.header.stamp = this->now();    
        restruct_ros_cloud.width = length1+length2+length3+length4;
        restruct_ros_cloud.data.resize( restruct_ros_cloud.point_step*restruct_ros_cloud.width);
        restruct_ros_cloud.row_step = restruct_ros_cloud.data.size();
        memcpy(restruct_ros_cloud.data.data() , buff_restruct_cloud[0].data(), length1*sizeof (TransClound_S));
        memcpy(restruct_ros_cloud.data.data()+length1*sizeof (TransClound_S) , buff_restruct_cloud[1].data(), length2*sizeof (TransClound_S));
        memcpy(restruct_ros_cloud.data.data()+length1*sizeof (TransClound_S)+length2*sizeof (TransClound_S), buff_restruct_cloud[2].data(), length3*sizeof (TransClound_S));
        memcpy(restruct_ros_cloud.data.data()+length1*sizeof (TransClound_S)+length2*sizeof (TransClound_S)+length3*sizeof (TransClound_S) , buff_restruct_cloud[3].data(), length4*sizeof (TransClound_S));
        pub_cloud->publish(restruct_ros_cloud);
        rclcpp::spin_some(this->get_node_base_interface());
      }
    }

  }
  else if (m_input_para.display_mode=="snapshot")
  {
    for (int i=0;i<4;i++)
    {
      if(numMirror[i]==0)
        continue;
        restruct_ros_cloud.header.stamp = this->now();       
      restruct_ros_cloud.width = numMirror[i];
      int data_size=restruct_ros_cloud.point_step*restruct_ros_cloud.width;
      restruct_ros_cloud.data.resize(data_size);
      restruct_ros_cloud.row_step = restruct_ros_cloud.data.size();
      memcpy(&restruct_ros_cloud.data[0] , buff_mirror_cloud[i].data(), data_size);
      pub_cloud->publish(restruct_ros_cloud);
      rclcpp::spin_some(this->get_node_base_interface());
      std::this_thread::sleep_for(chrono::milliseconds(1));
    }
  }

}

int Rfans_Driver::realTimeMode()
{
  while (!(int)m_heart.device_id)
  {
    usleep(1);
  }

  m_sdk_para.cfg_opt.cfg_Itn  =CFG_FILE;      //  文件头或配置文件
  //多文件循环解算
  int cfg_ret = m_sdk_obj._cfg_ptr->getLidarCfg(header_buff, ISF_HEADER, m_sdk_para.cfg_para, m_sdk_para.cfg_opt);
  /* 标定文件对不上时这里会失败，而失败的后果是点云角度整体偏 —— 不会崩，
   * 很难从数据上看出是标定问题。所以留一条明确的告警。 */
  if (cfg_ret < 0)
    RCLCPP_WARN(this->get_logger(), "getLidarCfg 失败(返回 %d)：标定文件 %s 与 %s 不匹配？",
                cfg_ret, m_sdk_para.cfg_para.cfg_path.c_str(), m_input_para.device_name.c_str());
  std::thread cal_thread=std::thread(&Rfans_Driver::calculation,this);
  cal_thread.detach();
  ((float*)header_buff)[ADR_D_DEVICE_ID]=m_heart.device_id;//从心跳包拿设备编号
  if(m_is_isf)//是否保存ISF文件
  {m_isf_fp.write(header_buff,ISF_HEADER);
    m_isf_count+=ISF_HEADER;}

  m_sdk_obj._dec_ptr->setMetaHeader(header_buff, ISF_HEADER); 	//解码初始化
  //（3）解算初始化
  memcpy(m_sdk_para.cal_iniPara.lidarPara, header_buff, ISF_HEADER);	//将参数传递给解算
  if (m_sdk_para.cal_algMode == Config_ALG)
    m_sdk_para.cal_iniPara.algOpt = m_sdk_para.cal_algOpt;//后期该参数m_sdk_para.cal_algOpt，可从界面更改
  //解算初始化
  m_sdk_obj._cal_ptr->init(m_sdk_buff.decStream, m_sdk_buff.calcStream,m_sdk_para.cal_iniPara,m_sdk_para.cal_algMode);

  //单文件循环解算
  int surpBufSize = 0;
  while (true)
  {
    /* 每次只收【一个完整】UDP 数据报：recvfrom 给的长度小于报文长度会把包截断，
     * 所以读取长度固定用 UDP_MAX_SIZE，但落点要接在已缓存数据的后面。
     * 缓冲空间不够一个整包时就先不读，等解码器消费出空间。 */
    int space = m_sdk_buff.rawStream.buff_capacity - m_sdk_buff.rawStream.cur_size;
    int rd_n = 0;
    if (space >= UDP_MAX_SIZE)
      rd_n = m_data_socket->read((unsigned char*)m_sdk_buff.rawStream._ptr
                                 + m_sdk_buff.rawStream.cur_size, UDP_MAX_SIZE);
    if (rd_n > 0)
      m_sdk_buff.rawStream.cur_size += rd_n;
    if((int)m_heart.device_id==0 || m_sdk_buff.rawStream.cur_size <= 0)
    {
     continue;
    }
    //ROS_INFO("Line503:decStream.tgidx_vec.size()=%d\n",m_sdk_buff.decStream.tgidx_vec.size());
    if(m_is_isf&&rd_n>0)//是否保存ISF文件
    {
      // 本包的起点不再是缓冲首地址，而是刚读进来的那一段
      char* pkt=m_sdk_buff.rawStream._ptr+m_sdk_buff.rawStream.cur_size-rd_n;
      m_isf_count+=rd_n;
      if(m_isf_count<MAX_ISF_SIZE)
        m_isf_fp.write(pkt,rd_n);
      else
      {
        int cnt=MAX_ISF_SIZE-(m_isf_count-rd_n);
        if(cnt>0)
        {
          temp_buf.resize(MAX_ISF_SIZE-(m_isf_count-UDP_MAX_SIZE),0);
          m_isf_fp.write(temp_buf.data(),temp_buf.size());
        }
        m_isf_fp.close();
        m_isf_count=0;
        creatIsf(m_input_para.isf_path);
        m_isf_fp.write(header_buff,ISF_HEADER);
        m_isf_count+=ISF_HEADER;
      }
    }
    m_sdk_buff.decStream.tgidx_vec.resize(0);
    int rtn = m_sdk_obj._dec_ptr->decFastRun(m_sdk_buff.rawStream, m_sdk_buff.decStream, m_sdk_para.dec_para, m_sdk_para.dec_opt);
    /* 把解码器已消耗掉的 rtn 字节移出缓冲、剩余数据前移 —— 和 playBackMode 的写法一致。
     * 少了这一步，cur_size 永远不变、每个包都从缓冲头部重新解一次，
     * 解码器永远看不到连续的码流，帧边界（tgidx）自然出不来。 */
    if (rtn > 0)
    {
      if (rtn >= m_sdk_buff.rawStream.cur_size)
        m_sdk_buff.rawStream.cur_size = 0;
      else
      {
        memmove(m_sdk_buff.rawStream._ptr,
                m_sdk_buff.rawStream._ptr + rtn,
                m_sdk_buff.rawStream.cur_size - rtn);
        m_sdk_buff.rawStream.cur_size -= rtn;
      }
    }
    //调试代码

    if (m_sdk_buff.decStream.tgidx_vec.size() <= 0)
      continue;
    //——————————————————————————————分帧处理————————————————————————————————
    int offset = 0;
    int preIndx = 0;
    for (int i = 0; i < m_sdk_buff.decStream.tgidx_vec.size(); i++)
    {
      DecStream_S frameStream;
      frameStream.deviceID = m_sdk_buff.decStream.deviceID;
      if (i > 0)    preIndx = m_sdk_buff.decStream.tgidx_vec[i - 1] + offset;
      frameStream._lpoints = &m_sdk_buff.decStream._lpoints[preIndx];
      frameStream.las_num = m_sdk_buff.decStream.tgidx_vec[i] - preIndx;
      offset = 1;
      if (frameStream.las_num <= 0)  continue;
      memset(&m_sdk_para.cal_updPara, 0, sizeof(CalcCfgPara_S));
      m_sdk_para.cal_updPara.temper=m_heart.temperature/100;      //更新温度
      //ROS_INFO("FRAM_LAM=%d",frameStream.las_num);
      if(frameStream.las_num>250000)
        continue;
      std::unique_lock<std::mutex> locker(queue_mutex);
      memcpy(m_sdk_buff.frame_stream._lpoints,frameStream._lpoints,sizeof (LaserPoint_S)*frameStream.las_num);
      m_sdk_buff.frame_stream.las_num=frameStream.las_num;
      m_sdk_buff.frame_stream.deviceID=frameStream.deviceID;
      locker.unlock();
      cond.notify_one();
    }
    //不满帧buff处理
    surpBufSize = (m_sdk_buff.decStream.las_num - m_sdk_buff.decStream.tgidx_vec.back()) * sizeof(LaserPoint_S);
    memmove(m_sdk_buff.decStream._lpoints, m_sdk_buff.decStream._lpoints + m_sdk_buff.decStream.tgidx_vec.back(), surpBufSize);
    m_sdk_buff.decStream.las_num = m_sdk_buff.decStream.las_num - m_sdk_buff.decStream.tgidx_vec.back();
    memset(m_sdk_buff.decStream._lpoints + m_sdk_buff.decStream.las_num, 0, m_sdk_buff.decStream.buff_boundary - m_sdk_buff.decStream.las_num * sizeof(LaserPoint_S));
  }
  return 0;
}


void Rfans_Driver::calculation()
{
  while(1)
  {
    std::unique_lock<std::mutex> locker(queue_mutex);
    cond.wait(locker);
    m_sdk_para.cal_opt.navConfig = false;
    m_sdk_obj._cal_ptr->pointRun(m_sdk_buff.frame_stream, m_sdk_buff.calcStream, m_sdk_para.cal_updPara, m_sdk_para.cal_opt);
    for (int i=0;i<m_sdk_buff.frame_stream.las_num;i++)
    {
      this->cal2RosClound(&m_sdk_buff.frame_stream._lpoints[i],&m_sdk_buff.calcStream._calcPt[i]);
    }
    locker.unlock();
    rosCloundPulish();
    dumpFrame();          // 锁外落盘：本帧点数就是 m_cur_count
    netFrame();           // 锁外 UDP 转发；只在 net_forward=true 时真正发包
    //ROS_INFO("pub_count=%d",m_cur_count);
    //帧数初始化
    if(m_input_para.display_mode=="overlay")
      m_cur_count=0;
    else if (m_input_para.display_mode=="snapshot" || m_input_para.display_mode=="pipeline")
      memset(numMirror,0,sizeof (int)*4);
  }
}

//获取
filterXYZ_S Rfans_Driver::getFilterPara(double utc_time,vector<crdFilterPara_S>& filter_para )
{

  filterXYZ_S filter;//当前帧的判断条件
  filter.frameFlag=false ;
  for(int i=0;i<filter_para.size();i++)
  {
    float filetime=filter_para.at(i).time;
    //判断当前帧的时间是否在过滤条件范围内
    if(utc_time >filetime -timeVal  && utc_time <filetime +timeVal )
    {
      for(int ptNum=0;ptNum<filter_para.at(i).num;ptNum++)
      { //输出点的过滤条件
        filter.ptMin[ptNum].x=filter_para.at(i).pt[ptNum].x- xyVal;
        filter.ptMax[ptNum].x=filter_para.at(i).pt[ptNum].x+xyVal;
        filter.ptMin[ptNum].y=filter_para.at(i).pt[ptNum].y- xyVal;
        filter.ptMax[ptNum].y=filter_para.at(i).pt[ptNum].y+ xyVal;
        filter.ptMin[ptNum].z=filter_para.at(i).pt[ptNum].z- zVal;
        filter.ptMax[ptNum].z=filter_para.at(i).pt[ptNum].z+ zVal;
      }
      filter.ptNum=filter_para.at(i).num;
      filter.frameFlag=true ;
      break;//只可能有一个判断条件，所以说遇到一个判定条件满足之后，就可以退出
    }
  }
  return filter;
}


int Rfans_Driver::playBackMode()
{
  int buff_surplus = 0;//统计多文件未消耗的数据量

  m_sdk_para.cfg_opt.cfg_Itn  =CFG_AND_RAW_FILE;      //  文件头或配置文件
  rclcpp::Rate rate_loop(static_cast<double>(m_input_para.scnSpeed));
  RCLCPP_INFO(this->get_logger(), "rate_loop=%d", m_input_para.scnSpeed);
  //多文件循环解算
  for (size_t fileNum = 0; fileNum < m_file_list.size(); fileNum++)
  {
    //m_sdk_para.io_para.file_path=m_file_list.at(fileNum);
    m_sdk_para.cfg_para.raw_path=m_file_list.at(fileNum);
    m_sdk_para.io_para.data_type=FAST_DATA;
    m_sdk_obj._cfg_ptr->getLidarCfg(header_buff, ISF_HEADER, m_sdk_para.cfg_para, m_sdk_para.cfg_opt);     //文件头或配置文件
    if(m_input_para.device_name=="CK-128")
      ((float*)header_buff)[ADR_D_DEVICE_ID]=0XA0;
    int deviceID = ((float*)header_buff)[ADR_D_DEVICE_ID];
    RCLCPP_INFO(this->get_logger(), "deviceID=%d", deviceID);

    m_sdk_obj._dec_ptr->setMetaHeader(header_buff, ISF_HEADER); 	//解码初始化
    //（2）读快数据
    int rtn =m_sdk_obj._io_ptr->openLidarFile((char*)m_file_list.at(fileNum).c_str(),m_sdk_para.io_para);
    if (rtn < 0)		 continue;
   // m_sdk_buff.dec_input.ptr = m_sdk_buff.rawStream._ptr;
    //（3）解算初始化
    memcpy(m_sdk_para.cal_iniPara.lidarPara, header_buff, ISF_HEADER);	//将参数传递给解算
    if (m_sdk_para.cal_algMode == Config_ALG)
      m_sdk_para.cal_iniPara.algOpt = m_sdk_para.cal_algOpt;//后期该参数m_sdk_para.cal_algOpt，可从界面更改
    //解算初始化
    m_sdk_obj._cal_ptr->init(m_sdk_buff.decStream, m_sdk_buff.calcStream,m_sdk_para.cal_iniPara,m_sdk_para.cal_algMode);


    //单文件循环解算
    int surpBufSize = 0;
    //while ((m_sdk_buff.dec_input.ptr + BLOCK_SIZE) < (m_sdk_buff.rawStream._ptr + m_sdk_buff.rawStream.cur_size))
    while(1)
    {
      int read_num = m_sdk_obj._io_ptr->getFastBuff(m_sdk_buff.rawStream);
        m_sdk_buff.rawStream.cur_size += read_num;
        if (read_num <= 0)
          break;
      //解码
      m_sdk_buff.decStream.tgidx_vec.resize(0);
      int rtn = m_sdk_obj._dec_ptr->decFastRun(m_sdk_buff.rawStream, m_sdk_buff.decStream, m_sdk_para.dec_para, m_sdk_para.dec_opt);
      memmove(m_sdk_buff.rawStream._ptr, m_sdk_buff.rawStream._ptr + rtn, m_sdk_buff.rawStream.cur_size - rtn);//将未解析完的数据拷贝到buff的前面
      m_sdk_buff.rawStream.cur_size = m_sdk_buff.rawStream.cur_size - rtn;                          //buff中有效数据的位置偏移量
      if (m_sdk_buff.decStream.tgidx_vec.size() == 0) continue;
//      std::cout << "progress[" << fileNum << "]=" << (m_sdk_buff.dec_input.ptr + rtn - m_sdk_buff.rawStream._ptr) * 1.0 / m_sdk_buff.rawStream.cur_size << std::endl;
//      std::cout << "m_sdk_buff.decStream.las_num=" << m_sdk_buff.decStream.las_num<< std::endl;
      //m_sdk_buff.dec_input.ptr += rtn;
      if (m_sdk_buff.decStream.tgidx_vec.size() <= 0)
        continue;
      //——————————————————————————————分帧处理————————————————————————————————
      int offset = 0;
      int preIndx = 0;
      for (int i = 0; i < m_sdk_buff.decStream.tgidx_vec.size(); i++)
      {
        DecStream_S frameStream;
        frameStream.deviceID = m_sdk_buff.decStream.deviceID;
        if (i > 0)    preIndx = m_sdk_buff.decStream.tgidx_vec[i - 1] + offset;
        frameStream._lpoints = &m_sdk_buff.decStream._lpoints[preIndx];
        frameStream.las_num = m_sdk_buff.decStream.tgidx_vec[i] - preIndx;
        //ROS_INFO("frameStream.las_num=%d",frameStream.las_num);
        offset = 1;
        if (frameStream.las_num <= 0)  continue;
        memset(&m_sdk_para.cal_updPara, 0, sizeof(CalcCfgPara_S));
        getTemper(m_sdk_para.cal_updPara.temper, m_file_list.at(fileNum));      //更新温度
        m_sdk_para.cal_opt.navConfig = false;
        //解算
        m_sdk_obj._cal_ptr->pointRun(frameStream, m_sdk_buff.calcStream, m_sdk_para.cal_updPara, m_sdk_para.cal_opt);

        if(is_filter_flag)
          m_filter=getFilterPara(frameStream._lpoints[0].utc_time,m_filter_xyz_vec);

        for (int i=0;i<frameStream.las_num;i++)
        {
          this->cal2RosClound(&frameStream._lpoints[i],&m_sdk_buff.calcStream._calcPt[i]);
        }
        rosCloundPulish();

        //帧数初始化
        if(m_input_para.display_mode=="overlay")
          m_cur_count=0;
        else if (m_input_para.display_mode=="snapshot" || m_input_para.display_mode=="pipeline")
          memset(numMirror,0,sizeof (int)*4);
      rate_loop.sleep();
      }


      //不满帧buff处理
      surpBufSize = (m_sdk_buff.decStream.las_num - m_sdk_buff.decStream.tgidx_vec.back()) * sizeof(LaserPoint_S);
      memmove(m_sdk_buff.decStream._lpoints, m_sdk_buff.decStream._lpoints + m_sdk_buff.decStream.tgidx_vec.back(), surpBufSize);
      m_sdk_buff.decStream.las_num = m_sdk_buff.decStream.las_num - m_sdk_buff.decStream.tgidx_vec.back();
      memset(m_sdk_buff.decStream._lpoints + m_sdk_buff.decStream.las_num, 0, m_sdk_buff.decStream.buff_boundary - m_sdk_buff.decStream.las_num * sizeof(LaserPoint_S));
      //buff_surplus = m_sdk_buff.rawStream.cur_size - (m_sdk_buff.dec_input.ptr - m_sdk_buff.rawStream._ptr);
    }

    memcpy(m_sdk_buff.rawStream._ptr, m_sdk_buff.rawStream._ptr + m_sdk_buff.rawStream.cur_size - buff_surplus, buff_surplus);
    m_sdk_buff.rawStream.cur_size = buff_surplus;
  }


  //循环播放
  if(!m_input_para.read_once)
  {
   m_sdk_buff.rawStream.cur_size=0;
   playBackMode();
  }


  return 0;
}
/** @brief Rfnas Driver Core */
int Rfans_Driver::spinOnce()
{
  static bool exe_once=true;
  if(exe_once)
  {
    exe_once=false;
    if(m_is_real_time)//synchronous;
    {
      return realTimeMode();
    }
    else {//Asynchronous;
      return   playBackMode();
    }
  }else
  {
    RCLCPP_INFO(this->get_logger(), "END OF SINGLE PLAY");

    usleep(1000);
    return 0;
  }

}

void Rfans_Driver::InitPointcloud2(sensor_msgs::msg::PointCloud2 &initCloud)
{

  static const size_t DataSize = 0;
  initCloud.data.clear();
  initCloud.data.resize( DataSize);
  initCloud.is_bigendian = false ;
  initCloud.fields.resize(9);
  initCloud.is_dense = false;

  int tmpOffset = 0 ;
  for(int i=0; i < initCloud.fields.size() ;i++) {
    switch(i) {
    case 0:
      initCloud.fields[i].name = "x" ;
      initCloud.fields[i].datatype = 7u;
      break;
    case 1:
      initCloud.fields[i].name = "y" ;
      initCloud.fields[i].datatype = 7u;
      tmpOffset += 4;
      break;
    case 2:
      initCloud.fields[i].name = "z" ;
      initCloud.fields[i].datatype = 7u;
      tmpOffset += 4;
      break;
    case 3:
      initCloud.fields[i].name = "intensity" ;
      initCloud.fields[i].datatype = 7u;
      tmpOffset += 4;
      break;
    case 4:
      initCloud.fields[i].name = "v_angle" ;
      initCloud.fields[i].datatype = 7u;
      tmpOffset += 4;
      break;
    case 5:
      initCloud.fields[i].name = "h_angle" ;
      initCloud.fields[i].datatype = 7u;
      tmpOffset += 4;
      break;
    case 6:
      initCloud.fields[i].name = "range" ;
      initCloud.fields[i].datatype = 7u;
      tmpOffset += 4;
      break;
    case 7:
      initCloud.fields[i].name = "timestamp";
      initCloud.fields[i].datatype = 8u;
      tmpOffset +=4;
      break;
    case 8:
      initCloud.fields[i].name = "laserid";
      initCloud.fields[i].datatype = 5u;
      tmpOffset +=8;
      break;
    }
    initCloud.fields[i].offset = tmpOffset ;
    initCloud.fields[i].count = 1 ;
  }
  initCloud.height = 1;
  initCloud.point_step = sizeof(TransClound_S);
  initCloud.row_step = DataSize ;
  initCloud.width = 0 ;
  std::string frame_id_str;
  bool frame_flag = this->get_parameter_or<std::string>("frame_id", frame_id_str, "world");
  if(frame_flag)
  {
    initCloud.header.frame_id = frame_id_str;
  }else
  {
    initCloud.header.frame_id = "world";
  }
}
void Rfans_Driver::setupNodeParams()
{
  m_input_para.device_name = this->declare_parameter<std::string>("model", "R-Fans-32");
  m_input_para.display_mode = this->declare_parameter<std::string>("display_mode", "overlay");

  if ((m_input_para.device_name == "R-Fans-32") || (m_input_para.device_name == "R-Fans-16"))
    is_Rfans = true;
  else
    is_Rfans = false;

  m_input_para.dataport      = this->declare_parameter<int>("device_port", 2014);
  m_input_para.heart_port    = this->declare_parameter<int>("heart_port", 2030);
  m_input_para.save_xyz      = this->declare_parameter<bool>("save_xyz", false);
  /* 二进制落盘：满速采集用这个，别用 save_xyz（文本模式只能跑 1239 包/秒） */
  m_save_bin                 = this->declare_parameter<bool>("save_bin", false);
  m_bin_path                 = this->declare_parameter<std::string>("OutBIN_path", "");
  /* UDP 转发：只在 qemu 模拟的容器里需要（DDS 组播发现用不了）。
   * 车机上保持 false —— 那里节点直接 publish 就能被发现。 */
  m_net_forward              = this->declare_parameter<bool>("net_forward", false);
  m_net_host                 = this->declare_parameter<std::string>("net_host", "127.0.0.1");
  m_net_port                 = this->declare_parameter<int>("net_port", 7500);
  m_input_para.use_gps       = this->declare_parameter<bool>("use_gps", true);
  m_input_para.device_ip     = this->declare_parameter<std::string>("device_ip", "192.168.0.3");
  m_input_para.scnSpeed      = this->declare_parameter<int>("rps", 20);
  m_input_para.simu_filepath = this->declare_parameter<std::string>(
                                  "readfile_path",
                                  "/home/bkth/cfans128/data-20210722-190504-71.000.isf");
  m_input_para.cfg_path      = this->declare_parameter<std::string>(
                                  "cfg_path",
                                  "/home/bkth/cfans128/revise.ini");
  m_input_para.dual_echo     = this->declare_parameter<bool>("use_double_echo", false);
  m_input_para.export_path   = this->declare_parameter<std::string>("OutXYZ_path", "");
  m_input_para.read_once     = this->declare_parameter<bool>("read_once", false);
  m_input_para.alg_flag      = this->declare_parameter<bool>("algorithm_default", false);
  m_input_para.filter_path   = this->declare_parameter<std::string>("filter_path", "");
  m_input_para.save_isf      = this->declare_parameter<bool>("save_isf", false);
  m_input_para.isf_path      = this->declare_parameter<std::string>("OutISF_path", ".");
  m_input_para.msgport       = this->declare_parameter<int>("msg_port", 2015);
}


rfans_driver::IOAPI* Rfans_Driver::getDevInstance(){
  return m_ctl_socket;
}
/** @brief control the device
     *  @param .parameters
     */
int Rfans_Driver::progSet(lidarAPi::DEB_PROGRM_S &program)
{
  unsigned int tmpData = 0;
  if((m_input_para.device_name=="R-Fans-32")||(m_input_para.device_name=="R-Fans-16"))
  {
    switch (program.scnSpeed) {
    case ANGLE_SPEED_10HZ:
      tmpData |= CMD_SCAN_ENABLE;
      tmpData |= CMD_SCAN_SPEED_10HZ;
      break;
    case ANGLE_SPEED_20HZ:
      tmpData |= CMD_SCAN_ENABLE;
      tmpData |= CMD_SCAN_SPEED_20HZ;
      break;
    case ANGLE_SPEED_5HZ:
      tmpData |= CMD_SCAN_ENABLE;
      tmpData |= CMD_SCAN_SPEED_5HZ;
      break;
    default:
      tmpData |= CMD_SCAN_ENABLE;
      tmpData |= CMD_SCAN_SPEED_5HZ;
      break;
    }
  }
  else if ((m_input_para.device_name=="C-Fans-128")||(m_input_para.device_name=="C-Fans-32")||(m_input_para.device_name=="C-Fans-256")||(m_input_para.device_name=="CK-128")||(m_input_para.device_name=="CK-128"))
  {

    switch (program.scnSpeed) {
    case ANGLE_SPEED_0HZ_cfan:
      tmpData |= 0x0;
      tmpData |= CMD_SCAN_SPEED_10HZ_C;
      break;
    case ANGLE_SPEED_10HZ_cfan:
      tmpData |= CMD_SCAN_ENABLE;
      tmpData |= CMD_SCAN_SPEED_10HZ_C;
      break;
    case ANGLE_SPEED_20HZ_cfans:
      tmpData |= CMD_SCAN_ENABLE;
      tmpData |= CMD_SCAN_SPEED_20HZ_C;
      break;
    case ANGLE_SPEED_40HZ_cfans:
      tmpData |= CMD_SCAN_ENABLE;
      tmpData |= CMD_SCAN_SPEED_40HZ_C;
      break;
    case ANGLE_SPEED_60HZ_cfans:
      tmpData |= CMD_SCAN_ENABLE;
      tmpData |= CMD_SCAN_SPEED_60HZ_C;
      break;
    case ANGLE_SPEED_80HZ_cfans:
      tmpData |= CMD_SCAN_ENABLE;
      tmpData |= CMD_SCAN_SPEED_80HZ_C;
      break;
    default:
      tmpData |= CMD_SCAN_ENABLE;
      tmpData |= CMD_SCAN_SPEED_10HZ_C;
      break;
    }
  }

  switch (program.cmdstat) {
  case lidarAPi::eDevCmdWork://默认为工作状态，此值在QT中设为常量
    m_ctl_socket->HW_WRREG(0, REG_DEVICE_CTRL_OLD, tmpData);
    m_ctl_socket->HW_WRREG(0, REG_DEVICE_CTRL, tmpData);
    break;
  case lidarAPi::eDevCmdIdle://待机状态
    tmpData = CMD_RCV_CLOSE;
    m_ctl_socket->HW_WRREG(0, REG_DEVICE_CTRL_OLD, tmpData);
    m_ctl_socket->HW_WRREG(0, REG_DEVICE_CTRL, tmpData);
    break;
  case lidarAPi::eDevCmdAsk:
    break;
  default:
    break;
  }

  return 0;

}

int Rfans_Driver::dataLevelSet(lidarAPi::DEB_PROGRM_S &program)
{
  unsigned int regData =0;
  switch (program.dataLevel) {
  case lidarAPi::LEVEL0_ECHO:
    regData = CMD_LEVEL0_ECHO;
    break;
  case lidarAPi::LEVEL0_DUAL_ECHO:
    regData= CMD_LEVLE0_DUAL_ECHO;
    break;
  case lidarAPi::LEVEL1_ECHO:
    regData = CMD_LEVEL1_ECHO;
    break;
  case lidarAPi::LEVEL1_DUAL_ECHO:
    regData = CMD_LEVEL1_DUAL_ECHO;
    break;
  case lidarAPi::LEVEL2_ECHO:
    regData = CMD_LEVEL2_ECHO;
    break;
  case lidarAPi::LEVEL2_DUAL_ECHO:
    regData = CMD_LEVEL2_DUAL_ECHO;
    break;
  case lidarAPi::LEVEL3_ECHO:
    regData = CMD_LEVEL3_ECHO;
    break;
  case lidarAPi::LEVEL3_DUAL_ECHO:
    regData = CMD_LEVEL3_DUAL_ECHO;
    break;
  default:
    break;
  }

  switch (program.cmdstat) {//界面默认为1，参数属于const
  case lidarAPi::eDevCmdWork://选择为1的时候,
    m_ctl_socket->HW_WRREG(0, REG_DATA_LEVEL, regData);
    m_ctl_socket->HW_WRREG(0, REG_DATA_LEVEL_OLD, regData);
    break;
  case lidarAPi::eDevCmdAsk:
    break;
  default:
    break;
  }
  return 0;

}

void Rfans_Driver::configDeviceParams()
{
  bool dual_echo = m_input_para.dual_echo;

  lidarAPi::DEB_PROGRM_S params;
  params.cmdstat = lidarAPi::eDevCmdWork;
  params.dataFormat = lidarAPi::eFormatCalcData;

  // set start rps
  params.scnSpeed =  m_input_para.scnSpeed;
  progSet(params);

  unsigned int regData =0;
  regData =0;
  if(dual_echo)
  {
    regData = CFANS_DUAL;
  }
  else {
    regData = CFANS_ECHO;
  }

  m_ctl_socket->HW_WRREG(0, REG_DATA_LEVEL, regData);
  m_ctl_socket->HW_WRREG(0, REG_DATA_LEVEL_OLD, regData);
}




} //rfans_driver namespace
