/***************************************
 *          control_node.cpp
 *   ROS2 Humble 完整移植版本
 **************************************/
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <decision_planning_msgs/msg/decision_planning.hpp>
#include <can_control_msgs/msg/autocontrol.hpp>
#include <can_control_msgs/msg/vehicle_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "../include/can_com/maindata.h"
#include "../include/can_com/int.h"
#include "can_com/RoutePlanning.h"
#include "can_com/AutoVehicleControl.h"

#include <arpa/inet.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// ===================== 宏 & 常量（全部保留） =====================
#define BUFFER_SIZE 650
#define Remoter_IP  "192.168.5.137"
#define Remoter_PORT 9999
#define VCU_174h_ID 0x174
#define ACU_24Ah_ID 0x24A
#define EPS_112h_ID 0x112
#define EPS_114h_ID 0x114
#define WCBS_92h_ID 0x92
#define WCBS_93h_ID 0x93
#define WCBS_96h_ID 0x96
#define BCM_152h_ID 0x152
#define APA_170h_ID 0x170
#define APA_166h_ID 0x166
#define ADAS_160h_ID 0x160
#define ADAS_162h_ID 0x162
#define ADAS_163h_ID 0x163
#define CAN_SFF_MASK 0x000007FFU
#define CAN_EFF_MASK 0x1FFFFFFFU
#define CAN_ERR_MASK 0x1FFFFFFFU
#define CAN_EFF_FLAG 0x80000000U

using namespace std;
using rclcpp::Clock;
using rclcpp::Rate;
using rclcpp::Time;

// ===================== 全局变量（与原版保持一致） =====================
void MSG_Radar(void);
can_control_msgs::msg::Autocontrol msg_Lidar_self;
decision_planning_msgs::msg::DecisionPlanning msg_Plan;

int count = 0;
pthread_mutex_t count_mutex;
pthread_cond_t count_threshold_cv;
int sendbuf[BUFFER_SIZE];
uint8_t recvbuf[BUFFER_SIZE];
int i_count = 0;
uint8_t msg_count = 0;
float heading_delta_angle;
float send_time;
int detect_conut_garbge = 0;
bool detect_leaf = false;

// ===================== 底层 CAN 发送线程 =====================
void *auto_driver_send(void *arg)
{
    int can_skt = *(int *)arg;
    rclcpp::Rate loop_rate(50);
    uint8_t count_data = 0;
    ofstream ofs("send.txt", ios::out);

    while (rclcpp::ok())
    {
        auto start_ms = chrono::duration_cast<chrono::milliseconds>(
                            chrono::system_clock::now().time_since_epoch())
                            .count();

        RoutePlanning_step();       // Static RoutePlanning
        AutoVehicleControl_step();  // Decision & Control
        JMEV_Control_Output();      // Package Control CAN Message

        for (int i = 0; i < 5; ++i)
        {
            int bytes = write(can_skt, &JMEVmsg_ViewcarCtrl[i],
                              sizeof(JMEVmsg_ViewcarCtrl[i]));
            if (bytes != sizeof(JMEVmsg_ViewcarCtrl[i]))
            {
                printf("send msg to vehicle Error\n!");
                continue;
            }
            ofs << dec << start_ms << '\t'
                << hex << int(JMEVmsg_ViewcarCtrl[i].can_id & CAN_SFF_MASK)
                << '\t';
            for (int j = 0; j < 8; ++j)
                ofs << hex << int(JMEVmsg_ViewcarCtrl[i].data[j]) << '\t';
            ofs << endl;
        }
        count_data++;
        loop_rate.sleep();
        auto end_ms = chrono::duration_cast<chrono::milliseconds>(
                          chrono::system_clock::now().time_since_epoch())
                          .count();
        cout << "send_time: " << end_ms - start_ms << " ms" << endl;
    }
    ofs.close();
    pthread_exit(nullptr);
}

// ===================== 底层 CAN 接收线程 =====================
void *auto_driver_receive(void *arg)
{
    int can_skt = *(int *)arg;
    rclcpp::Rate loop_rate(50);
    struct can_frame frame;
    ofstream ofs("read.txt", ios::out);

    while (rclcpp::ok())
    {
        bool VCU_174h = false, ACU_24Ah = false, EPS_112h = false,
             EPS_114h = false, WCBS_92h = false, WCBS_93h = false,
             WCBS_96h = false, BCM_152h = false;

        auto start_ms = chrono::duration_cast<chrono::milliseconds>(
                            chrono::system_clock::now().time_since_epoch())
                            .count();

        while (true)
        {
            int bytes = read(can_skt, &frame, sizeof(frame));
            if (bytes != sizeof(frame))
            {
                printf("recv msg Error\n!");
                continue;
            }
            ViewVehicleData = frame;
            ofs << dec << start_ms << '\t'
                << hex << int(ViewVehicleData.can_id & CAN_SFF_MASK) << '\t';
            for (int j = 0; j < 8; ++j)
                ofs << int(ViewVehicleData.data[j]) << '\t';
            ofs << endl;

            JMEV_Data_Input();

            switch (frame.can_id & CAN_SFF_MASK)
            {
            case VCU_174h_ID: VCU_174h = true; break;
            case ACU_24Ah_ID: ACU_24Ah = true; break;
            case EPS_112h_ID: EPS_112h = true; break;
            case EPS_114h_ID: EPS_114h = true; break;
            case WCBS_92h_ID: WCBS_92h = true; break;
            case WCBS_93h_ID: WCBS_93h = true; break;
            case WCBS_96h_ID: WCBS_96h = true; break;
            case BCM_152h_ID: BCM_152h = true; break;
            default: break;
            }
            if (VCU_174h && ACU_24Ah && EPS_112h && EPS_114h &&
                WCBS_92h && WCBS_93h && WCBS_96h && BCM_152h)
                break;
        }
        JMEV_SIGNAL_Parsed();
        loop_rate.sleep();
        auto end_ms = chrono::duration_cast<chrono::milliseconds>(
                          chrono::system_clock::now().time_since_epoch())
                          .count();
        cout << "read_time: " << end_ms - start_ms << " ms" << endl;
    }
    ofs.close();
    pthread_exit(nullptr);
}

// ===================== CAN 驱动类，与原版相同 =====================
class can_vhicle_auto_driver
{
public:
    can_vhicle_auto_driver();
    ~can_vhicle_auto_driver();
    bool can_vhicle_auto_driver_start();

    int get_send_socket() const { return can_skt_send; }
    int get_recv_socket() const { return can_skt_rec; }

private:
    int can_skt_send;
    int can_skt_rec;
    struct sockaddr_can addr;
    struct ifreq ifr;
    pthread_t _pthread_send;
    pthread_t _pthread_receive;
    pthread_attr_t attr_send;
    pthread_attr_t attr_receive;
};

bool can_vhicle_auto_driver::can_vhicle_auto_driver_start()
{
    pthread_attr_init(&attr_send);
    pthread_attr_setdetachstate(&attr_send, 1);
    pthread_attr_init(&attr_receive);
    pthread_attr_setdetachstate(&attr_receive, 1);

    if (pthread_create(&_pthread_send, &attr_send, auto_driver_send,
                       &can_skt_send) != 0)
    {
        cerr << "send thread error\n";
        return false;
    }
    if (pthread_create(&_pthread_receive, &attr_receive, auto_driver_receive,
                       &can_skt_rec) != 0)
    {
        cerr << "receive thread error\n";
        return false;
    }
    return true;
}

can_vhicle_auto_driver::can_vhicle_auto_driver()
{
    // 启动 CAN 口脚本
    system("echo 'nvidia' | sudo -S bash /home/nvidia/ZHITAI/qingling_ros2/src/"
           "control/can_com/can_start.sh");

    // recv socket
    can_skt_rec = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(ifr.ifr_name, "can0");
    ioctl(can_skt_rec, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(can_skt_rec, (struct sockaddr *)&addr, sizeof(addr));

    // send socket
    can_skt_send = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(ifr.ifr_name, "can0");
    ioctl(can_skt_send, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(can_skt_send, (struct sockaddr *)&addr, sizeof(addr));

    // 过滤器
    struct can_filter rfilter[8] = {
        {VCU_174h_ID, CAN_SFF_MASK}, {ACU_24Ah_ID, CAN_SFF_MASK},
        {EPS_112h_ID, CAN_SFF_MASK}, {EPS_114h_ID, CAN_SFF_MASK},
        {WCBS_92h_ID, CAN_SFF_MASK}, {WCBS_93h_ID, CAN_SFF_MASK},
        {WCBS_96h_ID, CAN_SFF_MASK}, {BCM_152h_ID, CAN_SFF_MASK}};
    setsockopt(can_skt_send, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0);
    setsockopt(can_skt_rec, SOL_CAN_RAW, CAN_RAW_FILTER, rfilter,
               sizeof(rfilter));
}

can_vhicle_auto_driver::~can_vhicle_auto_driver()
{
    close(can_skt_send);
    close(can_skt_rec);
    cout << "can_vhicle_auto_driver destruct\n";
}

// ===================== 远程平台 Client 线程 =====================
static void *Client(void * /*unused*/)
{
    std_msgs::msg::UInt16MultiArray msg_Vecle;
    msg_Vecle.data.resize(26);
    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok())
    {
        int sock_cli = socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in servaddr {};
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(Remoter_PORT);
        servaddr.sin_addr.s_addr = inet_addr(Remoter_IP);

        if (connect(sock_cli, (sockaddr *)&servaddr, sizeof(servaddr)) < 0)
        {
            perror("connect");
            ::close(sock_cli);
            loop_rate.sleep();
            continue;
        }

        // Vehicle_Platform();
        send(sock_cli, msg_Platfom, 13, 0);
        recv(sock_cli, msg_FromPlatfom, sizeof(msg_FromPlatfom), 0);
        msg_count++;

        memset(msg_Platfom, 0, sizeof(msg_Platfom));
        memset(msg_FromPlatfom, 0, sizeof(msg_FromPlatfom));
        ::close(sock_cli);
        loop_rate.sleep();
    }
    pthread_exit(nullptr);
}

// ===================== ROS2 回调函数 =====================
void Planning_Callback(const decision_planning_msgs::msg::DecisionPlanning &msg)
{
    V_SlopeResisf32s20 = 0;
    v_road_typeu8 = 0;

    o_traiedel_x = msg.aim_x;
    o_traiedel_y = msg.aim_y;
    V_TrajeSpdf16s4 = msg.speed;

    V_LaneWidthf16s4 = 3.7;
    V_RefPoint = msg.vfpoint;

    V_VehPosXdou = V_VehPosX = msg.loca_x;
    V_VehPosYdou = V_VehPosY = msg.loca_y;
    V_VehPosAngdou = msg.loca_yaw;

    GPS_Heading = msg.loca_yaw * 180 / 3.1415926;
    vhicle_stop =
        (msg.trace_driving || msg.traffic_light_driving || msg.trace_stop);
}

void chatterCallback_fusion(const can_control_msgs::msg::Autocontrol & /*msg*/)
{
    // 该订阅暂未使用
}

void chatterCallback(const can_control_msgs::msg::Autocontrol &msg)
{
    RT1_Width_Rel = msg.front_width;
    V_Objwidth = RT1_Width_Rel;
    RT1_L_Long_Rel = msg.front_l_long_obj;
    RT1_L_Lat_Rel = msg.front_l_lat_obj;
    RT1_V_Long_Rel = msg.front_v_long_obj;
    RT1_V_Lat_Rel = msg.front_v_lat_obj;
    RT1_Class_Rel = msg.front_class;

    RT3_Width_Rel = msg.leftfront_width;
    RT3_L_Long_Rel = msg.leftfront_l_long_obj;
    RT3_L_Lat_Rel = msg.leftfront_l_lat_obj;
    RT3_V_Long_Rel = msg.leftfront_v_long_obj;
    RT3_V_Lat_Rel = msg.leftfront_v_lat_obj;
    RT3_Class_Rel = msg.leftfront_class;

    RT4_Width_Rel = msg.rightfront_width;
    RT4_L_Long_Rel = msg.rightfront_l_long_obj;
    RT4_L_Lat_Rel = msg.rightfront_l_lat_obj;
    RT4_V_Long_Rel = msg.rightfront_v_long_obj;
    RT4_V_Lat_Rel = msg.rightfront_v_lat_obj;
    RT4_Class_Rel = msg.rightfront_class;

    RT5_Width_Rel = msg.back_width;
    RT5_L_Long_Rel = msg.back_l_long_obj;
    RT5_L_Lat_Rel = msg.back_l_lat_obj;
    RT5_V_Long_Rel = msg.back_v_long_obj;
    RT5_V_Lat_Rel = msg.back_v_lat_obj;
    RT5_Class_Rel = msg.back_class;

    RT2_Width_Rel = msg.leftback_width;
    RT2_L_Long_Rel = msg.leftback_l_long_obj;
    RT2_L_Lat_Rel = msg.leftback_l_lat_obj;
    RT2_V_Long_Rel = msg.leftback_v_long_obj;
    RT2_V_Lat_Rel = msg.leftback_v_lat_obj;
    RT2_Class_Rel = msg.leftback_class;

    RT6_Width_Rel = msg.rightback_width;
    RT6_L_Long_Rel = msg.rightback_l_long_obj;
    RT6_L_Lat_Rel = msg.rightback_l_lat_obj;
    RT6_V_Long_Rel = msg.rightback_v_long_obj;
    RT6_V_Lat_Rel = msg.rightback_v_lat_obj;
    RT6_Class_Rel = msg.rightback_class;

    fusion_count = msg.fusion_count;
}

// ===================== ControlNode 类（ROS2 节点） =====================
class ControlNode : public rclcpp::Node
{
public:
    explicit ControlNode(shared_ptr<can_vhicle_auto_driver> driver)
        : Node("control_node"), driver_(std::move(driver))
    {
        sub_filter_ = create_subscription<can_control_msgs::msg::Autocontrol>(
            "route_filter/fusion", 100,
            std::bind(chatterCallback, std::placeholders::_1));

        sub_fusion_ = create_subscription<can_control_msgs::msg::Autocontrol>(
            "fusion", 100,
            std::bind(chatterCallback_fusion, std::placeholders::_1));

        plan_sub_ =
            create_subscription<decision_planning_msgs::msg::DecisionPlanning>(
                "decision_planning/aim", 100, Planning_Callback);

        vehicle_status_pub_ =
            create_publisher<can_control_msgs::msg::VehicleStatus>(
                "/vehicle_status", 10);

        timer_ = create_wall_timer(
            chrono::milliseconds(20),
            std::bind(&ControlNode::publish_vehicle_status, this));

        // 启动 RoutePlanning 初始化
        RoutePlanning_initialize();
    }

private:
    void publish_vehicle_status()
    {
        can_control_msgs::msg::VehicleStatus msg;
        msg.vehicle_spd = V_VehSpd;
        vehicle_status_pub_->publish(msg);
    }

    // -------------------- 成员 --------------------
    shared_ptr<can_vhicle_auto_driver> driver_;

    rclcpp::Subscription<can_control_msgs::msg::Autocontrol>::SharedPtr
        sub_filter_;
    rclcpp::Subscription<can_control_msgs::msg::Autocontrol>::SharedPtr
        sub_fusion_;
    rclcpp::Subscription<decision_planning_msgs::msg::DecisionPlanning>::SharedPtr
        plan_sub_;
    rclcpp::Publisher<can_control_msgs::msg::VehicleStatus>::SharedPtr
        vehicle_status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// ===================== main =====================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto driver = make_shared<can_vhicle_auto_driver>();
    driver->can_vhicle_auto_driver_start();

    // 可选：再开一个线程跑 Client（如果平台通信仍需要）
    pthread_t client_thread;
    pthread_create(&client_thread, nullptr, Client, nullptr);

    auto node = make_shared<ControlNode>(driver);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    system("echo 'nvidia' | sudo -S "
           "/home/nvidia/ZHITAI/qingling_ros2/src/control/can_com/can_down.sh");
    rclcpp::shutdown();
    return 0;
}
