/*****softwaredata input**********************
  File Name   : Vechicle_Data_input();
  Date        : 2019/11/11
  Version     : 1.0.1
  Desciption  : Vechicle CAN data input;
 *******************************/
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "../include/can_com/int.h"
#include "can_com/RoutePlanning.h"
#include "can_com/AutoVehicleControl.h"
#include <stdio.h>
#include <linux/can.h>
#include <linux/can/raw.h>
//#include <can_com/msg/radardate.hpp>
#include "std_msgs/msg/u_int16_multi_array.hpp"

//              JMEV
//recieve
#define VCU_174h_ID 0x174
#define ACU_24Ah_ID 0x24A
#define EPS_112h_ID 0x112
#define EPS_114h_ID 0x114
#define WCBS_92h_ID 0x92
#define WCBS_93h_ID 0x93
#define WCBS_96h_ID 0x96
#define BCM_152h_ID 0x152
//send
#define APA_170h_ID 0x170
#define APA_166h_ID 0x166
#define ADAS_160h_ID 0X160
#define ADAS_162h_ID 0x162
#define ADAS_163h_ID 0x163

#define CAN_SFF_MASK 0x000007FFU /* 标准帧格式 (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFU /* 扩展帧格式 (EFF) */
#define CAN_ERR_MASK 0x1FFFFFFFU /* 忽略 EFF, RTR, ERR 标志 */


#define CAN_EFF_FLAG 0x80000000U //发送扩展帧的标识

//recieve
VCU_174h VCU_174h_1 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
ACU_24Ah ACU_24Ah_1 = {0, 0, 0, 0, 0, 0, 0, 0};
EPS_112h EPS_112h_1 = {0, 0, 0, 0, 0, 0, 0};
EPS_114h EPS_114h_1 = {0, 0, 0, 0, 0, 0, 0, 0};
WCBS_92h WCBS_92h_1 = {0, 0, 0, 0, 0, 0, 0};
WCBS_93h WCBS_93h_1 = {0, 0, 0, 0, 0, 0, 0, 0};
WCBS_96h WCBS_96h_1 = {0, 0, 0, 0, 0, 0, 0, 0};
BCM_152h BCM_152h_1 = {0, 0, 0, 0, 0, 0, 0, 0, 0};

//send
APA_170h APA_170h_1 = {0, 0, 0, 0, 0, 0, 0, 0, 0};
ADAS_160h ADAS_160h_1 = {0, 0, 0, 0, 0, 0, 0, 0};
ADAS_162h ADAS_162h_1 = {0, 0, 0, 0, 0, 0, 0, 0, 0};
ADAS_163h ADAS_163h_1 = {0, 0, 0, 0, 0, 0};
APA_166h APA_166h_1 = {0, 0, 0, 0, 0, 0, 0, 0};

/***********************qingling_ros2*********************************/

uint16_t control_count = 0;
uint16_t fusion_count = 0;

struct can_frame ViewVehicleData;
uint8_t VehCANDATA[650];
uint8_t RadarDATA_1[650];
uint8_t msg_ViewcarCtrl[52];
//uint8_t JMEVmsg_ViewcarCtrl[78];
//uint8_t JMEVmsg_ViewcarCtrl[52];/*20210906 39-->52***/
can_frame JMEVmsg_ViewcarCtrl[8];
uint8_t msg[52];
uint8_t msg_Platfom[13];
uint8_t msg_FromPlatfom[650];
ViewVehicle_CANDATA ViewVehicle_CANDATA_1;

/*****************************************


// **************************/
// void Remote_Control()
// {
//   int Remote_count = 0;
//   for (Remote_count = 0; Remote_count < 50; Remote_count++)
//   {
//     Plat_CANDATA_1.MsgLengh.D = msg_FromPlatfom[Remote_count];
//     Plat_CANDATA_1.MsgID.D = (msg_FromPlatfom[Remote_count + 1] << 24) + (msg_FromPlatfom[Remote_count + 2] << 16) + (msg_FromPlatfom[Remote_count + 3] << 8) + msg_FromPlatfom[Remote_count + 4];
//     Plat_CANDATA_1.DATA0.D = msg_FromPlatfom[Remote_count + 5];
//     Plat_CANDATA_1.DATA1.D = msg_FromPlatfom[Remote_count + 6];
//     Plat_CANDATA_1.DATA2.D = msg_FromPlatfom[Remote_count + 7];
//     Plat_CANDATA_1.DATA3.D = msg_FromPlatfom[Remote_count + 8];
//     Plat_CANDATA_1.DATA4.D = msg_FromPlatfom[Remote_count + 9];
//     Plat_CANDATA_1.DATA5.D = msg_FromPlatfom[Remote_count + 10];
//     Plat_CANDATA_1.DATA6.D = msg_FromPlatfom[Remote_count + 11];
//     Plat_CANDATA_1.DATA7.D = msg_FromPlatfom[Remote_count + 12];
//     if (Plat_CANDATA_1.MsgID.bit.ID == AutoControlData601_ID)
//     {
//       AutoControlData_1.MsgData0.D = Plat_CANDATA_1.DATA0.D;
//       AutoControlData_1.MsgData1.D = Plat_CANDATA_1.DATA1.D;
//       AutoControlData_1.MsgData23.D = (Plat_CANDATA_1.DATA2.D << 8) + Plat_CANDATA_1.DATA3.D;
//       AutoControlData_1.MsgData45.D = (Plat_CANDATA_1.DATA4.D << 8) + Plat_CANDATA_1.DATA5.D;
//       AutoControlData_1.MsgData6.D = Plat_CANDATA_1.DATA6.D;
//       AutoControlData_1.MsgData7.D = Plat_CANDATA_1.DATA7.D;
//     }
//   }
// }

void JMEV_Data_Input()
{
  ViewVehicle_CANDATA_1.MsgID.D = ViewVehicleData.can_id & CAN_EFF_MASK; //((ViewVehicleData[1] <<24)+(ViewVehicleData[2] <<16)+(ViewVehicleData[3] <<8) + ViewVehicleData[4]);
  ViewVehicle_CANDATA_1.DATA0.D = ViewVehicleData.data[0];
  ViewVehicle_CANDATA_1.DATA1.D = ViewVehicleData.data[1];
  ViewVehicle_CANDATA_1.DATA2.D = ViewVehicleData.data[2];
  ViewVehicle_CANDATA_1.DATA3.D = ViewVehicleData.data[3];
  ViewVehicle_CANDATA_1.DATA4.D = ViewVehicleData.data[4];
  ViewVehicle_CANDATA_1.DATA5.D = ViewVehicleData.data[5];
  ViewVehicle_CANDATA_1.DATA6.D = ViewVehicleData.data[6];
  ViewVehicle_CANDATA_1.DATA7.D = ViewVehicleData.data[7];
  if (ViewVehicle_CANDATA_1.MsgID.bit.ID == VCU_174h_ID)
  {
    VCU_174h_1.DATA0.D = ViewVehicle_CANDATA_1.DATA0.D;
    VCU_174h_1.DATA1.D = ViewVehicle_CANDATA_1.DATA1.D;
    VCU_174h_1.DATA2.D = ViewVehicle_CANDATA_1.DATA2.D;
    VCU_174h_1.DATA3.D = ViewVehicle_CANDATA_1.DATA3.D;
    VCU_174h_1.DATA4.D = ViewVehicle_CANDATA_1.DATA4.D;
    VCU_174h_1.DATA5.D = ViewVehicle_CANDATA_1.DATA5.D;
    VCU_174h_1.DATA6.D = ViewVehicle_CANDATA_1.DATA6.D;
    VCU_174h_1.DATA7.D = ViewVehicle_CANDATA_1.DATA7.D;
  }

  if (ViewVehicle_CANDATA_1.MsgID.bit.ID == ACU_24Ah_ID)
  {
    ACU_24Ah_1.DATA0.D = ViewVehicle_CANDATA_1.DATA0.D;
    ACU_24Ah_1.DATA1.D = ViewVehicle_CANDATA_1.DATA1.D;
    ACU_24Ah_1.DATA23.D = (ViewVehicle_CANDATA_1.DATA2.D << 8) + ViewVehicle_CANDATA_1.DATA3.D;
    ACU_24Ah_1.DATA45.D = (ViewVehicle_CANDATA_1.DATA4.D << 8) + ViewVehicle_CANDATA_1.DATA5.D;
    ACU_24Ah_1.DATA6.D = ViewVehicle_CANDATA_1.DATA6.D;
    ACU_24Ah_1.DATA7.D = ViewVehicle_CANDATA_1.DATA7.D;
  }

  if (ViewVehicle_CANDATA_1.MsgID.bit.ID == EPS_112h_ID)
  {
    EPS_112h_1.DATA01.D = (ViewVehicle_CANDATA_1.DATA0.D << 8) + ViewVehicle_CANDATA_1.DATA1.D;
    EPS_112h_1.DATA23.D = (ViewVehicle_CANDATA_1.DATA2.D << 8) + ViewVehicle_CANDATA_1.DATA3.D;
    EPS_112h_1.DATA45.D = (ViewVehicle_CANDATA_1.DATA4.D << 8) + ViewVehicle_CANDATA_1.DATA5.D;
    EPS_112h_1.DATA6.D = ViewVehicle_CANDATA_1.DATA6.D;
    EPS_112h_1.DATA7.D = ViewVehicle_CANDATA_1.DATA7.D;
  }

  if (ViewVehicle_CANDATA_1.MsgID.bit.ID == EPS_114h_ID)
  {
    EPS_114h_1.DATA01.D = (ViewVehicle_CANDATA_1.DATA0.D << 8) + ViewVehicle_CANDATA_1.DATA1.D;
    EPS_114h_1.DATA2.D = ViewVehicle_CANDATA_1.DATA2.D;
    EPS_114h_1.DATA3.D = ViewVehicle_CANDATA_1.DATA3.D;
    EPS_114h_1.DATA45.D = (ViewVehicle_CANDATA_1.DATA4.D << 8) + ViewVehicle_CANDATA_1.DATA5.D;
    EPS_114h_1.DATA6.D = ViewVehicle_CANDATA_1.DATA6.D;
    EPS_114h_1.DATA7.D = ViewVehicle_CANDATA_1.DATA7.D;
  }

  if (ViewVehicle_CANDATA_1.MsgID.bit.ID == WCBS_92h_ID)
  {
    WCBS_92h_1.DATA01.D = (ViewVehicle_CANDATA_1.DATA0.D << 8) + ViewVehicle_CANDATA_1.DATA1.D;
    WCBS_92h_1.DATA23.D = (ViewVehicle_CANDATA_1.DATA2.D << 8) + ViewVehicle_CANDATA_1.DATA3.D;
    WCBS_92h_1.DATA45.D = (ViewVehicle_CANDATA_1.DATA4.D << 8) + ViewVehicle_CANDATA_1.DATA5.D;
    WCBS_92h_1.DATA6.D = ViewVehicle_CANDATA_1.DATA6.D;
    WCBS_92h_1.DATA7.D = ViewVehicle_CANDATA_1.DATA7.D;
  }

  if (ViewVehicle_CANDATA_1.MsgID.bit.ID == WCBS_93h_ID)
  {
    WCBS_93h_1.DATA0.D = ViewVehicle_CANDATA_1.DATA0.D;
    WCBS_93h_1.DATA12.D = (ViewVehicle_CANDATA_1.DATA1.D << 8) + ViewVehicle_CANDATA_1.DATA2.D;
    WCBS_93h_1.DATA34.D = (ViewVehicle_CANDATA_1.DATA3.D << 8) + ViewVehicle_CANDATA_1.DATA4.D;
    WCBS_93h_1.DATA5.D = ViewVehicle_CANDATA_1.DATA5.D;
    WCBS_93h_1.DATA6.D = ViewVehicle_CANDATA_1.DATA6.D;
    WCBS_93h_1.DATA7.D = ViewVehicle_CANDATA_1.DATA7.D;
  }

  if (ViewVehicle_CANDATA_1.MsgID.bit.ID == WCBS_96h_ID)
  {
    WCBS_96h_1.DATA01.D = (ViewVehicle_CANDATA_1.DATA0.D << 8) + ViewVehicle_CANDATA_1.DATA1.D;
    WCBS_96h_1.DATA23.D = (ViewVehicle_CANDATA_1.DATA2.D << 8) + ViewVehicle_CANDATA_1.DATA3.D;
    WCBS_96h_1.DATA4.D = ViewVehicle_CANDATA_1.DATA4.D;
    WCBS_96h_1.DATA5.D = ViewVehicle_CANDATA_1.DATA5.D;
    WCBS_96h_1.DATA6.D = ViewVehicle_CANDATA_1.DATA6.D;
    WCBS_96h_1.DATA7.D = ViewVehicle_CANDATA_1.DATA7.D;
  }

  if (ViewVehicle_CANDATA_1.MsgID.bit.ID == BCM_152h_ID)
  {
    BCM_152h_1.DATA0.D = ViewVehicle_CANDATA_1.DATA0.D;
    BCM_152h_1.DATA1.D = ViewVehicle_CANDATA_1.DATA1.D;
    BCM_152h_1.DATA2.D = ViewVehicle_CANDATA_1.DATA2.D;
    BCM_152h_1.DATA3.D = ViewVehicle_CANDATA_1.DATA3.D;
    BCM_152h_1.DATA4.D = ViewVehicle_CANDATA_1.DATA4.D;
    BCM_152h_1.DATA5.D = ViewVehicle_CANDATA_1.DATA5.D;
    BCM_152h_1.DATA67.D = (ViewVehicle_CANDATA_1.DATA6.D << 8) + ViewVehicle_CANDATA_1.DATA7.D;
  }
}

/***************************************************************

parsedCANData Transfers to Variables 

******************************/
void JMEV_SIGNAL_Parsed()
{
  // MC_Mode = BCM_152h_1.DATA0.bit.BCM_PositionLampSts; //BCM_152h_1.DATA2.bit.BCM_HazardWarnLampSt;
  /****************Vehicle Signal*************************/
  // int Sys_State = RTK_323_1.DATA0.D;
  // int StateStatus = RTK_323_1.DATA2.D;
  //RTK_323_1.DATA2.bit.StateStatus
  // if (MC_Mode == 1)
  // {
  //   if ((Sys_State == 2) && (StateStatus == 4))
  //   {
  //     MC_Mode = 1;
  //   }
  //   else
  //   {
  //     printf("Unstable RTK positioning status");
  //     MC_Mode = 0;
  //   }
  // }
  // else
  // {
  //   MC_Mode = 0;
  // }
  //int BCM_LowBeamStatus = BCM_152h_1.DATA0.bit.BCM_LowBeamStatus;
  RCLCPP_INFO(rclcpp::get_logger("control_node"), "BCM_HazardWarnLampSt %d\n",BCM_152h_1.DATA0.bit.BCM_PositionLampSts);
  MC_Mode = BCM_152h_1.DATA0.bit.BCM_PositionLampSts; //BCM_152h_1.DATA2.bit.BCM_HazardWarnLampSt;

  ESP_VehicleSpeed = (WCBS_92h_1.DATA01.bit.WCBS_ESC_VehSpd * 0.05625) * 16; // km/h
  //RCLCPP_INFO(rclcpp::get_logger("control_node"), "control: [%d], raw_data: [%f]",ESP_VehicleSpeed,Speed_Control_ACU23C_1.DATA56.D*0.1);
  TCU_GearShiftPositon = 3; //档位
  //VCU_ACU20C_1.DATA6.bit.VCU_ACU_VSpeed_Gear; //VCU_ACU20C_1.DATA6.bit.VCU_ACU_VSpeed_Gear;       //PRND
  //RCLCPP_INFO(rclcpp::get_logger("control_node"), "control: [%d], raw_data: [%d]", TCU_GearShiftPositon, VCU_ACU20C_1.DATA6.bit.VCU_ACU_VSpeed_Gear);
  EMS_BrakePedalStatus = 0; /*EMB_ACU21C_1.DATA0.bit.Front_EMB_Brake_Enable*/
  //RCLCPP_INFO(rclcpp::get_logger("control_node"), "control: [%d], raw_data: [%d]",EMS_BrakePedalStatus ,EMB_ACU21C_1.DATA0.bit.Front_EMB_Brake_Enable);
  V_VehSpd = WCBS_92h_1.DATA01.bit.WCBS_ESC_VehSpd * 0.05625 / 3.6; //m/s
  //RCLCPP_INFO(rclcpp::get_logger("control_node"), "V_VehSpd: [%f]",V_VehSpd);

  ESP_LongAccel = (WCBS_96h_1.DATA5.bit.WCBS_ESC_LongAcc * 0.01 - 1.27) * 256;
  //RCLCPP_INFO(rclcpp::get_logger("control_node"), "control: [%d], raw_data: [%f]",ESP_LongAccel ,WCBS_96h_1.DATA5.bit.WCBS_ESC_LongAcc*0.01-1.27);
  AT_ActualGear = 1; //123456
  EMS_MaxIndicatedTorq = 100 * 2;
  Wheel_Speed_RL_Data = 0 * 16;
  Wheel_Speed_RR_Data = 0 * 16;
  EPS_APA_ControlFeedback = 1;
  EPS_APA_Abortfeedback = 0;

  SAS_SteeringAngle = ((EPS_112h_1.DATA23.bit.EPS_SteerWheelAng * 0.1 - 780) * 16);
  //RCLCPP_INFO(rclcpp::get_logger("control_node"), "control: [%d], raw_data: [%f]", SAS_SteeringAngle, EPS_112h_1.DATA23.bit.EPS_SteerWheelAng * 0.1 - 780);
  ESP_YawRate = 0 * 256;
  EMS_AccPedal = 0 * 2;
  EMS_EngineSpeed = 0 * 4;
  EMS_IndicatedRealEngTorq = 10 * 256;
  EMS_MinIndicatedTorq = 10 * 2;
  EMS_IndicatedRealEngTorq = 10 * 256;
  EPS_APA_EpasFAILED = 0;
  BCM_TurnLightSwitchSts = 0;


  //OBU
  // V_TrafficLightDet = 0;
}

/***************************************************************

Control CAN MSG PACK 

******************************/
// uint8_t APA_ControlSts_AliveCounter = 0;
// uint8_t ADAS_160h_RollingCounter = 0;
// uint8_t ADAS_162h_RollingCounter = 0;
// uint8_t ADAS_163h_RollingCounter = 0;
// uint8_t APA_LCControl2_AliveCounter = 0;

int JL_Gear_Status = 0; //车辆当前档位状态临时接口
//int JL_Gear_number = 0;//车辆当前档位状态临时接口

void JMEV_Control_Output()
{
  uint8_t APA_ControlSts_CheckSUM = 0;
  uint8_t ADAS_160h_CheckSUM = 0;
  uint8_t ADAS_162h_CheckSUM = 0;
  uint8_t ADAS_163h_CheckSUM = 0;
  uint8_t APA_LCControl2_CheckSum = 0;
  static uint8_t APA_ControlSts_AliveCounter = 0;
  static uint8_t ADAS_160h_RollingCounter = 0;
  static uint8_t ADAS_162h_RollingCounter = 0;
  static uint8_t ADAS_163h_RollingCounter = 0;
  static uint8_t APA_LCControl2_AliveCounter = 0;
  //static int JL_steer_number = 0;//车辆当前转向状态临时接口

  /*******************************************控制报文初始化↓************************************************/
  /*******************************************APA_170h——档位控制************************************************/
  APA_170h_1.MsgLengh.D = 0x08;
  APA_170h_1.MsgID.bit.ID = APA_170h_ID;
  APA_170h_1.DATA0.bit.APA_GearRequest = 0;       //factor为1，Maximum为7，（无请求/P/R/N/D——0/1/2/3/4）
  APA_170h_1.DATA1.bit.APA_BrakeModeSts = 0;      //factor为1，Maximum为7，（纵向控制初始化/待机/激活/完成/中断/退出——0/1/2/3/4/5）
  APA_170h_1.DATA1.bit.APA_BrakeFunctionMode = 3; //factor为1，Maximum为7（无动作/舒适制动/紧急制动/动态制动——0/1/2/3）
  APA_170h_1.DATA2.bit.APA_SpeedLimit = 0 / 0.1;  //factor为0.1，Maximum为25.2（车速限值）
  APA_170h_1.DATA34.bit.APA_StopDistance = 0;     //factor为1，Maximum为4095（目标停车距离）
  APA_170h_1.DATA5.bit.APA_DegreeReqSignCmd = 0;  //factor为1，Maximum为3（转向灯请求：无请求/左转/右转——0/1/2）

  /*******************************************ADAS_166h**************************************************************/
  APA_166h_1.MsgLengh.D = 0x08;
  APA_166h_1.MsgID.bit.ID = APA_166h_ID;                          //换挡时ADAS处于静默状态，不能同时开启 ADAS_162h_1.DATA4.bit.ACC_ModeReq = 2;
  APA_166h_1.DATA0.bit.APA_EPS_Control_Request = 0;               //（无请求/请求EPS控制/EPS控制激活/非法值——0/1/2/3
  APA_166h_1.DATA12.bit.APA_SetSteeringWheelAng = (0 + 780) * 10; //转向角度请求，转速520°/s,车辆内部PID控制
  /*******************************************ADAS_163h**************************************************************/
  ADAS_163h_1.MsgLengh.D = 0x08;
  ADAS_163h_1.MsgID.bit.ID = ADAS_163h_ID;
  ADAS_163h_1.DATA01.bit.ACC_UpperJerkLimit = 1 / 0.01;             //加速度变化率上限
  ADAS_163h_1.DATA2345.bit.ACC_LowerJerkLimit = (-1.5 + 25) / 0.01; //(-25 + 25) / 0.01//加速度变化率下限
  ADAS_163h_1.DATA2345.bit.ACC_ComfortUpBand = (0 + 5) / 0.01;      //舒适加速度上区间
  ADAS_163h_1.DATA2345.bit.ACC_ComfortLoBand = (0 + 5) / 0.01;      //舒适加速度下区间
  /*******************************************ADAS_162h **************************************************************/
  ADAS_162h_1.MsgLengh.D = 0x08;
  ADAS_162h_1.MsgID.bit.ID = ADAS_162h_ID;                            //aeb/apa/acc只能同时使用一个，或者自己进行优先级判断
  ADAS_162h_1.DATA0.bit.AEB_ABA_Level = 0;                            //液压制动辅助级别（正常压力/低压/中压/高压——0/1/2/3）
  ADAS_162h_1.DATA0.bit.AEB_ABA_Req = 0;                              //制动辅助请求（无请求/请求——0/1）
  ADAS_162h_1.DATA0.bit.AEB_Prefill_Request = 0;                      //AEB制动预充请求（无请求/请求——0/1）
  ADAS_162h_1.DATA0.bit.AEB_AWB_Req = 0;                              //警告制动请求（无请求/请求——0/1）
  ADAS_162h_1.DATA0.bit.AEB_state = 0;                                //指示AEB系统状态(不可用/关闭/待命/激活无介入/激活——0/1/2/3/4)
  ADAS_162h_1.DATA12.bit.AEB_DecCtlReq = 0;                           //紧急制动减速控制请求（无请求/请求——0/1）
  ADAS_162h_1.DATA12.bit.FCW_State = 0;                               //指示FCW状态(不可用/关闭/待命/激活无介入/激活——0/1/2/3/4)
  ADAS_162h_1.DATA12.bit.AEB_Decelerate_Request = (0 + 20.47) / 0.01; //AEB请求的减速度值(-20.7+20.47)/0.01,单位m/s²
  ADAS_162h_1.DATA3.bit.ACC_MiniBraking = 0;                          //轻微制动请求（无请求/请求——0/1）
  ADAS_162h_1.DATA3.bit.ADAS_Sensorstatus = 0;                        //传感器故障状态（无故障/摄像头初始化/摄像头被挡/摄像头系统故障——0/1/2/3）
  ADAS_162h_1.DATA3.bit.HW_Warning_Level = 0;                         //指示HW当前报警等级（无报警/等级1/等级2——0/1/2）
  ADAS_162h_1.DATA3.bit.FCW_Warning_Level = 0;                        //（无报警/等级1/等级2——0/1/2）
  ADAS_162h_1.DATA4.bit.ACC_CIPV_indicator = 0;                       //指示最近在径前车状态
  ADAS_162h_1.DATA4.bit.ACC_Auto_Cancel = 0;                          //指示系统自动取消了ACC功能
  ADAS_162h_1.DATA4.bit.ACC_Driver_Cancel = 0;                        //指示驾驶员取消ACC
  ADAS_162h_1.DATA4.bit.ACC_ModeReq = 2;                              //自适应巡航工作模式请求(关闭/被动/等待/激活控制/仅制动/覆盖/停顿/错误——0/1/2/3/4/5/6/7)
  ADAS_162h_1.DATA5.bit.AEB_AWB_Level = 0;                            //警告制动级别
  ADAS_162h_1.DATA5.bit.ACC_ShutDownMode = 0;                         //3退出模式(缓慢关闭/快速关闭/立刻关闭/无请求——0/1/2/3)
  ADAS_162h_1.DATA5.bit.ACC_DriveOff = 0;                             //车辆起步（未激活/激活——0/1）
  ADAS_162h_1.DATA5.bit.ACC_BrkPreferred = 0;                         //制动优先——（未激活/激活——0/1）
  ADAS_162h_1.DATA5.bit.ACC_Go_Indicator = 0;                         //stop & go功能的前进指示（无请求/请求——0/1）
  ADAS_162h_1.DATA5.bit.ACC_Takeover_Indicator = 0;                   //请求驾驶员接管（无请求/请求——0/1）
  /******************************************* ADAS_160h ***********************************************/
  ADAS_160h_1.MsgLengh.D = 0x08;
  ADAS_160h_1.MsgID.bit.ID = ADAS_160h_ID;
  ADAS_160h_1.DATA0.bit.ACC_Set_Speed = 0;                         //驾驶员选择的巡航速度
  ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = (0 + 5) / 0.005; //(-5 + 5) / 0.005
  ADAS_160h_1.DATA12.bit.ACC_DecToStop = 0;                        //减速停车（未激活/激活——0/1）
  ADAS_160h_1.DATA12.bit.ACC_Set_Headway = 0;                      //驾驶员选择的跟车距离4
  ADAS_160h_1.DATA45.bit.TJAHWA_Auto_Cancel = 0;
  ADAS_160h_1.DATA45.bit.ALC_Auto_Ccancel = 0;

  // if(changelane_status > 0){
  //   ROS_WARN("changelane_status:%d",changelane_status);
  //   ROS_WARN("max_steer_angle:%d",max_steer_angle);
  //     if(std::fabs(StC_SteeringAngleRequest/16 -steer_angle_old )>max_steer_angle){
  //       ROS_WARN("steer_angle_old1:%d",steer_angle_old);
  //          ROS_WARN("StC_SteeringAngleRequest1:%d",StC_SteeringAngleRequest);
  // if(StC_SteeringAngleRequest/16 -steer_angle_old >0){
  //     StC_SteeringAngleRequest = (steer_angle_old+max_steer_angle)*16;
  //   steer_angle_old = StC_SteeringAngleRequest/16;
  // }
  // else{
  //   StC_SteeringAngleRequest = (steer_angle_old+max_steer_angle)*16;
  //   steer_angle_old = StC_SteeringAngleRequest/16;
  // }
  //
  //        ROS_WARN("steer_angle_old2:%d",steer_angle_old);
  //         ROS_WARN("StC_SteeringAngleRequest2:%d",StC_SteeringAngleRequest);
  //   }
  // }
  // else{
  //         steer_angle_old = StC_SteeringAngleRequest/16;
  //        ROS_WARN("steer_angle_old3:%d",steer_angle_old);
  // }

  /*******************************************控制报文初始化↑************************************************/
  /*******************************************杂项************************************************/
  static int trigger = 0;
  //static int JL_Gear_Status = 0;//车辆当前档位状态临时接口
  // MC_Mode = 1;
  //RCLCPP_INFO(rclcpp::get_logger("control_node"), "V_VehSpd: %f", V_VehSpd);
  JL_Gear_Status = VCU_174h_1.DATA0.bit.VCU_GearPosition;
  RCLCPP_ERROR(rclcpp::get_logger("control_node"), " V_RefPoint :%d", V_RefPoint );
  if (MC_Mode == 1)
  {
    /*********************************************档位控制↓*******************************************************/
    if (JL_Gear_Status != 4)
    {
      ADAS_162h_1.DATA4.bit.ACC_ModeReq = 2; //自适应巡航工作模式请求(关闭/被动/等待/激活控制/仅制动/覆盖/停顿/错误——0/1/2/3/4/5/6/7)

      if (WCBS_93h_1.DATA34.bit.WCBS_EHB_APA_LC_Status == 1)
      {
        APA_170h_1.DATA1.bit.APA_BrakeModeSts = 2; //factor为1，Maximum为7，（纵向控制初始化/待机/激活/完成/中断/退出——0/1/2/3/4/5）
      }
      else if (WCBS_93h_1.DATA34.bit.WCBS_EHB_APA_LC_Status == 4)
      {
        APA_170h_1.DATA0.bit.APA_GearRequest = 4;       //factor为1，Maximum为7，（无请求/P/R/N/D——0/1/2/3/4）
        APA_170h_1.DATA1.bit.APA_BrakeModeSts = 2;      //factor为1，Maximum为7，（纵向控制初始化/待机/激活/完成/中断/退出——0/1/2/3/4/5）
        APA_170h_1.DATA1.bit.APA_BrakeFunctionMode = 3; //(无动作/舒适性制动/紧急制动/自动泊车制动模式制动——0/1/2/3/)
        APA_170h_1.DATA2.bit.APA_SpeedLimit = 0;        //0 / 0.1//factor为0.1，Maximum为25.2（车速限值）
        APA_170h_1.DATA34.bit.APA_StopDistance = 0;     //factor为1，MaxiAPA_BrakeModeStsAPA_BrakeModeStsum为4095（目标停车距离）

        // if(JL_Gear_number<100)
        // {
        //   JL_Gear_number = JL_Gear_number+1 ;
        // }
        // else
        // {
        //   JL_Gear_Status = 1;
        // }
      }
      else
      {
        APA_170h_1.DATA1.bit.APA_BrakeModeSts = 1; //factor为1，Maximum为7，（纵向控制初始化/待机/激活/完成/中断/退出——0/1/2/3/4/5）
        //ADAS_162h_1.DATA4.bit.ACC_ModeReq = 2;//自适应巡航工作模式请求(关闭/被动/等待/激活控制/仅制动/覆盖/停顿/错误——0/1/2/3/4/5/6/7)
      }
      //RCLCPP_INFO(rclcpp::get_logger("control_node"), "WCBS_EHB_APA_LC_Status: %d", WCBS_93h_1.DATA34.bit.WCBS_EHB_APA_LC_Status);
    }

    else if (JL_Gear_Status == 4)
    {
      /*******************************************APA_166h——转向控制************************************************/
      // RCLCPP_ERROR(rclcpp::get_logger("control_node"), "EPS_APA_CONTROL_STS_0: %d", EPS_114h_1.DATA2.bit.EPS_APA_CONTROL_STS);
      //RCLCPP_INFO(rclcpp::get_logger("control_node"), "EPS_APA_FAULT_STS_0: %d", EPS_114h_1.DATA2.bit.EPS_APA_FAULT_STS);
      //RCLCPP_INFO(rclcpp::get_logger("control_node"), "JL_steer_number: %d", JL_steer_number);
      //if((EPS_114h_1.DATA2.bit.EPS_APA_CONTROL_STS == 4)&&(JL_steer_number==0))
      if (EPS_114h_1.DATA2.bit.EPS_APA_CONTROL_STS == 4)
      {
        APA_166h_1.DATA0.bit.APA_EPS_Control_Request = 0;
        APA_166h_1.DATA12.bit.APA_SetSteeringWheelAng = (0 + 780) / 0.1;
      }
      else if (EPS_114h_1.DATA2.bit.EPS_APA_CONTROL_STS == 2)
      //APA_EPS_Control_Request发1，EPS_APA_CONTROL_STS返回2——APA_EPS_Control_Request发1，EPS_APA_CONTROL_STS返回3
      {
        APA_166h_1.DATA0.bit.APA_EPS_Control_Request = 1;
        // APA_166h_1.DATA12.bit.APA_SetSteeringWheelAng = (20 + 780)/0.1;//(-780 + 780) * 10,自己进行有效性判断——100
        if((StC_SteeringAngleRequest / 16)>450)
            APA_166h_1.DATA12.bit.APA_SetSteeringWheelAng = (450 + 780) * 10;
        else if((StC_SteeringAngleRequest / 16)<-450)
            APA_166h_1.DATA12.bit.APA_SetSteeringWheelAng = (-450 + 780) * 10;
        else
            APA_166h_1.DATA12.bit.APA_SetSteeringWheelAng = ((StC_SteeringAngleRequest / 16)+ 780) * 10;
        // if (changelane_status > 0)-450
        // {
        //   printf("11111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111");
        //   APA_166h_1.DATA12.bit.APA_SetSteeringWheelAng = ((StC_SteeringAngleRequest/16) + 780)* 10;
        // }
      }
      else if (EPS_114h_1.DATA2.bit.EPS_APA_CONTROL_STS == 3)
      {
        APA_166h_1.DATA0.bit.APA_EPS_Control_Request = 2;
        if((StC_SteeringAngleRequest / 16)>450)
            APA_166h_1.DATA12.bit.APA_SetSteeringWheelAng = (450 + 780) * 10;
        else if((StC_SteeringAngleRequest / 16)<-450)
            APA_166h_1.DATA12.bit.APA_SetSteeringWheelAng = (-450 + 780) * 10;
        else
            APA_166h_1.DATA12.bit.APA_SetSteeringWheelAng = ((StC_SteeringAngleRequest / 16)+ 780) * 10;
        // if (changelane_status > 0)
        // {
        //   APA_166h_1.DATA12.bit.APA_SetSteeringWheelAng = ((StC_SteeringAngleRequest/16) + 780)* 10;
        //   printf("222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222");
        // }
        // APA_166h_1.DATA12.bit.APA_SetSteeringWheelAng = (20 + 780)/0.1;//(-780 + 780) * 10,自己进行有效性判断——100
        //RCLCPP_INFO(rclcpp::get_logger("control_node"), "EPS_APA_CONTROL_STS: %d", EPS_114h_1.DATA2.bit.EPS_APA_CONTROL_STS);
        //RCLCPP_INFO(rclcpp::get_logger("control_node"), "EPS_APA_FAULT_STS: %d", EPS_114h_1.DATA2.bit.EPS_APA_FAULT_STS);
        //JL_steer_number=1;
      }
      //else if((EPS_114h_1.DATA2.bit.EPS_APA_CONTROL_STS == 1)||(EPS_114h_1.DATA2.bit.EPS_APA_CONTROL_STS == 2))
      else //APA_EPS_Control_Request发1，EPS_APA_CONTROL_STS返回2——APA_EPS_Control_Request发1，EPS_APA_CONTROL_STS返回3
      {
        APA_166h_1.DATA0.bit.APA_EPS_Control_Request = 0;
        APA_166h_1.DATA12.bit.APA_SetSteeringWheelAng = (0 + 780) / 0.1;
      }
      // RCLCPP_ERROR(rclcpp::get_logger("control_node"), "WCBS_93h_1.DATA0.bit.WCBS_EHB_ACC_VLC_Active:%d",WCBS_93h_1.DATA0.bit.WCBS_EHB_ACC_VLC_Active);
      // RCLCPP_ERROR(rclcpp::get_logger("control_node"), " V_RefPoint :%d", V_RefPoint );

      /*******************************************APA_160h——加减速控制************************************************/
      if (((V_VehSpd >= C_TrajeSpd[V_RefPoint]) && (V_RefPoint <= (20000)) && ((RT1_L_Long_Rel > 15) || (RT1_L_Long_Rel < 0.01))) || (V_VehSpd >= 4))
      {
        if ((WCBS_93h_1.DATA0.bit.WCBS_EHB_ACC_VLC_Active == 0 && WCBS_93h_1.DATA0.bit.WCBS_EHB_ACC_VLC_Available == 1) || (WCBS_93h_1.DATA0.bit.WCBS_EHB_ACC_VLC_Active == 1 && WCBS_93h_1.DATA0.bit.WCBS_EHB_ACC_VLC_Available == 1))
        {
          ADAS_162h_1.DATA4.bit.ACC_ModeReq = 3;
          ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = (0 + 5) / 0.005;
          //(((TqR_AccTrqReq/100)*20)/256+5)/0.005;//加速减速都用此值，最大减速度-0.5
          ADAS_160h_1.DATA12.bit.ACC_DecToStop = 0; //减速至停车
          ADAS_160h_1.DATA0.bit.ACC_Set_Speed = 40; //速度限制40
          ADAS_160h_1.DATA12.bit.ACC_Set_Headway = 2;
          ///*((V_RefPoint > 490)&&(V_RefPoint < 1943))
          if (TqR_AccTrqReqEna > 0)
          {
            // ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = ((TqR_AccTrqReq/256) * 1 + 5)/0.005;
            ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = ((0 + 5) / 0.005);
            // RCLCPP_ERROR(rclcpp::get_logger("control_node"), "3333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333");
          }

          if (TqR_CDDAxEnable > 0)
          {
            // ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = ((TqR_ACCTargetAccelerationReq/16)* 1 + 5)/0.005;
            ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = (-0.3 + 5) / 0.005;
            // RCLCPP_ERROR(rclcpp::get_logger("control_node"), "4444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444");
          }
          //*/
        }
        if (WCBS_93h_1.DATA0.bit.WCBS_EHB_ACC_VLC_Active == 1 && WCBS_93h_1.DATA0.bit.WCBS_EHB_ACC_VLC_Available == 1)
        {
          if (V_VehSpd > 1)
          {
            trigger = 1;
          }
          ADAS_162h_1.DATA4.bit.ACC_ModeReq = 3;
          if (TqR_AccTrqReqEna > 0) //加速控制
          {
            // ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = ((TqR_AccTrqReq / 256) * 0.05 + 5) / 0.005;
            ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = ((0 + 5) / 0.005);
            // RCLCPP_ERROR(rclcpp::get_logger("control_node"), "1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111");
          }

          if (TqR_CDDAxEnable > 0) //减速控制
          {
            // ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = ((TqR_ACCTargetAccelerationReq/16)* 1 + 5)/0.005;
            ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = (-0.3 + 5) / 0.005;
            // RCLCPP_ERROR(rclcpp::get_logger("control_node"), "22222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222");
          }
        }
        // else if ((RT1_L_Long_Rel > 10) && (RT1_L_Long_Rel <= 20) && (forbid_change_lane == true))
        else if ((RT1_L_Long_Rel > 15) && (RT1_L_Long_Rel <= 20) )
        {
          if ((WCBS_93h_1.DATA0.bit.WCBS_EHB_ACC_VLC_Active == 0 && WCBS_93h_1.DATA0.bit.WCBS_EHB_ACC_VLC_Available == 1) || (WCBS_93h_1.DATA0.bit.WCBS_EHB_ACC_VLC_Active == 1 && WCBS_93h_1.DATA0.bit.WCBS_EHB_ACC_VLC_Available == 1))
          {
            ADAS_162h_1.DATA4.bit.ACC_ModeReq = 3;
            ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = (0 + 5) / 0.005;
            //(((TqR_AccTrqReq/100)*20)/256+5)/0.005;//加速减速都用此值，最大减速度-0.5
            ADAS_160h_1.DATA12.bit.ACC_DecToStop = 0; //减速至停车
            ADAS_160h_1.DATA0.bit.ACC_Set_Speed = 40; //速度限制40
            ADAS_160h_1.DATA12.bit.ACC_Set_Headway = 2;
            ///*((V_RefPoint > 490)&&(V_RefPoint < 1943))
            if (TqR_AccTrqReqEna > 0)
            {
              // ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = ((TqR_AccTrqReq/256) * 1 + 5)/0.005;
              ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = ((0 + 5) / 0.005);
            }

            if (TqR_CDDAxEnable > 0)
            {
              // ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = ((TqR_ACCTargetAccelerationReq/16)* 1 + 5)/0.005;
              ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = (-0.3 + 5) / 0.005;
            }
            //*/
          }
        }
        else if (trigger == 1)
        {

          ADAS_162h_1.DATA4.bit.ACC_ModeReq = 3;
          ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = (-1 + 5) / 0.005;
          // ADAS_162h_1.DATA0123.bit.AEB_DecColReq = 1;
          // ADAS_162h_1.DATA0123.bit.AEB_DecReq = (-4+20.47)/0.01;
          if (V_VehSpd < 1)
          {
            trigger = 0;
          }
        }
      }
      else
      {
        if ((WCBS_93h_1.DATA0.bit.WCBS_EHB_ACC_VLC_Active == 0 && WCBS_93h_1.DATA0.bit.WCBS_EHB_ACC_VLC_Available == 1) || (WCBS_93h_1.DATA0.bit.WCBS_EHB_ACC_VLC_Active == 1 && WCBS_93h_1.DATA0.bit.WCBS_EHB_ACC_VLC_Available == 1))
        {
          // RCLCPP_ERROR(rclcpp::get_logger("control_node"), "jiajiansu");
          // RCLCPP_ERROR(rclcpp::get_logger("control_node"), "TqR_AccTrqReqEna:%f",TqR_AccTrqReqEna);
          ADAS_162h_1.DATA4.bit.ACC_ModeReq = 3;
          ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = (0 + 5) / 0.005;
          //(((TqR_AccTrqReq/100)*20)/256+5)/0.005;//加速减速都用此值，最大减速度-0.5
          ADAS_160h_1.DATA12.bit.ACC_DecToStop = 0; //减速至停车
          ADAS_160h_1.DATA0.bit.ACC_Set_Speed = 40; //速度限制40
          ADAS_160h_1.DATA12.bit.ACC_Set_Headway = 2;
          ///*
          if (TqR_AccTrqReqEna > 0)
          {
            ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = ((TqR_AccTrqReq / 256) * 1 + 5) / 0.005;
            //  ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = ((0.5) * 1 + 5) / 0.005;
          }

          if (TqR_CDDAxEnable > 0)
          {
            ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = ((TqR_ACCTargetAccelerationReq / 16) * 1.3 + 5) / 0.005;
            // ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = ((0)* 1 + 5)/0.005;
            APA_170h_1.DATA34.bit.APA_StopDistance = 0;     //factor为1，MaxiAPA_BrakeModeStsAPA_BrakeModeStsum为4095（目标停车距离）
            if((V_VehSpd>10)&&(RT1_L_Long_Rel<20))
            {
              ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = ((-0.5)* 1 + 5)/0.005;
            }
          }
           //ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = ((2)* 1 + 5)/0.005;
          //*/
        }
        if (WCBS_93h_1.DATA0.bit.WCBS_EHB_ACC_VLC_Active == 1 && WCBS_93h_1.DATA0.bit.WCBS_EHB_ACC_VLC_Available == 1)
        {
          // RCLCPP_ERROR(rclcpp::get_logger("control_node"), "1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111");
          if (V_VehSpd > 1)
          {
            trigger = 1;
          }
          ADAS_162h_1.DATA4.bit.ACC_ModeReq = 3;
          if (TqR_AccTrqReqEna > 0) //加速控制
          {
            ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = ((TqR_AccTrqReq / 256) * 0.05 + 5) / 0.005;
            // ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = ((TqR_AccTrqReq / 256) * 0.05 + 5) / 0.005;
            // ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = ((0.5) * 1 + 5) / 0.005;

            //ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = (0 + 5) / 0.005;
            // RCLCPP_ERROR(rclcpp::get_logger("control_node"), "1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111");
          }

          if (TqR_CDDAxEnable > 0) //减速控制
          {
            ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = ((TqR_ACCTargetAccelerationReq / 16) * 1.3 + 5) / 0.005;
            // ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = ((0)* 1 + 5)/0.005;
            if((V_VehSpd>10)&&(RT1_L_Long_Rel<20))
            {
              ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = ((-0.5)* 1 + 5)/0.005;
            }
          }
          //ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = ((2)* 1 + 5)/0.005;

        }
        else if (trigger == 1)
        {

          ADAS_162h_1.DATA4.bit.ACC_ModeReq = 3;
          ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = (-1 + 5) / 0.005;
          // RCLCPP_ERROR(rclcpp::get_logger("control_node"), "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000");
          // ADAS_162h_1.DATA0123.bit.AEB_DecColReq = 1;
          // ADAS_162h_1.DATA0123.bit.AEB_DecReq = (-4+20.47)/0.01;
          if (V_VehSpd < 1)
          {
            trigger = 0;
          }
        }
      }
      // RCLCPP_ERROR(rclcpp::get_logger("control_node"), "ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request: %d", ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request);

      // RCLCPP_ERROR(rclcpp::get_logger("control_node"), "TqR_ACCTargetAccelerationReq: %f", TqR_ACCTargetAccelerationReq);
    }
    //RCLCPP_INFO(rclcpp::get_logger("control_node"), "APA_GearRequest: %d", APA_170h_1.DATA0.bit.APA_GearRequest);
  }

  else
  {
    ADAS_162h_1.DATA4.bit.ACC_ModeReq = 2;
    ADAS_160h_1.DATA12.bit.ACC_Accelerate_Request = (0 + 5) / 0.005;
    // ADAS_162h_1.DATA0123.bit.AEB_DecColReq = 0;
    // ADAS_162h_1.DATA0123.bit.AEB_DecReq = (0+20.47)/0.01;
  }

  /*******************************************控制报文输出组包↓************************************************/
  if (APA_ControlSts_AliveCounter > 15)
  {
    APA_ControlSts_AliveCounter = 0;
  }
  APA_166h_1.DATA6.bit.APA_ControlSts_AliveCounter = APA_ControlSts_AliveCounter;
  APA_ControlSts_AliveCounter = APA_ControlSts_AliveCounter + 1;

  JMEVmsg_ViewcarCtrl[0].can_id =APA_166h_1.MsgID.D;
  JMEVmsg_ViewcarCtrl[0].can_dlc = APA_166h_1.MsgLengh.D;
  JMEVmsg_ViewcarCtrl[0].data[0] = APA_166h_1.DATA0.D;
  JMEVmsg_ViewcarCtrl[0].data[1] = APA_166h_1.DATA12.D >> 8 & 0xFF;
  JMEVmsg_ViewcarCtrl[0].data[2] = APA_166h_1.DATA12.D & 0xFF;
  JMEVmsg_ViewcarCtrl[0].data[3] = APA_166h_1.DATA3.D & 0xFF;
  JMEVmsg_ViewcarCtrl[0].data[4] = APA_166h_1.DATA45.D  >> 8 & 0xFF;
  JMEVmsg_ViewcarCtrl[0].data[5] = APA_166h_1.DATA45.D & 0xFF;
  JMEVmsg_ViewcarCtrl[0].data[6] = APA_166h_1.DATA6.D & 0xFF;
  APA_ControlSts_CheckSUM = ((JMEVmsg_ViewcarCtrl[0].data[0] +
                              JMEVmsg_ViewcarCtrl[0].data[1] + JMEVmsg_ViewcarCtrl[0].data[2] +
                              JMEVmsg_ViewcarCtrl[0].data[3] + JMEVmsg_ViewcarCtrl[0].data[4] +
                              JMEVmsg_ViewcarCtrl[0].data[5] + JMEVmsg_ViewcarCtrl[0].data[6]) ^
                             (0xFF));
  APA_166h_1.DATA7.bit.APA_ControlSts_CheckSUM = APA_ControlSts_CheckSUM;
  JMEVmsg_ViewcarCtrl[0].data[7] = APA_166h_1.DATA7.D & 0xFF;

  if (ADAS_160h_RollingCounter > 15)
  {
    ADAS_160h_RollingCounter = 0;
  }
  ADAS_160h_1.DATA6.bit.ADAS_160h_RollingCounter = ADAS_160h_RollingCounter;
  ADAS_160h_RollingCounter = ADAS_160h_RollingCounter + 1;

  JMEVmsg_ViewcarCtrl[1].can_id = ADAS_160h_1.MsgID.D;
  JMEVmsg_ViewcarCtrl[1].can_dlc = ADAS_160h_1.MsgLengh.D;
  JMEVmsg_ViewcarCtrl[1].data[0] = ADAS_160h_1.DATA0.D & 0xFF;
  JMEVmsg_ViewcarCtrl[1].data[1] = ADAS_160h_1.DATA12.D >> 8 & 0xFF;
  JMEVmsg_ViewcarCtrl[1].data[2] = ADAS_160h_1.DATA12.D & 0xFF;
  JMEVmsg_ViewcarCtrl[1].data[3] = ADAS_160h_1.DATA3.D & 0xFF;
  JMEVmsg_ViewcarCtrl[1].data[4] = ADAS_160h_1.DATA45.D >> 8 & 0xFF;
  JMEVmsg_ViewcarCtrl[1].data[5] = ADAS_160h_1.DATA45.D & 0xFF;
  JMEVmsg_ViewcarCtrl[1].data[6] = ADAS_160h_1.DATA6.D & 0xFF;
  ADAS_160h_CheckSUM = ((JMEVmsg_ViewcarCtrl[1].data[0] +
                         JMEVmsg_ViewcarCtrl[1].data[1] + JMEVmsg_ViewcarCtrl[1].data[2] +
                         JMEVmsg_ViewcarCtrl[1].data[3] + JMEVmsg_ViewcarCtrl[1].data[4] +
                         JMEVmsg_ViewcarCtrl[1].data[5] + JMEVmsg_ViewcarCtrl[1].data[6]) ^
                        (0xFF));
  ADAS_160h_1.DATA7.bit.ADAS_160h_CheckSUM = ADAS_160h_CheckSUM;
  JMEVmsg_ViewcarCtrl[1].data[7] = ADAS_160h_1.DATA7.D & 0xFF;

  if (ADAS_162h_RollingCounter > 15)
  {
    ADAS_162h_RollingCounter = 0;
  }
  // //printf("ADAS_162h_RollingCounter: %d \n", ADAS_162h_RollingCounter);
  ADAS_162h_1.DATA6.bit.ADAS_162h_RollingCounter = ADAS_162h_RollingCounter;
  ADAS_162h_RollingCounter = ADAS_162h_RollingCounter + 1;

  JMEVmsg_ViewcarCtrl[2].can_id = ADAS_162h_1.MsgID.D;
  JMEVmsg_ViewcarCtrl[2].can_dlc = ADAS_162h_1.MsgLengh.D;
  JMEVmsg_ViewcarCtrl[2].data[0] = ADAS_162h_1.DATA0.D & 0xFF;
  JMEVmsg_ViewcarCtrl[2].data[1] = ADAS_162h_1.DATA12.D >> 8 & 0xFF;
  JMEVmsg_ViewcarCtrl[2].data[2] = ADAS_162h_1.DATA12.D & 0xFF;
  JMEVmsg_ViewcarCtrl[2].data[3] = ADAS_162h_1.DATA3.D & 0xFF;
  JMEVmsg_ViewcarCtrl[2].data[4] = ADAS_162h_1.DATA4.D & 0xFF;
  JMEVmsg_ViewcarCtrl[2].data[5] = ADAS_162h_1.DATA5.D & 0xFF;
  JMEVmsg_ViewcarCtrl[2].data[6] = ADAS_162h_1.DATA6.D & 0xFF;
  ADAS_162h_CheckSUM = ((JMEVmsg_ViewcarCtrl[2].data[0] +
                         JMEVmsg_ViewcarCtrl[2].data[1] + JMEVmsg_ViewcarCtrl[2].data[2] +
                         JMEVmsg_ViewcarCtrl[2].data[3] + JMEVmsg_ViewcarCtrl[2].data[4] +
                         JMEVmsg_ViewcarCtrl[2].data[5] + JMEVmsg_ViewcarCtrl[2].data[6]) ^
                        (0xFF));
  ADAS_162h_1.DATA7.bit.ADAS_162h_CheckSUM = ADAS_162h_CheckSUM;
  JMEVmsg_ViewcarCtrl[2].data[7] = ADAS_162h_1.DATA7.D & 0xFF;

  if (ADAS_163h_RollingCounter > 15)
  {
    ADAS_163h_RollingCounter = 0;
  }
  ADAS_163h_1.DATA6.bit.ADAS_163h_RollingCounter = ADAS_163h_RollingCounter;
  ADAS_163h_RollingCounter = ADAS_163h_RollingCounter + 1;
  JMEVmsg_ViewcarCtrl[3].can_id = ADAS_163h_1.MsgID.D;
  JMEVmsg_ViewcarCtrl[3].can_dlc = ADAS_163h_1.MsgLengh.D;
  JMEVmsg_ViewcarCtrl[3].data[0] = ADAS_163h_1.DATA01.D >> 8 & 0xFF;
  JMEVmsg_ViewcarCtrl[3].data[1] = ADAS_163h_1.DATA01.D & 0xFF;
  JMEVmsg_ViewcarCtrl[3].data[2] = ADAS_163h_1.DATA2345.D >> 24 & 0xFF;
  JMEVmsg_ViewcarCtrl[3].data[3] = ADAS_163h_1.DATA2345.D >> 16 & 0xFF;
  JMEVmsg_ViewcarCtrl[3].data[4] = ADAS_163h_1.DATA2345.D >> 8 & 0xFF;
  JMEVmsg_ViewcarCtrl[3].data[5] = ADAS_163h_1.DATA2345.D & 0xFF;
  JMEVmsg_ViewcarCtrl[3].data[6] = ADAS_163h_1.DATA6.D & 0xFF;
  ADAS_163h_CheckSUM = ((JMEVmsg_ViewcarCtrl[3].data[0] +
                         JMEVmsg_ViewcarCtrl[3].data[1] + JMEVmsg_ViewcarCtrl[3].data[2] +
                         JMEVmsg_ViewcarCtrl[3].data[3] + JMEVmsg_ViewcarCtrl[3].data[4] +
                         JMEVmsg_ViewcarCtrl[3].data[5] + JMEVmsg_ViewcarCtrl[3].data[6]) ^
                        (0xFF));
  ADAS_163h_1.DATA7.bit.ADAS_163h_CheckSUM = ADAS_163h_CheckSUM;
  JMEVmsg_ViewcarCtrl[3].data[7] = ADAS_163h_1.DATA7.D & 0xFF;

  if (APA_LCControl2_AliveCounter > 15)
  {
    APA_LCControl2_AliveCounter = 0;
  }
  APA_170h_1.DATA6.bit.APA_LCControl2_AliveCounter = APA_LCControl2_AliveCounter;
  APA_LCControl2_AliveCounter = APA_LCControl2_AliveCounter + 1;


  JMEVmsg_ViewcarCtrl[4].can_id = APA_170h_1.MsgID.D;
  JMEVmsg_ViewcarCtrl[4].can_dlc = APA_170h_1.MsgLengh.D;
  JMEVmsg_ViewcarCtrl[4].data[0] =  APA_170h_1.DATA0.D & 0xFF;
  JMEVmsg_ViewcarCtrl[4].data[1] = APA_170h_1.DATA1.D & 0xFF;
  JMEVmsg_ViewcarCtrl[4].data[2] = APA_170h_1.DATA2.D & 0xFF;
  JMEVmsg_ViewcarCtrl[4].data[3] =APA_170h_1.DATA34.D >> 8 & 0xFF;
  JMEVmsg_ViewcarCtrl[4].data[4] = APA_170h_1.DATA34.D & 0xFF;
  JMEVmsg_ViewcarCtrl[4].data[5] = APA_170h_1.DATA5.D & 0xFF;
  JMEVmsg_ViewcarCtrl[4].data[6] =APA_170h_1.DATA6.D & 0xFF;
  APA_LCControl2_CheckSum = ((JMEVmsg_ViewcarCtrl[4].data[0] +
                         JMEVmsg_ViewcarCtrl[4].data[1] + JMEVmsg_ViewcarCtrl[4].data[2] +
                         JMEVmsg_ViewcarCtrl[4].data[3] + JMEVmsg_ViewcarCtrl[4].data[4] +
                         JMEVmsg_ViewcarCtrl[4].data[5] + JMEVmsg_ViewcarCtrl[4].data[6]) ^
                        (0xFF));
  APA_170h_1.DATA7.bit.APA_LCControl2_CheckSum = APA_LCControl2_CheckSum;
  JMEVmsg_ViewcarCtrl[4].data[7] = APA_170h_1.DATA7.D & 0xFF;
}

/******************************************************************
*File Name   : VehiclePosInit
*Date        : 2020/04/26
*  Version     : 1.0.1
*Desciption  : VehiclePosInit
***************************************/
void VehiclePosInit()
{
  //real_T V_VehPosXdou;
  //real_T V_VehPosYdou;
  real_T Vehicle_Dis1 = 0, Vehicle_Dis2 = 0, Vehicle_Dis3 = 0;
  int i = 0;
  C_IniRefPointu16 = 1;

  //RCLCPP_INFO(rclcpp::get_logger("control_node"), "V_VehPosXdou: [%f] V_VehPosYdou: [%f]", V_VehPosXdou,V_VehPosYdou);
  Vehicle_Dis1 = (C_RoadTraj1Xtable_f32s23[0] - V_VehPosXdou) * (C_RoadTraj1Xtable_f32s23[0] - V_VehPosXdou) + (C_RoadTraj1Ytable_f32s23[0] - V_VehPosYdou) * (C_RoadTraj1Ytable_f32s23[0] - V_VehPosYdou);
  /*
  for(i=1;i<2585;i++)
  {
    if((abs(C_RoadTraj1Xtable_f32s23[i]-V_VehPosXdou)<10)&&(abs(C_RoadTraj1Ytable_f32s23[i]-V_VehPosYdou)<10))
    {
      //Vehicle_Dis1 = (C_RoadTraj1Xtable_f32s23[i-1]-V_VehPosXdou)*(C_RoadTraj1Xtable_f32s23[i-1]-V_VehPosXdou)+(C_RoadTraj1Ytable_f32s23[i-1]-V_VehPosYdou)*(C_RoadTraj1Ytable_f32s23[i-1]-V_VehPosYdou);
      Vehicle_Dis2 = (C_RoadTraj1Xtable_f32s23[i]-V_VehPosXdou)*(C_RoadTraj1Xtable_f32s23[i]-V_VehPosXdou)+(C_RoadTraj1Ytable_f32s23[i]-V_VehPosYdou)*(C_RoadTraj1Ytable_f32s23[i]-V_VehPosYdou);
      //Vehicle_Dis3 = (C_RoadTraj1Xtable_f32s23[i+1]-V_VehPosXdou)*(C_RoadTraj1Xtable_f32s23[i+1]-V_VehPosXdou)+(C_RoadTraj1Ytable_f32s23[i+1]-V_VehPosYdou)*(C_RoadTraj1Ytable_f32s23[i+1]-V_VehPosYdou);
      
      //RCLCPP_INFO(rclcpp::get_logger("control_node"), "Vehicle_Dis2: [%f]", Vehicle_Dis2);
      if(Vehicle_Dis2<Vehicle_Dis1)
    {
      Vehicle_Dis1 = Vehicle_Dis2;
      C_IniRefPointu16 = i;
    }    
    
    }
  }
*/
}
