/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

#ifndef _FRAME_LIB_H_
#define _FRAME_LIB_H_
#include <iostream>
#include <string>
#include "rfans_driver/msg/rfans_packet.hpp"
#include <rfans_driver/point_types.h>
using namespace std;

const static int REG_DEVICE_CTRL = (0x2040);
const static int REG_DEVICE_CTRL_OLD = (0x0040);
const static int REG_DATA_LEVEL = 0x2008;
const static int REG_DATA_LEVEL_OLD = 0x0008;

const static int CMD_SCAN_SPEED_5HZ = 0;
const static int CMD_SCAN_SPEED_10HZ = 0x60;//转速10hz
const static int CMD_SCAN_SPEED_20HZ = 0xF0;


const static int CMD_SCAN_SPEED_10HZ_C = 0x00;
const static int CMD_SCAN_SPEED_20HZ_C = 0x20;
const static int CMD_SCAN_SPEED_40HZ_C = 0x60;
const static int CMD_SCAN_SPEED_60HZ_C = 0xA0;
const static int CMD_SCAN_SPEED_80HZ_C = 0xE0;


const static int CFANS_ECHO = 0x36;//cfans 0x36
const static int CFANS_DUAL = 0x39;//cfans 0x36

const static int CMD_SCAN_ENABLE = 0x3;
const static int CMD_LASER_ENABLE = 0x2;
const static int CMD_CALC_DATA = 0x2;
const static int CMD_DEBUG_DATA = 0x5;
const static int CMD_RCV_CLOSE = 0x0;
const static int CMD_LEVEL0_ECHO = 0x00000000 ;
const static int CMD_LEVLE0_DUAL_ECHO = 0x00000002;
const static int CMD_LEVEL1_ECHO = 0x01000200;
const static int CMD_LEVEL1_DUAL_ECHO = 0x01000202;
const static int CMD_LEVEL2_ECHO = 0x0201BF00;
const static int CMD_LEVEL2_DUAL_ECHO = 0x0201BF02;
const static int CMD_LEVEL3_ECHO = 0x03000100;
const static int CMD_LEVEL3_DUAL_ECHO = 0x03000102;



const static int ANGLE_SPEED_5HZ = 5;
const static int ANGLE_SPEED_10HZ = 10;
const static int ANGLE_SPEED_20HZ = 20;

const static int ANGLE_SPEED_0HZ_cfan = 0;
const static int ANGLE_SPEED_10HZ_cfan = 10;
const static int ANGLE_SPEED_20HZ_cfans = 20;
const static int ANGLE_SPEED_40HZ_cfans = 40;
const static int ANGLE_SPEED_60HZ_cfans = 60;
const static int ANGLE_SPEED_80HZ_cfans = 80;



static unsigned short DEVICE_PORT_NUMBER = 2015;
static unsigned short PC_PORT_NUMBER = 2015;
static std::string DEVICE_IP_STRING = "192.168.0.3";

static unsigned short UDP_FRAME_MIN =(18);

#define DEB_FRAME_WRITE (0xA5)  //write head sync
#define DEB_FRAME_READ  (0x5a)  //read head sync
#define DEB_FRAME_ERROR  (0xE7) //err  haad sync

#define FRAME_MSG_LENGTH (1024) 

const int UDPREG_MAX_COUNT = 256;
const int ROMREG_MAX_COUNT = 0x7FF;
namespace lidarAPi{
typedef enum{
  eCmdWrite,  //write command
  eCmdRead,   //read command
  eCmdQuery,  //Query command
} SCD_FRAME_TYPE_E ;

typedef struct _frams_buffer{
  char msgStream[FRAME_MSG_LENGTH];
  int writeIdx;
  int readIdx;
  int length;
} FRAMS_BUFFER_S;

#pragma pack(1)
typedef struct {                 //! rfans udp command package
  unsigned char msgHead;         //!< head sync
  unsigned char msgCheckSum;     //!< check sum
  unsigned short regAddress;     //!< register add
  unsigned int regData;          //!< register data
} DEB_FRAME_S;

typedef enum {
  eDevCmdIdle = 0,
  eDevCmdWork,//1
  eDevCmdSimu,
  eDevCmdBreak,
  eDevCmdReset,
  eDevCmdAsk,
} DEB_CMD_E;

typedef enum {
  eFormatCalcData = 0x2,
  eFormatDebugData = 0x5,
}DEB_DFORMAT_E;


static int DEVICE_MOTOR_HZ = 5;
typedef struct {
  DEB_CMD_E cmdstat;
  DEB_DFORMAT_E dataFormat;
  int scnSpeed;
  int dataLevel;
  int lsrFreq;
  float rangeMin, rangeMax;
}DEB_PROGRM_S;

static const size_t upd_packet_size = 0x8000;  //32KB
const unsigned char ID_RFANSBLOCKV2_SYNC = 0x96;
const unsigned char ID_RFANSBLOCKV32_0_15_SYNC = 0x97;
const unsigned char ID_RFANSBLOCKV32_16_31_SYNC = 0x98;

const unsigned char ID_RFANSBLOCKV6G_0_15_SYNC = 0x99;
const unsigned char ID_RFANSBLOCKV6G_16_31_SYNC = 0x9A;

const unsigned char ID_RFANSBLOCKV32_GM_0_15_SYNC = 0x87;
const unsigned char ID_RFANSBLOCKV32_GM_16_31_SYNC = 0x88;

const unsigned char ID_RFANSBLOCK_GM_SYNC = 0xee;
const int UDP_PACKET_SIZE_V5A = 1380;
const int UDP_PACKET_SIZE_V6G = 1206;
const int UDP_PACKET_SIZE_DATA_LEVEL_ORI=1406;

typedef struct {
  unsigned short angle  : 16 ;          //scan angle, 0.01��
  unsigned short rangeOne  : 16 ;       //echo 1,  cm
  unsigned short rangeTwo  : 16 ;       //echo 2,  cm
  unsigned char  intentOne : 8 ;        //0~255
  unsigned char  intentTwo : 8 ;        //0~255
}SCDRFANS_POINT_S;

const int RFANS_LASER_COUNT = 16 ;
typedef struct {                                        //138 byte
  unsigned char           dataID : 8;                   //Sync Number
  unsigned char           chksum : 8;                   //chksum  Bytes[3,138]
  unsigned int            t0stampH  : 32;               //T0 Bloceek
  unsigned int            t0stampL  : 32;               //T0 tag
  SCDRFANS_POINT_S        laserData[RFANS_LASER_COUNT] ;//
}SCDRFANS_BLOCK_S;

static const int DECODE_BUFFER_SIZE = 0x40000; // 256KB

static const int UDP_PACKET_SIZE = 2*1024; // 256KB
static const int UDP_PACKET_BUFFER_SIZE = 16*1024; // 256KB
typedef struct  {
  int packetsize ;
  unsigned char buffer[UDP_PACKET_SIZE];
}UDP_PACKET_S;

typedef struct {
  int wrHead, rdTail, bufSize;
  UDP_PACKET_S buffer[UDP_PACKET_BUFFER_SIZE];
} UDP_PACKET_BUFFER_S;


typedef struct {
  int wrHead, rdTail, bufSize;
  unsigned char buffer[DECODE_BUFFER_SIZE];
} UDP_DECBUFFER_S;

typedef struct
{
  unsigned short range;
  unsigned char intensity;
}RFans_Laser32Block_S;

const unsigned short RFANS_UDPFRAMV6G_FLAT = 0xFFEE;
const unsigned short LASER32BLOCK_COUNT = 32;
typedef struct
{
  unsigned short flag; 
  unsigned short azimuthAngle;
  RFans_Laser32Block_S laserBlock[32];
}RFans_DataBlock_S;
const unsigned short UDP32FRAMEV6G_COUNT = 12;
const unsigned short RFANS_GM_16_FLAG = 0x3732;
const unsigned short RFANS_V6_GM_33_FLAG = 0x3733;


typedef struct
{
  SCDRFANS_BLOCK_S blockdata[10];
}RFans_UDPFRAMEV5_S;

typedef struct
{
  RFans_DataBlock_S dataBlock[UDP32FRAMEV6G_COUNT];
  unsigned int gpsTimestamp;
  unsigned char gmReservedA;
  unsigned char gmReservedB;
}RFans_UDP32FRAMEV6G_S;


typedef enum{
    LEVEL0_ECHO=0,
    LEVEL0_DUAL_ECHO,
    LEVEL1_ECHO,
    LEVEL1_DUAL_ECHO,
    LEVEL2_ECHO,
    LEVEL2_DUAL_ECHO,
    LEVEL3_ECHO,
    LEVEL3_DUAL_ECHO,

}MULTI_LEVEL_DATA;


typedef enum {
    DATA_LEVEL_ORI = 0,     
    DATA_LEVEL_CALIB,       
    DATA_LEVEL_USER,        
    DATA_LEVEL_USER_SIMPLE, 
} DATA_LEVEL_E;

// algorithm flag
typedef enum {
    ALGORITHM_ID_PREPROCESS = 0x00, //Ԥ����ģ��
    ALGORITHM_ID_DISTANCE = 0x10,   //����궨
    ALGORITHM_ID_INTENSITY = 0x10,  //�Ҷȱ궨
    ALGORITHM_ID_ANGLE = 0x20,      //�Ƕȱ궨
    ALGORITHM_ID_STRETCHing = 0x30, //�Ҷ�����
} ALGORITHM_ID_E;


// mirror id

typedef enum {
    MIRROR_ID_CFANS_00 = 0x0,   //CFANS ...
    MIRROR_ID_CFANS_01 = 0x1,
    MIRROR_ID_CFANS_02 = 0x2,
    MIRROR_ID_RFANS = 0x3,      //RFANS�����ʶ
} MIRROR_ID_E;


// data packet id
typedef enum {
    PACK_ID_DUALECHO_MIX    = 0x00,   //˫�ز�������
    PACK_ID_DUALECHO        = 0x01,   //˫�ز����
    PACK_ID_STRONG_ECHO     = 0x02,   //��ǿ�ز����
    PACK_ID_FIRST_ECHO      = 0x03,   //��һ�ز����
} PACK_ID_E;
}


#pragma pack()

#ifdef __cplusplus
extern "C"
{
#endif
int swapchar( unsigned char * _data, int size_ ) ;
int checkSum(unsigned char * _dataBuf, int count_ ) ;

lidarAPi::DEB_FRAME_S packDEBV3Frame(lidarAPi::SCD_FRAME_TYPE_E flag, int regAddress_, int regData_);

void writeFrameBuffer(lidarAPi::FRAMS_BUFFER_S *mtFrameMsgBuf, char * _mt_frame, int mt_size);

void readDEBFrameBuffer(lidarAPi::FRAMS_BUFFER_S *mtFrameMsgBuf, lidarAPi::DEB_FRAME_S *mtRegMap);

//int  searchUDPPacket(UDP_DECBUFFER_S *mtUdpBuffer, BLOCK_VECTOR_S *outBlocks);


#ifdef __cplusplus
}
#endif


#endif
