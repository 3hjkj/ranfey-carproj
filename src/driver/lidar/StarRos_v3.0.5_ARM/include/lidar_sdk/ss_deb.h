#ifndef SS_DEB_H
#define SS_DEB_H
#include <string>
#include"compile.h"

#define DEB_FRAME_WRITE_SS (0xA5)  //write head sync
#define DEB_FRAME_READ_SS  (0x5a)  //read head sync
#define DEB_FRAME_ERROR_SS  (0xE7) //err  haad sync
#define UDP_MIN_SIZE_CTR_CMD 18
#define UPGRADE_BEGIN           (0x06070809)
#define UPGRADE_SUCCESS HEARTBEAT_FLAG  //最初是ABABABAB，后调整为心跳包标识
#define SYNC_ANSWER_ITV         (1000)  //回应09 08 07 06 的时间间隔
#define SYNC_ERASE_ITV          (2000)  //同步结束后到第一条擦除指令的时间间隔
#define ERASE_ITV               (1000)  //擦除命令的时间间隔
#define ERASE_TO_BURN_ITV       (1000)  //擦除到写命令之间的时间间隔
#define BURN_DEB_ITV            (10)    //写DEB命令的时间间隔
#define BURN_MTDC_ITV           (30)    //写MTDC命令的时间间隔
#define ERASE_ADDR_START        (0x10)  //擦除的起始地址
#define ERASE_ADDR_END          (0x2F)  //擦除的终止地址
#define DEB_ADDR_START          (0x10)  //写DEB的起始地址
#define DEB_ADDR_END            (0x2F)  //写DEB的终止地址
#define MTDC_ADDR_START         (0x10)  //写MTDC的起始地址
#define MTDC_ADDR_END           (0x2F)  //写MTDC的终止地址
#define HANDSHAKE_ENABLE        (1)     //是否使能握手机制
#define HANDSHAKE_TIMEOUT       (600)   //单次握手超时时间
#define HANDSHAKE_MAX_TIMES     (5)     //单次命令最大握手次数
#define HANDSHAKE_FAILED_CNTU   (0)     //握手失败是否继续擦除再继续升级
#define DEB_FRAME_WRITE_SS (0xA5)  //write head sync
#define DEB_FRAME_READ_SS  (0x5a)  //read head sync
#define DEB_FRAME_ERROR_SS  (0xE7) //err  haad sync



#define UDP_MIN_SIZE_CTR_CMD 18
#define WRITE_DATA_SZIE (256)
#define FRAME_MSG_LENGTH (1024*32)//32K
const static int SS_ANGLE_SPEED_5HZ = 5;
const static int SS_ANGLE_SPEED_10HZ = 10;
const static int SS_ANGLE_SPEED_20HZ = 20;

const static int SS_ANGLE_SPEED_0HZ_CFAN = 0;
const static int SS_ANGLE_SPEED_10HZ_CFAN = 10;
const static int SS_ANGLE_SPEED_20HZ_CFAN = 20;
const static int SS_ANGLE_SPEED_40HZ_CFAN = 40;
const static int SS_ANGLE_SPEED_60HZ_CFAN = 60;
const static int SS_ANGLE_SPEED_80HZ_CFAN = 80;

const static int SS_CMD_SCAN_SPEED_5HZ = 0;
const static int SS_CMD_SCAN_SPEED_10HZ = 0x50;
const static int SS_CMD_SCAN_SPEED_20HZ = 0xF0;


const static int SS_CMD_SCAN_SPEED_10HZ_C = 0x00;
const static int SS_CMD_SCAN_SPEED_20HZ_C = 0x20;
const static int SS_CMD_SCAN_SPEED_40HZ_C = 0x60;
const static int SS_CMD_SCAN_SPEED_60HZ_C = 0xA0;
const static int SS_CMD_SCAN_SPEED_80HZ_C = 0xE0;
const static int SS_CMD_SCAN_ENABLE = 0x3;
const static int SS_CMD_LASER_ENABLE = 0x2;

const static int SS_CMD_RCV_CLOSE = 0x0;
const static int SS_ANGLE_SPEED_0HZ_cfan = 0;
const static int SS_ANGLE_SPEED_10HZ_cfan = 10;
const static int SS_ANGLE_SPEED_20HZ_cfans = 20;
const static int SS_ANGLE_SPEED_40HZ_cfans = 40;
const static int SS_ANGLE_SPEED_60HZ_cfans = 60;
const static int SS_ANGLE_SPEED_80HZ_cfans = 80;
const static int ROMREG_MAX_COUNT = 0x7FF;
const static int REG_DEVICE_CTRL = (0x2040);
const int HEAD_A_MTDC = 0xA6;
const int HEAD_A_DEB = 0xA5;

//const int MTDC_HEAD_A = 0x6A;
//const int DEB_HEAD_A = 0x5A;
const int HEAD_B_WRITE = 0x02;
const int READ_HEAD_B = 0x03;
const int HEAD_B_CLEAR = 0xD8;
const int FLASH_BASE_ADDRESS = 0x100000;
const int FLASH_ADDRESS_OFFSET = 0x100;
const unsigned char FLASH_BASE_ADDRESS_H = 0x10;
const unsigned char FLASH_MAX_ADDRESS_H = 0x2F;
const unsigned char FLASH_MAX_ADDRESS_SURVEY = 0x4F;

// ================ upgrade =================
const unsigned char DEB_START[] = { 0xff, 0xee, 0xdd, 0xcc, 0xbb, 0xaa };
const unsigned char DEB_STOP[] = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xab };
const unsigned char MTDC_START[] = { 0x55, 0xbb, 0xcc, 0xdd, 0xee, 0x55 };
const unsigned char MTDC_STOP[] = { 0x43, 0x23, 0x45, 0x67, 0x89, 0x43 };
const unsigned char MTDC_SVY_FILE_INFO[] = { 0x49, 0x4e, 0x46, 0x4f }; //INFO


const unsigned int HEARTBEAT_FLAG = 0xE4E3E2E1;
// survey checksum error cmd: 46 41 49 4C A6 03 10 00 00 00 01 00; 100000为地址，000100为读取包大小
const unsigned int PKT_SURVEY_UPGRADE_FAILED = 0x4C494146; //FAIL 0x4641494C
// survey progress, 4bytes Header + 1bytes type + 1byte length + length byte value
//                  50 52 4F 47 00 03 00 00 01    1%
const unsigned int PKT_SURVEY_UPGRADE_PROGRESS = 0x474F5250; // PROG 0x50524F47 测绘升级程序通知升级写数据的真实进度
struct CMD_HEADER_S{
	unsigned char headA;
	unsigned char headB;
	unsigned char addrA;
	unsigned char addrB;
	unsigned char addrC;
} ;

struct CMD_WRITE_S
{
	CMD_HEADER_S head;
	char data[WRITE_DATA_SZIE];
} ;

#pragma pack(1)
typedef struct
{
	unsigned char msgHead;
	unsigned char msgCheckSum;
	unsigned short regAddress;
	unsigned int regData;
}SS_DEB_FRAME;
#pragma pack()
typedef enum {
	eFormatCalcData = 0x2,
	eFormatDebugData = 0x5,
}SS_DEB_DFORMAT;


typedef enum {
	eCmdWrite,  //write command
	eCmdRead,   //read command
	eCmdQuery,  //Query command
} SCD_TYPE;


typedef enum {
	eDevCmdIdle_ = 0,
	eDevCmdWork_,//1
} SS_DEB_CMD_E;


typedef struct {
	SS_DEB_CMD_E cmdstat;
	int scnSpeed;
	int dataLevel;
	int lsrFreq;
	float rangeMin, rangeMax;
}SS_DEB_PROGRM;


typedef enum {
	DEVICE_NAVIGATION,
	DEVICE_SURVEY
} DEVICE_E_;

typedef enum {
	FPGA_DEB,
	FPGA_MTDC
} FPGA_E;



struct CfgPara_S
{
	int sync_answer_itv;
	int sync_erase_itv;
	int erase_itv;
	int erase_to_burn_itv;
	int burn_deb_itv;
	int burn_mtdc_itv;
	int erase_addr_start;
	int erase_addr_end;
	int deb_addr_start;
	int deb_addr_end;
	int mtdc_addr_start;
	int mtdc_addr_end;
	int handshake_enable;
	int handshake_timeout;
	int handshake_max_times;
	int handshake_failed_continue;
	CfgPara_S()
	{
		sync_answer_itv = SYNC_ANSWER_ITV;
		sync_erase_itv = SYNC_ERASE_ITV;
		erase_itv = ERASE_ITV;
		erase_to_burn_itv = ERASE_TO_BURN_ITV;
		burn_deb_itv = BURN_DEB_ITV;
		burn_mtdc_itv = BURN_MTDC_ITV;
		erase_addr_start = ERASE_ADDR_START;
		erase_addr_end = ERASE_ADDR_END;
		deb_addr_start = DEB_ADDR_START;
		deb_addr_end = DEB_ADDR_END;
		mtdc_addr_start = MTDC_ADDR_START;
		mtdc_addr_end = MTDC_ADDR_END;
		handshake_enable = HANDSHAKE_ENABLE;
		handshake_timeout = HANDSHAKE_TIMEOUT;
		handshake_max_times = HANDSHAKE_MAX_TIMES;
		handshake_failed_continue = HANDSHAKE_FAILED_CNTU;	
	}
};

 struct BasePara_S 
 {
	 DEVICE_E_ device_type;
	FPGA_E fpga_type;
	string bin_filename;
	string ip;
	uint16_t port;
	BasePara_S() {
		device_type = DEVICE_NAVIGATION;
		fpga_type = FPGA_DEB;
		bin_filename = "";
		ip = "192.168.0.3";
		port = 2030;
	}
} ;

 enum UPGRADE_STATUS {
	 UPGRADE_NONE,       //
	 UPGRADE_DETECTING,  //upgrade_detecting
	 UPGRADE_DETECTED,
	 UPGRADE_MTDC_ACTION,
	 UPGRADE_ERASE_START,
	 UPGRADE_ERASING,    //
	 UPGRADE_ERASED,
	 UPGRADE_FIRING,     //
	 UPGRADE_FINISHING,  //
	 UPGRADE_FINISH      //
 } ;

 struct UpgradeCmd_S {
	 unsigned char cmd[512];
	 int itv;               //ms
	 int cmd_size;
	 UPGRADE_STATUS status;

	 UpgradeCmd_S() {
		 memset(cmd, 0, sizeof(cmd));
		 itv = 1000;//1s
		 cmd_size = 256;
		 status = UPGRADE_NONE;
	 }
 };

 typedef struct _frams_buffer {
	 char msgStream[FRAME_MSG_LENGTH];
	 int writeIdx;
	 int readIdx;
	 int length;
 } FRAMS_BUFFER_S;

 //typedef struct {
	// unsigned int regAddr;          //!< ?????????????????
	//// float regValue;         //!< ????????????????????
	// unsigned int regValue;
	// //int reserved[13];             //!<total 60 Byte
 //}DEB_REGIST_S;


#pragma pack(1)
 typedef struct  //256 Byte
 {
	 unsigned int pkgflag;
	 unsigned int pkgnumber;
	 unsigned char	gps_time[6];
	 unsigned int maca;
	 unsigned short macb;
	 unsigned short dataport;
	 unsigned short msgport;
	 unsigned char motorspd;
	 unsigned int deviceType;
	 unsigned short phaseAngle;
	 //according new heartbeat package
	 unsigned char pack_format;
	 unsigned char device_id;
	 short temperature;
	 unsigned int err_chksum;
	 unsigned int point_freq;
	 unsigned int dev_status;
	 unsigned int low_sn;
	 unsigned int high_sn;
	 unsigned char padding[201];
 }HEARTBEAT_S;
#pragma pack()

#endif // SSSOCKET_H
