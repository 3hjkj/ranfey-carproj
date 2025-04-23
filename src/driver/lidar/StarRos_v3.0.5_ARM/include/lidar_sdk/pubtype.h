#ifndef __pubtype__
#define  __pubtype__

#include <vector>
#include <map>
#include <algorithm>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include<stdio.h>
//#include<memory.h>
using namespace std;

#define PACKETSIZE 0X40000
#define BUFFSIZE (0X40000*5)
#define SLOW_VEC_SIZE 500
#define PPS_STAMP_DEFAULT (0x80000000) //max T0,2^31, 2147483648
#define PPS_DIFF_DEFAULT  (5000000)
#define UTC_STAMP_DEFAULT (0.0)
#define T0_STEP (0x80000000) // 2^31, 2147483648
const int HEADSIZE = 81920;

#ifndef __dllexport
#if defined(_MSC_VER)
#define __dllexport __declspec(dllexport)
#define __dllimport __declspec(dllimport)
#define __dllhidden
#elif defined(__MINGW32__) || defined(__SYGWIN__)
#define __dllexport __attribute__((dllexport))
#define __dllimport __attribute__((dllimport))
#define __dllhidden __attribute__((visibility("hidden")))
#else
#define __dllexport
#define __dllimport
#define __dllhidden
#endif
#endif

typedef vector<int> INTV;
typedef vector<double> DBV;
typedef vector<DBV> DBVV;
typedef unsigned char BYTE;
typedef vector <BYTE> BYTEV;
typedef map<int, vector<float>> cfgDATA;

#pragma pack(1)
//每个回波的信息
typedef struct {
	unsigned short wYear;
	unsigned short wMonth;
	unsigned short wDayOfWeek;
	unsigned short wDay;
	unsigned short wHour;
	unsigned short wMinute;
	unsigned short wSecond;
	unsigned short wMilliseconds;
}CSYSTEMTIME, *PCSYSTEMTIME, LPCSYSTEMTIME;


typedef unsigned long  long         uint_64;
const int DATA_POINT_COUNT = 0x10000; //激光点个数为64K

//CRS 轮廓断面数据头结构
typedef struct _CRSHeader {
	unsigned int   packFlag;    //!< 数据包标识: 0xE7E7E7E7
	unsigned short versions;    //!< 版本号
	unsigned short devNum;      //!< 设备编号
	unsigned int   serialNum;   //!< 序列号
	unsigned char  devStaFlag;  //!< 设备状态标识 0正常；非零异常 ； 11 | 11 | 11(激光器状态) | 11(扫描电机状态)
	unsigned int   scanFrq;     //!< 扫描频率   单位 RPS(转/秒)
	unsigned int   pointFrq;    //!< 点频       单位 Hz(个/秒)
	unsigned int   angleRes;    //!< 角度分辨率 单位 千分之一度
	uint_64   pulseNum;    //!< DMI脉冲计数
	unsigned int   pointsCount; //!< 一个轮廓断面总点数
	CSYSTEMTIME svrSysTime;     //!< 断面采集时间，零位时间。
}CRS_HEADER_S;

//CRS 点数据结构
typedef struct _CRSPoint {
	//unsigned int Tstamp ;
	float angle;               //!< 扫描角度
	unsigned short range;      //!< 测距 单位mm
	unsigned short intension;  //!< 反射强度:
	_CRSPoint() {
		angle = 0;
		range = 0;
		intension = 0;
	}
}CRS_POINT_S;

typedef struct {
	CRS_HEADER_S dataHead;
	CRS_POINT_S dataPoints[DATA_POINT_COUNT];
}CRS_LINE_S;


struct FilePath_S {
	string fileForderPath;
	string lidarPath;	     //原始激光点数据
	string cfgPath;
	string rawPosPath;
	string posPath;    //轨迹路径和命名
	string lasPath;           //输出las文件路径
	string posCfgPath;
};

struct IMU_S
{
	float time;
	float accel_x;
	float accel_y;
	float accel_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float roll;
	float pitch;
	float yaw;
	IMU_S()
	{
		time=0.0;
		accel_x=0.0;
		accel_y=0.0;
		accel_z=0.0;
		gyro_x=0.0;
		gyro_y=0.0;
		gyro_z=0.0;
		roll=0.0;
		pitch=0.0;
		yaw=0.0;
	}
};

//IMU DATA
typedef struct tagPRE_IMULCI100_S {
	double utcTime;
	double xAng;
	double yAng;
	double zAng;
	double xSpd;
	double ySpd;
	double zSpd;
	tagPRE_IMULCI100_S()
	{
		double utcTime = 0.0;
		double xAng = 0.0;
		double yAng = 0.0;
		double zAng = 0.0;
		double xSpd = 0.0;
		double ySpd = 0.0;
		double zSpd = 0.0;
	}
}PRE_IMULCI100_S;

//GPS_data
const uint32_t GPS_DATA_SIZE = 48;
typedef struct {
	uint16_t dataId : 16;//8
	uint8_t gpsData[GPS_DATA_SIZE];
} SCD_GPS_S;

//GPS_IMU_STREAM
struct slowStream
{

	using imuStream = vector<PRE_IMULCI100_S>;//imu
	using gpsStream = vector<SCD_GPS_S>;
	imuStream imu_stream_out;
	gpsStream gps_stream_out;
	slowStream()
	{
		imu_stream_out.reserve(1000);
		imu_stream_out.reserve(1000);

	}

};

#pragma pack()


struct OPTION_DECODER_S
{
	bool GPS;
	bool ISS;

};



#pragma pack()

#endif // __PUBTYPE__
