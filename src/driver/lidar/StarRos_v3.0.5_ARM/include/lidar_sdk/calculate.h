#ifndef __CALCULATE__
#define  __CALCULATE__

#include <deque>
#include <vector>
#include <map>
#include <algorithm>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include<stdio.h>
#include"compile.h"
using namespace std;

const double T0_Clock_Freq = 5000000;         //测绘 ：点频 =5000000/时间戳差值
const double T0_Clock_Freq_nav = 1000000;    //导航 ：点频 =1000000/时间戳差值    16线点频300k  导航32线点频576k

#ifndef PI 
#define PI 3.1415926535897932384626433832
#endif

#ifndef D2RAD 
#define D2RAD 0.017453292520 
#endif

#ifndef RAD2D 
#define RAD2D 57.295779513082
#endif

//#define D2RAD  (PI/180.0) 
//#define RAD2D (180.0/PI)

#define ZERONR 1e-5
#define ZEROF 1e-7
#define ZEROD 1.0e-12


#define TURN_ANGEL_DIFF_DEFAULT  (-999)  // 5M  //1秒钟的T0 Count值
#define TURN_ANGEL_DEFAULT (0.0)
#define PPS_STAMP_DEFAULT (0x80000000) //max T0,2^31, 2147483648
#define PPS_DIFF_DEFAULT  (5000000)  // 5M  //1秒钟的T0 Count值
#define UTC_STAMP_DEFAULT (0.0)
#define T0_STEP (0x80000000) // 2^31, 2147483648
#define UTC_LOSEUPDATA_VALUE (30)  //30s
#define ECHO_NUM 4
//元数据中   metaHeader
//const double T0_Clock_Freq = 5000000;         //测绘 ：点频 =5000000/时间戳差值
//const double T0_Clock_Freq_nav = 1000000;    //导航 ：点频 =1000000/时间戳差值    16线点频300k  导航32线点频576k   导航32线点频450k 


#define TEMPER_MIN 0
#define TEMPER_MAX 99
#define TEMPER_NUM 100 

//一字节对齐
#pragma pack(1)

//算法配置
typedef struct algConfig {
	unsigned int algId;
	char name[32];
	char descripts[256];
	char path[256]; // for self defined algorithms
	union {
		struct {
			int iConf[8];
			float fConf[8];
			char strConf[64];
		} _exam;
		char strRsv[128];
		int intRsv[32];
		float floatRsv[32]; // each float has 4 bytes
		int paramset; // 0 means default
	};
	algConfig()
	{
		algId = 0;
		memset(name, '\0', 32);
		memset(descripts, '\0', 256);
		memset(path, '\0', 256);
		memset(_exam.strConf, '\0', 64);
		memset(strRsv, '\0', 128);
	}
} algConfig_S;



////PPS时间对
//typedef struct tag_PosPPS_S {
//	uint32_t ppsStamp;
//	int32_t diffPpsStamp;
//	double utcStamp;
//	tag_PosPPS_S()
//	{
//		ppsStamp = PPS_STAMP_DEFAULT;
//		diffPpsStamp = PPS_DIFF_DEFAULT;
//		utcStamp = UTC_STAMP_DEFAULT;
//	}
//}PosPPS_S;
//typedef vector<PosPPS_S> PPS_V;

//指北针
typedef struct narrow_S {
	double narrowAngle;
	double turnAngle;
	narrow_S() {
		narrowAngle = 0;
		turnAngle = 0;
	}
}Narrow_S;
//轨迹
typedef struct SBET_Rec_tag { //3*5*8+2*8 =136
	double ti;			///< 时间，单位:周秒(time gps-sec. of week)
	double p[3];		///< 纬度 经度 高度 24
	double wVel[3];		///< x方向速度 y方向速度 Z方向速度 24
	double rph[3];		///< roll, pitch, heading ,单位:弧度 24
	double w;			///< 游移方位角
	double force[3];		///< x方向加速度 y方向加速度 Z方向加速度 24
	double aRate[3];		///< x方向角速度 y方向角速度 Z方向角速度 24
}SBET_Rec;
typedef vector<SBET_Rec> POS_V;
typedef deque<SBET_Rec> POS_D;

//实时轨迹
//#define POS_RINGBUF_FULL -2
//#define INTER_POS_FAILED -1

enum INTER_STATE {
	INTER_SUCSESS=0,
	INTER_POS_FAILED=1,
	POS_RINGBUF_FULL=2,
};

typedef struct SBET_SIMPLE {
	double ti;			///< 时间，单位:周秒(time gps-sec. of week)
	double p[3];		///< 纬度 经度 高度 24	
	double rph[3];		///< roll, pitch, heading ,单位:弧度 24	
}SBET_SIMPLE_S;
typedef vector<SBET_SIMPLE_S> POS_SIMPLE_V;


// 惯导更新频率（200 400 1000hz）
//若转速3000r/min，则一帧的时间为60/3000 =0.02s ,若惯导更新频率为1000hz，则一帧更新0.02*1000=20个pos数据
const int POS_BUFFER_NUM = 20;      //解算模块中POS_REALTIME_S中pos数据的缓存个数
const int POS_UPDATE_NUM = 5;   

typedef struct {
	SBET_Rec PosRing[POS_BUFFER_NUM];
	int wrHead;
	int rdTail;
}POS_REALTIME_S;


typedef struct tag_PlatForm_S {
	uint32_t ppsStamp;
	float angle;
	tag_PlatForm_S()
	{
		ppsStamp = PPS_STAMP_DEFAULT;
		angle = 0;
	}
}PlatForm_S;
typedef vector<PlatForm_S> platFormV;

typedef struct point3d {
	double x;
	double y;
	double z;
	point3d()
	{
		x = 0;
		y = 0;
		z = 0;
	}
}Point3d;

//解算回波信息
typedef struct CacPulse {
	bool flag;             //激光点的标识   //过滤
	float range;           //修正后距离(m)   //标定
	uint16_t intensity;    //修正后灰度
	float pulseWidth;      //修正后脉宽(m)
	//uint8_t echoNum;
	double Xs,Ys,Zs;		// // 坐标系：设备坐标(m)    //用于CFans飞行标定
	double x,y,Z;		// 坐标系：大地坐标(m)    //pos联合解算
	//double afa, bta;       //极坐标角度    //用于滤波平滑
	float rise_edge;
	uint8_t flagT0; //mta使用
	CacPulse() {
		flag = 0;
		range = 0;
		intensity = 0;
		pulseWidth = 0;	
		x = y = Z = 0;
		Xs = Ys = Zs = 0;	
		//afa = bta = 0;
		rise_edge = 0;
	}
}CacPulse_S;

//内插姿态
typedef struct PosMsg {
	double gpstime;
	double heading;
	double pitch;
	double roll;
	double latitude;
	double lontitude;
	double height;
	PosMsg()
	{
		gpstime = 0;
		heading = 0;
		pitch = 0;
		roll = 0;
		latitude = 0;
		lontitude = 0;
		height = 0;		
	}
}PosMsg_S;

//镜面编号:AP CFans
//enum MirrorNum_E {
//	oneMirror = 0,
//	twoMirror = 1,
//	threeMirror = 2,
//	fourMirror = 3,
//	zeroAngSignal = 4,
//	updataFrame = 5,
//};

//解算结构体
typedef struct CalcLaserPt
{
	//标定后角度距离灰度等
	float scanAngle;      //扫描电机角度：标定/转换后
	PosMsg_S pos;          //内插位姿   转台角度  	
	CacPulse_S pulse[4];    //回波信息
	uint16_t lmac[4];      //0:  laserID  1: mirrorID 2:angleArea 3:channelAB	
	uint16_t transID;
	uint32_t stamp_time;
	uint8_t echoType;     //回波数：距离不为0判断

	//阳光噪点算法：过滤点标识
	bool filterFlag;    

	//mta
	int matNum;            //mta周期数
	uint8_t flagPower;     //mta点处理类型（mtaType）：基下点、断点、电力线点、相对高度内连续点、相对高度点
	uint8_t flagCont;      //mta：单侧连续性标识（true/false）
} CalcLaserPt_S;

typedef vector<CalcLaserPt_S> calcLasV;

typedef struct CalcStream
{
	CalcLaserPt_S *_calcPt;
	int32_t buff_size; //快数据buffer size
	int32_t las_num; //点云的个数
	DataBuff_S error_log;    //错误日志
	//std::vector<int> tag_idx;//储存过零点的标记索引
}CalcStream_S;


enum ROTATE_CLOCK_E
{
	CLOCK = 0,     //顺时帧
	UNTI_CLOCK = 1,  //逆时针
};

typedef struct InitConfig
{	
	float *revise_map_;
	int platNum;
	PlatForm_S *  platArr;
	ROTATE_CLOCK_E rotateClock;
	//platFormV  platArr;
	DataBuff_S *error_log;    //错误日志

}InitConfig_S;

typedef struct IniOption
{
	bool isErrorLog;
	bool isCFansAngleFilter;
	bool isFlightCalib;
}IniOption_S;



//内部数据处理用4个字节对齐

typedef struct tagLidarRAW {
	double gpst;
	double x, y, Z;
	double angle, dst;
	uint32_t gray;
	tagLidarRAW()
	{
		gpst = 0;
		x = y = Z = 0;
		angle = dst = 0;
		gray = 0;
	}
}LidarRAW;
typedef vector<LidarRAW>  LidarRAWV;

#define GRID_SIZE  14400               //3600*4   最小角度分辨率0.01度
//#define GRID_SIZE  115200           //3600*32    32路:最小角度分辨率0.01度
typedef struct GRIDE_HEAD {
	bool flag; // int
	uint32_t ptSize;
	double firstUtc;
	GRIDE_HEAD() {
		flag = false;
		firstUtc = -999.0;
		ptSize = GRID_SIZE;
	}
}GRIDE_HEAD_S;

typedef struct GRIDE_POINT {
	uint8_t intensity;
	uint16_t range;  //单位cm，最大值2^16=65536， 即最大距离655.99m
	//float angle;
	//uint16_t utc;   //时间为小数点后3位，最大值2^16=65536
	//bool flag;
	//double x,y,Z;
	//double utc2;
	//uint32_t tzero;
	GRIDE_POINT()
	{
		intensity = 0;
		range = 0;
	}
}GRIDE_POINT_S;


typedef struct GRIDE_NAV_POINT {
	float range;  //单位m
	uint16_t intensity;
	//float wide;	
	//float angle;
	//uint16_t utc;   //时间为小数点后3位，最大值2^16=65536
	//bool flag;
	//double x,y,Z;
	//double utc2;
	//uint32_t tzero;
	GRIDE_NAV_POINT()
	{
		intensity = 0;
		range = 0;
		//wide = 0;
	}
}GRIDE_NAV_POINTT_S;


typedef struct GRIDE_DATA {
	//bool flag; // int
	//uint32_t ptSize;
	//double firstUtc;
	GRIDE_HEAD_S header;
	GRIDE_POINT_S gridPt[GRID_SIZE];
	//int rowNum, colNum;
	//union {
	//GRIDE_POINT_S gridPt[GRID_SIZE];
	//GRIDE_POINT_S gridPt[rowNum][colNum];
	//GRIDE_POINT_S & gridRef ; // gridRef = malloc(size) ; gridRef[1...n] ;
	//}
	GRIDE_DATA() {
		memset(gridPt, 0, sizeof(GRIDE_POINT_S)* GRID_SIZE);
	}
}GRIDE_DATA_S;
typedef vector<GRIDE_DATA_S> GridDataV;

typedef struct TAG_GRIDE_NET_DATA {
	uint16_t grideDataID;
	uint32_t grideDataTotalSize;
	GRIDE_DATA_S sGrideData;
}GRIDE_NET_DATA_S;

#define ANGLE_RES_CFANS64  0.1    //((1.0/25600)*360)=0.014°

#define GRID_ROWARY_NUM  1029   
#define GRID_COLARY_NUM  32   //角度分辨率0.1° 保留视场角100°   //6400=(360 / 4) / (ANGLE_RES_CFANS64);
//#define GRID_FRAME_SIZE 600     //50   600

const int GRID_NAV_POINTT_SIZE = sizeof(GRIDE_NAV_POINTT_S);
const int GRID_COLBUF_SIZE = GRID_NAV_POINTT_SIZE * GRID_ROWARY_NUM;
const int GRID_ROWBUF_SIZE = GRID_NAV_POINTT_SIZE *GRID_COLARY_NUM;
const int GRID_GRDPNT_BUFSIZE = GRID_ROWBUF_SIZE * GRID_ROWARY_NUM;

typedef struct NavGridCfg {
	float temperature;
	int frameFreq;
}NavGridCfg_S;

////指针数组
typedef struct GRIDE_DATA_NAV {
	//int8_t temperature;
	GRIDE_NAV_POINTT_S (*gridPt)[GRID_COLARY_NUM];
	GRIDE_DATA_NAV() {
		//temperature = 40;
	}
}GRIDE_DATA_NAV_S;


struct RangeCorr_Temper_S {
	float temperMin;
	float temperMax;
	double rangeCoff[3];
};

//二维指针
//typedef struct GRIDE_DATA_NAV {
//	int8_t temperature;
//	GRIDE_POINT_S **gridPt;
//	GRIDE_DATA_NAV() {
//		temperature = 40;
//	}
//}GRIDE_DATA_NAV_S; 

////数组指针
//typedef struct GRIDE_DATA_NAV {
//	int8_t temperature;
//	GRIDE_POINT_S (*gridPt)[GRID_COL_SIZE];
//	GRIDE_DATA_NAV() {
//		temperature = 40;
//	}
//}GRIDE_DATA_NAV_S;

//slam格网
typedef struct GRIDE_PROP_SLAM
{
	float angleView;
	int navLidarNum;
	float angleRes;
	GRIDE_PROP_SLAM()
	{
		angleRes = 0.18;
		angleView = 360;
		navLidarNum = 16;
	}
}GRIDE_PROP_SLAM_S;
namespace fancy{
	namespace slam	{
		union LidarPoint	{
			float data[8];
			struct
			{
				float x;         //x
				float y;         //y
				float z;         //z
				float intensity; //强度
				float hangle;    //水平角
				float range; //距离
				int laserid;     //线号
				float timeflag;  //时间戳	
				//float pulseWidth;		
				//uint8_t mirrorid;
			};
		};
	}
}
typedef struct GRIDE_SLAM {
	bool flag;
	fancy::slam::LidarPoint * gridPt;
}GRIDE_SLAM_S;



typedef struct
{
	float time;
	float  accel_x;
	float  accel_y;
	float  accel_z;
	float  gyro_x;
	float  gyro_y;
	float  gyro_z;
	float  roll;
	float  pitch;
	float  yaw;
}IMU_DATE_OUT;

typedef vector<IMU_DATE_OUT> ImuVec;
#pragma pack()

#endif // __CALCULATE__
