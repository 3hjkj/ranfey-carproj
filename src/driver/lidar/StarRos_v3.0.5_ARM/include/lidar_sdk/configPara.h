#ifndef __FILTER_ANLIGN_PARA_H__
#define  __FILTER_ANLIGN_PARA_H__
using namespace std;
#include <vector>
#include"calculate.h"
#define GPSTIME_FILTER_NUM 40
#define ISF_HEADER 0x40000    //256*1024
#define IMP_HEADER 0x14000    //80*1024
#define DEC_BLOCK_COFF 5;            //可修改(>=1.5)

#define DECODER_SHOT_SIZE 250000//考虑到cfan系列的一帧最大的点达到640K/2.5hz
#define FAST_MAX_SIZE 0x7FEC000//快数据的最大值128M-256K
#define IMP_ISF_SIZE 0x8A00000//  
//#define ISF_HEADER 0X40000 //256k文件头
//#define ISF_HEADER 0x3FFFC//ISF的快数据大小


#define ZERO_ECHO_SIZE 12
#define ERORRLOG_BUFFSIZE  0x10000    

#define PACKET0_SIZE 1412      //一个udp的字节数
#define PACKET0_POINT_NUM 192   //6*32
#define UDP_MAX_SIZE 1500
#define DEC_POINT_SIZE 300000 
#define UDP_CALIBPARA_SIZE 290 

#define CALIBPARA_OFFSET 32
#define CALIBPARA_VALID_SIZE 256 
#define CALIBPARA_VALID_PACKET_NUM 90

#define CALIBPARA_START_ADDR 0xC4  
#define CALIBPARA_FLASH_ADDR 0x380000

#define CALIBPARA_READ_ADDR 0xC8  
#define CALIBPARA_READ_SIZE 0x6400

#define ANGLE_RES_MIN 0.05
#define ADR_D_ANGLE_RES 0x20F4
#define ADR_D_DEVICE_ID  0x20F0
#define ADR_D_TEMPERATURE 0x20C0

#pragma pack(1)
//采用数组的形式可以隐藏imp文件头的参数
typedef struct Header
{
	char data[IMP_HEADER];
}Header_S;

typedef struct DataRange {
	double min;
	double max;
	DataRange()
	{
		min = { 0 };
		max = { 0 };
	}
}DataRange_S;

//有效距离
typedef struct RangeFilter {
	bool enable;
	DataRange_S range;
}RangeFilter_S;

//距离灰度
typedef struct RangeIntensityFilter {
	bool enable;
	DataRange_S range;
	DataRange_S intensity;
}RangeIntensityFilter_S;

typedef struct AngleFilter {
	bool enable;	
	DataRange_S angle;
	bool isInvert;
}AngleFilter_S;


//高度
typedef struct HeightFilter {
	bool enable;
	DataRange_S height;
}HeightFilter_S;


//抽稀
typedef struct Resample {
	bool enable;
	uint32_t ratio;	
	float flightWide;
	float angleViewL;
	float angleViewR;
	Resample()
	{
		enable = false;
		ratio = 100;
		flightWide = 1000000;
		angleViewL =-90;
		angleViewR = 90;
	}
}Resample_S;


typedef struct EchoFilter {
	bool enable;
	bool echo[4];
	EchoFilter()
	{
		enable = false;
		echo[0] = echo[1] = echo[2] = echo[3] = 1;
	}
}EchoFilter_S;

typedef struct timeFilter {
	bool enable;
	float  time[GPSTIME_FILTER_NUM];
}timeFilter_S;

////interplation
//typedef struct Interp {
//	bool enable;
//	//int maxUtcGap;
//	//int maxIncltGap;
//	//int utcOffset;
//	//int gridSize;
//	Interp()
//	{
//		enable = false;
//		//maxUtcGap = 3600;
//		//maxIncltGap = 360;
//		//utcOffset = 0;
//		//gridSize = 1;
//	}
//}Interp_S;

//安装角
typedef struct lidarImuPlacePara {
	bool enable;
	int calcMode;
	int imuClock[3];
	int	frd_zdirect;
	float imu_frd_anlign[3];
	float lidar_frd_anlign[3];
	lidarImuPlacePara()
	{
		imuClock[0] = imuClock[1] = imuClock[2] = 1;
		frd_zdirect = -1;
		imu_frd_anlign[0] = 1; imu_frd_anlign[1] = 2; imu_frd_anlign[2] = 3;
		lidar_frd_anlign[0] = lidar_frd_anlign[1] = lidar_frd_anlign[2] = 0;
	}
	
}lidarImuPlacePara_S;

//安置角偏心量
typedef struct lidarAnlignPara {
	bool  utc_enable;   //轨迹内插
	bool   btd_enable;  
	double  utc2gps;
	double deltAngle[3];
	double deltXYZ[3];
	lidarAnlignPara()
	{
		utc_enable = false;
		btd_enable = false;
		utc2gps = 18;	
		deltAngle[0] = deltAngle[1] = deltAngle[2] = 0;
		deltXYZ[0] = deltXYZ[1] = deltXYZ[2] = 0;	
	}
}lidarAnlignPara_S;

enum AnlignParaType {
	ANLIGN_DEFAULT = 0,
	ANLIGN_FREQ,
	ANLIGN_HEIGHT,
};
//安装角安置角偏心量
typedef struct AnlignParaFlight {
	AnlignParaType  type;
	float freq_filght;
	double deltAngle[3];
	double deltXYZ[3];
	AnlignParaFlight()
	{
		type = ANLIGN_DEFAULT;
		freq_filght = 0;
		deltAngle[0] = deltAngle[1] = deltAngle[2] = 0;
		deltXYZ[0] = deltXYZ[1] = deltXYZ[2] = 0;
	}
}AnlignParaFlight_S;  //56

//基站流动站
typedef struct gpsPara {
	double longtitude;
	double latitude;
	double altitude;
	double centralMeridian;
	double baseH;
	double gpsDeltX;
	double gpsDeltY;
	double gpsDeltZ;
	
}gpsPara_S;

//投影参数
typedef enum {
	EastLongitude = 0,
	WestLongitude = 1,
	NorthLatitude = 2,
	SouthLatitude = 3,
}LongLatType_E;

typedef enum {
	WGS84 = 0,
	BeiJing54 = 1,
	XiAn80 = 2,
	GJ2000 = 3,
}EllipsoidType_E;

typedef enum {
	UTM = 0,
	GUSS = 1,
	TM = 2,
}ProjectionType_E;

typedef enum {
	STRPNUM6 = 0,
	STRPNUM3 = 1,	
	CONFIG = 2,
}StripNumType_E;

typedef struct  GeoProjection {
	bool enable;
	ProjectionType_E projectiontype; //投影类型
	EllipsoidType_E ellipsolidtype;  //椭球类型
	//double CentralMeridian; //中央子午线经度 单位度	
	//double centralLat;	
	LongLatType_E longitudetype;//经度类型
	LongLatType_E latitudetype;//纬度类型
	bool switch_centMerid; //跨度处理	
	StripNumType_E stripNum;      //投影带号 （3/6度带）
	GeoProjection() {
		enable = false;
		ellipsolidtype = WGS84;
		projectiontype = UTM;
		//CentralMeridian = 117.0;
		longitudetype = EastLongitude;
		latitudetype = NorthLatitude;
		switch_centMerid = false;
		stripNum = STRPNUM3;
	}
}GeoProjection_S;

typedef struct GpsUAPosition {
	bool enable;
	uint32_t centralMeridian;
	double B;
	double L;
	float H;
	GpsUAPosition()
	{
		enable = false;
		centralMeridian = 117;
		B = 34.0;
		L = 117.0;
		H = 0.0;
	}
}GpsUAPosition_S;

typedef struct OrientUA {
	bool enable;
	float Hs;
	float pointStart[3];
	float pointEnd[3];
	float turnAngle;
	float deltAngle[3];
	float deltXYZ[3];
	OrientUA()
	{
		enable = false;
		Hs = 0;
		pointStart[0] = 0; pointStart[1] = 0; pointStart[2] = 0;
		pointEnd[0] = 0; pointEnd[1] = 0; pointEnd[2] = 0;
		turnAngle = 0;
		deltAngle[0] = 0; deltAngle[1] = 0; deltAngle[2] = 0;
		deltXYZ[0] = 0; deltXYZ[1] = 0; deltXYZ[2] = 0;
	}
}OrientUA_S;

struct preFilterPara_S {
	AngleFilter_S angleFilter;
	Resample_S resample;
	EchoFilter_S echoFilter;
	float platAngleFilter;
	
};


typedef struct _ErroOutPut {
	unsigned int BorderT0; //相邻的两个时间点的T0
	unsigned int CheckT0;
	double  BorderScanAngle; //相邻的两个时间点的ScanAngle
	double CheckScanAngle;
	
	_ErroOutPut() {
		BorderT0 = 0; //相邻的两个时间点的T0
		CheckT0 = 0;
		BorderScanAngle = 0; //相邻的两个时间点的ScanAngle
		CheckScanAngle = 0;	
	}
}ErroOutPut_S;


typedef struct lidarCalcPara{
	//bool enableRA; // 灰度距离 默认为1
	//bool enableI;   //灰度标定 
	bool enableXYZ;    
	lidarCalcPara() {
		//enableRA = 1;
		//enableI = 1;
		enableXYZ = 1;
	}
}lidarCalcPara_S;


struct postFilterPara_S {
	RangeFilter_S rangeFilter;
	RangeIntensityFilter_S rangeIntensityFilter;
	HeightFilter_S heightFilter;
	timeFilter_S timeFilter;
};

struct posProjPara_S {
	//Interp_S interp;
	gpsPara_S gpsPara;
	lidarImuPlacePara_S lidarImuPlace;
	lidarAnlignPara_S posCmpCoor;
	GeoProjection_S geoProjection;
};

struct UAPosition_S {
	GpsUAPosition_S gpsUAPosition;
	OrientUA_S orientUA;
};

typedef struct mtaUserPara
{
	bool enable;
	float flightH;     //相对高度
	int neighPtNum;    //邻域点个数
	float angleVal;     //基下点角度区间 10
	float rangeT0;      //T0距离

	//距离连续性阈值
	float diffH_min;   //距离接近的判断阈值：  30
	float diffH_max;   //距离变化的判断阈值：  180

	 //断点位置的距离阈值
	float breakR_min;
	float breakR_max;    

	//暂时不用
	int indexInterp;  //100
	float diffH_power;   //50 
	//int neighNum_con;    //连续区域邻域点个数
	//float edgeAngleVal;  //边缘角度  >35
	//float edgeRangeVal;   //边缘最大距离  200
	mtaUserPara()
	{
		enable = false;
		flightH = 60;
		neighPtNum = 6;
		angleVal = 10;
		rangeT0 = 370;
	}
}mtaUserPara_S;


typedef struct transPtFormat
{
	bool enable;
	transPtFormat() {
		enable = false;
	}
}transPtFormat_S;

typedef struct OUTFILE {	
	bool isOutlas;
	bool isOurblh;
	bool isOutCrs;
	bool iserrorLog;	
	//bool isOutxyziT0A;
	//bool isOutxyziGps;
	//bool isOutagr;
	//bool isOutDecode;
	bool isOutxyzAll;      //解算数据	
	bool isOutCalibAG;    //地面角度（测绘）
	bool isOutCalibRG;     //地面距离 （测绘）
	bool isOutCalibAA;    //飞行角度 （AP/TP）
	bool isOutCalibAK;    //飞行角度 （AK/RA/RFans）
	bool isOutrfans;      //导航CFans/RFans飞行（激光器编号）
	bool isOutcfans;      //导航CFans飞行（镜面编号）
	bool isOutbagrT0;
	bool isOutbagrGps;
	bool isOutTransRaw;
	OUTFILE()
	{		
		isOutlas = false;
		isOurblh= false;
		isOutCrs = false;
		iserrorLog = false;
		isOutxyzAll = false;	
		isOutCalibAG = false;
		isOutCalibRG = false;
		isOutCalibAA = false;
		isOutCalibAK = false;
		isOutrfans = false;
		isOutcfans = false;
		isOutbagrT0 = false;
		isOutbagrGps = false;
		isOutTransRaw = false;
	}
}OUTFILE_S;

struct OutFileOption_S
{
	bool  id_pack;
	bool  id_pack_fast;
	bool  id_simulate;
	bool  id_scan_camera;
	bool  id_inclt_echo;
	bool  id_envir;
	bool  id_imu;
	bool  id_imu_lc100;
	bool  id_discard_lc100;
	bool  lfp_imu_lc100;
	bool  bfp_imu_lc100;
	bool  id_imu_ic100;
	bool  id_gps;
	bool  id_dmi;
	bool  id_pos2010;
	bool  id_gi510;
	bool  id_hg4930;
	bool  id_uimu_ic;
	bool  id_other;
	bool  id_synchron;
	bool  fp_error_log;
	bool  fp_imu_raw;
	bool  fp_imu_raw_dat_01;
	bool  fp_imu_raw_dat_02;
	bool  fp_imu_raw_lc100_01;
	bool  fp_imu_raw_lc100_02;
	bool  fp_imu_raw_lc100;
	bool  fp_imu_raw_dat;
	bool  fp_imu_gwxt_dat;
	bool  fp_imu_gwxt_lc100;
	bool  fp_decoder_rawData;
	bool  fp_imuRawData_IE;
	OutFileOption_S()
	{
		id_pack = false;
		id_pack_fast = false;
		id_simulate = false;
		id_scan_camera = false;
		id_inclt_echo = false;
		id_envir = false;
		id_imu = false;
		id_imu_lc100 = false;
		id_discard_lc100 = false;
		lfp_imu_lc100 = false;
		bfp_imu_lc100 = false;
		id_imu_ic100 = false;
		id_gps = false;
		id_dmi = false;
		id_pos2010 = false;
		id_gi510 = false;
		id_gi510 = false;
		id_gi510 = false;
		id_hg4930 = false;
		id_uimu_ic = false;
		id_other = false;
		id_synchron = false;
		fp_error_log = false;
		fp_imu_raw = false;
		fp_imu_raw_dat_01 = false;
		fp_imu_raw_dat_02 = false;
		fp_imu_raw_lc100_01 = false;
		fp_imu_raw_lc100_02 = false;
		fp_imu_raw_lc100 = false;
		fp_imu_raw_dat = false;
		fp_imu_gwxt_dat = false;
		fp_imu_gwxt_lc100 = false;
		fp_decoder_rawData = false;
		fp_imuRawData_IE=true;
	}
};



//整理lidarsystem配置文件得出结构体，考虑慢数据过多，挑出几个常用的慢数据作为外部参数
struct decSysOption_S
{
	bool is_imu;           //保存imu的二进制数据
	bool is_gps;           //保存imu的字符串文本数据
	bool is_camera;        //相机触发/曝光信息 ,0XFD20-0XFD26,0XFD2F
	bool is_pps;           //保存gps的二进制数据 
	bool is_pack;          //A7A7A7A7打包标记
	bool is_simulate;       //B7B7 仿真数据 count从1开始累加：测试数据传输、FPU单板测试;
	bool is_envir;         // f0f0 环境参数
	bool is_raw;           //上升沿文件的数据文本
	decSysOption_S()
	{
		is_imu=false; 
		is_gps=false;
		is_camera=false;
		is_pps=false;   //相
		is_pack=false;
		is_simulate=false;
		is_envir=false;
		is_raw=false;
	}
};


//整理lidarsystem配置文件得出结构体，考虑慢数据过多，挑出几个常用的慢数据作为外部参数
struct IoFpOption_S
{
	bool is_imu;           //保存imu的二进制数据
	bool is_gps;           //保存imu的字符串文本数据
	bool is_camera;        //相机触发/曝光信息 ,0XFD20-0XFD26,0XFD2F
	bool is_str_imu;           //保存gps的二进制数据 
	bool is_las;          //A7A7A7A7打包标记
	bool is_errlog;      //错误日志信息
	bool is_pps;
	bool is_fpps;
	IoFpOption_S()
	{
		is_imu=false;
		is_gps = false;
		is_camera = false;
		is_str_imu = false;
		is_las = false;
		is_errlog = false;
		is_pps = false;
		is_fpps = false;
	}
};




typedef struct connectAdr {
	char  sendAdr[256];
	char  revAdr[256];
}connectAdr_S;

typedef struct SLAM_GRIDE_PROP
{
	int pointFreq;
	int scanSpeed;    //10hz
	int laser_Max;
}SLAM_GRIDE_PROP_S;


//格网数据
typedef struct GRIDE_PROP
{
	bool update;
	float angleView;
	float angleRes;
	int intentVal;
	float angleVal;
	GRIDE_PROP()
	{
		update = false;
		angleView = 70;
		angleRes = 0.1;
		intentVal = 60;
		angleVal = 0.01;
	}
}GRIDE_PROP_S;

//集成参数配置
typedef struct lidarSysPara {	
	lidarImuPlacePara_S   lidarImuPlacePara; //0x4000 
	lidarAnlignPara_S   lidarAnlignPara;          //0x4001
	AnlignParaFlight_S  AnlignParaFlight;   //0x4002
	gpsPara_S gpsPara;     //0x4010
	GeoProjection_S geoProjection;    //0x4011
	GpsUAPosition_S gpsUAPosition;    //0x4020
	OrientUA_S orientUA;              //0x4021
	char reservedA[424];    //480-56

	RangeFilter_S rangeFilter;      //0x5000
	RangeIntensityFilter_S rangeIntensityFilter; //0x5001
	AngleFilter_S angleFilter;      //0x5002
	HeightFilter_S heightFilter;      //0x5003
	Resample_S resample;             //0x5004
	EchoFilter_S echoFilter;         //0x5005
	timeFilter_S timeFilter;
	char reservedB[476];
	//#0x5020
	mtaUserPara_S mtaPara;        //0x5020
	GRIDE_PROP  gridPara;
	char reservedC[1903];    //1920 -17

	decSysOption_S outSlowOption;     //0xF000
	OUTFILE_S outFile_S;               //0xF001
	connectAdr_S address;     //0xFFFF	
	bool validFlag[16];          //每个模块参数是否有效
	char reservedD[730];      
}lidarSysPara_S;    //4736  


// struct imp_lidar_sys {
//	char reservedA[66940]; //66940
//	lidarSysPara_S lidarSysPara;// 4736
//	char reservedB[10244];	
//} ;//total 81920 Byte






typedef struct SurveyDevOpt
{
	bool calibR;
	bool calibA;
	bool calibI;
	bool smoothRange;
	bool strechGray_SWTX;
	bool mtaCalc;
	bool lidarCalc;
	bool interpPps;          // 0x21
	SurveyDevOpt() {
		calibR = false;
		calibA = false;
		calibI = false;
		smoothRange = false;
		strechGray_SWTX = false;
		mtaCalc = false;
		lidarCalc = false;
		interpPps = false;
	}

}SurveyDevOpt_S;
//




typedef struct NavOptions
{
	//MultiAlgMode_E mode;	   //算法配置
	//0级
	bool Level0A;  //useTimeWindowFilter 
	bool Level0B;   //useRangeIntensityWideFilter
	bool Level0C;   //useAngleFilter
	bool Level0D;   //useMultiEchoWideFilter 
	bool Level0E;  //cfans 过滤
	//1级
	bool Level1A;   // useSingleLidarRangeRevise
	bool Level1B;   //useSingleLidarWideRevise
	bool Level1C;
	bool Level1D;
	bool Level1E;
	bool Level1F;  //useRangeTempratureRevise
	bool Level1G;  //useMidFarCalibRevise
	bool Level1H;  //echo2filter阳光噪点 ：机载
	bool Level1I;  //useLasifilter
	bool Level1J;  //useWideRangeRevise
	bool Level1K;   //useWideLaserRevise
	bool Level1L;  //useTrans12bitIntensity
	bool Level1P;  //电力线细化
	bool Level1Q;  //  阳光噪点
	bool Level1S;   //车载灰度标定
	//2级
	bool Level2A;     //12bit转8bit：线性拉伸     与Level1B 、Level1L同时作用
	bool Level2B;     //12转8bit:区间赋值        与 Level1B 、Level1L同时作用
	bool Level2C;     //脉宽转8bit（蜂鸟飞行）   不需要Level1B 和Level1L
	//高级
	bool LevelAA;  //获取镜面标  默认为true
	bool LevelAB;  //零位角修正  默认为true
	bool LevelAC;  //cfans角度修正
	bool LevelAD;   //useSmoothRange 
	bool LevelAE;   //useSmoothXY
	bool LevelAF;   //均匀化
	bool LevelAG;   //转换竖直角度和激光器编号 		
	//bool lidarCalc;            //设备坐标解算
	NavOptions() {		
		Level0A = Level0B = Level0C = Level0D = Level0E = false;	
		Level1A = Level1B = Level1C = Level1D = Level1E = false;
		Level1F = Level1G = Level1H = Level1I = Level1J = false;
		Level1K = Level1L = Level1P = Level1Q = false;		
		Level2A = Level2B = Level2C = false;		
		//LevelAA = LevelAB = false;
		LevelAC = LevelAD = false;
		LevelAE = LevelAF = LevelAG = false;
		//lidarCalc = true;
	}
}NavOptions_S;

typedef struct AlgOpt {
	//SurveyDevOpt_S survOpt;
   // NavOptions_S navOpt;
	bool options[50];

}AlgOpt_S;

//typedef enum {		//设备类型
//	SurvDev = 0,
//	NavDev=1,
//} DEVICE_E;

typedef struct tag_PosPPS_S {
	uint32_t ppsStamp;
	int32_t diffPpsStamp;
	double utcStamp;
	tag_PosPPS_S()
	{
		ppsStamp = PPS_STAMP_DEFAULT;
		diffPpsStamp = PPS_DIFF_DEFAULT;
		utcStamp = UTC_STAMP_DEFAULT;
	}
}PosPPS_S;

//转台插值结构体
struct turn_itp_S {
	uint32_t ppsStamp;
	int diffTurnCnt;
	int TurnCnt;
	turn_itp_S()
	{
		ppsStamp = PPS_STAMP_DEFAULT;
		diffTurnCnt = TURN_ANGEL_DIFF_DEFAULT;
		TurnCnt = TURN_ANGEL_DEFAULT;
	}
};

typedef vector<PosPPS_S> PPS_V;


enum DecType_E
{
	ISF_DEC = 0,
	IMP_DEC = 1, //both fast and slow data
};


typedef struct calConfigPara
{
	DecType_E deviceType;    //设备类型
	ROTATE_CLOCK_E rotateClock;
	char lidarPara[ISF_HEADER];   //内外参数	
	AlgOpt_S algOpt;       //算法模块
	char posPath[255];      //轨迹
	//lidarSysPara_S sysPara;        //外参   
	//vector<PosPPS_S> ppsV;   //时间对
}CalcInitPara_S;


//typedef struct calConfigPara
//{
//	DEVICE_E deviceType;    //设备类型
//	char lidarPara[ISF_HEADER];     //内参
//	lidarSysPara_S sysPara;        //外参
//	AlgOpt_S algOpt;       //算法模块
//	
//
//	////（2）设备参数
//	//SurveyPara_S surveyPara_S;
//	//NaviPara_S naviPara_S;
//	////DevPara_S devPara_S;
//	////（3）地理参数
//	//GeoPara_S geoPara_S;
//	////（4）高级参数
//	//AdvancePara_S advPara_S;
// //   //（5）慢数据
//	vector<PosPPS_S> ppsV;
//    char posPath[255];
//	////string posPath; 
//	//////（6）输出
//	////OUTFILE_S outFile_S
//}CalcInitPara_S;

//typedef struct TemperRT
//{
//	bool update;
//	float temper;
//}TemperRT_S;
//
//typedef struct IntenStreRT
//{
//	bool update;
//	float para[8];
//}IntenStreRT_S;

typedef struct PosUpdateInfo
{
	bool update;
	POS_D posD;
	//int curIndex;   //解算当前位置
	//int posNum;    //点个数
}PosUpdateInfo_S;


typedef struct calcOption {
	bool navConfig;
}calcOption_S;

typedef enum AlgOption {
	Config_ALG = 0,	
	Nav_DevCalc_ALG,
	Surv_DevCalc_ALG,
	Nav_GeoCalc_ALG,
	Surv_GeoCalc_ALG,
	Nav_GridCalc_ALG,
	Surv_GridCalc_ALG,
	Surv_CRS_ALG,
}AlgMode_E;

typedef enum AlgSysOpt{
	AlgSysPara_Assign=0,
	AlgSysPara_File=1,
}AlgSysOpt_E;

typedef struct CalcGridPara_S {
	GRIDE_PROP_S gridPara;
	PosUpdateInfo_S posData;
}CalcGridPara_S;

typedef struct CalcCfgPara
{	
	float temper;
	float intStrePara[8];  // ABCD-XYZP
	NavOptions_S navOptions;       //平行算法
	float revise_map_[0xFFFF];  //导航内参	
	int deviceID;	
	CalcGridPara_S gridPara;
}CalcCfgPara_S;



#pragma pack()

#endif  //__FILTER_ANLIGN_PARA_H__