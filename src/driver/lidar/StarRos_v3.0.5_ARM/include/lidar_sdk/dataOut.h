#ifndef DATAOUT_H
#define DATAOUT_H
#include<string>
#include"compile.h"
#include"ICD_Scada_API.h"
#include"calculate.h"
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

#pragma pack(1)
//struct GRIDE_NAV_PNT_S
//{
//	float range;
//	uint16_t intensity;
//};

////指针数组

//每个group的点云
struct GRIDE_GROUP_S {
	int16_t angel;
	GRIDE_NAV_POINTT_S gridPt[GRID_COLARY_NUM];
};


//每个udp包的大小
struct GRIDE_PACKET_S
{
	uint32_t FLAG;
	int16_t frame_cnt;
	int16_t packet_cnt;
	int32_t utc_time;
	int32_t stamp;
	GRIDE_GROUP_S group[7];
	int16_t check_sum;//
	int32_t tail_syn;//帧尾同步字
};
#pragma pack()
#endif // SSSOCKET_H
