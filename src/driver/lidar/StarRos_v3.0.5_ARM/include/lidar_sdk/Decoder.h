#pragma once
#include <string.h>
#include <stdint.h>
#include <map>
#include <vector>
#include "slowdata.h"
#include"configPara.h"
#include"compile.h"
//#include"impHeader.h"
#define IMP_FAST_MAX_SIZE 0x7FEC000//快数据的最大值128M-80K
#define ISF_FAST_MAX_SIZE  0x7FC0000 //快数据的最大值128M-256K
#define ECHO_MIN_SIZE 12 //最小的回波值
#define ISF_IMU_MAX_SIZE 1134//ISF中的imu数据
#define ISF_SLOW_MAX_SIZE 1314//isf的慢数据的最大字节数
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



typedef std:: map<int, std::vector<float>> MapCfgItem ; // max item is ????
typedef std::vector<float> isfIni;
typedef vector<PosPPS_S> PPS_V;
struct EchoPulse_S {
    float    range ;       //距离(m)
    uint16_t intensity ;   //灰度
    float    pluse_width ; //脉宽(m)
    float    rise_edge ;   //上升沿(m???)
	float   rangH;
	float   rangL;
	uint16_t     intenH;
	uint16_t     intenL;
	EchoPulse_S()
	{
		range=0;
		intensity=0;
		pluse_width = 0;
		rise_edge=0;
	}
};

struct LaserPoint_S {
	uint32_t stamp_time;
	double      utc_time;//CRS解算将时间戳对其赋值   
    float     scan_angle;     //扫描角度（单位度）
    float     turn_angle;     //转台角度（单位度）
    /*
     *byte0：laser_id (激光器通道号); byte1:    receiver chnl_id（A\B通道）;
     *byte2: mirror_id （镜面标识） ; byte3: 0Xxxii: scope from (ii - xx);
    */
    uint32_t lsr_chnl_mir_pn ; // pn: pulse num from ii - xx
    EchoPulse_S pulse[4];
	LaserPoint_S()
	{
		utc_time = 0.0;
		scan_angle = 0.0;
		lsr_chnl_mir_pn = 0;
	}
};




struct SlowStream_S
{
	DataBuff_S pack_txt;
	DataBuff_S simulate;   //B7B7 仿真数据 count从1开始累加：测试数据传输、FPU单板测试;
	DataBuff_S scan_camera;//相机触发/曝光信息
	DataBuff_S envir;      //环境参数
	DataBuff_S imu_data;   //保存imu的二进制数据
	DataBuff_S imu_txt;    //保存imu的字符串文本数据
	DataBuff_S gps_data;   //保存gps的二进制数据
	//DataBuff_S gps_txt;    //保存gps的字符串文本数据
	DataBuff_S pos_data;    //保存pos2010的字符串文本数据
	//DataBuff_S isf_imu;    //保存isf中的imu数据
	DataBuff_S pps_txt;    //保存PPS的txt数据
	DataBuff_S turn_angel;    //保存转台信息的buff;
};



struct DecStream_S
{
	LaserPoint_S * _lpoints;  //快数指针
    uint32_t buff_boundary;   //快数据buffer的大小(边界)
    int32_t  las_num  ;       //buffer点云的个数
	std::vector<int> tgidx_vec;//储存过零点的标记索引
	SlowStream_S slow_stream; // 所有的慢数据buff
	DataBuff_S error_log;    //保存gps的字符串文本数据
	DataBuff_S dec_pnt_str; //点云的解码中间数据输出
	DataBuff_S fpps_err;
	PPS_V pps_pair;
	uint8_t deviceID;
	bool is_over_buff;
	DecStream_S()
	{
		is_over_buff = false;
	}
};





enum DATA_SOURCE_E
{

    HeadeDatar=0,
    FastData  =1,
    SlowData  =2,
};

struct DecCfgPara_S
{
    float angel_seg;
    float region_min;
    float region_max;
	int   scd_ver;//imp使用
	int   isf_ver;//isf的版本
	DecCfgPara_S()
	{
	angel_seg = 0.0;
	scd_ver = 3;
	isf_ver = 0;
	}
};

enum IMP_DEC_TYPE
{
	IMP_CRS = 1,
	IMP_2 = 2,
	IMP_3 = 3,
	IMP_3_1=4,
};


//struct DecSlowOption_S
//{
//	bool pack;       //A7A7A7A7打包标记
//	bool simulate;   //B7B7 仿真数据 count从1开始累加：测试数据传输、FPU单板测试;
//	bool scan_camera;//相机触发/曝光信息 ,0XFD20-0XFD26,0XFD2F
//	bool envir;         // f0f0 环境参数
//	bool bin_imu;     //保存imu的二进制数据
//	bool str_imu;      //保存imu的字符串文本数据
//	bool bin_gps;    //保存gps的二进制数据
//	//bool str_gps;     //保存gps的字符串文本数据
//	bool bin_pos2010;  //pos2020,gps的二进制数据
//	bool isf_imu;      //isf的imu输出
//	DecSlowOption_S()
//	{
//		pack = false;
//		simulate = false;
//		scan_camera = false;
//		envir = false;
//		bin_imu = false;
//		str_imu = false;
//		bin_gps = false;
//		//str_gps = false;
//		bin_pos2010 = false;
//		isf_imu = false;
//	}
//};
struct WrtFile_S
{
	FILE* pack;       //A7A7A7A7打包标记
	FILE* simulate;   //B7B7 仿真数据 count从1开始累加：测试数据传输、FPU单板测试;
	FILE* scan_camera;//相机触发/曝光信息 ,0XFD20-0XFD26,0XFD2F
	FILE* envir;         // f0f0 环境参数
	FILE* bin_imu;     //保存imu的二进制数据
	FILE* str_imu;      //保存imu的字符串文本数据
	FILE* bin_gps;    //保存gps的二进制数据
	FILE* isf_imu;
	FILE* pps_txt;
	FILE* fpps_txt;
	FILE* err_log; //错误日志信息
	FILE* s_pps_err;
	FILE* f_pps_err;
	FILE* las;
	FILE* crs;
	WrtFile_S()
	{
		pack = NULL;
		simulate = NULL;
		scan_camera = NULL;
		envir = NULL;
		bin_imu = NULL;
		str_imu = NULL;
		bin_gps = NULL;
		isf_imu = NULL;
		pps_txt = NULL;
		fpps_txt = NULL;
		err_log = NULL;
		s_pps_err = NULL;
		f_pps_err = NULL;
		las = NULL;
		crs = NULL;
	}
};


struct WrtFileCfg_S
{
	string  out_file_name;   //生成文件的名字 如果不设置，则设置当前时间为文件名称
	string  out_file_path;   //生成文件的路径 如果不设置，则设置为当前系统路径
};
