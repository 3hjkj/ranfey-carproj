
#ifndef __SDK_CONFIG
#define __SDK_CONFIG
#include <string.h>
#include"configPara.h"
#define NAV_MIN_ANGLE_RES 0.13
//#define NAV_MIN_ANGLE_RES 0.05
void configAlg(AlgMode_E &option, AlgOpt_S &algOpt, lidarSysPara_S &lidarSysPara);
void assignSysPara(lidarSysPara_S &lidarSysPara);
int readPos(std::string trackFile, POS_V &posV);


void assignAlg(DecType_E DEVICE_TYPE,AlgOpt_S &algOpt)
{
	NavOptions_S navOption;
	SurveyDevOpt_S survOptions;
	switch (DEVICE_TYPE)
	{
	case ISF_DEC:
		memset(&navOption, 0, sizeof(NavOptions_S));
    		//navOption.Level0B = true;      //单路距离
    		navOption.Level1A = true;      //单路距离
    		navOption.Level1B = true;      //单路灰度脉宽
    		navOption.Level1L = true;      //12bit
    		navOption.Level1G = true;     //中远距离
    		navOption.Level1F = true;     //温度
    		navOption.Level1Q = true;     //阳光噪点
    		navOption.Level2A = true;     //12bit转8bit
    		navOption.LevelAB = true;     //零位角
    		navOption.LevelAC = true;     //cfans角度标定
    		//navOption.LevelAF = true;
   		//navOption.LevelAG = true;     //cfans转角度和lidarID
		memcpy(&algOpt, &navOption, sizeof(NavOptions_S));
		break;
	case IMP_DEC:
		memset(&survOptions, 0, sizeof(SurveyDevOpt_S));
		survOptions.calibR = true;      //距离角度修正
		survOptions.calibA = true;       //零位角度修正
		survOptions.smoothRange = true;  //距离平滑
		survOptions.lidarCalc = true;   //设备坐标解算
		survOptions.interpPps = false;
		memcpy(&algOpt, &survOptions, sizeof(SurveyDevOpt_S));
		break;
	default:
		break;
	}
}

void assignSysPara(lidarSysPara_S &lidarSysPara)
{
	memset(&lidarSysPara, 0, sizeof(lidarSysPara_S));
	lidarSysPara.outSlowOption.is_imu = true;
	////0x4000 
	//lidarImuPlacePara_S  * lidarImuPlacePara = &lidarSysPara.lidarImuPlacePara;
	//lidarImuPlacePara->imuClock[0] = 1; lidarImuPlacePara->imuClock[1] = 1; lidarImuPlacePara->imuClock[2] = 1;
	//lidarImuPlacePara->frd_zdirect = -1;
	//lidarImuPlacePara->lidar_frd_anlign[0] = 170;	lidarImuPlacePara->lidar_frd_anlign[1] = 0;	lidarImuPlacePara->lidar_frd_anlign[2] = -90;  //°
	// //0x4001
	//lidarAnlignPara_S   *lidarAnlignPara = &lidarSysPara.lidarAnlignPara;
	//lidarAnlignPara->utc_enable = true;  lidarAnlignPara->btd_enable = true;
	//lidarAnlignPara->utc2gps = 18;
	//lidarAnlignPara->deltAngle[0] = 0.02;	lidarAnlignPara->deltAngle[1] = 0.03;	lidarAnlignPara->deltAngle[2] = 0;
	//lidarAnlignPara->deltXYZ[0] = 10.4;	lidarAnlignPara->deltXYZ[1] = -81.73;	lidarAnlignPara->deltXYZ[2] = 3;
	////0x4010
	//gpsPara_S *gpsPara = &lidarSysPara.gpsPara;
	//gpsPara->centralMeridian = 113;
	////0x4011
	//GeoProjection_S *geoProjection = &lidarSysPara.geoProjection;
	//geoProjection->enable = true;
	//geoProjection->projectiontype = UTM;	geoProjection->ellipsolidtype = WGS84;
	//geoProjection->longitudetype = EastLongitude;  geoProjection->latitudetype = NorthLatitude;
	//geoProjection->switch_centMerid = 0;  geoProjection->stripNum = STRPNUM6;
	////0x5000
	//RangeFilter_S *rangeFilter = &lidarSysPara.rangeFilter;
	//rangeFilter->enable = false;
	//rangeFilter->range.min = 0; rangeFilter->range.max = 1800;
	////0x5001
	//RangeIntensityFilter_S *rangeIntFilter = &lidarSysPara.rangeIntensityFilter;
	//rangeIntFilter->enable = false;
	//rangeIntFilter->range.min = 0; rangeIntFilter->range.max = 1800;
	//rangeIntFilter->intensity.min = 0; rangeIntFilter->intensity.max = 8191;
	////0x5002
	//AngleFilter_S *angleFilter = &lidarSysPara.angleFilter;
	//angleFilter->enable = false;
	//angleFilter->angle.min = 0; angleFilter->angle.max = 360;
	//angleFilter->isInvert = 0;
	////0x5003
	//HeightFilter_S *heightFilter = &lidarSysPara.heightFilter;
	//heightFilter->enable = false;
	//heightFilter->height.min = 0; heightFilter->height.max = 100;
	////0x5004
	//Resample_S *resample = &lidarSysPara.resample;
	//resample->enable = false;
	//resample->ratio = 1; resample->angleView = 80; resample->flightWide = 10000;
	////0x5005
	//EchoFilter_S *echoFilter = &lidarSysPara.echoFilter;
	//echoFilter->enable = false;
	//echoFilter->echo[0] = 1; echoFilter->echo[1] = 1; echoFilter->echo[2] = 1; echoFilter->echo[3] = 1;
	////轨迹时间过滤
	//timeFilter_S *timeFilter = &lidarSysPara.timeFilter;
	//timeFilter->enable = false		;

	////格网参数
	//GRIDE_PROP_S *calcGridePara = &lidarSysPara.gridPara;
	//calcGridePara->angleRes = 0.1;
	//calcGridePara->angleVal = 0.01;
	//calcGridePara->angleView = 180;  //180
	//calcGridePara->intentVal = 0;
	//calcGridePara->update = true;

	////0x6000
	//mtaUserPara_S *mtaPara = &lidarSysPara.mtaPara;
	//mtaPara->enable = false;
	//mtaPara->neighPtNum = 20;  //邻域点个数
	//mtaPara->flightH = 40;   //相对高度
	//mtaPara->angleVal = 15;  //基下点角度范围
	//mtaPara->rangeT0 = 370;

	//decSysOption_S *outSlowOption = &lidarSysPara.outSlowOption;     //0xF000
	//OUTFILE_S *outFile_S = &lidarSysPara.outFile_S;               //0xF001
	//outFile_S->isOutlas = true;
	//outFile_S->isOutCrs = true;
	//outFile_S->isOutrfans = true;
	//outFile_S->isOutxyziGps = true;
	//outFile_S->isOutagr = true;
	//outFile_S->isOutxyzAll = true;
}



void configAlg(AlgMode_E &option, AlgOpt_S &algOpt, lidarSysPara_S &lidarSysPara)
{
	NavOptions_S navOption;
	SurveyDevOpt_S survOptions;
	switch (option)
	{
	case Config_ALG:
		memset(&navOption, 0, sizeof(NavOptions_S));
		navOption.Level1A = true;      //单路距离
		navOption.Level1B = true;      //单路灰度脉宽
		navOption.Level1L = true;      //12bit
		navOption.Level1G = true;     //中远距离
		navOption.Level1F = true;     //温度
		navOption.Level1Q = true;     //阳光噪点
		navOption.Level2A = true;     //12bit转8bit
		navOption.LevelAA = true;     //镜面标识
		navOption.LevelAB = true;     //零位角
		navOption.LevelAC = true;     //cfans角度标定
		navOption.LevelAG = true;     //cfans转角度和lidarID		
		memcpy(&algOpt, &navOption, sizeof(NavOptions_S));
	case Nav_DevCalc_ALG:	
		memset(&navOption, 0, sizeof(NavOptions_S));
		navOption.Level1A = true;      //单路距离
		navOption.Level1B = true;      //单路灰度脉宽
		navOption.Level1L = true;      //12bit
		navOption.Level1G = true;     //中远距离
		navOption.Level1F = true;     //温度
		navOption.Level1Q = true;     //阳光噪点
		navOption.Level2A = true;     //12bit转8bit
		navOption.LevelAA = true;     //镜面标识
		navOption.LevelAB = true;     //零位角
		navOption.LevelAC = true;     //cfans角度标定
		navOption.LevelAG = true;     //cfans转角度和lidarID		
		memcpy(&algOpt, &navOption, sizeof(NavOptions_S));
		lidarSysPara.lidarAnlignPara.utc_enable = false;
		lidarSysPara.lidarAnlignPara.btd_enable = false;
		lidarSysPara.geoProjection.enable = false;
		break;
	case Nav_GeoCalc_ALG:	
		memset(&navOption, 0, sizeof(NavOptions_S));
		navOption.Level1A = true;
		navOption.Level1G = true;
		navOption.Level1F = true;
		navOption.Level1H = true;       //蜂鸟阳光噪点
		navOption.Level2C = true;       //蜂鸟灰度
		navOption.LevelAA = true;
		navOption.LevelAB = true;
		navOption.LevelAC = true;
		navOption.LevelAG = true;
		memcpy(&algOpt, &navOption, sizeof(NavOptions_S));
		lidarSysPara.lidarAnlignPara.utc_enable = true;   //轨迹文件插值
		lidarSysPara.lidarAnlignPara.btd_enable = true;   //解算到BLH
		lidarSysPara.geoProjection.enable = true;        //投影计算
		break;
	case Surv_DevCalc_ALG:	
		memset(&survOptions, 0, sizeof(SurveyDevOpt_S));
		survOptions.calibR = true;      //距离角度修正
		survOptions.calibA = true;       //零位角度修正
		survOptions.smoothRange = true;  //距离平滑
		survOptions.lidarCalc = true;   //设备坐标解算
		survOptions.interpPps = false;
		memcpy(&algOpt, &survOptions, sizeof(SurveyDevOpt_S));
		lidarSysPara.lidarAnlignPara.utc_enable = false;
		lidarSysPara.lidarAnlignPara.btd_enable = false;
		lidarSysPara.geoProjection.enable = false;
		break;
	case Surv_GeoCalc_ALG:		
		memset(&survOptions, 0, sizeof(SurveyDevOpt_S));
		survOptions.calibR = true;      //距离角度修正
		survOptions.calibA = true;       //零位角度修正
		survOptions.smoothRange = true;  //距离平滑
		survOptions.lidarCalc = true;   //设备坐标解算
		survOptions.interpPps = true;
		memcpy(&algOpt, &survOptions, sizeof(SurveyDevOpt_S));
		lidarSysPara.lidarAnlignPara.utc_enable = true;   //轨迹文件插值
		lidarSysPara.lidarAnlignPara.btd_enable = true;   //解算到BLH
		lidarSysPara.geoProjection.enable = true;        //投影计算
		break;
	case Nav_GridCalc_ALG:	
		memset(&navOption, 0, sizeof(NavOptions_S));     //格网数据已标定距离和灰度
		memcpy(&algOpt, &navOption, sizeof(NavOptions_S));
		lidarSysPara.lidarAnlignPara.utc_enable = false;    //轨迹实时插值
		lidarSysPara.lidarAnlignPara.btd_enable = true;
		lidarSysPara.geoProjection.enable = true;
		break;
	case Surv_GridCalc_ALG:
		memset(&survOptions, 0, sizeof(SurveyDevOpt_S));    //格网数据已标定距离、灰度和角度
		survOptions.lidarCalc = true;    //设备坐标解算
		memcpy(&algOpt, &survOptions, sizeof(SurveyDevOpt_S));
		lidarSysPara.lidarAnlignPara.utc_enable = false;    //轨迹实时插值
		lidarSysPara.lidarAnlignPara.btd_enable = true;
		lidarSysPara.geoProjection.enable = true;
		break;
	}
}



int readPos(std::string trackFile, POS_V &posV)
{
	FILE * fp = fopen(trackFile.c_str(), "rb+");
	if (fp == NULL) {
		return -1;
	}

	fseek(fp, 0, SEEK_END); //定位到文件末尾
	int posSize = ftell(fp);   //文件长度
	int posCount = posSize / sizeof(SBET_Rec);
	posV.resize(posCount);
	fseek(fp, 0, SEEK_SET);   //定位到文件开始
	fread(posV.data(), sizeof(SBET_Rec), posCount, fp);
}

void getTemper(float& temper, string  filePath_)
{

	int count_ = 0;
	int degin_ = -1;
	while ((degin_ = filePath_.find("-", degin_ + 1)) != string::npos)
	{
		count_++;
		degin_ += 1;
	}

	if (count_ == 2)
	{
		temper = -999;
	}
	else if (count_ == 3)
	{
		int idx_s = filePath_.find_last_of("-");
		int idx_e = filePath_.find_last_of(".");
		string temp_ = filePath_.substr(idx_s + 1, idx_e - idx_s - 1);
		temper = atof(temp_.c_str());
	}

}
#endif
