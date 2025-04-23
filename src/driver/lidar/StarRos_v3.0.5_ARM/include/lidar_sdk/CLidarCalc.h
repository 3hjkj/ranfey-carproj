
#ifndef __LIDAR_CALC_H
#define __LIDAR_CALC_H

#include"compile.h"
#include "Decoder.h"
#include "calculate.h"
#include "configPara.h"
#include "slamPara.h"

class __dllexport CLidarCalc { 

public:
	CLidarCalc(){}
	~CLidarCalc(){}

	//初始化
	virtual int init(DecStream_S &decStream, CalcStream_S  &calcStream,CalcInitPara_S &configPara, AlgMode_E &option)=0;

	//点云解算
	virtual int pointRun(DecStream_S &decStream, CalcStream_S &calcStream, CalcCfgPara_S &updatePara, calcOption_S &calOption)=0;

	//测绘格网数据
	virtual int run(DecStream_S &decStream, CalcStream_S &calcStream, GRIDE_DATA_S &gridData, CalcCfgPara_S &updatePara, calcOption_S &option)=0;
	virtual int run(GRIDE_DATA_S &gridData, DecStream_S &decStream, CalcStream_S &calcStream, CalcCfgPara_S &updatePara, calcOption_S &option)=0;

	//slam格网
	virtual int run(DecStream_S &decStream, CalcStream_S &calcStream, SLAM_GRIDE_S & gridCloud, CalcCfgPara_S &updatePara, calcOption_S &option)=0;

	//Cfans64格网数据
	virtual int  run(DecStream_S &decStream, GRIDE_DATA_NAV &gridNavData, NavGridCfg_S &navGridCfg) =0;
	virtual int run(GRIDE_DATA_NAV &gridNavData, CalcStream_S &calcStream)=0;
protected:
	

private:


};

__dllexport CLidarCalc *getCalcPtr();

#endif
