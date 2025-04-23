
#ifndef __SDK_INTERFACE
#define __SDK_INTERFACE
#include <string.h>
#include "CLidarIO.h"
#include "CLidarDecoder.h"
#include "CLidarCfg.h"
#include "CLidarCalc.h"
#define NAV_MIN_ANGLE_RES 0.13
#define BLOCK_SIZE 0X100000          //输入数据流大小1M:  可修改

struct SDK_OBJ_S
{
	CLidarIO*       _io_ptr;
	CLidarCalc*    _cal_ptr;
	CLidarDecoder* _dec_ptr;
	CLidarCfg*    _cfg_ptr;
	SDK_OBJ_S()
	{
		_io_ptr  = NULL;
		_cal_ptr = NULL;
		_dec_ptr = NULL;
		_cfg_ptr = NULL;
	}
};

struct SDK_PARA_S
{
	CfgCfgPara_S       cfg_para;
	CfgOption_S     cfg_opt ;
	IoCfgPara_S     io_para ;
	DecCfgPara_S    dec_para;
	DecOption_S     dec_opt ;
	CalcInitPara_S  cal_iniPara;
	CalcCfgPara_S   cal_updPara;
	calcOption_S    cal_opt;
	AlgOpt_S        cal_algOpt;
	AlgMode_E       cal_algMode;
	DecType_E       dec_mode;
};









#endif