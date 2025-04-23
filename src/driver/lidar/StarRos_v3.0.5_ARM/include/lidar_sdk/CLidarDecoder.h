#pragma once
//
//////////////////////////////////////////////////////////////////////
#ifndef CLIDAR_DECODER_H
#define CLIDAR_DECODER_H
#include <string>
#include"ICD_LiDAR_API.h"
#include"compile.h"
#include "Decoder.h"
//#include "impHeader.h"


struct RawPntBuff_S
{
    char* ptr ;
    uint32_t   size ;
};

enum StreamType_E
{
    unknown = 0,
    fast_stream = 1,
    slow_stream = 2,
    fast_slow_b = 3, //both fast and slow data
};


struct DecOption_S
{
 bool angelSel;     //是否要输出过零位点的索引
 bool regionSel;    //区域范围内的点云选择
 bool is_clear_pnt;//将点云输出buff的点云计数清零
 DecOption_S()
 {
	 is_clear_pnt = false;//点云计数清零
	 angelSel=true;     //过零位点的区域选择
	 regionSel=false;    //区域范围内的点云选择
 }
};


class CLidarDecoderISF;

class   __dllexport CLidarDecoder
{
private:


protected:


public:
    CLidarDecoder(){};
    ~ CLidarDecoder(){};

	virtual int reset() =0;
	/*
	* 传入解码元信息;
	* @param _addr    输入元数据流buff的首指针；
	* @param  size    输入元数据流buff的大小；
	* @return 成功返回true，失败返回false
	*/
    virtual int setMetaHeader(char * _addr,int size)=0;


	/**
	 * 解码快数据;
	 * @param data_stream 输入：快数据的数据流；
	 * @param stream_out  输出：快数据的解码输出；
	 * @param configure  配置：参数的配置；
	 * @param option  选项：功能的选项；
	 * @return 返回实际解码消耗的字节数，若<0表示解码失败，=0无解码输出，
	 */
	virtual int decFastRun(DataBuff_S&  data_stream, DecStream_S&  stream_out, DecCfgPara_S& configure, DecOption_S& option) { return 0; };


	/**
	 * 解码慢数据;
	* @param data_stream 输入：快数据的数据流；
	* @param stream_out  输出：快数据的解码输出；
	* @param configure  配置：参数的配置；
	* @param option  选项：功能的选项；
	 * @return 返回实际解码消耗的字节数，若<0表示解码失败，=0无解码输出，
	 */
	virtual int decslowRun(DataBuff_S&  data_stream, DecStream_S&  stream_out, DecCfgPara_S& configure, DecOption_S& option) { return 0; };


};


__dllexport CLidarDecoder *createDecObj(DecType_E dec_obj);

 CLidarDecoder *createIMP();

 CLidarDecoder *createIsf();

#endif // BKBLOCKCALC_H
