#ifndef LidarIO_h
#define LidarIO_h

#include <string.h>
//#include "sssocket_global.h"
#include"compile.h"
#include"configPara.h"
#include"Decoder.h"
#include"dataOut.h"
enum DataType_E
{
	OTHER_DATA = 0,
	FAST_DATA = 1,
	SLOW_DATA = 2,
	
};

enum ErrorCode_E
{
	NO_Error=0,
	NameTooLong_Error,
	GpsOpenFailed_Error,
	ImuOpenFailed_Error,
	StrImuOpenFailed_Error,
	ScanCameraOpenFailed_Error,
	LasOpenFailed_Error,
	ErrorLogOpenFailed_Error,
};

//设置参数
struct IoCfgPara_S
{
	//char* file_path;//配置文件的路径
	DataType_E data_type;
	bool is_iss_flag;//是否含有ISS文件
};

class __dllexport CLidarIO
{
private:
protected:
public:

	//多个架次解算的初始化操作，例如多个架次的要将不同架次的满数据输出到不同的文件中，需要执行此操作
	//virtual void reset()=0;

	//慢数据和快数据两个功能是分开的，所以快慢数据初始化也要分开初始化，避免在慢数据初始化的过程生成LAS文件
	/*1. 设置慢数据输出的选择项，以及配置输出文本的路径及名称,在多文件循环外部创建一次;*/
	//virtual int initSlow(char* out_file_name, char*  out_file_path) =0;

	/* 2.快数据的文本输出，以及配置输出文本的路径及名称;在多文件循环内部循环调用*/
	//virtual int initHeader(const char* imp_header, int imp_header_size)=0;


	virtual int openLidarFile(char* _file_path, IoCfgPara_S config)=0;

	virtual void closeLidarFile() = 0;


	virtual  char* getHeaderBuff()=0;



	/*
	* brief: 初始化文件参数，并创建文件
	*@ out_file_name 输入文件名
	*@ out_file_path 输入文件路径
	*@ _option  创建文件的选项
	* return
	*/
	 virtual ErrorCode_E creatFile(char* out_file_name, char* out_file_path, const char* imp_header, int imp_header_size, IoFpOption_S _option)=0;
	/*
   * 从ISF/IMP文件获取文件头数据;
   * @param input_buff   输入buff，存放快数据；
   * @param config   配置参数；
   * @param io_type  输出选项；
   * @return 成功返回读取的实际大小，失败返回-1
   */


	//从文件中获取数据流，实时采集的过程中不需要该接口
	virtual int getFastBuff(DataBuff_S & input_buff)=0;


	//从文件中获取数据流，实时采集的过程中不需要该接口
	virtual int getSlowBuff(DataBuff_S & input_buff)=0;

	virtual int decWrtFile(DecStream_S& dec_input) = 0;


	virtual int decCalWrtFile( DecStream_S& dec_input,  CalcStream_S& cal_input)=0;

	/*
	* 从文件夹中获取文件列表
	*@ file_path 输入：文件夹路径
	*@ file_list 输出：文件名数组
	*/
	virtual int multFileFrPath(string  file_path,vector<string> &file_list) = 0;



	virtual string  getNameFrPath(string path_)=0;

	virtual int closeFile()=0;

	virtual bool getCrsStream(DecStream_S& dec_input, CalcStream_S& cal_input, CRS_LINE_S& dataLins) = 0;
	//virtual int setImpHeader(char* imp_header,int header_size)=0;


};

__dllexport CLidarIO *getLidarIO(DecType_E io_type);
__dllexport bool strSplice(char*src, vector<float>& value_);
__dllexport bool strSplice(char*src, vector<int>& value_);
__dllexport bool strSplice(char*src, vector<double>& value_);
#endif 
