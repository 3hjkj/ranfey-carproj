#ifndef REGISTER_H
#define REGISTER_H
#include <string>
#include<iostream>
#include<fstream>
#include"Cregister.h"
#include"compile.h"
#include"sssocket.h"
#include"ss_deb.h"


class  __dllexport DebRegister :public CRegister
{
public:
	DebRegister(void);
	/**
    * 设置控制转速的参数
    * @param deviceType 设备类型
    * @param speed 转速大小
    */
	bool modSpeed(string deviceType, int speed);

	/**
	* 传入socket对象
	* @ss_socket socket对象
	*/
	bool setInternet(string ip_,int port_dev,int port_my=2015);



	/* 往目标的寄存器写入数据
	**@regAddress 寄存器地址
	**@regData    写入数据
	**@return 返回true，写入成功，返回false，写入失败
	*/
	bool  wrReg(int regAddress, int regData);




	/* 往目标的寄存器写入数据
    **@regAddress 寄存器地址
    **@regData    写入数据
    **@return 返回true，写入成功，返回false，写入失败
    */
	bool  rdReg(DEB_REGIST_S& msg);




private:

	/**
	* 绑定socket，绑定对方主机ip地址和端口，以及自身端口
	* @param SCD_TYPE 设置命令的方式（读写、查询）
	* @param regAddress 寄存器地址
	* @param regData 寄存器数据值
	* @return 控制命令packet
	*/
	SS_DEB_FRAME packTransFrame(SCD_TYPE FLAG, int regAddress, int regData);

	/**
    * 计算校验和
    * @param _dataBuf 数组首地址
    * @param count_ 数组大小
    * @return 校验和
    */
	int checkSum(unsigned char * _dataBuf, int count_);



    int HW_RDREG(int flag, int regAddress, unsigned int & regData)
	{
		int rtn = 0;
		regData = m_regMap[regAddress%ROMREG_MAX_COUNT].regData;
		regData = regData & 0xFF;
		return rtn;
	}

private:
	SsSocket m_net;//example:R-Fans,C-Fans
	SS_DEB_FRAME m_regMap[ROMREG_MAX_COUNT];
	FRAMS_BUFFER_S m_frameBuffer;
};



int __dllexport loadIniFile(std::string iniPath, std::map<std::string, std::string> &ini);
int  writeFrameBuffer(FRAMS_BUFFER_S *mtFrameMsgBuf, char * _mt_frame, int mt_size);
bool readDEBFrameBuffer(FRAMS_BUFFER_S *mtFrameMsgBuf, SS_DEB_FRAME *mtRegMap);
#endif // SSSOCKET_H
