#ifndef CREGISTER_H
#define CREGISTER_H
#include <string>
#include<iostream>
#include<fstream>
#include"compile.h"
#include"sssocket.h"
//网络接口前面加WSA为Winsock2的函数
 struct DEB_REGIST_S 
 {
	unsigned int regAddr;          
	unsigned int regValue;
 };
 class __dllexport CRegister
{
private:
protected:
public:
	CRegister(void) {};
	~CRegister(void) {};
	/**
    * 设置控制转速的参数
    * @param deviceType 设备类型
    * @param speed 转速大小
    */
	virtual bool modSpeed(string deviceType, int speed)=0;

	/**
	* 传入socket对象
	* @ss_socket socket对象
	*/
	virtual bool setInternet(string ip_,int port_dev,int port_my=2015)=0;



	/* 往目标的寄存器写入数据
	**@regAddress 寄存器地址
	**@regData    写入数据
	**@return 返回true，写入成功，返回false，写入失败
	*/
	virtual bool  wrReg(int regAddress, int regData)=0;




	/* 往目标的寄存器写入数据
    **@regAddress 寄存器地址
    **@regData    写入数据
    **@return 返回true，写入成功，返回false，写入失败
    */
	virtual bool  rdReg(DEB_REGIST_S& msg)=0;
};

__dllexport CRegister* USEREG();

#endif // SSSOCKET_H
