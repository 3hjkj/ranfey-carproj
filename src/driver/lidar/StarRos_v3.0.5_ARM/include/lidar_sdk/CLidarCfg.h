#ifndef _ICLIDAR_CFG
#define _ICLIDAR_CFG

#include<vector>
#include"compile.h"
#include"configPara.h"
using namespace std;

/*
*内参（internal）+集成（integrate）参数的选择，二选一，不可同时选择
*/
enum  Cfg_Itn_E
{

	/* CFG文件(0x1- 0xF) */
	CFG_FILE = 0,


	/* imp文件/isf文件(0x1- 0xF) */
	RAW_FILE = 1,  
	/*
	*若是测绘：从lidar.cfg（0X3-0Xf）和 imp文件（0X1 -0X2）\
	*若是导航：从revise.ini（0x6-0xF），从isf文件头0x1-0x5\
	*/
	CFG_AND_RAW_FILE = 2,  

	/* imp_header/isf_header(0x1- 0xF) */
	HEADER_DATA = 3,

};  


/*
*集成参数的选择,可从lidarSys文件或者界面指定的参数的获取\
*如果值都是false，则从内参的选择获取集成参数的值
*/
enum Cfg_Itg_E
{
	 Is_ldSys=0,//从lidarSys文件获取
	 Is_Ui=1,//从界面ui获取配置参数
	 No_Sel=2,//都不选择
}; 



struct CfgOption_S 
{
	Cfg_Itn_E cfg_Itn;//internal 内参（internal）
	Cfg_Itg_E cfg_Itg;//integrate 集成（integrate）
};

struct CfgCfgPara_S 
{
	string raw_path;//isf文件/imp文件
	string cfg_path;//lidar.cfg文件/revise.ini文件
	string cfg_sys_path;//lidarSys文件
	string file_out_path;//lidarSys文件
	lidarSysPara_S lidarSys_ui;//界面传进来的结构体变量
	DataBuff_S imp_header;//传进来的V1格式数据头
	CfgCfgPara_S()
	{
		memset(&lidarSys_ui,0,sizeof(lidarSys_ui));
	}
};



class __dllexport CLidarCfg
{
public:
	CLidarCfg() {};
	~CLidarCfg() {};

	/*
    * 从文件头lidar.cfg/Revise.ini获取参数
    * @revise_buff 输出参数的数组
    * @buff_size 输出参数的buff的大小、
	* @cfg_path  isf文件/revise文件的路径大小
    * @return 成功返回true，失败返回false
    */
    virtual int getLidarCfg(char* revise_buff, int buff_size, CfgCfgPara_S  cfg_path, CfgOption_S  cgf_source)=0;


	/*
	* 读取revise路径，获取vec形式的数组
	* @iniPath 输入cfg文件的路径
	* @ini 输出vector的数组
	* @return 成功返回true，失败返回false
	*/
	virtual int getIniVec(string iniPath, vector<float> &ini) = 0;

private:

};

__dllexport CLidarCfg* creatLidarCfg(DecType_E cfg_type);
















#endif