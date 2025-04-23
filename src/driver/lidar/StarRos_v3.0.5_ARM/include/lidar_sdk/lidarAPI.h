/********************************************************************
 * $I
 * @Technic Support: <sdk@isurestar.com>
 * All right reserved, Sure-Star Coop.
 ********************************************************************/
#ifndef SS_LIDAR_API_H_
#define SS_LIDAR_API_H_

#include "ICD_LiDAR_API.h"
#include "ICD_LiDAR_TSK.h"
#include "ICD_Scada_API.h"
#include "ICD_UsbCamera.h"

//added by zhubing 2020.1.3
#ifndef _WINDOWS
	#include "../navigator/include/ssNavigator.h"
	#include "../navigator/include/nav_ccoordconvert.h"
	#include "../navigator/include/nav_datatype.h"
	#include <mutex>

#endif // !WINDOWS

#ifdef BINARY_CONTROL
//added by zhubing 2020.4.27
#include <sdk-decoder/dev/star/Packet.h>
#include <sdk-decoder/msg/cmd/Channel.h>
#include <sdk-decoder/Socket.h>
//end added by zhubing 2020.4.27
#endif

//added by linlianming 20210310 SRS-726
#ifdef LIDAR_HARDWARE 
#include "../si-share/BkUdpBroadcast.h"
#endif


class CUsbCamera ;
class CScada ;
class CLidarConfig ;
class IdataHandle ;
class CLifetime ;
class CLidarReporter ;
class CPalmMode ;
class CStrPtcTool ;
class CSvrSocket ;
class CNavSerial ;
class CdataCom ;
//typedef struct {
//	unsigned int dataid;
//	unsigned int len;
//	unsigned int groundHigh;
//}netData_groundHigh_S;


#if defined(_MSC_VER)
#ifdef LIDARAPI_EXPORTS
#define LIDAR_API_DLL __declspec(dllexport)
#else //LIDARAPI_EXPORTS
#define LIDAR_API_DLL __declspec(dllimport)
#endif //LIDARAPI_EXPORTS
#else //defined(_MSC_VER)
#define LIDAR_API_DLL
#endif //defined(_MSC_VER)


class LIDAR_API_DLL CLidarAPI {
  private:
    //<! LiDAR State
    LDRPROG_STAT_E m_eprogstat;
    LDRPROG_STAT_E cmrScanSta_;
    GPSMSG_RINGBUF_S m_gpsMsgRing ;
    SHOTS_RINGBUF_S shotsRing_;		//!< for lidar data collecting
    IMAGE_RINGBUF_S imageRing_[USBCmr_MAXNUMBER]; //一个相机一个ring Buffer，异步事件数据(回调函数)串行处理
    MESSG_RINGBUF_S messgRing_;   //!< for GUI message box
    CFG_RINGBUF_S   m_serverCfgRing_ ;    //buf
    CFG_RINGBUF_S   m_clientCfgRing_ ;    //配置文件信息buf
    SLOW_RINGBUF_S slowRing_ ;    //慢数据Buffer
    STRMSG_RINGBUF_S strRprtRing_ ; //字符命令反馈buf
	//FILE *fp_test;
	//int buf_full;
	
    int m_camNeedCnt;
    int m_imgCnt;
    int preCmrCount;
    bool m_cmrIsTrige ;   // 外置相机触发 
    int cmrCount ; //debug
    CScada *s_scada;
    CLidarConfig *s_config;
	//added by zhubing 2020.1.3
#ifndef _WINDOWS
	Point_S m_pStart_;
	Point_S	m_pNext_;
	CNavigator *tmpNavigator_;
	double	m_fSumDis_;
	SCDCMD_CAMERA_S tmpCamera;
	void cameraRangeTrigger(Pilot_S &data);	
	FILE *cmrTriggerlog;
	char *buf_gps;
#endif
	bool cameraRangeTrigger_flag;
    //<! Camera
    CUsbCamera * _usbCmr;
    //USBCMR_CTRL_S usbCmrCtrl_[USBCmr_MAXNUMBER];   //!< 相机参数控制
   // USBCMR_TRIG_S usbTrig_;

    LiDAR_DATAINFO_S curData_; //!< 实时显示数据 liyp
    
    SCADA_POST_S posDataStat_ ; //!< pos 状态 liyp

    CLifetime            *_lidarLife  ;       //!< 上电时间记录
    CLidarReporter       *_lidarRecord ;	    //!< 网络命令记录
    unsigned short       reportItv_ ;         //!< 间隔时间
    LiDAR_STAT_E         lidarCfgStat_ ;      //!< lidarConfig 接受记录
    CPalmMode *_m_palmMode ;                  //!< palm 控制模块
    SCADA_TYPE_E m_scada ; //scadaType
    LiDAR_TYPE_E m_lidar;
    
    bool m_openLidar ;                  //The LIDAR is Opened.
    int m_cmfFlashCount ;
    CStrPtcTool *_mPtcTool ;
    bool m_dataFlwRun ;  //初始化fast ring buffer 信号
    bool m_dataTrans;
    int m_StgMsgUp ;
    int m_StgMsgCount ;
    LDRPROG_STAT_E m_cmdSpinStat ;
    LDRPROG_STAT_LIST m_cmdStatList ;

    LDRFILE_STAT_LIST m_fileCtrlList ;
    CSvrSocket *_svrSock ;
    CdataCom *_m_svrUrt ;
	#ifdef LIDAR_HARDWARE 
	BkUdpBroadcast* bkUdpBroadcast;   //usded to broadcast ground height data 20200310 linlianming
	#endif
    CStrPtcTool *_m_strCmdHnd ;
    CStrPtcTool *_m_strMsgHnd ; 
    time_t m_StartTime ;  //开始采集数据时间
    time_t m_StopTime ;   //结束采集数据时间
    LDRPROG_STAT_E m_slAtuoCtrlStatus;
    UartScreenStatus m_usStatus;// uart screen的工作状态
    UartSceenParam m_usParam;
  private:
    void impDataCtrl(SCDCMD_PROGRM_S progPara_) ;
    
    void initUsbCamera( ) ;
    void initScada(SCADA_TYPE_E mtscada ) ;
  public:
    //<! 
    CLidarAPI(SCADA_TYPE_E scada_e=eScadaNet,LiDAR_TYPE_E mtlidar=eLidarGui);
    ~CLidarAPI() ;
    static CLidarAPI* getInstance() ;
    LiDAR_TYPE_E getLidarType();
    SCADA_TYPE_E getScadaType();

    /*** (1) for receiving commands from up-side controller ***/
    int cmdHandle(SCADA_CMDRPT_U & cmd) ;
    int cmdRprt(SCADA_CMDRPT_U & cmdrcv) ;
    
// added by zhubing 2020.3.19
	//void getMF_Parameter(double &tmp_h, double &tmp_v, double &tmp_Drol, double &tmp_Draw,bool &tmp_mf_falg);
	void setMFParameter(float tmp_mf_num_, bool tmp_mf_flag, float tmp_mf_freq_);

	int setLidarSvrReset(bool tmp_flag);
    /*** (2) for SCADA Hardware communicating ***/
    int openLidar(char *ip );
    int closeLidar(void);
    int pingLidar(void);

    //<! start up a scanning production
    LDRPROG_STAT_E prodStart(LiDAR_RoiProg_S * roids=NULL);

	int getImpVersion_s();

    //<! stop the scanning procedure
    LDRPROG_STAT_E prodStop();
    //<! take a break, stop laser firing or simu
    LDRPROG_STAT_E prodBreak();

    int dtaRecvHandle() ;
    int msgHandle(SCADA_CMDRPT_U mtMsgs) ;
    int msgRecvHandle() ;
    int systemDateTimeUpdate();
    int msgRecv(char * _mt_msgBuff, int mt_msgSize ) ;
	int getGpsMsg(char * _mt_msgBuff, int mt_msgSize);
	int gpsmsgRecv(char * _mt_msgBuff, int mt_msgSize);
    int slwdtaRecvHandle() ;  //慢数据流接收
	int navmsgHandle();//added by zhubing 2020.1.3

    /*** (3) share the ring buffers ***/
    //<! 
    SHOTS_RINGBUF_S * shotsOut() { return &shotsRing_; }
    //<! 
    IMAGE_RINGBUF_S * imageOut() { return imageRing_; }
    //<! 
    MESSG_RINGBUF_S * messgOut() { return &messgRing_; }
    
    CFG_RINGBUF_S * serverCfgRingOut() { return &m_serverCfgRing_ ; } 
    CFG_RINGBUF_S * clientCfgRingOut() { return &m_clientCfgRing_ ; } 
   
    SLOW_RINGBUF_S * slwRingOut() { return &slowRing_ ;}   //慢数据流内存地址
    STRMSG_RINGBUF_S * strMsgRingOut() { return &strRprtRing_ ;}  //字符串消息反馈内存地址
    CLidarConfig * ldrCfgOut() {return s_config; } 
    
    LDRFILE_STAT_LIST * fileCtrlListOut() {return &m_fileCtrlList; }
    /*** (4) .... ***/
    //<! usb camera options
    int usbCmrOpen();
    int usbCmrClose();

    //<! read and write the camera config parameters
    int usbCmrRead(USBCMR_CTRL_S * usbcmrPara);
    int usbCmrWrite(USBCMR_CTRL_S usbcmrPara);
    int usbCmrSetupTrig(USBCMR_TRIG_S usbTrigSetup );
    int usbCmrTrig(USBCMR_TRIG_S usbCmrTrig); // take a picture 

    int usbCameraCtrl(USBCMR_TRIG_S mt_usbCmrPara) ;
    int usbCameraRprt(USBCMR_TRIG_S *_mt_usbCmrPara) ;

    LDRPROG_STAT_E usbCameraScanStart(USBCMR_TRIG_S mt_usbCmrPara) ;
    LDRPROG_STAT_E usbCameraScanStop();


    /*** (5) .... *****/
    //<! used for data header packaging, for CssTask and IDataHandle
    //fileType: 0:字符串  1:二进制
    int lidarLoadCfg(char * fname , int fileType=0 ) ;
    int lidarLoadCfg(CScada * scada, char * fname , int fileType=0) ;

	int lidarDumpCfg();
    
    char * getImpHead() ;
    int packImpHead(char * buf, int size=IMP_HEADER_SIZE) ;
    int packTskRecd(char * buf) ;

    int packConfig() ;  
    int depackConfig(const char * buf) ;
    
    int spinMachine() ;
    int spinCmrTrigger() ;
    int initSlowRing() ;
    int initFastRing() ;

#ifdef BINARY_CONTROL
  public:
    //Advanced Usage
//added by zhubing 2020.4.27
	  typedef enum cmd_set_type {
		  CMD_SET_LIDAR = 0x01,
		  CMD_SET_DATA = 0x02,
		  CMD_SET_CAMERA = 0x03,
	  } cmd_set_;
	  typedef enum cmd_lidar_id_type {
		  SET_LIDAR_BASIC_PARAMETER = 0x13,
		  GET_LIDAR_BASIC_REPORT = 0x14,
		  SET_LIDAR_LASER_PARAMETER = 0x15,
		  GET_LIDAR_LASER_REPORT = 0x16,
		  SET_LIDAR_SCAN_PARAMETER = 0x17,
		  GET_LIDAR_SCAN_REPORT = 0x18,		  
		  GET_LIDAR_DEVICE_REPORT = 0x19,
		  GET_LIDAR_ENVIRONMENT_REPORT = 0x20,
		  GET_LIDAR_CFG_REPORT = 0x21,
		  GET_LIDAR_LIMITS_REPORT = 0x22,
		  SET_TIME_SYNCHRONIZATION = 0x23,
	  } cmd_lidar_id_;
	  typedef enum cmd_camera_id_type {
		  SET_CAMERA_BASIC_PARAMETER = 0x08,
		  GET_CAMERA_BASIC_REPORT = 0x09,
	  } cmd_camera_id_;

	  int case_cmd_set(int tmp_cmd_set, int tmp_cmd_id_set, ss::dev::cmd::star::Packet packet);
	  int cmd_lidar_id_case(int tmp_cmd_id_set, ss::dev::cmd::star::Packet packet);
	  int cmd_camera_id_case(int tmp_cmd_id_set, ss::dev::cmd::star::Packet packet);
	  int lidar_basic_control(ss::dev::cmd::star::Packet packet);
	  int lidar_laser_control(ss::dev::cmd::star::Packet packet);
	  int lidar_scan_control(ss::dev::cmd::star::Packet packet);
	  int lidar_basic_report(ss::dev::cmd::star::Packet &packet);
	  int lidar_laser_report(ss::dev::cmd::star::Packet &packet);
	  int lidar_scan_report(ss::dev::cmd::star::Packet &packet);
	  int lidar_device_report(ss::dev::cmd::star::Packet &packet);
	  int lidar_environment_report(ss::dev::cmd::star::Packet &packet);
	  int lidar_cfg_report(ss::dev::cmd::star::Packet packet);
	  //end added by zhubing 2020.4.27
	  //added by zhubing 2020.5.7
	  int camera_basic_control(ss::dev::cmd::star::Packet packet);
	  int camera_basic_report(ss::dev::cmd::star::Packet &packet);
	  //end added by zhubing 2020.5.7
	  int lidar_limits_report(ss::dev::cmd::star::Packet &packet);
	  //added by zhubing 2020.5.15
	  int lidar_time_synchronization(ss::dev::cmd::star::Packet packet);

	  void set_utc_time(unsigned int tmp_time);
	  unsigned int get_uct_time();
private:
	//added by zhubing 2020.5.13
	ss::dev::cmd::star::Packet          response;
	unsigned int utc_time_;
#endif

public:
    int ctrlParaRprt(SCADA_CONTROL_S *ctrRpt ) ;
    // including enable the dma
    int progCtrl(SCDCMD_PROGRM_S prgCmd);
    int progSetup(SCDCMD_PROGRM_S progPara) ;
    int progRprt(SCDCMD_PROGRM_S * prgRpt);
    
    //device control, add by dengbj
    int envirCtrl(SCDCMD_ENVIR_S envirPara);
    int envirSetup(SCDCMD_ENVIR_S envirPara);
    int envirRprt(SCDCMD_ENVIR_S *envirReport);
    
    int laserCtrl(SCDCMD_LASER_S laserPara);
    int laserSetup(SCDCMD_LASER_S laserPara);
    int laserRprt(SCDCMD_LASER_S *laserReport);
    
    int scanerCtrl(SCDCMD_SCANER_S scanerPara);
    int scanerSetup(SCDCMD_SCANER_S scanerPara);
    int scanerRprt(SCDCMD_SCANER_S *scanRprt);
    int scanerDriver(bool strStat ) ;
    
    int cameraCtrl(SCDCMD_CAMERA_S cameraPara);
    int cameraSetup(SCDCMD_CAMERA_S cameraPara);
    int cameraRprt(SCDCMD_CAMERA_S *cmrRprt);
    int cameraPosTrig();                        // add for nav by dengbj
		    
    int turretCtrl(SCDCMD_TURRET_S turretPara);
    int turretSetup(SCDCMD_TURRET_S turretPara);
    int turretRprt(SCDCMD_TURRET_S *turretReport);
    int turretWaitScaner();
    int turretStatProcess() ;
    
    //<! access the register directly  
    int regWrite(SCDCMD_REGIST_S reg);
    int regRead(SCDCMD_REGIST_S *reg);

    void setStationNbr(unsigned short nbr);
    
    int configRprt(LiDAR_CFGFILE_S *config); //! 配置文件数据返回
 
    LiDAR_DATAINFO_S* dataInfoOut();
    int dataInfoRprt( LiDAR_DATAINFO_S *_mt_Info ) ;
    int dataInfoSetup( LiDAR_DATAINFO_S mt_Info ) ;

    LDRPROG_STAT_E cameraScanStart(SCDCMD_CAMERA_S mt_cmrPara); 
    LDRPROG_STAT_E cameraScanStop(); //! 停止外置相机曝光

    int posCtrl(SCADA_POST_S posPara_ ) ;     //!< pos 数据数据控制
    int posRprt(SCADA_POST_S * _posPara ) ;   //!< pos 数据传输状态返回

    int fpgaStatRprt(SCADA_FPGA_STATE * _fpgaStaPara) ; //!< FPGA 状态监控


    LiDAR_STAT_E getLidarConfigStat( void ) ;  //!<liyp：读取配置文件接受状态


    int impFileStateCtrl(LiDARImp_Action_E impPara ) ;

    int searchUsbRegister() ;
//    #ifdef _WINDOWS
//    //palm 控制模块调用
//    int palmTranCtrl(SCDCMD_PALM_S mt_palmPata_) ;
//    int palmTranRprt(SCDCMD_PALM_S *_mt_palmPara) ;
//    
//    int palmCmrCtrl(SCDCMD_CAMERA_S mt_cmrPara) ;
//    int palmCmrRprt(SCDCMD_CAMERA_S *_mt_cmrState) ;
//
//    int palmStgCtrl(SCDCMD_PALMSTG_S mt_stgPara) ;
//    int palmStgRprt(SCDCMD_PALMSTG_S *_mt_stgPara) ;
//
//    int palmReadRegister(SCDCMD_REGIST_S *_mt_reg) ;
//    int palmWriteRegister(SCDCMD_REGIST_S mt_reg) ;
//    int palmDataDirSetup(char * _mt_Path ) ;
//    int palmDataHandle() ;
//    int palmPing();
//#endif
    unsigned int issFileSizeRprt() ;
    int dmiSetup(SCDCMD_DMI_S mt_para) ;
    int dmiRprt(SCDCMD_DMI_S *_mt_para) ;
    
    int strCmdSent(char *_mt_msg, int mt_size ) ;
    LDRPROG_STAT_E lidarMachStateRprt() { return m_eprogstat ; }

    int askLidarState() ;
    bool imageCollectRprt() ;  //反馈影像采集状态
    bool imageWiFiTransfer() ;

    int scanParaRprt(SCADA_DEFAULTCTR_S *defPara ) ;
    int testImpHeadDump() ;

    int syncTimeCtrl(SCDCMD_TIMSYN_S mtTime) ;

    //int storageSetup(SCADA_DATAFUN_S mtPara) ;
    //int storageCtrl(SCADA_DATAFUN_S mtPara) ;
    //int storageRprt(SCADA_DATAFUN_S *mtPara) ;

    void msgRingHangle(void) ;
    void strMsgHangle(void) ;
    int  msgAutoRprt() ;
    CSvrSocket * getSvrSocket() ;
    void checkDataChannel() ;
    int svcCmdflow() ;
    void initLidar(LiDAR_TYPE_E mtlidar) ;

    void msgSent(unsigned char *mtMsg, int mtMsgSize, LiDAR_MSGID_E mtMsgId ) ;

    int limitParaRprt(SCDCFG_LIMIT_S *mtPara) ; //设备硬件参数限制范围

    void setupSimuPath(char *mtDir);
    int intervalWork();
    void scadaPing();
    bool issFileIsOpen() ;
	bool serialScreenCommandHandle(void *mtComMsg, size_t size, char* mtStrCmd);
	int  sendScada2UartCmd(const unsigned char* cmd, size_t size);
	int  getUartCmdValue(void* cmdStr, size_t size, unsigned char* data);
	int  updateserialScreenInfo();
	void serialScreenStateReportCmd(unsigned char type, void*cmd);  
	void setupLaserStrCmd(char *buf);

public:
	/*zks--- should be in SCADA::state_get()
	void getTemp(float &tempA,float &tempB);
	void setTemp(float tempA,float tempB);
	*/
	bool getDataFlag();
	void setDataFlag(bool flag);
	bool getFastDataFlag();

//added by zhubing 2020.5.20
	//void setMF_Pulse_num(int tmp,bool tmp_flag);

	//added by zhubing 2020.9.4
	float temperatureGet();

 private:

	 float _tempA;
	 float _tempB;
	 bool  tcp_flag;
	 bool  _gps_ask_flag;
#ifndef _WINDOWS
	 std::mutex _gps_flag_lock;
#endif
// added by zhubing 2020.2.15
	 SCDCMD_MF_S _mf_parameter;
	 bool   _mf_flag;
	 float  _m_pulse;
	 int    _position_exposure;
	 bool   _position_exposure_flag;
// added by zhubing 2020.2.17
	 bool  _mf_open_flag;
	 bool  _mf_run_flag;
	 bool  _lidarsvr_reset_flag;
// added by chengyin 2021.3.1
	 bool m_dataflag;

     char sendGroundHeightBuffer[100];
     
};
#endif
  
