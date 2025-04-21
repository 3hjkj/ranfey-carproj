#include "math.h"

struct AutoControlData //0x601 little
{

	union
	{ 
		uint8_t D;
		struct
		{  
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;     
 
	union 
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t RemoteDriveID : 8;
		} bit;
	} MsgData0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t BrakePedalReq : 8;
		} bit;
	} MsgData1;
	union
	{
		uint16_t D;
		struct
		{
			uint16_t AccPedalReq : 16;
		} bit;
	} MsgData23;

	union
	{
		int16_t D;
		struct
		{
			int16_t Steeringangle : 16;
		} bit;
	} MsgData45;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t GearReq : 3;
			uint8_t Indicator_Right : 1;
			uint8_t Indicator_Left : 1;
			uint8_t Reserved : 3;
		} bit;
	} MsgData6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t LiveCount : 8;
		} bit;
	} MsgData7;
};

//0x611 little

struct RemoteConVehData
{

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t EngSpd : 16;
		} bit;
	} MsgData01;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t AECACC : 2;
			uint8_t ATFalt : 2;
			uint8_t ShiftPosition : 4;
		} bit;
	} MsgData2;
	union
	{
		uint8_t D;
		struct
		{
			uint8_t CruiseSwitchonoff : 1;
			uint8_t TCSFailure : 1;
			uint8_t ABSFailure : 1;
			uint8_t ESPQDCACC : 2;
			uint8_t SASfailure : 1;
			uint8_t EPBFailure : 2;
		} bit;
	} MsgData3;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t VehSpdkmph : 16;
		} bit;
	} MsgData45;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t GearReq : 2;
			uint8_t Indicator_Right : 2;
			uint8_t EPSfaild : 1;
			uint8_t RemoteConState : 1;
			uint8_t ComuniFault : 1;
			uint8_t Reserved : 1;
		} bit;
	} MsgData6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t LiveCount : 4;
			uint8_t TurnLightSt : 2;
			uint8_t ControlMode : 2;
		} bit;
	} MsgData7;
};

//0x111 little
struct AutoControlData111
{

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t ACCTarget : 8;
		} bit;
	} MsgData0;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t AEBTarget : 16;
		} bit;
	} MsgData12;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 4;
			uint8_t DecToStop : 1;
			uint8_t Driveoff : 1;
			uint8_t AEBDecAva : 1;
			uint8_t ACCCDDEnble : 1;
		} bit;
	} MsgData3;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t SteeringAngleReqPro : 2;
			uint16_t Reserved : 3;
			uint16_t AccTqReqEnble : 1;
			uint16_t AccTqReq : 10;
		} bit;
	} MsgData45;

	union
	{
		int16_t D;
		struct
		{
			int16_t SteeringAngleReq : 16;
		} bit;
	} MsgData67;
};

//0x112 little
struct AutoControlData112
{

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} MsgData0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t BCMhorns : 1;
			uint8_t Reserved1 : 1;
			uint8_t LKSStatus : 3;
			uint8_t Reserved : 3;
		} bit;
	} MsgData1;
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 1;
			uint8_t Door : 2;
			uint8_t Windows : 2;
			uint8_t Wipes : 3;
		} bit;
	} MsgData2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t RearFogLight : 1;
			uint8_t FrontFogLight : 1;
			uint8_t HeadLight : 1;
			uint8_t LowLight : 2;
			uint8_t SideLamps : 1;
			uint8_t TurnLight : 2;
		} bit;
	} MsgData3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 5;
			uint8_t EPBRequestVaild : 1;
			uint8_t EPBRequest : 2;
		} bit;
	} MsgData4;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 3;
			uint8_t ShiftPositionReqVaid : 1;
			uint8_t ShiftPositionReq : 3;
			uint8_t ShiftPositionNbl : 1;
		} bit;
	} MsgData5;

	union
	{
		int16_t D;
		struct
		{
			int16_t Reserved : 16;
		} bit;
	} MsgData67;
};

//0x208 little
struct EMS_208
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 4;
			uint8_t EMSESPTrqReqVail : 1;
			uint8_t EMSTrqFailure : 1;
			uint8_t EMSTCUTrqReqVail : 1;
			uint8_t EMSCreepINhibit : 1;
		} bit;
	} DATA0;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t EMSFrictionalTrq : 16;
		} bit;
	} DATA12;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t RealEngTorq : 16;
		} bit;
	} DATA34;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t IndicateDriverTrqReq : 16;
		} bit;
	} DATA56;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t LiveCount : 4;
			uint8_t Checksum : 4;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : EMS_258
  Date        : 2019/11/07
  Version     : 1.0.1
  Desciption  : EMS_258

  little
*******************************************************************************/
struct EMS_258
{

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t EMSCruiseControlSt : 2;
			uint8_t EMSClutchPedalSt : 2;
			uint8_t EMSCompressorSt : 1;
			uint8_t EMSConditionIdle : 1;
			uint8_t BrakepedalSt : 2;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t AirTempError : 1;
			uint8_t Reserved1 : 1;
			uint8_t AccpedalErro : 1;
			uint8_t ThrottleValveError : 1; //Temp_Difference_Fault
			uint8_t SpeedErro : 1;
			uint8_t Reserved : 1;
			uint8_t EngFuelCurOff : 1;
			uint8_t EMSKickdown : 1;
		} bit;
	} DATA1;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t EngSpd : 16; //Monomer_UnderVol_Fault
		} bit;
	} DATA23;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t EngThrottlePosition : 8; //TotalVol_Sensor_Fault
		} bit;
	} DATA4;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Accpedal : 8; //TotalVol_Sensor_Fault
		} bit;
	} DATA5;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t EngIntakeAirTem : 8; //Monomer_UnderVol_Fault
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t LiveCount : 4;
			uint8_t Checksum : 4;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : EMS_265
  Date        : 2019/11/08
  Version     : 1.0.1
  Desciption  : EMS_265

  little
*******************************************************************************/

struct EMS_26A
{

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved1 : 1;
			uint8_t EMS_AutoStopReq : 1;
			uint8_t EMS_AutoStartReq : 1;
			uint8_t EMS_PTOpenReq : 1;
			uint8_t Reserved : 4;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t RealAccPedal : 8;
		} bit;
	} DATA4;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t EngTorqMax : 8;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t EngTorqMin : 8;
		} bit;
	} DATA3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved1 : 2;
			uint8_t EngState : 2;
			uint8_t Reserved : 4;
		} bit;
	} DATA5;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t EngTorqueConstant : 8;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t LiveCount : 4;
			uint8_t Checksum : 4;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : EMS_26D
  Date        : 2019/11/08
  Version     : 1.0.1
  Desciption  : EMS_26D

  little
*******************************************************************************/

struct EMS_26D
{

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 7;
			uint8_t AccReqPossible : 1;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA1;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t ECGPOvrd : 1;
			uint16_t QECACC : 2;
			uint16_t EngStatusSTT : 3;
			uint16_t VacuumPressure : 10;
		} bit;
	} DATA23;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved : 16; //VCUFunctionFault
		} bit;
	} DATA45;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 4;
			uint8_t LiveCount : 4;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Checksum : 4;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : EMS_358
  Date        : 2019/11/08
  Version     : 1.0.1
  Desciption  : EMS_358

  little
*******************************************************************************/

struct EMS_358
{

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t CruiseActive : 1;
			uint8_t SSMWarningLamp : 1;
			uint8_t CruiseUnVail : 1;
			uint8_t IdleStopStatus : 1;
			uint8_t CuiseIndicate : 1;
			uint8_t EMSMIL : 2;
			uint8_t EMSSVS : 1;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t FuelConsumpotion : 8;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t EngineType : 4;
			uint8_t RemindGear : 2;
			uint8_t ClearDiagnostInfo : 2;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint16_t TotalOdometerHigh : 8;
		} bit;
	} DATA3;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t TotalOdometerLow : 16;
		} bit;
	} DATA45;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t TargetCruiseSpd : 8;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t PFCcondition : 1;
			uint8_t StartPrompt : 2;
			uint8_t CruiseDistance : 1;
			uint8_t CruiseSwitchSE : 1;
			uint8_t CruiseSwitchCA : 1;
			uint8_t CruiseSwitchReq : 1;
			uint8_t CruiseSwitchOnOff : 1;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : EPBI_21B
  Date        : 2019/11/08
  Version     : 1.0.1
  Desciption  : EPBI_21B

  little
*******************************************************************************/

struct EPBI_21B
{

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t ESPActiveSt : 1;
			uint8_t ESPFunctionSt : 1;
			uint8_t EBDFailState : 1;
			uint8_t ABSfailState : 1;
			uint8_t TCSFailState : 1;
			uint8_t GearHoldReqVail : 1;
			uint8_t GearHoldReq : 1;
			uint8_t ABSActiveStatue : 1;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t EngTrqInc : 8;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t EngTrqDecFast : 8;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t EngTrqDecSlow : 8;
		} bit;
	} DATA3;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t VehicleSpeed : 13;
			uint16_t VehicleSpdVail : 1;
			uint16_t EngTrqIncActive : 1;
			uint16_t EngTrqDecActive : 1;
		} bit;
	} DATA45;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t LiveCount : 4;
			uint8_t Reserved : 4;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Checksum : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : EPBI_27A
  Date        : 2019/11/08
  Version     : 1.0.1
  Desciption  : EPBI_27A

  little
*******************************************************************************/

struct EPBI_27A
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved1 : 4;
			uint8_t ABAVail : 1;
			uint8_t ABAActive : 1;
			uint8_t Reserved : 2;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 6;
			uint8_t PrefillVail : 1;
			uint8_t PrefillActive : 1;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved2 : 2;
			uint8_t BrakeForce : 1;
			uint8_t Reserved1 : 1;
			uint8_t CDDAvail : 1;
			uint8_t Reserved : 3;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 2;
			uint8_t BrakeOverTem : 1;
			uint8_t CDDActive : 1;
			uint8_t VDCActive : 1;
			uint8_t TCSActive : 1;
			uint8_t OnlyABSActive : 1;
			uint8_t Reserved1 : 1;
		} bit;
	} DATA3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved1 : 5;
			uint8_t AWBActive : 1;
			uint8_t AEBdevActive : 1;
			uint8_t Reserved : 1;
		} bit;
	} DATA4;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved2 : 1;
			uint8_t QDCACC : 2;
			uint8_t Reserved1 : 2;
			uint8_t AWBAvail : 1;
			uint8_t AEBAvail : 1;
			uint8_t Reserved : 1;
		} bit;
	} DATA5;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t LiveCount : 4;
			uint8_t Reserved1 : 2;
			uint8_t VehicleStandStill : 1;
			uint8_t Reserved : 1;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Checksum : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : EPBI_27B
  Date        : 2019/11/08
  Version     : 1.0.1
  Desciption  : EPBI_27B

  little
*******************************************************************************/

struct EPBI_27B
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t ESP_TODTrqMaxLimt : 12;
			uint16_t ESP_HDCStatue : 2;
			uint16_t ABS_TODFastOpen : 2;
		} bit;
	} DATA01;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t YawRate : 14;
			uint32_t LongAccel : 10;
			uint32_t LatAccel : 8;
		} bit;
	} DATA2345;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t HBBFuctionSt : 2;
			uint8_t HHCAvail : 1;
			uint8_t HBBActiveSt : 1;
			uint8_t Reserved : 3;
			uint8_t YawRateVail : 1;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Checksum : 4;
			uint8_t LiveCount : 4;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : EPBI_2A3
  Date        : 2019/11/08
  Version     : 1.0.1
  Desciption  : EPBI_2A3

  little
*******************************************************************************/

struct EPBI_2A3
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t ElectPowerConsumption : 8;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t TrqSensonSt : 1;
			uint8_t EPSFaild : 1;
			uint8_t Reserved1 : 2;
			uint8_t APAFeedBack : 3;
			uint8_t Reserved : 1;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved1 : 5;
			uint8_t APAControlFeedBack : 1;
			uint8_t Reserved : 2;
		} bit;
	} DATA3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t SteeringTrq : 8;
		} bit;
	} DATA4;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t LiveCount : 4;
			uint8_t EPS_ConcusAvailSt : 2;
			uint8_t Reserved : 2;
		} bit;
	} DATA5;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved1 : 1;
			uint8_t TrqAssistMode : 3;
			uint8_t Reserved : 4;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Checksum : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : SAS_183
  Date        : 2019/11/08
  Version     : 1.0.1
  Desciption  : SAS_183

  little
*******************************************************************************/

struct SAS_183
{

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t SteeringAngleSpd : 8;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA2;

	union
	{
		int16_t D;
		struct
		{
			int16_t SteeringAngle : 16;

		} bit;
	} DATA34;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t InternalStsFlg : 5;
			uint8_t SASCailrate : 1;
			uint8_t SASFails : 1;
			uint8_t SteeringAngleLev : 1;
		} bit;
	} DATA5;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 3;
			uint8_t LiveCount : 4;
			uint8_t SAS_TrinmmingSt : 1;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Checksum : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : TCU_26B
  Date        : 2019/11/08
  Version     : 1.0.1
  Desciption  : TCU_26B

  little
*******************************************************************************/

struct TCU_26B
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t VehicleSpdVail : 1;
			uint8_t STLKFail : 1;
			uint8_t MLLReq : 1;
			uint8_t TransFailSt : 2;
			uint8_t ReverseControl : 1;
			uint8_t NeuatralControl : 1;
			uint8_t HeavyDecelaration : 1;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t ActGear : 4;
			uint8_t ShiftPosition : 4;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 4; //TorqSpeedUpperLimit
			uint8_t TargetGear : 4;
		} bit;
	} DATA2;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t VehicleSpeed : 13;
			uint16_t ShiftPositionVaid : 1;
			uint16_t ActGearValid : 1;
			uint16_t TargetGearValid : 1;
		} bit;
	} DATA34;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t LiveCount : 4;
			uint16_t TurbinSpeed : 12;
		} bit;
	} DATA56;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Checksum : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : ACM_24C
  Date        : 2019/11/08
  Version     : 1.0.1
  Desciption  : ACM_24C

  little
*******************************************************************************/

struct ACM_24C
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 4;
			uint8_t ActPRNDSt : 4;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t ACMActSt : 1; //TorqSpeedUpperLimit
			uint8_t Reserved : 7;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 4;
			uint8_t APADriveInterrupt : 2;
			uint8_t APAReqEnble : 2;
		} bit;
	} DATA4;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA5;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t LiveCount : 4;
			uint8_t Reserved : 4;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Checksum : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : EPS_2A4
  Date        : 2019/11/08
  Version     : 1.0.1
  Desciption  : EPS_2A4

  little
*******************************************************************************/

struct EPS_2A4
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA0;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t EPS_SteeringAngle : 14;
			uint16_t EPS_LkaHandsonSt : 2;
		} bit;
	} DATA12;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t LiveCount : 4;
			uint32_t Reserved : 3;
			uint32_t MeasureTorsionBarTrqVald : 1;
			uint32_t EPS_SteeringAngleSpeed : 10;
			uint32_t AvailStatus : 3;
			uint32_t MeasureTorsionBarTrq : 11;
		} bit;
	} DATA3456;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Checksum : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : EPBI_20B
  Date        : 2019/11/08
  Version     : 1.0.1
  Desciption  : EPBI_20B

  little
*******************************************************************************/

struct EPBI_20B
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t RRWheelSpeed : 13;
			uint16_t RRWheelSpeedDirection : 2;
			uint16_t RRWheelSpeedVaild : 1;
		} bit;
	} DATA01;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t RLWheelSpeed : 13;
			uint16_t RLWheelSpeedDirection : 2;
			uint16_t RLWheelSpeedVaild : 1;
		} bit;
	} DATA23;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t FRWheelSpeed : 13;
			uint16_t FRWheelSpeedDirection : 2;
			uint16_t FRWheelSpeedVaild : 1;
		} bit;
	} DATA45;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t FLWheelSpeed : 13;
			uint16_t FLWheelSpeedDirection : 2;
			uint16_t FLWheelSpeedVaild : 1;
		} bit;
	} DATA67;
};

/*******************************************************************************

  File Name   : TCU_23B
  Date        : 2019/11/08
  Version     : 1.0.1
  Desciption  : TCU_23B

  little
*******************************************************************************/

struct TCU_23B
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 2;
			uint8_t ATClutchStatus : 2;
			uint8_t ATShiftProgressVaild : 1;
			uint8_t ATShiftProgress : 1;
			uint8_t TrqReqModel : 2;
		} bit;
	} DATA0;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t EngTrqReq : 16;
		} bit;
	} DATA12;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t TrqLimitationReq : 16;
		} bit;
	} DATA34;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t TurbinSpeed : 8;
		} bit;
	} DATA5;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t LiveCount : 4;
			uint8_t Reserved : 1;
			uint8_t StopStartInhibit : 1;
			uint8_t ReadyForAutoStop : 1;
			uint8_t OpenPowertrain : 1;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Checksum : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : EPBI_25B
  Date        : 2019/11/08
  Version     : 1.0.1
  Desciption  : EPBI_25B

  little
*******************************************************************************/

struct EPBI_25B
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t MasCyPressure : 12;
			uint16_t Reserved1 : 2;
			uint16_t BrakeLightonReq : 2;
		} bit;
	} DATA01;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t SpeedFLDrection : 2;
			uint8_t AutoHoldActive : 1;
			uint8_t AutoHoldVaild : 1;
			uint8_t ECADActive : 1;
			uint8_t ECDAVaild : 1;
			uint8_t AutoHoldStandby : 1;
			uint8_t MasCyPressureVaild : 1;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t LiveCount : 4;
			uint8_t Checksum : 4;
		} bit;
	} DATA3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t RLWheelSpeedPulse : 8;
		} bit;
	} DATA4;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t FRWheelSpeedPulse : 8;
		} bit;
	} DATA5;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t FLWheelSpeedPulse : 8;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t RRWheelSpeedPulse : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : EPBI_27C
  Date        : 2019/11/08
  Version     : 1.0.1
  Desciption  : EPBI_27C

  little
*******************************************************************************/

struct EPBI_27C
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved1 : 2;
			uint8_t LeftWheelDistanceVaild : 1;
			uint8_t Reserved : 5;
		} bit;
	} DATA0;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t LeftWheelDistanceTimestamp : 16;
		} bit;
	} DATA12;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved1 : 2;
			uint8_t RightWheelDistanceVaild : 1;
			uint8_t Reserved : 5;
		} bit;
	} DATA4;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t RightWheelDistanceTimestamp : 16;
		} bit;
	} DATA56;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : TCU_33B
  Date        : 2019/11/08
  Version     : 1.0.1
  Desciption  : TCU_33B

  little
*******************************************************************************/

struct TCU_33B
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved1 : 4;
			uint8_t ODLamp : 1;
			uint8_t WNTLamp : 1;
			uint8_t Reserved : 1;
			uint8_t PWRLamp : 1;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t ClearDiagnostInfo : 2;
			uint8_t Reserved : 6;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 4;
			uint8_t DriveMode : 4;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t SlopeRatio : 8;
		} bit;
	} DATA3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 7;
			uint8_t FailureLamp : 1;
		} bit;
	} DATA4;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t TransFluidTemp : 8;
		} bit;
	} DATA5;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 4;
			uint8_t GearForDisplay : 4;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : EPBI_259
  Date        : 2019/11/08
  Version     : 1.0.1
  Desciption  : EPBI_259

  little
*******************************************************************************/

struct EPBI_259
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved1 : 4;
			uint8_t EPBIStatus : 2;
			uint8_t EPBIFailSt : 2;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved1 : 1;
			uint8_t AchievedClampForce : 5;
			uint8_t Reserved : 2;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 2;
			uint8_t EPBSwPosition : 2;
			uint8_t EPBSwPositionVaild : 1;
			uint8_t Reserved1 : 3;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 1;
			uint8_t EPBClutchSensor : 7;
		} bit;
	} DATA3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 4;
			uint8_t FunctionLamp : 2;
			uint8_t BrakeLightOnReq : 2;
		} bit;
	} DATA4;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t TextDisplay : 4;
			uint8_t EPBFailureLamp : 2;
			uint8_t Reserved : 2;
		} bit;
	} DATA5;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t LiveCount : 4;
			uint8_t Reserved : 4;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Checksum : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : EMS_268
  Date        : 2019/11/08
  Version     : 1.0.1
  Desciption  : EMS_268

  little
*******************************************************************************/

struct EMS_268
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t IndicatorTrqMax : 8;
			uint32_t VehicleSpeed : 13;
			uint32_t VehicleSpdVail : 1;
			uint32_t IntakeMainFoldPressure : 10;
		} bit;
	} DATA0123;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t IndicatorTrqMin : 8;
		} bit;
	} DATA4;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t EngTrqTargetWithoutTCU : 8;
		} bit;
	} DATA5;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t EngTorqueConstant : 8;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t LiveCount : 4;
			uint8_t IntakeMainFoldPresVaild : 1;
			uint8_t IGNPosition : 1;
			uint8_t EngStatus : 2;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : EMS_278
  Date        : 2019/11/08
  Version     : 1.0.1
  Desciption  : EMS_278

  little
*******************************************************************************/

struct EMS_278
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t EngStartUpEnble : 1;
			uint8_t ImmoCanfig : 1;
			uint8_t ImmoFeedBack : 2;
			uint8_t EngLimpHome : 1;
			uint8_t DrivingCycle : 1;
			uint8_t WarmUpCycle : 1;
			uint8_t CatalyWarmUp : 1;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t CoolTempError : 1;
			uint8_t BatteryVolError : 1;
			uint8_t Reserved : 1;
			uint8_t GeneralMin : 1;
			uint8_t GeneralMax : 1;
			uint8_t IgnitionCycleCount : 1;
			uint8_t Reserved1 : 2;
		} bit;
	} DATA3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t IdleRefSpeed : 8;
		} bit;
	} DATA4;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t AltitudeFactor : 8;
		} bit;
	} DATA5;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t EngCoolTemp : 8;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t BatteryVol : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : EMS_298
  Date        : 2019/11/08
  Version     : 1.0.1
  Desciption  : EMS_298

  little
*******************************************************************************/

struct EMS_298
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t ASLTargetSpeed : 8;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t ASLControlSt : 2;
			uint8_t ASLExceedTarSpeed : 1;
			uint8_t ASLFault : 1;
			uint8_t Neutralgear : 2;
			uint8_t ChargeLightSt : 1;
			uint8_t Reserved : 1;
		} bit;
	} DATA3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 5;
			uint8_t RemindGear : 3;
		} bit;
	} DATA4;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA5;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : Radar_Yaw130
  Date        : 2019/11/11
  Version     : 1.0.1
  Desciption  : Radar_Yaw130

  little
*******************************************************************************/

struct Radar_Yaw130
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t IMUYawRtPri : 13;
			uint16_t Reserved : 3;
		} bit;
	} DATA01;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved1 : 5;
			uint8_t IMUYawRtPriVail : 1;
			uint8_t Reserved : 2;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 5;
		} bit;
	} DATA4;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA5;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : Radar_Spd3E9
  Date        : 2019/11/11
  Version     : 1.0.1
  Desciption  : Radar_Spd3E9

  little
*******************************************************************************/

struct Radar_Spd3E9
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t VehicleSpeed : 15;
			uint16_t VehicleSpdVail : 1;
		} bit;
	} DATA01;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA3;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t VehicleSpeed : 15;
			uint16_t VehicleSpdVail : 1;
		} bit;
	} DATA45;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : Radar_RTS1A740
  Date        : 2019/11/11
  Version     : 1.0.1
  Desciption  : Radar_RTS1A740

  little
*******************************************************************************/

struct Radar_RTS1A740
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t L_LongRel : 12;
			uint32_t L_LatRel : 12;
			uint32_t TrackID : 8;
		} bit;
	} DATA0123;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 15;
			uint16_t Reserved : 1;
		} bit;
	} DATA45;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Status : 4;
			uint16_t A_LongObj : 8;
			uint16_t DetectionSenson : 2;
			uint16_t LiveCount : 2;
		} bit;
	} DATA67;
};

/*******************************************************************************

  File Name   : Radar_RTS1B741
  Date        : 2019/11/11
  Version     : 1.0.1
  Desciption  : Radar_RTS1B741

  little
*******************************************************************************/

struct Radar_RTS1B741
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t V_LongObj : 12;
			uint32_t V_LatObj : 12;
			uint32_t Reserved : 8;
		} bit;
	} DATA0123;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA4;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 4;
			uint16_t A_LatObj : 8;
			uint16_t Movement : 4;
		} bit;
	} DATA56;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 6;
			uint8_t LiveCount : 2;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : Radar_RTS1C742
  Date        : 2019/11/11
  Version     : 1.0.1
  Desciption  : Radar_RTS1C742

  little
*******************************************************************************/

struct Radar_RTS1C742
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Width : 5;
			uint8_t Reserved : 3;
		} bit;
	} DATA0;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved : 16;
		} bit;
	} DATA12;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 7;
			uint16_t ObjectClass : 4;
			uint16_t Reserved : 5;
		} bit;
	} DATA34;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA5;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 6;
			uint16_t VisTrkID : 4;
			uint16_t Reserved : 4;
			uint16_t LiveCount : 2;
		} bit;
	} DATA67;
};

/*******************************************************************************

  File Name   : Radar_RTS2A745
  Date        : 2019/11/11
  Version     : 1.0.1
  Desciption  : Radar_RTS2A745

  little
*******************************************************************************/

struct Radar_RTS2A745
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t L_LongRel : 12;
			uint32_t L_LatRel : 12;
			uint32_t TrackID : 8;
		} bit;
	} DATA0123;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 15;
			uint16_t Reserved : 1;
		} bit;
	} DATA45;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Status : 4;
			uint16_t A_LongObj : 8;
			uint16_t DetectionSenson : 2;
			uint16_t LiveCount : 2;
		} bit;
	} DATA67;
};

/*******************************************************************************

  File Name   : Radar_RTS2B746
  Date        : 2019/11/11
  Version     : 1.0.1
  Desciption  : Radar_RTS2B746

  little
*******************************************************************************/

struct Radar_RTS2B746
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t V_LongObj : 12;
			uint32_t V_LatObj : 12;
			uint32_t Reserved : 8;
		} bit;
	} DATA0123;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA4;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 4;
			uint16_t A_LatObj : 8;
			uint16_t Movement : 4;
		} bit;
	} DATA56;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 6;
			uint8_t LiveCount : 2;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : Radar_RTS2C747
  Date        : 2019/11/11
  Version     : 1.0.1
  Desciption  : Radar_RTS2C747

  little
*******************************************************************************/

struct Radar_RTS2C747
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Width : 5;
			uint8_t Reserved : 3;
		} bit;
	} DATA0;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved : 16;
		} bit;
	} DATA12;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 7;
			uint16_t ObjectClass : 4;
			uint16_t Reserved : 5;
		} bit;
	} DATA34;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA5;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 6;
			uint16_t VisTrkID : 4;
			uint16_t Reserved : 4;
			uint16_t LiveCount : 2;
		} bit;
	} DATA67;
};

/*******************************************************************************

  File Name   : Radar_RTS3A74A
  Date        : 2019/11/11
  Version     : 1.0.1
  Desciption  : Radar_RTS3A74A

  little
*******************************************************************************/

struct Radar_RTS3A74A
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t L_LongRel : 12;
			uint32_t L_LatRel : 12;
			uint32_t TrackID : 8;
		} bit;
	} DATA0123;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 15;
			uint16_t Reserved : 1;
		} bit;
	} DATA45;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Status : 4;
			uint16_t A_LongObj : 8;
			uint16_t DetectionSenson : 2;
			uint16_t LiveCount : 2;
		} bit;
	} DATA67;
};

/*******************************************************************************

  File Name   : Radar_RTS3B74B
  Date        : 2019/11/11
  Version     : 1.0.1
  Desciption  : Radar_RTS3B74B


*******************************************************************************/

struct Radar_RTS3B74B
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t V_LongObj : 12;
			uint32_t V_LatObj : 12;
			uint32_t Reserved : 8;
		} bit;
	} DATA0123;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA4;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 4;
			uint16_t A_LatObj : 8;
			uint16_t Movement : 4;
		} bit;
	} DATA56;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 6;
			uint8_t LiveCount : 2;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : Radar_RTS3C74C
  Date        : 2019/11/11
  Version     : 1.0.1
  Desciption  : Radar_RTS3C74C

  little
*******************************************************************************/

struct Radar_RTS3C74C
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Width : 5;
			uint8_t Reserved : 3;
		} bit;
	} DATA0;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved : 16;
		} bit;
	} DATA12;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 7;
			uint16_t ObjectClass : 4;
			uint16_t Reserved : 5;
		} bit;
	} DATA34;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA5;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 6;
			uint16_t VisTrkID : 4;
			uint16_t Reserved : 4;
			uint16_t LiveCount : 2;
		} bit;
	} DATA67;
};

/*******************************************************************************

  File Name   : Radar_RTS4A74F
  Date        : 2019/11/11
  Version     : 1.0.1
  Desciption  : Radar_RTS4A74F

  little
*******************************************************************************/

struct Radar_RTS4A74F
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t L_LongRel : 12;
			uint32_t L_LatRel : 12;
			uint32_t TrackID : 8;
		} bit;
	} DATA0123;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 15;
			uint16_t Reserved : 1;
		} bit;
	} DATA45;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Status : 4;
			uint16_t A_LongObj : 8;
			uint16_t DetectionSenson : 2;
			uint16_t LiveCount : 2;
		} bit;
	} DATA67;
};

/*******************************************************************************

  File Name   : Radar_RTS4B750
  Date        : 2019/11/11
  Version     : 1.0.1
  Desciption  : Radar_RTS4B750

  little
*******************************************************************************/

struct Radar_RTS4B750
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t V_LongObj : 12;
			uint32_t V_LatObj : 12;
			uint32_t Reserved : 8;
		} bit;
	} DATA0123;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA4;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 4;
			uint16_t A_LatObj : 8;
			uint16_t Movement : 4;
		} bit;
	} DATA56;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 6;
			uint8_t LiveCount : 2;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : Radar_RTS4C751
  Date        : 2019/11/11
  Version     : 1.0.1
  Desciption  : Radar_RTS4C751

  little
*******************************************************************************/

struct Radar_RTS4C751
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Width : 5;
			uint8_t Reserved : 3;
		} bit;
	} DATA0;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved : 16;
		} bit;
	} DATA12;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 7;
			uint16_t ObjectClass : 4;
			uint16_t Reserved : 5;
		} bit;
	} DATA34;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA5;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 6;
			uint16_t VisTrkID : 4;
			uint16_t Reserved : 4;
			uint16_t LiveCount : 2;
		} bit;
	} DATA67;
};

/*******************************************************************************

  File Name   : MSG_CANDATA
  Date        : 2019/11/21
  Version     : 1.0.1
  Desciption  : MSG_CANDATA

  little
*******************************************************************************/

struct MSG_CANDATA
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Width : 5;
			uint8_t Reserved : 3;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA4;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA5;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t LiveCount : 8;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : Radar_CANDATA
  Date        : 2019/11/21
  Version     : 1.0.1
  Desciption  : Radar_CANDATA

  little
*******************************************************************************/

struct Radar_CANDATA
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Width : 5;
			uint8_t Reserved : 3;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA4;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA5;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t LiveCount : 8;
		} bit;
	} DATA6;

	union
	{
		uint64_t D;
		struct
		{
			uint64_t data7 : 8;
			uint64_t data6 : 8;
			uint64_t data5 : 8;
			uint64_t data4 : 8;
			uint64_t data3 : 8;
			uint64_t data2 : 8;
			uint64_t data1 : 8;
			uint64_t data0 : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : Plat_CANDATA
  Date        : 2019/11/21
  Version     : 1.0.1
  Desciption  : Plat_CANDATA

  little
*******************************************************************************/

struct Plat_CANDATA
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Width : 5;
			uint8_t Reserved : 3;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA4;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA5;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t LiveCount : 8;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : Radar_RT1A760
  Date        : 2019/11/28
  Version     : 1.0.1
  Desciption  : Radar_RT1A760

  little
*******************************************************************************/

struct Radar_RT1A760
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t L_LongObj : 12;
			uint32_t V_LongObj : 12;
			uint32_t Reserved : 8;
		} bit;
	} DATA012;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t L_LatObj : 12;
			uint32_t V_LatObj : 12;
			uint32_t Reserved : 8;
		} bit;
	} DATA345;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t A_LongObj : 8;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 4;
			uint8_t DetectionSenson : 2;
			uint8_t LiveCount : 2;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : Radar_RT1B761
  Date        : 2019/11/28
  Version     : 1.0.1
  Desciption  : Radar_RT1B761

  little
*******************************************************************************/

struct Radar_RT1B761
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t TrackID : 8;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 4;
			uint8_t Status : 4;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA4;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 4;
			uint16_t A_LatObj : 8;
			uint16_t Movement : 4;
		} bit;
	} DATA56;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 6;
			uint8_t LiveCount : 2;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : Radar_RT1C762
  Date        : 2019/11/28
  Version     : 1.0.1
  Desciption  : Radar_RT1C762

  little
*******************************************************************************/

struct Radar_RT1C762
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Width : 5;
			uint8_t Reserved : 3;
		} bit;
	} DATA0;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved : 16;
		} bit;
	} DATA12;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 7;
			uint16_t ObjectClass : 4;
			uint16_t Reserved : 5;
		} bit;
	} DATA34;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA5;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 6;
			uint16_t VisTrkID : 4;
			uint16_t Reserved : 4;
			uint16_t LiveCount : 2;
		} bit;
	} DATA67;
};

/*******************************************************************************

  File Name   : Radar_RT2A765
  Date        : 2019/11/28
  Version     : 1.0.1
  Desciption  : Radar_RT2A765

  little
*******************************************************************************/

struct Radar_RT2A765
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t L_LongObj : 12;
			uint32_t V_LongObj : 12;
			uint32_t Reserved : 8;
		} bit;
	} DATA012;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t L_LatObj : 12;
			uint32_t V_LatObj : 12;
			uint32_t Reserved : 8;
		} bit;
	} DATA345;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t A_LongObj : 8;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 4;
			uint8_t DetectionSenson : 2;
			uint8_t LiveCount : 2;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : Radar_RT2B766
  Date        : 2019/11/28
  Version     : 1.0.1
  Desciption  : Radar_RT2B766

  little
*******************************************************************************/

struct Radar_RT2B766
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t TrackID : 8;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 4;
			uint8_t Status : 4;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA4;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 4;
			uint16_t A_LatObj : 8;
			uint16_t Movement : 4;
		} bit;
	} DATA56;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 6;
			uint8_t LiveCount : 2;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : Radar_RT2C767
  Date        : 2019/11/28
  Version     : 1.0.1
  Desciption  : Radar_RT2C767

  little
*******************************************************************************/

struct Radar_RT2C767
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Width : 5;
			uint8_t Reserved : 3;
		} bit;
	} DATA0;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved : 16;
		} bit;
	} DATA12;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 7;
			uint16_t ObjectClass : 4;
			uint16_t Reserved : 5;
		} bit;
	} DATA34;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA5;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 6;
			uint16_t VisTrkID : 4;
			uint16_t Reserved : 4;
			uint16_t LiveCount : 2;
		} bit;
	} DATA67;
};

/*******************************************************************************

  File Name   : Radar_RT3A76A
  Date        : 2019/11/28
  Version     : 1.0.1
  Desciption  : Radar_RT3A76A

  little
*******************************************************************************/

struct Radar_RT3A76A
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t L_LongObj : 12;
			uint32_t V_LongObj : 12;
			uint32_t Reserved : 8;
		} bit;
	} DATA012;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t L_LatObj : 12;
			uint32_t V_LatObj : 12;
			uint32_t Reserved : 8;
		} bit;
	} DATA345;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t A_LongObj : 8;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 4;
			uint8_t DetectionSenson : 2;
			uint8_t LiveCount : 2;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : Radar_RT3B76B
  Date        : 2019/11/28
  Version     : 1.0.1
  Desciption  : Radar_RT3B76B

  little
*******************************************************************************/

struct Radar_RT3B76B
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t TrackID : 8;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 4;
			uint8_t Status : 4;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA4;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 4;
			uint16_t A_LatObj : 8;
			uint16_t Movement : 4;
		} bit;
	} DATA56;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 6;
			uint8_t LiveCount : 2;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : Radar_RT3C76C
  Date        : 2019/11/28
  Version     : 1.0.1
  Desciption  : Radar_RT3C76C

  little
*******************************************************************************/

struct Radar_RT3C76C
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Width : 5;
			uint8_t Reserved : 3;
		} bit;
	} DATA0;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved : 16;
		} bit;
	} DATA12;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 7;
			uint16_t ObjectClass : 4;
			uint16_t Reserved : 5;
		} bit;
	} DATA34;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA5;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 6;
			uint16_t VisTrkID : 4;
			uint16_t Reserved : 4;
			uint16_t LiveCount : 2;
		} bit;
	} DATA67;
};

/*******************************************************************************

  File Name   : Radar_RT4A76F
  Date        : 2019/11/28
  Version     : 1.0.1
  Desciption  : Radar_RT4A76F

  little
*******************************************************************************/

struct Radar_RT4A76F
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t L_LongObj : 12;
			uint32_t V_LongObj : 12;
			uint32_t Reserved : 8;
		} bit;
	} DATA012;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t L_LatObj : 12;
			uint32_t V_LatObj : 12;
			uint32_t Reserved : 8;
		} bit;
	} DATA345;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t A_LongObj : 8;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 4;
			uint8_t DetectionSenson : 2;
			uint8_t LiveCount : 2;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : Radar_RT4B770
  Date        : 2019/11/28
  Version     : 1.0.1
  Desciption  : Radar_RT4B770

  little
*******************************************************************************/

struct Radar_RT4B770
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t TrackID : 8;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 4;
			uint8_t Status : 4;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA4;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 4;
			uint16_t A_LatObj : 8;
			uint16_t Movement : 4;
		} bit;
	} DATA56;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 6;
			uint8_t LiveCount : 2;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : Radar_RT4C771
  Date        : 2019/11/28
  Version     : 1.0.1
  Desciption  : Radar_RT4C771

  little
*******************************************************************************/

struct Radar_RT4C771
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Width : 5;
			uint8_t Reserved : 3;
		} bit;
	} DATA0;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved : 16;
		} bit;
	} DATA12;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 7;
			uint16_t ObjectClass : 4;
			uint16_t Reserved : 5;
		} bit;
	} DATA34;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA5;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 6;
			uint16_t VisTrkID : 4;
			uint16_t Reserved : 4;
			uint16_t LiveCount : 2;
		} bit;
	} DATA67;
};

/*******************************************************************************

  File Name   : Radar_RT5A774
  Date        : 2019/11/28
  Version     : 1.0.1
  Desciption  : Radar_RT5A774

  little
*******************************************************************************/

struct Radar_RT5A774
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t L_LongObj : 12;
			uint32_t V_LongObj : 12;
			uint32_t Reserved : 8;
		} bit;
	} DATA012;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t L_LatObj : 12;
			uint32_t V_LatObj : 12;
			uint32_t Reserved : 8;
		} bit;
	} DATA345;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t A_LongObj : 8;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 4;
			uint8_t DetectionSenson : 2;
			uint8_t LiveCount : 2;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : Radar_RT5B775
  Date        : 2019/11/28
  Version     : 1.0.1
  Desciption  : Radar_RT5B775

  little
*******************************************************************************/

struct Radar_RT5B775
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t TrackID : 8;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 4;
			uint8_t Status : 4;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA4;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 4;
			uint16_t A_LatObj : 8;
			uint16_t Movement : 4;
		} bit;
	} DATA56;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 6;
			uint8_t LiveCount : 2;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : Radar_RT5C776
  Date        : 2019/11/28
  Version     : 1.0.1
  Desciption  : Radar_RT5C776

  little
*******************************************************************************/

struct Radar_RT5C776
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Width : 5;
			uint8_t Reserved : 3;
		} bit;
	} DATA0;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved : 16;
		} bit;
	} DATA12;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 7;
			uint16_t ObjectClass : 4;
			uint16_t Reserved : 5;
		} bit;
	} DATA34;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA5;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 6;
			uint16_t VisTrkID : 4;
			uint16_t Reserved : 4;
			uint16_t LiveCount : 2;
		} bit;
	} DATA67;
};

/*******************************************************************************

  File Name   : Radar_RT6A779
  Date        : 2019/11/28
  Version     : 1.0.1
  Desciption  : Radar_RT6A779

  little
*******************************************************************************/

struct Radar_RT6A779
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t L_LongObj : 12;
			uint32_t V_LongObj : 12;
			uint32_t Reserved : 8;
		} bit;
	} DATA012;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t L_LatObj : 12;
			uint32_t V_LatObj : 12;
			uint32_t Reserved : 8;
		} bit;
	} DATA345;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t A_LongObj : 8;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 4;
			uint8_t DetectionSenson : 2;
			uint8_t LiveCount : 2;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : Radar_RT6B77A
  Date        : 2019/11/28
  Version     : 1.0.1
  Desciption  : Radar_RT6B77A

  little
*******************************************************************************/

struct Radar_RT6B77A
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t TrackID : 8;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 4;
			uint8_t Status : 4;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA4;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 4;
			uint16_t A_LatObj : 8;
			uint16_t Movement : 4;
		} bit;
	} DATA56;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 6;
			uint8_t LiveCount : 2;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : Radar_RT6C77B
  Date        : 2019/11/28
  Version     : 1.0.1
  Desciption  : Radar_RT6C77B

  little
*******************************************************************************/

struct Radar_RT6C77B
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Width : 5;
			uint8_t Reserved : 3;
		} bit;
	} DATA0;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved : 16;
		} bit;
	} DATA12;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 7;
			uint16_t ObjectClass : 4;
			uint16_t Reserved : 5;
		} bit;
	} DATA34;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA5;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved1 : 6;
			uint16_t VisTrkID : 4;
			uint16_t Reserved : 4;
			uint16_t LiveCount : 2;
		} bit;
	} DATA67;
};

/*******************************************************************************

  File Name   : RTK_324
  Date        : 2020/04/02
  Version     : 1.0.1
  Desciption  : RTK_324

  little
*******************************************************************************/

struct RTK_324
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t Poslat : 32;
		} bit;
	} DATA0123;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t PosLong : 32;
		} bit;
	} DATA4567;
};

/*******************************************************************************

  File Name   : RTK_32A
  Date        : 2020/04/02
  Version     : 1.0.1
  Desciption  : RTK_32A

  little
*******************************************************************************/

struct RTK_32A
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t AngleHeading : 16;
		} bit;
	} DATA01;

	union
	{
		int16_t D;
		struct
		{
			int16_t AnglePitch : 16;
		} bit;
	} DATA23;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t AngleRoll : 16;
		} bit;
	} DATA45;
};

/*******************************************************************************

  File Name   : RTK_320
  Date        : 2020/04/02
  Version     : 1.0.1
  Desciption  : RTK_320

  little
*******************************************************************************/

struct RTK_320
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t GpsWeek : 16;
		} bit;
	} DATA01;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t GpsTime : 32;
		} bit;
	} DATA2345;
};

/*******************************************************************************

  File Name   : RTK_323
  Date        : 2020/04/02
  Version     : 1.0.1
  Desciption  : RTK_323

  little
*******************************************************************************/

struct RTK_323
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Sys_State : 8;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t GpsNumStats : 8;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t StateStatus : 8;
		} bit;
	} DATA2;
};

/*******************2020.07.30************************/
/*******************************************************************************

  File Name   : RTK_329
  Date        : 2020/07/30
  Version     : 1.0.1
  Desciption  : RTK_329

  little
*******************************************************************************/

struct RTK_329
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		int32_t D;
		struct
		{
			int32_t AccelX : 20;
			int32_t Reserved : 12;
		} bit;
	} DATA0123;

	union
	{
		int32_t D;
		struct
		{
			int32_t Reserved : 12;
			int32_t AccelY : 20;
		} bit;
	} DATA1234;

	union
	{
		int32_t D;
		struct
		{
			int8_t Reserved : 8;
			int32_t AccelZ : 20;
			int8_t Reserved1 : 4;
		} bit;
	} DATA4567;
};


/*******************************************************************************

  File Name   : RTK_327
  Date        : 2021/12/02
  Version     : 1.0.1
  Desciption  : RTK_327

  little
*******************************************************************************/

struct RTK_327
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		int32_t D;
		struct
		{
			int32_t VelE : 16;
			int32_t VelN : 16;
		} bit;
	} DATA0123;

	union
	{
		int32_t D;
		struct
		{
			int32_t VelU : 16;
			int32_t Vel : 16;
		} bit;
	} DATA4567;
};

/*******************************************************************************

  File Name   : RTK_325
  Date        : 2021/12/02
  Version     : 1.0.1
  Desciption  : RTK_325

  little
*******************************************************************************/

struct RTK_325
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		int32_t D;
		struct
		{
			int32_t PosAlt : 32;
		} bit;
	} DATA0123;

	union
	{
		int32_t D;
		struct
		{
			int32_t Reserved : 16;
		} bit;
	} DATA4567;
};
/*******************2020.04.18************************/
/*******************************************************************************

  File Name   : Radar_VIS6A4
  Date        : 2020/04/17
  Version     : 1.0.1
  Desciption  : Radar_VIS6A4

  little
*******************************************************************************/

struct Radar_VIS6A4
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t lane_left_type : 4;
			uint8_t Reserved : 4;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t lane_right_type : 4;
			uint8_t Reserved : 4;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t LaneLeftNeigLKACON : 2;
			uint8_t Reserved : 6;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved1 : 4;
			uint8_t LaneLeftIndLKACON : 2;
			uint8_t Reserved : 2;
		} bit;
	} DATA3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t LaneRightNeigLKACON : 2;
			uint8_t Reserved : 6;
		} bit;
	} DATA4;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved1 : 4;
			uint8_t LaneRightIndLKACON : 2;
			uint8_t Reserved : 2;
		} bit;
	} DATA5;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t lane_left_neig_type : 4;
			uint8_t lane_right_neig_type : 4;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t LaneChange : 2;
			uint8_t Reserved : 6;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : Radar_VIS6A8
  Date        : 2020/04/17
  Version     : 1.0.1
  Desciption  : Radar_VIS6A8

  little
*******************************************************************************/

struct Radar_VIS6A8
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t lane_right_neig_a0 : 12;
			uint32_t LaneRightNeigA2 : 12;
			uint32_t Reserved : 8;
		} bit;
	} DATA012;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t lane_right_neig_col : 2;
			uint16_t LaneRightNeigA1 : 11;
			uint16_t Reserved : 3;
		} bit;
	} DATA34;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t Reserved1 : 5;
			uint32_t LaneRightNeigRange : 9;
			uint32_t LaneRightNeigA3 : 16;
			uint32_t Reserved : 2;
		} bit;
	} DATA4567;
};
/*******************************************************************************

  File Name   : Radar_VIS6A9
  Date        : 2020/04/17
  Version     : 1.0.1
  Desciption  : Radar_VIS6A9


*******************************************************************************/

struct Radar_VIS6A9
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t lane_left_neig_a0 : 12;
			uint32_t LaneLeftNeigA2 : 12;
			uint32_t Reserved : 8;
		} bit;
	} DATA012;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t lane_left_neig_col : 2;
			uint16_t LaneLeftNeigA1 : 11;
			uint16_t Reserved : 3;
		} bit;
	} DATA34;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t Reserved1 : 5;
			uint32_t LaneLeftNeigRange : 9;
			uint32_t LaneLeftNeigA3 : 16;
			uint32_t Reserved : 2;
		} bit;
	} DATA4567;
};
/*******************************************************************************

  File Name   : Radar_VIS6AB
  Date        : 2020/04/17
  Version     : 1.0.1
  Desciption  : Radar_VIS6AB

  little
*******************************************************************************/

struct Radar_VIS6AB
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t lane_right_indv_a0 : 12;
			uint32_t LaneRightIndvA2 : 12;
			uint32_t Reserved : 8;
		} bit;
	} DATA012;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t lane_right_indv_col : 2;
			uint16_t LaneRightIndvA1 : 11;
			uint16_t Reserved : 3;
		} bit;
	} DATA34;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t Reserved1 : 5;
			uint32_t LaneRightIndvRange : 9;
			uint32_t LaneRightIndvA3 : 16;
			uint32_t Reserved : 2;
		} bit;
	} DATA4567;
};
/*******************************************************************************

  File Name   : Radar_VIS6AC
  Date        : 2020/04/17
  Version     : 1.0.1
  Desciption  : Radar_VIS6AC

  little
*******************************************************************************/

struct Radar_VIS6AC
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t lane_left_indv_a0 : 12;
			uint32_t LaneLeftIndvA2 : 12;
			uint32_t Reserved : 8;
		} bit;
	} DATA012;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t lane_left_indv_col : 2;
			uint16_t LaneLeftIndvA1 : 11;
			uint16_t Reserved : 3;
		} bit;
	} DATA34;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t Reserved1 : 5;
			uint32_t LaneLeftIndvRange : 9;
			uint32_t LaneLeftIndvA3 : 16;
			uint32_t Reserved : 2;
		} bit;
	} DATA4567;
};

/*******************************************************************************

  File Name   : ACU_EMB213
  Date        : 2020/07/29
  Version     : 1.0.1
  Desciption  : ACU_EMB213

  little
*******************************************************************************/

struct ACU_EMB213
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t ACU_Brake_Opening : 8;
		} bit;
	} DATA0;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t Reserved : 32;
		} bit;
	} DATA1234;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved : 16;
		} bit;
	} DATA56;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t ACU_Brake_Enable : 1;
			uint8_t ACU_Handbrake_Enable : 1;
			uint8_t Reserved : 6;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : ACU_EPS223
  Date        : 2020/07/29
  Version     : 1.0.1
  Desciption  : ACU_EPS223

  little
*******************************************************************************/

struct ACU_EPS223
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t Reserved : 32;
		} bit;
	} DATA0123;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA4;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t ACU_EPS_Angle_Control : 16;
		} bit;
	} DATA56;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t ACU_EPS_Enable : 1;
			uint8_t Reserved : 7;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : ACU_Speed_Control233
  Date        : 2020/07/29
  Version     : 1.0.1
  Desciption  : ACU_Speed_Control233

  little
*******************************************************************************/

struct ACU_Speed_Control233
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA0;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t ACU_Torque_Control : 16;
		} bit;
	} DATA12;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved : 16;
		} bit;
	} DATA34;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t ACU_VSpeed_Control : 16;
		} bit;
	} DATA56;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t ACU_VSpeed_Control_Enable : 1;
			uint8_t ACU_Torque_Control_Enable : 1;
			uint8_t Reserved : 6;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : ACU_VCU203
  Date        : 2020/07/29
  Version     : 1.0.1
  Desciption  : ACU_VCU203

  little
*******************************************************************************/

struct ACU_VCU203
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t Reserved : 32;
		} bit;
	} DATA0123;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved : 16;
		} bit;
	} DATA45;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t ACU_VCU_VSpeed_Gear : 2;
			uint8_t Reserved : 6;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Choice : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : EMB_ACU21C
  Date        : 2020/07/29
  Version     : 1.0.1
  Desciption  : EMB_ACU21C

  little
*******************************************************************************/

struct EMB_ACU21C
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Front_EMB_Brake_Enable : 1;
			uint8_t Front_EMB_HandBrake_Enable : 1;
			uint8_t Reserved : 6;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Front_EMB_Left_Motor_Current : 8;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Front_EMB_Right_Motor_Current : 8;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Front_Left_Brake_LevelRsp : 8;
		} bit;
	} DATA4;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Front_Right_Brake_LevelRsp : 8;
		} bit;
	} DATA5;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved : 16;
		} bit;
	} DATA67;
};

/*******************************************************************************

  File Name   : EPS_ACU22C
  Date        : 2020/07/29
  Version     : 1.0.1
  Desciption  : EPS_ACU22C

  little
*******************************************************************************/

struct EPS_ACU22C
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t Reserved : 32;
		} bit;
	} DATA0123;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA4;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t EPS_Angle_Control : 16;
		} bit;
	} DATA56;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t EPS_ACU_Enable : 1;
			uint8_t Reserved : 7;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : Speed_Control_ACU23C
  Date        : 2020/07/29
  Version     : 1.0.1
  Desciption  : Speed_Control_ACU23C

  little
*******************************************************************************/

struct Speed_Control_ACU23C
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA0;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t SC_Motor_Speed : 16;
		} bit;
	} DATA12;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved : 16;
		} bit;
	} DATA34;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t SC_VSpeed_Control : 16;
		} bit;
	} DATA56;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t SC_VSpeed_Enable : 1;
			uint8_t SC_Torque_Control_Enable : 1;
			uint8_t Reserved : 6;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : VCU_ACU20C
  Date        : 2020/07/29
  Version     : 1.0.1
  Desciption  : VCU_ACU20C

  little
*******************************************************************************/

struct VCU_ACU20C
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t Reserved : 32;
		} bit;
	} DATA0123;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved : 16;
		} bit;
	} DATA45;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t VCU_ACU_VSpeed_Gear : 2;
			uint8_t Reserved : 6;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 6;
			uint8_t VCU_Remote_Enable : 1;
			uint8_t VCU_Automatic_or_Manual : 1;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : ViewVehicle_CANDATA
  Date        : 2020/07/29
  Version     : 1.0.1
  Desciption  : ViewVehicle_CANDATA

  little
*******************************************************************************/

struct ViewVehicle_CANDATA
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA4;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA5;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA7;
};

/*****************************江铃********************************************

  File Name   : VCU_174h
  Date        : 2021/12/08
  Version     : 1.0.1
  Desciption  : VCU_174h JMEV

  little
*******************************************************************************/

struct VCU_174h //修改
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 2;
			uint8_t VCU_Err_AccPedal : 1;
			uint8_t VCU_InsuidDetcCtrl : 1;
			uint8_t VCU_GearPosition : 4;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t VCU_PrechrgAnodeRelay_Status : 2;
			uint8_t Reserved : 1;
			uint8_t VCU_DischrgAnodeRelay_Control : 1;
			uint8_t Reserved1 : 1;
			uint8_t VCU_DischrgAnodeRelay_Status : 2;
			uint8_t Reserved2 : 1;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 1;
			uint8_t VCU_DischrgCathodeRelay_Control : 1;
			uint8_t VCU_DischrgCathodeRelay_Status : 2;
			uint8_t Reserved1 : 1;
			uint8_t VCU_PrechrgAnodeRelay_Control : 1;
			uint8_t VCU_PTSystem : 2;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t VCU_VehSysStatus : 5;
			uint8_t VCU_ChrgRelay_Control : 1;
			uint8_t VCU_GearCtrlSts : 2;
		} bit;
	} DATA3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t VCU_Cruise_MainSwitch : 1;
			uint8_t VCU_OperationMode : 2;
			uint8_t VCU_DrivingMode : 2;
			uint8_t VCU_FMCU_unloadReq : 1;
			uint8_t VCU_RMCU_unloadReq : 1;
			uint8_t VCU_AC_unloadReq : 1;
		} bit;
	} DATA4;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 4;
			uint8_t VCU_Tortoiselndicator : 1;
			uint8_t Reserved1 : 1;
			uint8_t VCU_ActiveDischargeOrder : 1;
			uint8_t VCU_Cruise_Enable : 1;
		} bit;
	} DATA5;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t VCU_174h_RolingCounter : 4;
			uint8_t Reserved : 1;
			uint8_t VCU_UnlockSts : 2;
			uint8_t VCU_StartAllow : 1;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t ACU_174h_CheckSUM : 8;
		} bit;
	} DATA7;
};


/*****************************江铃********************************************

  File Name   : ACU_24Ah
  Date        : 2021/11/16
  Version     : 1.0.1
  Desciption  : ACU_24Ah JMEV

  little
*******************************************************************************/

struct ACU_24Ah //修改
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 4;
			uint8_t ACU_AirBagSysWrnLmpCmd : 1;
			uint8_t ACU_Crashinfo : 3;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t BCM_DrvSeatbeltBucklevalidity : 1;
			uint8_t BCM_DrvSeatbeltBucklestatus : 1;
			uint8_t BCM_PassSeatbeltBucklevalidity : 1;
			uint8_t BCM_PassSeatbeltBucklestatus : 1;
			uint8_t Reserved : 4;
		} bit;
	} DATA1;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved : 16;
		} bit;
	} DATA23;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved : 16;
		} bit;
	} DATA45;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t ACU_24Ah_RollingCounter : 4;
			uint8_t Reserved : 4;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t ACU_24Ah_CheckSUM : 8;
		} bit;
	} DATA7;
};

/*****************************江铃********************************************

  File Name   : APA_170h
  Date        : 2021/11/22
  Version     : 1.0.1
  Desciption  : APA_170h JMEV

  little
*******************************************************************************/

struct APA_170h //修改
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 4;
			uint8_t APA_GearRequest : 3;
			uint8_t Reserved1 : 1;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t APA_BrakeModeSts : 3;
			uint8_t APA_BrakeFunctionMode : 3;
			uint8_t Reserved : 2;
		} bit;
	} DATA1;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t APA_SpeedLimit : 16;
		} bit;
	} DATA2;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved : 4;
			uint16_t APA_StopDistance : 12;
		} bit;
	} DATA34;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t APA_DegreeReqSignCmd : 2;
			uint8_t Reserved : 6;
		} bit;
	} DATA5;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t APA_LCControl2_AliveCounter : 4;
			uint8_t Reserved : 4;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t APA_LCControl2_CheckSum : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : ADAS_160h
  Date        : 2021/01/04
  Version     : 1.0.1
  Desciption  : ADAS_160h

  little
*******************************************************************************/

struct ADAS_160h
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t ACC_Set_Speed : 8;
		} bit;
	} DATA0;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t ACC_Accelerate_Request : 11;
			uint16_t Reserved : 1;
			uint16_t ACC_DecToStop : 1;
			uint16_t ACC_Set_Headway : 3;
		} bit;
	} DATA12;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA3;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved : 8;
			uint16_t TJAHWA_Auto_Cancel : 2;
			uint16_t ALC_Auto_Ccancel : 2;
			uint16_t Reserved1 : 4;
		} bit;
	} DATA45;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t ADAS_160h_RollingCounter : 4;
			uint8_t Reserved : 4;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t ADAS_160h_CheckSUM : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : ADAS_162h
  Date        : 2021/01/04
  Version     : 1.0.1
  Desciption  : ADAS_162h

  little
*******************************************************************************/

struct ADAS_162h //修改
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t AEB_ABA_Level : 2;
			uint8_t AEB_ABA_Req : 1;
			uint8_t AEB_Prefill_Request : 1;
			uint8_t AEB_AWB_Req : 1;
			uint8_t AEB_state : 3;
		} bit;
	} DATA0;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t AEB_DecCtlReq : 1;
			uint16_t FCW_State : 3;
			uint16_t AEB_Decelerate_Request : 12;
		} bit;
	} DATA12;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t ACC_MiniBraking : 1;
			uint8_t ADAS_Sensorstatus : 3;
			uint8_t HW_Warning_Level : 2;
			uint8_t FCW_Warning_Level : 2;
		} bit;
	} DATA3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 1;
			uint8_t ACC_CIPV_indicator : 1;
			uint8_t ACC_Auto_Cancel : 2;
			uint8_t ACC_Driver_Cancel : 1;
			uint8_t ACC_ModeReq : 3;
		} bit;
	} DATA4;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t AEB_AWB_Level : 2;
			uint8_t ACC_ShutDownMode : 2;
			uint8_t ACC_DriveOff : 1;
			uint8_t ACC_BrkPreferred : 1;
			uint8_t ACC_Go_Indicator : 1;
			uint8_t ACC_Takeover_Indicator : 1;
		} bit;
	} DATA5;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t ADAS_162h_RollingCounter : 4;
			uint8_t Reserved : 4;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t ADAS_162h_CheckSUM : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : ADAS_163h
  Date        : 2021/01/04
  Version     : 1.0.1
  Desciption  : ADAS_163h

  little
*******************************************************************************/

struct ADAS_163h
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t ACC_UpperJerkLimit : 12;
			uint16_t Reserved : 4;
		} bit;
	} DATA01;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ACC_ComfortLoBand : 10;
			uint32_t ACC_ComfortUpBand : 10;
			uint32_t ACC_LowerJerkLimit : 12;
		} bit;
	} DATA2345;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t ADAS_163h_RollingCounter : 4;
			uint8_t Reserved : 4;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t ADAS_163h_CheckSUM : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : APA_166h
  Date        : 2021/01/04
  Version     : 1.0.1
  Desciption  : APA_166h

  little
*******************************************************************************/

struct APA_166h
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 6;
			uint8_t APA_EPS_Control_Request : 2;
		} bit;
	} DATA0;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t APA_SetSteeringWheelAng : 16;
		} bit;
	} DATA12;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA3;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved : 16;
		} bit;
	} DATA45;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t APA_ControlSts_AliveCounter : 4;
			uint8_t Reserved : 4;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t APA_ControlSts_CheckSUM : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : EPS_112h
  Date        : 2021/01/04
  Version     : 1.0.1
  Desciption  : EPS_112h

  little
*******************************************************************************/

struct EPS_112h //修改
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t EPS_TorsionBarTorque : 10;
			uint16_t EPS_SteeringWheelAngVD : 1;
			uint16_t EPS_SASFailureSts : 1;
			uint16_t EPS_TorsionBarTorqueValid : 1;
			uint16_t EPS_TorsionBarTorqueDir : 1;
			uint16_t EPS_Fault : 2;
		} bit;
	} DATA01;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t EPS_SteerWheelAng : 16;
		} bit;
	} DATA23;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t EPS_State_Mode : 3;
			uint16_t EPS_Modesetting_Available : 1;
			uint16_t Reserved : 3;
			uint16_t EPS_SteerWheelRotSpdVD : 1;
			uint16_t EPS_SteerWheelRotSpd : 8;
		} bit;
	} DATA45;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t EPS_112h_RollingCounter : 4;
			uint8_t Reserved : 4;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t EPS_112h__CheckSUM : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : EPS_114h
  Date        : 2021/01/04
  Version     : 1.0.1
  Desciption  : EPS_114h

  little
*******************************************************************************/

struct EPS_114h //修改
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t EPS_OverLay_Torque : 11;
			uint16_t Reserved : 2;
			uint16_t EPS_Handsoff_Mode : 2;
			uint16_t EPS_Handsoff_State : 1;
		} bit;
	} DATA01;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 1;
			uint8_t EPS_APA_FAULT_STS : 3;
			uint8_t EPS_APA_CONTROL_STS : 3;
			uint8_t EPS_Steering_Angle_Speed_Direction : 1;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 4;
			uint8_t EPS_Status : 4;
		} bit;
	} DATA3;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved : 16;
		} bit;
	} DATA45;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t EPS_114h_RollingCounter : 4;
			uint8_t Reserved : 4;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t EPS_114h_CheckSUM : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : GW_156h
  Date        : 2021/01/04
  Version     : 1.0.1
  Desciption  : GW_156h

  little
*******************************************************************************/

struct GW_156h //修改
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t SteerWheel_ACC_ACC_Cancel_Switch : 1;
			uint8_t SteerWheel_ACC_ACC_Set_Switch : 1;
			uint8_t SteerWheel_ACC_ACC_Resume_Switch : 1;
			uint8_t SteerWheel_ACC_ACC_Gpa_Dec_Switch : 1;
			uint8_t SteerWheel_ACC_ACC_Gpa_Inc_Switch : 1;
			uint8_t SteerWheel_ACC_ACC_Main_Switch_On : 1;
			uint8_t Reserved : 1;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 4;
		} bit;
	} DATA3;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved : 16;
		} bit;
	} DATA45;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved : 16;
		} bit;
	} DATA67;
};

/*******************************************************************************

  File Name   : WCBS_92h
  Date        : 2021/01/04
  Version     : 1.0.1
  Desciption  : WCBS_92h

  little
*******************************************************************************/

struct WCBS_92h
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t WCBS_ESC_VehSpd : 16;
		} bit;
	} DATA01;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t WCBS_ESC_WhlFLSpd : 16;
		} bit;
	} DATA23;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t WCBS_ESC_WhlFRSpd : 16;
		} bit;
	} DATA45;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t WCBS_92h_RollingCounter : 4;
			uint8_t Reserved : 1;
			uint8_t WCBS_ESC_WhlSpdFRVd : 1;
			uint8_t WCBS_ESC_WhlSpdFLVd : 1;
			uint8_t WCBS_ESC_VehSpdVd : 1;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t WCBS_92h_CheckSUM : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : WCBS_93h
  Date        : 2021/01/04
  Version     : 1.0.1
  Desciption  : WCBS_93h

  little
*******************************************************************************/

struct WCBS_93h
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t WCBS_EHB_ACC_BrkSw_Validity : 1;
			uint8_t WCBS_EHB_ACC_BrkSw_Sta : 1;
			uint8_t WCBS_EHB_ACC_CDD_Available : 1;
			uint8_t WCBS_EHB_ACC_CDD_Active : 1;
			uint8_t WCBS_EHB_ACC_VLC_Available : 1;
			uint8_t WCBS_EHB_ACC_VLC_Active : 1;
			uint8_t WCBS_EHB_ACC_VLC_Failure : 2;
		} bit;
	} DATA0;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t WCBS_EHB_axInternalTarget : 11;
			uint16_t Reserved : 1;
			uint16_t WCBS_EHB_APA_Available : 1;
			uint16_t WCBS_EHB_APA_EmgyAvailable : 1;
			uint16_t WCBS_EHB_APA_CDD_Active : 1;
			uint16_t WCBS_EHB_APA_GearReqActive : 1;
		} bit;
	} DATA12;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t WCBS_EHB_APA_LC_FailureSts : 4;
			uint16_t Reserved : 4;
			uint16_t WCBS_EHB_APA_GearTarget : 4;
			uint16_t WCBS_EHB_APA_LC_Status : 4;
		} bit;
	} DATA34;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA5;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t WCBS_93h_RollingCounter : 4;
			uint8_t Reserved : 4;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t WCBS_93h_CheckSUM : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : WCBS_96h
  Date        : 2021/01/04
  Version     : 1.0.1
  Desciption  : WCBS_96h

  little
*******************************************************************************/

struct WCBS_96h
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t WCBS_ESC_FLWheelWpdPulse : 16;
		} bit;
	} DATA01;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t WCBS_ESC_FRWheelWpdPulse : 16;
		} bit;
	} DATA23;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t WCBS_ESC_LateralAcc : 8;
		} bit;
	} DATA4;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t WCBS_ESC_LongAcc : 8;
		} bit;
	} DATA5;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t WCBS_96h_RollingCounter : 4;
			uint8_t WCBS_ESC_FRWheelWpdPulseValidity : 1;
			uint8_t WCBS_ESC_FLWheelWpdPulseValidity : 1;
			uint8_t WCBS_ESC_LongAccVd : 1;
			uint8_t WCBS_ESC_LateralAccVd : 1;
		} bit;
	} DATA6;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t WCBS_96h_CheckSUM : 8;
		} bit;
	} DATA7;
};

/*******************************************************************************

  File Name   : BCM_152h
  Date        : 2021/01/04
  Version     : 1.0.1
  Desciption  : BCM_152h

  little
*******************************************************************************/

struct BCM_152h //修改
{
	union
	{
		uint8_t D;
		struct
		{
			uint8_t Length : 4;
			uint8_t Reserved : 4;
		} bit;
	} MsgLengh;

	union
	{
		uint32_t D;
		struct
		{
			uint32_t ID : 32;
		} bit;
	} MsgID;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t BCM_RFDoorSwitchSt : 1;
			uint8_t BCM_LFDoorSwitchSt : 1;
			uint8_t BCM_PositionLampSts : 1;
			uint8_t BCM_FrontFoglightSt : 1;
			uint8_t BCM_BackFoglightSt : 1;
			uint8_t BCM_HightBeamStatus : 1;
			uint8_t BCM_LowBeamStatus : 1;
			uint8_t BCM_TrunkSt : 5;
		} bit;
	} DATA0;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t BCM_TurnLeverLoc : 2;
			uint8_t BCM_RightLigthFaultSt : 1;
			uint8_t BCM_RightligthSt : 1;
			uint8_t BCM_LeftLigthFaultSt : 1;
			uint8_t BCM_LeftlightSt : 1;
			uint8_t BCM_LRDorSwitchSt : 1;
			uint8_t BCM_RRDorSwitchSt : 1;
		} bit;
	} DATA1;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t BCM_CentralLockSt : 1;
			uint8_t BCM_Reverse_Light : 1;
			uint8_t Reserved : 1;
			uint8_t BCM_OutSwitchPressCmd : 1;
			uint8_t BCM_HazardwarnLampSt : 1;
			uint8_t Reserved1 : 2;
			uint8_t BCM_FrontCoverSt : 1;
		} bit;
	} DATA2;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t Reserved : 8;
		} bit;
	} DATA3;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t BCM_Auto_LightSwitch : 1;
			uint8_t BCM_Brake_Light : 1;
			uint8_t BCM_RearViewMirrorAutoFoldFucFeedback : 2;
			uint8_t BCM_Rain_Sensor : 2;
			uint8_t Reserved : 2;
		} bit;
	} DATA4;

	union
	{
		uint8_t D;
		struct
		{
			uint8_t BCM_PowerMode : 3;
			uint8_t Reserved : 1;
			uint8_t BCM_RearViewMirror : 2;
			uint8_t BCM_ChangeCoverLockSt : 1;
			uint8_t BCM_RearDeforstSts : 1;
		} bit;
	} DATA5;

	union
	{
		uint16_t D;
		struct
		{
			uint16_t Reserved : 16;
		} bit;
	} DATA67;
};
// int changelane_status=0;
extern ADAS_160h ADAS_160h_1;
extern ADAS_162h ADAS_162h_1;
extern ADAS_163h ADAS_163h_1;
extern APA_166h APA_166h_1;
extern EPS_112h EPS_112h_1;
extern EPS_114h EPS_114h_1;
extern WCBS_92h WCBS_92h_1;
extern WCBS_93h WCBS_93h_1;
extern WCBS_96h WCBS_96h_1;
extern BCM_152h BCM_152h_1;
extern ACU_24Ah ACU_24Ah_1;
extern GW_156h GW_156h_1;
extern VCU_174h VCU_174h_1;

extern ViewVehicle_CANDATA ViewVehicle_CANDATA_1;
extern ACU_EMB213 ACU_EMB213_1;
extern ACU_EPS223 ACU_EPS223_1;
extern ACU_Speed_Control233 ACU_Speed_Control233_1;
extern ACU_VCU203 ACU_VCU203_1;
extern EMB_ACU21C EMB_ACU21C_1;
extern EPS_ACU22C EPS_ACU22C_1;
extern Speed_Control_ACU23C Speed_Control_ACU23C_1;
extern VCU_ACU20C VCU_ACU20C_1;

extern AutoControlData AutoControlData_1;		//0x601
extern RemoteConVehData RemoteConVehData_1;		//0x611
extern AutoControlData111 AutoControlData111_1; //0x111
extern AutoControlData112 AutoControlData112_1; //0x112
extern EMS_208 EMS_208_1;
extern EMS_258 EMS_258_1;
extern EMS_26A EMS_26A_1;
extern EMS_26D EMS_26D_1;
extern EMS_358 EMS_358_1;
extern EPBI_21B EPBI_21B_1;
extern EPBI_27A EPBI_27A_1;
extern EPBI_27B EPBI_27B_1;
extern EPBI_2A3 EPBI_2A3_1;
extern SAS_183 SAS_183_1;
extern TCU_26B TCU_26B_1;
extern ACM_24C ACM_24C_1;
extern EPS_2A4 EPS_2A4_1;
extern EPBI_20B EPBI_20B_1;
extern TCU_23B TCU_23B_1;
extern EPBI_25B EPBI_25B_1;
extern EPBI_27C EPBI_27C_1;
extern TCU_33B TCU_33B_1;
extern EPBI_259 EPBI_259_1;
extern EMS_268 EMS_268_1;
extern EMS_278 EMS_278_1;
extern EMS_298 EMS_298_1;
extern Radar_Yaw130 Radar_Yaw130_1;
extern Radar_Spd3E9 Radar_Spd3E9_1;
extern Radar_RTS1A740 Radar_RTS1A740_1;
extern Radar_RTS1B741 Radar_RTS1B741_1;
extern Radar_RTS1C742 Radar_RTS1C742_1;
extern Radar_RTS2A745 Radar_RTS2A745_1;
extern Radar_RTS2B746 Radar_RTS2B746_1;
extern Radar_RTS2C747 Radar_RTS2C747_1;
extern Radar_RTS3A74A Radar_RTS3A74A_1;
extern Radar_RTS3B74B Radar_RTS3B74B_1;
extern Radar_RTS3C74C Radar_RTS3C74C_1;
extern Radar_RTS4A74F Radar_RTS4A74F_1;
extern Radar_RTS4B750 Radar_RTS4B750_1;
extern Radar_RTS4C751 Radar_RTS4C751_1;
extern Radar_RT1A760 Radar_RT1A760_1;
extern Radar_RT1B761 Radar_RT1B761_1;
extern Radar_RT1C762 Radar_RT1C762_1;
extern Radar_RT2A765 Radar_RT2A765_1;
extern Radar_RT2B766 Radar_RT2B766_1;
extern Radar_RT2C767 Radar_RT2C767_1;
extern Radar_RT3A76A Radar_RT3A76A_1;
extern Radar_RT3B76B Radar_RT3B76B_1;
extern Radar_RT3C76C Radar_RT3C76C_1;
extern Radar_RT4A76F Radar_RT4A76F_1;
extern Radar_RT4B770 Radar_RT4B770_1;
extern Radar_RT4C771 Radar_RT4C771_1;
extern Radar_RT5A774 Radar_RT5A774_1;
extern Radar_RT5B775 Radar_RT5B775_1;
extern Radar_RT5C776 Radar_RT5C776_1;
extern Radar_RT6A779 Radar_RT6A779_1;
extern Radar_RT6B77A Radar_RT6B77A_1;
extern Radar_RT6C77B Radar_RT6C77B_1;
extern MSG_CANDATA MSG_CANDATA_1;
extern Radar_CANDATA Radar_CANDATA_1;
extern Plat_CANDATA Plat_CANDATA_1;
extern RTK_324 RTK_324_1;
extern RTK_32A RTK_32A_1;
extern RTK_320 RTK_320_1;
extern RTK_323 RTK_323_1;
extern RTK_329 RTK_329_1;
extern RTK_327 RTK_327_1;
extern RTK_325 RTK_325_1;

/**/
extern Radar_VIS6A4 Radar_VIS6A4_1;
extern Radar_VIS6A8 Radar_VIS6A8_1;
extern Radar_VIS6A9 Radar_VIS6A9_1;
extern Radar_VIS6AB Radar_VIS6AB_1;
extern Radar_VIS6AC Radar_VIS6AC_1;
