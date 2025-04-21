/*
 * File: AutoVehicleControl.h
 *
 * Code generated for Simulink model 'AutoVehicleControl'.
 *
 * Model version                  : 1.14
 * Simulink Coder version         : 8.3 (R2012b) 20-Jul-2012
 * TLC version                    : 8.3 (Jul 21 2012)
 * C/C++ source code generated on : Tue May 09 13:55:51 2017
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Generic->64-bit Embedded Processor (LP64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_AutoVehicleControl_h_
#define RTW_HEADER_AutoVehicleControl_h_
#ifndef AutoVehicleControl_COMMON_INCLUDES_
# define AutoVehicleControl_COMMON_INCLUDES_
#include <math.h>
#include <float.h>
#include <string.h>
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#endif                                 /* AutoVehicleControl_COMMON_INCLUDES_ */

#include "AutoVehicleControl_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif
extern bool vhicle_stop;
/* Block signals (auto storage) */
typedef struct {
  real_T O_StreerAngReq;               /* '<S13>/P_SteerConP' */
  real_T O_DesSteerAng;                /* '<S13>/Switch' */
  real_T V_DesSteerAngEna;             /* '<S84>/F_SteerAngEna' */
  real_T V_DesSteerAng;                /* '<S83>/F_SteerAngTrans' */
  real_T Switch3[3];                   /* '<S11>/Switch3' */
  real_T V_ObjSpdms;                   /* '<S55>/Add1' */
  real_T testpoint1;                   /* '<S56>/Multiport Switch' */
  real_T O_DesBrk;                     /* '<S57>/Saturation' */
  real_T O_DesDrv;                     /* '<S57>/Saturation1' */
  real_T Switch2;                      /* '<S10>/Switch2' */
  real_T Switch3_p;                    /* '<S10>/Switch3' */
  real_T B_TrajEndEna;                 /* '<S10>/05_F_TrajEndStop' */
  real_T B_TrafficLightStopEna;        /* '<S10>/04_F_TrafficLightStop' */
  real_T V_DesState;                   /* '<S56>/03_F_DesState' */
  real_T Switch1;                      /* '<S66>/Switch1' */
  real_T Merge;                        /* '<S49>/Merge' */
  real_T V_TrajNum;                    /* '<S17>/01_F_SeleTrajLCProcess' */
  real_T V_LCTurnLeft;                 /* '<S17>/01_F_SeleTrajLCProcess' */
  real_T V_LCTurnRight;                /* '<S17>/01_F_SeleTrajLCProcess' */
  real_T V_LCProcess;                  /* '<S17>/01_F_SeleTrajLCProcess' */
  boolean_T Merge_h;                   /* '<S15>/Merge' */
  boolean_T Merge1;                    /* '<S15>/Merge1' */
} BlockIO_AutoVehicleControl;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T UnitDelay2_DSTATE;            /* '<S8>/Unit Delay2' */
  real_T UnitDelay_DSTATE[3];          /* '<S76>/Unit Delay' */
  real_T DiscreteTimeIntegrator_DSTATE;/* '<S71>/Discrete-Time Integrator' */
  real_T Delay1_DSTATE;                /* '<S57>/Delay1' */
  real_T DiscreteTimeIntegrator_DSTATE_e;/* '<S66>/Discrete-Time Integrator' */
  real_T UnitDelay_DSTATE_p;           /* '<S9>/Unit Delay' */
  real_T PrevY;                        /* '<S66>/P_DesAccSpd_RateLim' */
  uint32_T temporalCounter_i1;         /* '<S84>/F_SteerAngEna' */
  uint32_T presentTicks;               /* '<S84>/F_SteerAngEna' */
  uint32_T elapsedTicks;               /* '<S84>/F_SteerAngEna' */
  uint32_T previousTicks;              /* '<S84>/F_SteerAngEna' */
  uint32_T temporalCounter_i1_c;       /* '<S83>/F_SteerAngTrans' */
  uint32_T presentTicks_h;             /* '<S83>/F_SteerAngTrans' */
  uint32_T elapsedTicks_i;             /* '<S83>/F_SteerAngTrans' */
  uint32_T previousTicks_a;            /* '<S83>/F_SteerAngTrans' */
  uint32_T temporalCounter_i1_f;       /* '<S10>/05_F_TrajEndStop' */
  uint32_T presentTicks_b;             /* '<S10>/05_F_TrajEndStop' */
  uint32_T elapsedTicks_a;             /* '<S10>/05_F_TrajEndStop' */
  uint32_T previousTicks_o;            /* '<S10>/05_F_TrajEndStop' */
  uint32_T temporalCounter_i1_l;       /* '<S10>/04_F_TrafficLightStop' */
  uint32_T presentTicks_o;             /* '<S10>/04_F_TrafficLightStop' */
  uint32_T elapsedTicks_h;             /* '<S10>/04_F_TrafficLightStop' */
  uint32_T previousTicks_b;            /* '<S10>/04_F_TrafficLightStop' */
  uint32_T temporalCounter_i1_cr;      /* '<S17>/01_F_SeleTrajLCProcess' */
  uint32_T presentTicks_d;             /* '<S17>/01_F_SeleTrajLCProcess' */
  uint32_T elapsedTicks_o;             /* '<S17>/01_F_SeleTrajLCProcess' */
  uint32_T previousTicks_e;            /* '<S17>/01_F_SeleTrajLCProcess' */
  uint16_T UnitDelay3_DSTATE;          /* '<S74>/Unit Delay3' */
  uint16_T UnitDelay3_DSTATE_p;        /* '<S75>/Unit Delay3' */
  boolean_T Delay_DSTATE;              /* '<S57>/Delay' */
  int8_T DiscreteTimeIntegrator_PrevRese;/* '<S71>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_m;/* '<S66>/Discrete-Time Integrator' */
  uint8_T is_active_c4_AutoVehicleControl;/* '<S84>/F_SteerAngEna' */
  uint8_T is_c4_AutoVehicleControl;    /* '<S84>/F_SteerAngEna' */
  uint8_T is_Ena;                      /* '<S84>/F_SteerAngEna' */
  uint8_T is_active_c5_AutoVehicleControl;/* '<S83>/F_SteerAngTrans' */
  uint8_T is_c5_AutoVehicleControl;    /* '<S83>/F_SteerAngTrans' */
  uint8_T is_active_c2_AutoVehicleControl;/* '<S10>/05_F_TrajEndStop' */
  uint8_T is_c2_AutoVehicleControl;    /* '<S10>/05_F_TrajEndStop' */
  uint8_T is_active_c6_AutoVehicleControl;/* '<S10>/04_F_TrafficLightStop' */
  uint8_T is_c6_AutoVehicleControl;    /* '<S10>/04_F_TrafficLightStop' */
  uint8_T is_active_c3_AutoVehicleControl;/* '<S56>/03_F_DesState' */
  uint8_T is_c3_AutoVehicleControl;    /* '<S56>/03_F_DesState' */
  uint8_T DiscreteTimeIntegrator_IC_LOADI;/* '<S66>/Discrete-Time Integrator' */
  uint8_T is_active_c1_AutoVehicleControl;/* '<S17>/01_F_SeleTrajLCProcess' */
  uint8_T is_c1_AutoVehicleControl;    /* '<S17>/01_F_SeleTrajLCProcess' */
  boolean_T u_F_LaneChange_MODE;       /* '<S7>/01_F_LaneChange' */
  boolean_T u_F_LongControl_MODE;      /* '<S7>/02_F_LongControl' */
  boolean_T u_F_SpdControl_MODE;       /* '<S56>/02_F_SpdControl' */
  boolean_T u_F_SeleTraj_MODE;         /* '<S9>/03_F_SeleTraj' */
} D_Work_AutoVehicleControl;

/* Constant parameters (auto storage) */
typedef struct {
  /* Expression: T_RoadResY
   * Referenced by: '<S57>/T_RoadRes'
   */
  real_T T_RoadRes_tableData[6];

  /* Expression: T_RoadResX
   * Referenced by: '<S57>/T_RoadRes'
   */
  real_T T_RoadRes_bp01Data[6];

  /* Expression: T_TransSpdRaioY
   * Referenced by: '<S57>/T_TransSpdRaio'
   */
  real_T T_TransSpdRaio_tableD[6];

  /* Expression: T_TransSpdRaioX
   * Referenced by: '<S57>/T_TransSpdRaio'
   */
  real_T T_TransSpdRaio_bp01Da[6];

  /* Pooled Parameter (Expression: [1 -1 0])
   * Referenced by:
   *   '<S74>/1-D Lookup Table'
   *   '<S75>/1-D Lookup Table'
   */
  real_T pooled12[3];

  /* Pooled Parameter (Expression: [0 1 2])
   * Referenced by:
   *   '<S74>/1-D Lookup Table'
   *   '<S75>/1-D Lookup Table'
   */
  real_T pooled13[3];

  /* Computed Parameter: T_MaxSteerAngRate_ta
   * Referenced by: '<S13>/T_MaxSteerAngRate'
   */
  uint32_T T_MaxSteerAngRate_ta[11];

  /* Computed Parameter: T_MaxSteerAngRate_bp
   * Referenced by: '<S13>/T_MaxSteerAngRate'
   */
  uint32_T T_MaxSteerAngRate_bp[11];

  /* Computed Parameter: T_MaxSteerAng_tableD
   * Referenced by: '<S13>/T_MaxSteerAng'
   */
  uint32_T T_MaxSteerAng_tableD[14];

  /* Computed Parameter: T_MaxSteerAng_bp01Da
   * Referenced by: '<S13>/T_MaxSteerAng'
   */
  uint32_T T_MaxSteerAng_bp01Da[14];
} ConstParam_AutoVehicleControl;

/* Real-time Model Data Structure */
struct tag_RTM_AutoVehicleControl {
  const char_T * volatile errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    boolean_T firstInitCondFlag;
  } Timing;
};

/* Block signals (auto storage) */
extern BlockIO_AutoVehicleControl AutoVehicleControl_B;

/* Block states (auto storage) */
extern D_Work_AutoVehicleControl AutoVehicleControl_DWork;

/* Constant parameters (auto storage) */
extern const ConstParam_AutoVehicleControl AutoVehicleControl_ConstP;

/*
 * Exported States
 *
 * Note: Exported states are block states with an exported global
 * storage class designation.  Code generation will declare the memory for these
 * states and exports their symbols.
 *
 */
extern real_T V_VehPosYReqdou;         /* Simulink.Signal object 'V_VehPosYReqdou' */
extern real_T V_VehPosXReqdou;         /* Simulink.Signal object 'V_VehPosXReqdou' */
extern real_T RT3_Class_Rel;           /* Simulink.Signal object 'RT3_Class_Rel' */
extern real_T RT3_Width_Rel;           /* Simulink.Signal object 'RT3_Width_Rel' */
extern real_T RT3_V_Long_Rel;          /* Simulink.Signal object 'RT3_V_Long_Rel' */
extern real_T RT4_Width_Rel;           /* Simulink.Signal object 'RT4_Width_Rel' */
extern real_T RT4_V_Long_Rel;          /* Simulink.Signal object 'RT4_V_Long_Rel' */
extern real_T RT4_V_Lat_Rel;           /* Simulink.Signal object 'RT4_V_Lat_Rel' */
extern real_T RT6_Width_Rel;           /* Simulink.Signal object 'RT6_Width_Rel' */
extern real_T RT6_V_Long_Rel;          /* Simulink.Signal object 'RT6_V_Long_Rel' */
extern real_T RT6_V_Lat_Rel;           /* Simulink.Signal object 'RT6_V_Lat_Rel' */
extern real_T RT6_L_Lat_Rel;           /* Simulink.Signal object 'RT6_L_Lat_Rel' */
extern real_T RT6_L_Long_Rel;          /* Simulink.Signal object 'RT6_L_Long_Rel' */
extern real_T RT6_Class_Rel;           /* Simulink.Signal object 'RT6_Class_Rel' */
extern real_T RT5_Width_Rel;           /* Simulink.Signal object 'RT5_Width_Rel' */
extern real_T RT4_L_Lat_Rel;           /* Simulink.Signal object 'RT4_L_Lat_Rel' */
extern real_T RT5_V_Long_Rel;          /* Simulink.Signal object 'RT5_V_Long_Rel' */
extern real_T RT5_V_Lat_Rel;           /* Simulink.Signal object 'RT5_V_Lat_Rel' */
extern real_T RT5_L_Lat_Rel;           /* Simulink.Signal object 'RT5_L_Lat_Rel' */
extern real_T RT5_L_Long_Rel;          /* Simulink.Signal object 'RT5_L_Long_Rel' */
extern real_T RT5_Class_Rel;           /* Simulink.Signal object 'RT5_Class_Rel' */
extern real_T RT2_Width_Rel;           /* Simulink.Signal object 'RT2_Width_Rel' */
extern real_T RT2_V_Long_Rel;          /* Simulink.Signal object 'RT2_V_Long_Rel' */
extern real_T RT2_V_Lat_Rel;           /* Simulink.Signal object 'RT2_V_Lat_Rel' */
extern real_T RT2_L_Lat_Rel;           /* Simulink.Signal object 'RT2_L_Lat_Rel' */
extern real_T RT2_L_Long_Rel;          /* Simulink.Signal object 'RT2_L_Long_Rel' */
extern real_T RT4_L_Long_Rel;          /* Simulink.Signal object 'RT4_L_Long_Rel' */
extern real_T RT2_Class_Rel;           /* Simulink.Signal object 'RT2_Class_Rel' */
extern real_T RT1_Class_Rel;           /* Simulink.Signal object 'RT1_Class_Rel' */
extern real_T RT1_Width_Rel;           /* Simulink.Signal object 'RT1_Width_Rel' */
extern real_T RT1_V_Long_Rel;          /* Simulink.Signal object 'RT1_V_Long_Rel' */
extern real_T RT1_V_Lat_Rel;           /* Simulink.Signal object 'RT1_V_Lat_Rel' */
extern real_T RT1_L_Lat_Rel;           /* Simulink.Signal object 'RT1_L_Lat_Rel' */
extern real_T RT1_L_Long_Rel;          /* Simulink.Signal object 'RT1_L_Long_Rel' */
extern real_T RT3_V_Lat_Rel;           /* Simulink.Signal object 'RT3_V_Lat_Rel' */
extern real_T RT3_L_Lat_Rel;           /* Simulink.Signal object 'RT3_L_Lat_Rel' */
extern real_T RT4_Class_Rel;           /* Simulink.Signal object 'RT4_Class_Rel' */
extern real_T RT3_L_Long_Rel;          /* Simulink.Signal object 'RT3_L_Long_Rel' */
extern real_T GPS_Heading;             /* Simulink.Signal object 'GPS_Heading' */
extern real_T Latitude_B;              /* Simulink.Signal object 'Latitude_B' */
extern real_T Longitude_L;             /* Simulink.Signal object 'Longitude_L' */
extern real_T V_VehPosXdou;            /* Simulink.Signal object 'V_VehPosXdou' */
extern real_T V_VehPosYdou;            /* Simulink.Signal object 'V_VehPosYdou' */
extern real_T GPS_Pitch;               /* Simulink.Signal object 'GPS_Pitch' */
extern real_T V_VehPosAngdou;          /* Simulink.Signal object 'V_VehPosAngdou' */
extern real_T V_Traje2X;               /* Simulink.Signal object 'V_Traje2X' */
extern real_T V_Traje1X;               /* Simulink.Signal object 'V_Traje1X' */
extern real_T V_NearTrajeY;            /* Simulink.Signal object 'V_NearTrajeY' */
extern real_T V_TrajeSpdf16s4;         /* Simulink.Signal object 'V_TrajeSpdf16s4' */
extern real_T V_LaneWidthf16s4;        /* Simulink.Signal object 'V_LaneWidthf16s4' */
extern real_T V_SlopeResisf32s20;      /* Simulink.Signal object 'V_SlopeResisf32s20' */
extern real_T V_Traje3X;               /* Simulink.Signal object 'V_Traje3X' */
extern real_T V_DesAnglef32s20;        /* Simulink.Signal object 'V_DesAnglef32s20' */
extern real_T V_Traje2Y;               /* Simulink.Signal object 'V_Traje2Y' */
extern real_T V_Traje3Y;               /* Simulink.Signal object 'V_Traje3Y' */
extern real_T V_Traje1Y;               /* Simulink.Signal object 'V_Traje1Y' */
extern real_T V_NearTrajeX;            /* Simulink.Signal object 'V_NearTrajeX' */
extern uint32_T V_RefPoint;            /* Simulink.Signal object 'V_RefPoint' */
extern uint16_T Wheel_Speed_RR_Data;   /* Simulink.Signal object 'Wheel_Speed_RR_Data' */
extern uint16_T Wheel_Speed_RL_Data;   /* Simulink.Signal object 'Wheel_Speed_RL_Data' */
extern uint16_T ESP_VehicleSpeed;      /* Simulink.Signal object 'ESP_VehicleSpeed' */
extern uint16_T TqR_AccTrqReq;         /* Simulink.Signal object 'TqR_AccTrqReq' */
extern uint16_T EMS_IndicatedRealEngTorq;/* Simulink.Signal object 'EMS_IndicatedRealEngTorq' */
extern int16_T ESP_LongAccel;          /* Simulink.Signal object 'ESP_LongAccel' */
extern int16_T ESP_YawRate;            /* Simulink.Signal object 'ESP_YawRate' */
extern int16_T StC_SteeringAngleRequest;/* Simulink.Signal object 'StC_SteeringAngleRequest' */
extern int16_T SAS_SteeringAngle;      /* Simulink.Signal object 'SAS_SteeringAngle' */
extern uint16_T EMS_EngineSpeed;       /* Simulink.Signal object 'EMS_EngineSpeed' */
extern uint8_T StC_SteeringAngleReq;   /* Simulink.Signal object 'StC_SteeringAngleReq' */
extern uint8_T MC_Mode;                /* Simulink.Signal object 'MC_Mode' */
extern uint8_T V_VehDesNum;            /* Simulink.Signal object 'V_VehDesNum' */
extern uint8_T V_TrafficLightDet;      /* Simulink.Signal object 'V_TrafficLightDet' */
extern uint8_T v_road_typeu8;           /* Simulink.Signal object 'v_road_typeu8' */
extern uint8_T EMS_BrakePedalStatus;   /* Simulink.Signal object 'EMS_BrakePedalStatus' */
extern uint8_T AT_ActualGear;          /* Simulink.Signal object 'AT_ActualGear' */
extern uint8_T TCU_GearShiftPositon;   /* Simulink.Signal object 'TCU_GearShiftPositon' */
extern uint8_T BCM_TurnLightSwitchSts; /* Simulink.Signal object 'BCM_TurnLightSwitchSts' */
extern uint8_T EPS_APA_Abortfeedback;  /* Simulink.Signal object 'EPS_APA_Abortfeedback' */
extern boolean_T TqR_AccTrqReqEna;     /* Simulink.Signal object 'TqR_AccTrqReqEna' */
extern boolean_T TqR_CDDAxEnable;      /* Simulink.Signal object 'TqR_CDDAxEnable' */
extern boolean_T EPS_APA_EpasFAILED;   /* Simulink.Signal object 'EPS_APA_EpasFAILED' */
extern boolean_T EPS_APA_ControlFeedback;/* Simulink.Signal object 'EPS_APA_ControlFeedback' */
extern uint8_T EMS_AccPedal;           /* Simulink.Signal object 'EMS_AccPedal' */
extern uint8_T EMS_MinIndicatedTorq;   /* Simulink.Signal object 'EMS_MinIndicatedTorq' */
extern uint8_T EMS_MaxIndicatedTorq;   /* Simulink.Signal object 'EMS_MaxIndicatedTorq' */
extern int8_T TqR_ACCTargetAccelerationReq;/* Simulink.Signal object 'TqR_ACCTargetAccelerationReq' */

#ifdef __cplusplus

extern "C" {

#endif

  /* Model entry point functions */
  extern void AutoVehicleControl_initialize(void);
  extern void AutoVehicleControl_step(void);
  extern void AutoVehicleControl_terminate(void);

#ifdef __cplusplus

}
#endif

/* Real-time Model object */
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_AutoVehicleControl *const AutoVehicleControl_M;

#ifdef __cplusplus

}
#endif

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'AutoVehicleControl'
 * '<S1>'   : 'AutoVehicleControl/01VehicleData'
 * '<S2>'   : 'AutoVehicleControl/02HdMapdata'
 * '<S3>'   : 'AutoVehicleControl/03GPSdata'
 * '<S4>'   : 'AutoVehicleControl/04ObjData'
 * '<S5>'   : 'AutoVehicleControl/05OBUdata'
 * '<S6>'   : 'AutoVehicleControl/06PlatformData'
 * '<S7>'   : 'AutoVehicleControl/07_Perception&DecisionPlanning'
 * '<S8>'   : 'AutoVehicleControl/08_EnableLogic'
 * '<S9>'   : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange'
 * '<S10>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/02_F_LongControl'
 * '<S11>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/03_F_Position'
 * '<S12>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/04_F_TxToRTMaps'
 * '<S13>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/05_F_SteerControl'
 * '<S14>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/Compare To Constant3'
 * '<S15>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/01_F_UseableTraj'
 * '<S16>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/02_F_ObjSafeCal'
 * '<S17>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/03_F_SeleTraj'
 * '<S18>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/01_F_UseableTraj/01_F_MSeleTraj'
 * '<S19>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/01_F_UseableTraj/02_F_LSeleTraj'
 * '<S20>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/01_F_UseableTraj/03_F_RSeleTraj'
 * '<S21>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/01_F_UseableTraj/Compare To Constant'
 * '<S22>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/01_F_UseableTraj/Compare To Constant1'
 * '<S23>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/01_F_UseableTraj/Compare To Constant2'
 * '<S24>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/01_F_UseableTraj/01_F_MSeleTraj/Compare To Constant3'
 * '<S25>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/01_F_UseableTraj/01_F_MSeleTraj/Compare To Zero'
 * '<S26>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/01_F_UseableTraj/01_F_MSeleTraj/Compare To Zero1'
 * '<S27>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/01_F_UseableTraj/01_F_MSeleTraj/Compare To Zero2'
 * '<S28>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/01_F_UseableTraj/02_F_LSeleTraj/Compare To Constant3'
 * '<S29>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/01_F_UseableTraj/02_F_LSeleTraj/Compare To Zero1'
 * '<S30>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/01_F_UseableTraj/03_F_RSeleTraj/Compare To Constant3'
 * '<S31>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/01_F_UseableTraj/03_F_RSeleTraj/Compare To Zero1'
 * '<S32>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/02_F_ObjSafeCal/Compare To Constant'
 * '<S33>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/02_F_ObjSafeCal/Compare To Constant1'
 * '<S34>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/02_F_ObjSafeCal/Compare To Constant2'
 * '<S35>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/02_F_ObjSafeCal/Compare To Constant3'
 * '<S36>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/02_F_ObjSafeCal/Compare To Constant4'
 * '<S37>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/02_F_ObjSafeCal/Compare To Constant5'
 * '<S38>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/02_F_ObjSafeCal/Compare To Constant6'
 * '<S39>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/02_F_ObjSafeCal/Compare To Constant7'
 * '<S40>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/02_F_ObjSafeCal/Compare To Zero'
 * '<S41>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/02_F_ObjSafeCal/Compare To Zero1'
 * '<S42>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/02_F_ObjSafeCal/Compare To Zero2'
 * '<S43>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/02_F_ObjSafeCal/Compare To Zero3'
 * '<S44>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/02_F_ObjSafeCal/Compare To Zero4'
 * '<S45>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/02_F_ObjSafeCal/Compare To Zero5'
 * '<S46>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/02_F_ObjSafeCal/Compare To Zero6'
 * '<S47>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/02_F_ObjSafeCal/Compare To Zero7'
 * '<S48>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/03_F_SeleTraj/01_F_SeleTrajLCProcess'
 * '<S49>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/03_F_SeleTraj/02_F_LaneChAndTurnCorner'
 * '<S50>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/03_F_SeleTraj/02_F_LaneChAndTurnCorner/Compare To Constant1'
 * '<S51>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/03_F_SeleTraj/02_F_LaneChAndTurnCorner/Compare To Constant2'
 * '<S52>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/03_F_SeleTraj/02_F_LaneChAndTurnCorner/If Action Subsystem'
 * '<S53>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/03_F_SeleTraj/02_F_LaneChAndTurnCorner/If Action Subsystem1'
 * '<S54>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/01_F_LaneChange/03_F_SeleTraj/02_F_LaneChAndTurnCorner/If Action Subsystem2'
 * '<S55>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/02_F_LongControl/01_F_CheckDisControl'
 * '<S56>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/02_F_LongControl/02_F_CalDesAcc'
 * '<S57>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/02_F_LongControl/03_F_AcceCon'
 * '<S58>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/02_F_LongControl/04_F_TrafficLightStop'
 * '<S59>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/02_F_LongControl/05_F_TrajEndStop'
 * '<S60>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/02_F_LongControl/Compare To Constant'
 * '<S61>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/02_F_LongControl/Compare To Constant3'
 * '<S62>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/02_F_LongControl/01_F_CheckDisControl/Compare To Constant'
 * '<S63>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/02_F_LongControl/01_F_CheckDisControl/Compare To Constant1'
 * '<S64>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/02_F_LongControl/01_F_CheckDisControl/Compare To Constant2'
 * '<S65>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/02_F_LongControl/02_F_CalDesAcc/01_F_DisControl'
 * '<S66>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/02_F_LongControl/02_F_CalDesAcc/02_F_SpdControl'
 * '<S67>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/02_F_LongControl/02_F_CalDesAcc/03_F_DesState'
 * '<S68>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/02_F_LongControl/02_F_CalDesAcc/Compare To Constant'
 * '<S69>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/02_F_LongControl/03_F_AcceCon/Compare To Zero'
 * '<S70>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/02_F_LongControl/03_F_AcceCon/Integrator'
 * '<S71>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/02_F_LongControl/03_F_AcceCon/Integrator/Integrator'
 * '<S72>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/02_F_LongControl/03_F_AcceCon/Integrator/Integrator/Saturation1'
 * '<S73>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/03_F_Position/01_F_CheckHigPos'
 * '<S74>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/03_F_Position/02_F_RLDisCal'
 * '<S75>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/03_F_Position/03_F_RRDisCal'
 * '<S76>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/03_F_Position/04_F_CalVehPos'
 * '<S77>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/05_F_SteerControl/F_AngConversion'
 * '<S78>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/05_F_SteerControl/Saturation Dynamic'
 * '<S79>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/05_F_SteerControl/Saturation Dynamic1'
 * '<S80>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/05_F_SteerControl/F_AngConversion/If Action Subsystem'
 * '<S81>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/05_F_SteerControl/F_AngConversion/If Action Subsystem1'
 * '<S82>'  : 'AutoVehicleControl/07_Perception&DecisionPlanning/05_F_SteerControl/F_AngConversion/If Action Subsystem2'
 * '<S83>'  : 'AutoVehicleControl/08_EnableLogic/01_F_DesTrans'
 * '<S84>'  : 'AutoVehicleControl/08_EnableLogic/02_Enable'
 * '<S85>'  : 'AutoVehicleControl/08_EnableLogic/01_F_DesTrans/F_SteerAngTrans'
 * '<S86>'  : 'AutoVehicleControl/08_EnableLogic/01_F_DesTrans/If Action Subsystem3'
 * '<S87>'  : 'AutoVehicleControl/08_EnableLogic/01_F_DesTrans/If Action Subsystem4'
 * '<S88>'  : 'AutoVehicleControl/08_EnableLogic/01_F_DesTrans/If Action Subsystem5'
 * '<S89>'  : 'AutoVehicleControl/08_EnableLogic/01_F_DesTrans/If Action Subsystem6'
 * '<S90>'  : 'AutoVehicleControl/08_EnableLogic/02_Enable/F_SteerAngEna'
 * '<S91>'  : 'AutoVehicleControl/08_EnableLogic/02_Enable/If Action Subsystem1'
 * '<S92>'  : 'AutoVehicleControl/08_EnableLogic/02_Enable/If Action Subsystem2'
 * '<S93>'  : 'AutoVehicleControl/08_EnableLogic/02_Enable/If Action Subsystem3'
 * '<S94>'  : 'AutoVehicleControl/08_EnableLogic/02_Enable/If Action Subsystem4'
 */
#endif                                 /* RTW_HEADER_AutoVehicleControl_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
