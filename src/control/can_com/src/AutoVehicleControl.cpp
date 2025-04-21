/*
 * File: AutoVehicleControl.cpp
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

#include "can_com/AutoVehicleControl.h"
#include "can_com/AutoVehicleControl_private.h"
#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <fstream>
#include "can_com/RoutePlanning.h"

using namespace std;

/* Named constants for Chart: '<S17>/01_F_SeleTrajLCProcess' */
#define AutoVehicleC_IN_NO_ACTIVE_CHILD ((uint8_T)0U)
#define AutoVehicleControl_IN_Defult1  ((uint8_T)1U)
#define AutoVehicleControl_IN_Defult2  ((uint8_T)2U)
#define AutoVehicleControl_IN_Defult3  ((uint8_T)3U)
#define AutoVehicleControl_IN_Defult4  ((uint8_T)4U)
#define AutoVehicleControl_IN_TrajNum1 ((uint8_T)5U)
#define AutoVehicleControl_IN_TrajNum2 ((uint8_T)6U)
#define AutoVehicleControl_IN_TrajNum3 ((uint8_T)7U)

/* Named constants for Chart: '<S56>/03_F_DesState' */
#define AutoVehicleContro_IN_DisControl ((uint8_T)1U)
#define AutoVehicleContro_IN_SpdControl ((uint8_T)2U)

/* Named constants for Chart: '<S10>/04_F_TrafficLightStop' */
#define AutoVehicleControl_IN_NoStop   ((uint8_T)1U)
#define AutoVehicleControl_IN_Stop     ((uint8_T)2U)

/* Named constants for Chart: '<S83>/F_SteerAngTrans' */
#define AutoVehicleControl_IN_Defult2_j ((uint8_T)1U)
#define AutoVehicleControl_IN_Defult3_i ((uint8_T)2U)
#define AutoVehicleControl_IN_DisEna   ((uint8_T)3U)
#define AutoVehicleControl_IN_Ena      ((uint8_T)4U)

/* Named constants for Chart: '<S84>/F_SteerAngEna' */
#define AutoVehicleContro_IN_ESPConect1 ((uint8_T)2U)
#define AutoVehicleControl_IN_ESPConect ((uint8_T)1U)
bool vhicle_stop;
/* Exported block states */
real_T V_VehPosYReqdou;                /* Simulink.Signal object 'V_VehPosYReqdou' */
real_T V_VehPosXReqdou;                /* Simulink.Signal object 'V_VehPosXReqdou' */
real_T RT3_Class_Rel;                  /* Simulink.Signal object 'RT3_Class_Rel' */
real_T RT3_Width_Rel;                  /* Simulink.Signal object 'RT3_Width_Rel' */
real_T RT3_V_Long_Rel;                 /* Simulink.Signal object 'RT3_V_Long_Rel' */
real_T RT4_Width_Rel;                  /* Simulink.Signal object 'RT4_Width_Rel' */
real_T RT4_V_Long_Rel;                 /* Simulink.Signal object 'RT4_V_Long_Rel' */
real_T RT4_V_Lat_Rel;                  /* Simulink.Signal object 'RT4_V_Lat_Rel' */
real_T RT6_Width_Rel;                  /* Simulink.Signal object 'RT6_Width_Rel' */
real_T RT6_V_Long_Rel;                 /* Simulink.Signal object 'RT6_V_Long_Rel' */
real_T RT6_V_Lat_Rel;                  /* Simulink.Signal object 'RT6_V_Lat_Rel' */
real_T RT6_L_Lat_Rel;                  /* Simulink.Signal object 'RT6_L_Lat_Rel' */
real_T RT6_L_Long_Rel;                 /* Simulink.Signal object 'RT6_L_Long_Rel' */
real_T RT6_Class_Rel;                  /* Simulink.Signal object 'RT6_Class_Rel' */
real_T RT5_Width_Rel;                  /* Simulink.Signal object 'RT5_Width_Rel' */
real_T RT4_L_Lat_Rel;                  /* Simulink.Signal object 'RT4_L_Lat_Rel' */
real_T RT5_V_Long_Rel;                 /* Simulink.Signal object 'RT5_V_Long_Rel' */
real_T RT5_V_Lat_Rel;                  /* Simulink.Signal object 'RT5_V_Lat_Rel' */
real_T RT5_L_Lat_Rel;                  /* Simulink.Signal object 'RT5_L_Lat_Rel' */
real_T RT5_L_Long_Rel;                 /* Simulink.Signal object 'RT5_L_Long_Rel' */
real_T RT5_Class_Rel;                  /* Simulink.Signal object 'RT5_Class_Rel' */
real_T RT2_Width_Rel;                  /* Simulink.Signal object 'RT2_Width_Rel' */
real_T RT2_V_Long_Rel;                 /* Simulink.Signal object 'RT2_V_Long_Rel' */
real_T RT2_V_Lat_Rel;                  /* Simulink.Signal object 'RT2_V_Lat_Rel' */
real_T RT2_L_Lat_Rel;                  /* Simulink.Signal object 'RT2_L_Lat_Rel' */
real_T RT2_L_Long_Rel;                 /* Simulink.Signal object 'RT2_L_Long_Rel' */
real_T RT4_L_Long_Rel;                 /* Simulink.Signal object 'RT4_L_Long_Rel' */
real_T RT2_Class_Rel;                  /* Simulink.Signal object 'RT2_Class_Rel' */
real_T RT1_Class_Rel;                  /* Simulink.Signal object 'RT1_Class_Rel' */
real_T RT1_Width_Rel;                  /* Simulink.Signal object 'RT1_Width_Rel' */
real_T RT1_V_Long_Rel;                 /* Simulink.Signal object 'RT1_V_Long_Rel' */
real_T RT1_V_Lat_Rel;                  /* Simulink.Signal object 'RT1_V_Lat_Rel' */
real_T RT1_L_Lat_Rel;                  /* Simulink.Signal object 'RT1_L_Lat_Rel' */
real_T RT1_L_Long_Rel;                 /* Simulink.Signal object 'RT1_L_Long_Rel' */
real_T RT3_V_Lat_Rel;                  /* Simulink.Signal object 'RT3_V_Lat_Rel' */
real_T RT3_L_Lat_Rel;                  /* Simulink.Signal object 'RT3_L_Lat_Rel' */
real_T RT4_Class_Rel;                  /* Simulink.Signal object 'RT4_Class_Rel' */
real_T RT3_L_Long_Rel;                 /* Simulink.Signal object 'RT3_L_Long_Rel' */
real_T GPS_Heading;                    /* Simulink.Signal object 'GPS_Heading' */
real_T Latitude_B;                     /* Simulink.Signal object 'Latitude_B' */
real_T Longitude_L;                    /* Simulink.Signal object 'Longitude_L' */
real_T V_VehPosXdou;                   /* Simulink.Signal object 'V_VehPosXdou' */
real_T V_VehPosYdou;                   /* Simulink.Signal object 'V_VehPosYdou' */
real_T GPS_Pitch;                      /* Simulink.Signal object 'GPS_Pitch' */
real_T V_VehPosAngdou;                 /* Simulink.Signal object 'V_VehPosAngdou' */
real_T V_Traje2X;                      /* Simulink.Signal object 'V_Traje2X' */
real_T V_Traje1X;                      /* Simulink.Signal object 'V_Traje1X' */
real_T V_NearTrajeY;                   /* Simulink.Signal object 'V_NearTrajeY' */
real_T V_TrajeSpdf16s4;                /* Simulink.Signal object 'V_TrajeSpdf16s4' */
real_T V_LaneWidthf16s4;               /* Simulink.Signal object 'V_LaneWidthf16s4' */
real_T V_SlopeResisf32s20;             /* Simulink.Signal object 'V_SlopeResisf32s20' */
real_T V_Traje3X;                      /* Simulink.Signal object 'V_Traje3X' */
real_T V_DesAnglef32s20;               /* Simulink.Signal object 'V_DesAnglef32s20' */
real_T V_Traje2Y;                      /* Simulink.Signal object 'V_Traje2Y' */
real_T V_Traje3Y;                      /* Simulink.Signal object 'V_Traje3Y' */
real_T V_Traje1Y;                      /* Simulink.Signal object 'V_Traje1Y' */
real_T V_NearTrajeX;                   /* Simulink.Signal object 'V_NearTrajeX' */
uint32_T V_RefPoint;                   /* Simulink.Signal object 'V_RefPoint' */
uint16_T Wheel_Speed_RR_Data;          /* Simulink.Signal object 'Wheel_Speed_RR_Data' */
uint16_T Wheel_Speed_RL_Data;          /* Simulink.Signal object 'Wheel_Speed_RL_Data' */
uint16_T ESP_VehicleSpeed;             /* Simulink.Signal object 'ESP_VehicleSpeed' */
uint16_T TqR_AccTrqReq;                /* Simulink.Signal object 'TqR_AccTrqReq' */
uint16_T EMS_IndicatedRealEngTorq;     /* Simulink.Signal object 'EMS_IndicatedRealEngTorq' */
int16_T ESP_LongAccel;                 /* Simulink.Signal object 'ESP_LongAccel' */
int16_T ESP_YawRate;                   /* Simulink.Signal object 'ESP_YawRate' */
int16_T StC_SteeringAngleRequest;      /* Simulink.Signal object 'StC_SteeringAngleRequest' */
int16_T SAS_SteeringAngle;             /* Simulink.Signal object 'SAS_SteeringAngle' */
uint16_T EMS_EngineSpeed;              /* Simulink.Signal object 'EMS_EngineSpeed' */
uint8_T StC_SteeringAngleReq;          /* Simulink.Signal object 'StC_SteeringAngleReq' */
uint8_T MC_Mode;                       /* Simulink.Signal object 'MC_Mode' */
uint8_T V_VehDesNum;                   /* Simulink.Signal object 'V_VehDesNum' */
uint8_T V_TrafficLightDet;             /* Simulink.Signal object 'V_TrafficLightDet' */
uint8_T v_road_typeu8;                  /* Simulink.Signal object 'v_road_typeu8' */
uint8_T EMS_BrakePedalStatus;          /* Simulink.Signal object 'EMS_BrakePedalStatus' */
uint8_T AT_ActualGear;                 /* Simulink.Signal object 'AT_ActualGear' */
uint8_T TCU_GearShiftPositon;          /* Simulink.Signal object 'TCU_GearShiftPositon' */
uint8_T BCM_TurnLightSwitchSts;        /* Simulink.Signal object 'BCM_TurnLightSwitchSts' */
uint8_T EPS_APA_Abortfeedback;         /* Simulink.Signal object 'EPS_APA_Abortfeedback' */
boolean_T TqR_AccTrqReqEna;            /* Simulink.Signal object 'TqR_AccTrqReqEna' */
boolean_T TqR_CDDAxEnable;             /* Simulink.Signal object 'TqR_CDDAxEnable' */
boolean_T EPS_APA_EpasFAILED;          /* Simulink.Signal object 'EPS_APA_EpasFAILED' */
boolean_T EPS_APA_ControlFeedback;     /* Simulink.Signal object 'EPS_APA_ControlFeedback' */
uint8_T EMS_AccPedal;                  /* Simulink.Signal object 'EMS_AccPedal' */
uint8_T EMS_MinIndicatedTorq;          /* Simulink.Signal object 'EMS_MinIndicatedTorq' */
uint8_T EMS_MaxIndicatedTorq;          /* Simulink.Signal object 'EMS_MaxIndicatedTorq' */
int8_T TqR_ACCTargetAccelerationReq;   /* Simulink.Signal object 'TqR_ACCTargetAccelerationReq' */


/* Block signals (auto storage) */
BlockIO_AutoVehicleControl AutoVehicleControl_B;

/* Block states (auto storage) */
D_Work_AutoVehicleControl AutoVehicleControl_DWork;

/* Real-time model */
RT_MODEL_AutoVehicleControl AutoVehicleControl_M_;
RT_MODEL_AutoVehicleControl *const AutoVehicleControl_M = &AutoVehicleControl_M_;
uint32_T look1_iu32lu32n31_binlcsos(uint32_T u0, const uint32_T bp0[], const
  uint32_T table[], uint32_T maxIndex)
{
  uint32_T frac;
  uint32_T iRght;
  uint32_T iLeft;
  uint64_T tmp;

  /* Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear'
     Extrapolation method: 'Clip'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
     Rounding mode: 'simplest'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
     Rounding mode: 'simplest'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = 0U;
  } else if (u0 < bp0[maxIndex]) {
    /* Binary Search */
    frac = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[frac]) {
        iRght = frac;
      } else {
        iLeft = frac;
      }

      frac = (iRght + iLeft) >> 1U;
    }

    tmp = bp0[iLeft + 1U] - bp0[iLeft];
    frac = (uint32_T)(tmp == 0 ? MAX_uint64_T : ((uint64_T)(u0 - bp0[iLeft]) <<
      31) / tmp);
  } else {
    iLeft = maxIndex - 1U;
    frac = 2147483648U;
  }

  /* Interpolation 1-D
     Interpolation method: 'Linear'
     Use last breakpoint for index at or above upper limit: 'off'
     Rounding mode: 'simplest'
     Overflow mode: 'wrapping'
   */
  if (table[iLeft + 1U] >= table[iLeft]) {
    frac = (uint32_T)((uint64_T)(table[iLeft + 1U] - table[iLeft]) * (uint64_T)
                      frac >> 31) + table[iLeft];
  } else {
    frac = table[iLeft] - (uint32_T)((uint64_T)(table[iLeft] - table[iLeft + 1U])
      * (uint64_T)frac >> 31);
  }

  return frac;
}

real_T look1_binlxpw(real_T u0, const real_T bp0[], const real_T table[],
                     uint32_T maxIndex)
{
  real_T frac;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T bpIdx;

  /* Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear'
     Extrapolation method: 'Linear'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Linear'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    /* Binary Search */
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  /* Interpolation 1-D
     Interpolation method: 'Linear'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'portable wrapping'
   */
  return (table[iLeft + 1U] - table[iLeft]) * frac + table[iLeft];
}

real_T look1_linlxpw(real_T u0, const real_T bp0[], const real_T table[],
                     uint32_T maxIndex)
{
  real_T frac;
  uint32_T found;
  uint32_T bpIdx;

  /* Lookup 1-D
     Search method: 'linear'
     Use previous index: 'off'
     Interpolation method: 'Linear'
     Extrapolation method: 'Linear'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'linear'
     Extrapolation method: 'Linear'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    bpIdx = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    /* Linear Search */
    found = 0U;
    bpIdx = maxIndex >> 1U;
    while (found == 0U) {
      if (u0 < bp0[bpIdx]) {
        bpIdx--;
      } else if (u0 < bp0[bpIdx + 1U]) {
        found = 1U;
      } else {
        bpIdx++;
      }
    }

    frac = (u0 - bp0[bpIdx]) / (bp0[bpIdx + 1U] - bp0[bpIdx]);
  } else {
    bpIdx = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  /* Interpolation 1-D
     Interpolation method: 'Linear'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'portable wrapping'
   */
  return (table[bpIdx + 1U] - table[bpIdx]) * frac + table[bpIdx];
}

real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = -0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

real_T rt_modd_snf(real_T u0, real_T u1)
{
  real_T y;
  real_T tmp;
  if (u1 == 0.0) {
    y = u0;
  } else if (!((!rtIsNaN(u0)) && (!rtIsInf(u0)) && ((!rtIsNaN(u1)) && (!rtIsInf
                (u1))))) {
    y = (rtNaN);
  } else {
    tmp = u0 / u1;
    if (u1 <= floor(u1)) {
      y = u0 - floor(tmp) * u1;
    } else if (fabs(tmp - rt_roundd_snf(tmp)) <= DBL_EPSILON * fabs(tmp)) {
      y = 0.0;
    } else {
      y = (tmp - floor(tmp)) * u1;
    }
  }

  return y;
}

/* Model step function */
void AutoVehicleControl_step(void)
{
  /* local block i/o variables */
  uint8_T rtb_DataStoreRead4_g;
  boolean_T rtb_LogicalOperator_o;
  boolean_T rtb_LogicalOperator_l;
  uint32_T rtb_V_RefPoint;
  boolean_T rtb_Compare_n;
  uint8_T rtb_DataStoreRead4_b;
  uint8_T rtb_V_APA_Abortfeedback;
  real_T rtb_V_VehPosAng_f1;
  real_T rtb_DataStoreRead16;
  real_T rtb_DataStoreRead17;
  real_T rtb_V_TrajSpd;
  real_T rtb_V_Traje1X;
  real_T rtb_V_Traje2X;
  real_T rtb_V_Traje3X;
  real_T rtb_V_Traje1Y;
  real_T rtb_V_Traje2Y;
  real_T rtb_V_Traje3Y;
  real_T rtb_Add1_o;
  real_T rtb_Product_e;
  real_T rtb_MathFunction1;
  real_T rtb_Switch_n;
  real_T rtb_Switch2_c;
  boolean_T rtb_Compare_o2;
  boolean_T rtb_LogicalOperator_h;
  boolean_T rtb_Compare_ks;
  boolean_T rtb_LogicalOperator3;
  boolean_T rtb_LogicalOperator8;
  boolean_T rtb_LogicalOperator9;
  boolean_T rtb_LogicalOperator4;
  boolean_T rtb_Compare_fh;
  real_T rtb_TmpSignalConversionAtDotP_h[6];
  boolean_T rtb_Compare_ay;
  boolean_T rtb_LogicalOperator2_g;
  uint32_T rtb_V_VehSpdms;
  uint32_T rtb_T_MaxSteerAngRate;
  uint32_T rtb_T_MaxSteerAng;
  int8_T rtb_DataTypeConversion1_o;
  uint16_T rtb_Add3_a;
  int32_T i;
  real_T rtb_Add3_idx;
  real_T rtb_Add3_idx_0;
  real_T rtb_TmpSignalConversionAtDotP_0;

  /* DataStoreRead: '<S2>/Data Store Read1' */
  rtb_V_RefPoint = V_RefPoint;

  /* RelationalOperator: '<S14>/Compare' incorporates:
   *  Constant: '<S14>/Constant'
   *  DataStoreRead: '<S2>/Data Store Read1'
   */
  rtb_Compare_n = (V_RefPoint >= 28200U);

  /* DataStoreRead: '<S8>/Data Store Read4' */
  rtb_DataStoreRead4_b = MC_Mode;

  /* Gain: '<S1>/Gain' incorporates:
   *  DataStoreRead: '<S1>/Data Store Read1'
   */
  rtb_V_VehSpdms = 36409U * (uint32_T)ESP_VehicleSpeed;

  /* Lookup_n-D: '<S13>/T_MaxSteerAngRate' */
  rtb_T_MaxSteerAngRate = 1.3* look1_iu32lu32n31_binlcsos(rtb_V_VehSpdms, *(uint32_T  //20210108  add 1.5*
    (*)[11])AutoVehicleControl_ConstP.T_MaxSteerAngRate_bp, *(uint32_T (*)[11])
    AutoVehicleControl_ConstP.T_MaxSteerAngRate_ta, 10U);

  /* Lookup_n-D: '<S13>/T_MaxSteerAng' */
  rtb_T_MaxSteerAng = look1_iu32lu32n31_binlcsos(rtb_V_VehSpdms, *(uint32_T (*)
    [14])AutoVehicleControl_ConstP.T_MaxSteerAng_bp01Da, *(uint32_T (*)[14])
    AutoVehicleControl_ConstP.T_MaxSteerAng_tableD, 13U);

  /* DataStoreRead: '<S7>/Data Store Read4' */
  rtb_DataStoreRead4_g = MC_Mode;

  /* Outputs for Atomic SubSystem: '<S7>/03_F_Position' */
  /* Lookup_n-D: '<S74>/1-D Lookup Table' incorporates:
   *  DataStoreRead: '<S1>/Data Store Read16'
   *  DataTypeConversion: '<S74>/Data Type Conversion1'
   */
  rtb_Add1_o = look1_binlxpw((real_T)Wheel_Speed_RL_Data * 0.0625, *(real_T (*)
    [3])AutoVehicleControl_ConstP.pooled13, *(real_T (*)[3])
    AutoVehicleControl_ConstP.pooled12, 2U);

  /* Gain: '<S74>/P_Puse2DisFac' incorporates:
   *  DataStoreRead: '<S1>/Data Store Read12'
   *  Product: '<S74>/Divide1'
   *  Sum: '<S74>/Add3'
   *  Switch: '<S74>/Switch1'
   *  UnitDelay: '<S74>/Unit Delay3'
   */
  rtb_Product_e = (real_T)(uint16_T)(Wheel_Speed_RL_Data -
    AutoVehicleControl_DWork.UnitDelay3_DSTATE) * 0.0625 * rtb_Add1_o * 0.02545;

  /* Lookup_n-D: '<S75>/1-D Lookup Table' incorporates:
   *  DataStoreRead: '<S1>/Data Store Read15'
   *  DataTypeConversion: '<S75>/Data Type Conversion1'
   */
  rtb_Add1_o = look1_binlxpw((real_T)Wheel_Speed_RR_Data * 0.0625, *(real_T (*)
    [3])AutoVehicleControl_ConstP.pooled13, *(real_T (*)[3])
    AutoVehicleControl_ConstP.pooled12, 2U);

  /* Gain: '<S75>/P_Puse2DisFac' incorporates:
   *  DataStoreRead: '<S1>/Data Store Read14'
   *  Product: '<S75>/Divide1'
   *  Sum: '<S75>/Add3'
   *  Switch: '<S75>/Switch1'
   *  UnitDelay: '<S75>/Unit Delay3'
   */
  rtb_MathFunction1 = (real_T)(uint16_T)(Wheel_Speed_RR_Data -
    AutoVehicleControl_DWork.UnitDelay3_DSTATE_p) * 0.0625 * rtb_Add1_o *
    0.02545;

  /* Sum: '<S76>/Add1' */
  rtb_Add1_o = rtb_MathFunction1 - rtb_Product_e;

  /* Sum: '<S76>/Add2' incorporates:
   *  Gain: '<S76>/2P_TyreDisInv'
   *  UnitDelay: '<S76>/Unit Delay'
   */
  rtb_Switch_n = 0.30959752321981426 * rtb_Add1_o +
    AutoVehicleControl_DWork.UnitDelay_DSTATE[2];

  /* Gain: '<S76>/Gain' incorporates:
   *  Sum: '<S76>/Add'
   */
  rtb_MathFunction1 = (rtb_Product_e + rtb_MathFunction1) * 0.5;

  /* Product: '<S76>/Product' incorporates:
   *  Trigonometry: '<S76>/Trigonometric Function1'
   */
  rtb_Product_e = cos(rtb_Switch_n) * rtb_MathFunction1;

  /* Product: '<S76>/Product1' incorporates:
   *  Trigonometry: '<S76>/Trigonometric Function2'
   */
  rtb_MathFunction1 *= sin(rtb_Switch_n);

  /* Sum: '<S76>/Add3' incorporates:
   *  UnitDelay: '<S76>/Unit Delay'
   */
  rtb_Add3_idx = AutoVehicleControl_DWork.UnitDelay_DSTATE[0] + rtb_Product_e;
  rtb_Add3_idx_0 = AutoVehicleControl_DWork.UnitDelay_DSTATE[1] +
    rtb_MathFunction1;

  /* Switch: '<S73>/Switch' incorporates:
   *  Constant: '<S73>/Constant'
   *  DotProduct: '<S73>/Dot Product'
   *  Math: '<S73>/Math Function'
   *  RelationalOperator: '<S73>/Relational Operator'
   */
  if (rtb_DataStoreRead4_g != 0) {
    /* Sum: '<S73>/Add1' incorporates:
     *  Constant: '<S73>/P_PosErrMX'
     *  Gain: '<S73>/P_Ts'
     */
    rtb_Product_e = (real_T)(2748779069UL * (uint64_T)rtb_V_VehSpdms) *
      1.7347234759768071E-18 + 0.9;

    /* SignalConversion: '<S73>/TmpSignal ConversionAtDot ProductInport1' incorporates:
     *  DataStoreRead: '<S3>/Data Store Read2'
     *  DataStoreRead: '<S3>/Data Store Read3'
     *  Sum: '<S73>/Add2'
     *  Sum: '<S73>/Add3'
     */
    rtb_Switch_n = V_VehPosXdou - rtb_Add3_idx;
    rtb_TmpSignalConversionAtDotP_0 = V_VehPosYdou - rtb_Add3_idx_0;
    rtb_Switch_n = (real_T)(rtb_Switch_n * rtb_Switch_n +
      rtb_TmpSignalConversionAtDotP_0 * rtb_TmpSignalConversionAtDotP_0 >=
      rtb_Product_e * rtb_Product_e);
  } else {
    rtb_Switch_n = 0.0;
  }

  /* End of Switch: '<S73>/Switch' */

  /* Math: '<S76>/Math Function1' incorporates:
   *  Constant: '<S76>/Constant'
   *  Gain: '<S76>/P_TyreDisInv'
   *  Sum: '<S76>/Add4'
   *  UnitDelay: '<S76>/Unit Delay'
   */
  rtb_MathFunction1 = rt_modd_snf(0.61919504643962853 * rtb_Add1_o +
    AutoVehicleControl_DWork.UnitDelay_DSTATE[2], 6.2831853071795862);

  /* Switch: '<S11>/Switch3' incorporates:
   *  DataStoreRead: '<S3>/Data Store Read1'
   *  DataStoreRead: '<S3>/Data Store Read2'
   *  DataStoreRead: '<S3>/Data Store Read3'
   *  Switch: '<S76>/Switch'
   */
  if (/*rtb_Switch_n != 0.0*/0) {
    AutoVehicleControl_B.Switch3[0] = rtb_Add3_idx;
    AutoVehicleControl_B.Switch3[1] = rtb_Add3_idx_0;
    AutoVehicleControl_B.Switch3[2] = rtb_MathFunction1;

    /* Update for UnitDelay: '<S76>/Unit Delay' */
    AutoVehicleControl_DWork.UnitDelay_DSTATE[0] = rtb_Add3_idx;
    AutoVehicleControl_DWork.UnitDelay_DSTATE[1] = rtb_Add3_idx_0;
    AutoVehicleControl_DWork.UnitDelay_DSTATE[2] = rtb_MathFunction1;
  } else {
    AutoVehicleControl_B.Switch3[0] = V_VehPosXdou;
    AutoVehicleControl_B.Switch3[1] = V_VehPosYdou;
    AutoVehicleControl_B.Switch3[2] = V_VehPosAngdou;

    /* Update for UnitDelay: '<S76>/Unit Delay' incorporates:
     *  DataStoreRead: '<S3>/Data Store Read1'
     *  DataStoreRead: '<S3>/Data Store Read2'
     *  DataStoreRead: '<S3>/Data Store Read3'
     */
    AutoVehicleControl_DWork.UnitDelay_DSTATE[0] = V_VehPosXdou;
    AutoVehicleControl_DWork.UnitDelay_DSTATE[1] = V_VehPosYdou;
    AutoVehicleControl_DWork.UnitDelay_DSTATE[2] = V_VehPosAngdou;
  }

  /* End of Switch: '<S11>/Switch3' */

  /* Update for UnitDelay: '<S74>/Unit Delay3' incorporates:
   *  DataStoreRead: '<S1>/Data Store Read12'
   */
  AutoVehicleControl_DWork.UnitDelay3_DSTATE = Wheel_Speed_RL_Data;

  /* Update for UnitDelay: '<S75>/Unit Delay3' incorporates:
   *  DataStoreRead: '<S1>/Data Store Read14'
   */
  AutoVehicleControl_DWork.UnitDelay3_DSTATE_p = Wheel_Speed_RR_Data;

  /* End of Outputs for SubSystem: '<S7>/03_F_Position' */

  /* Sum: '<S13>/Add' incorporates:
   *  DataStoreRead: '<S2>/Data Store Read2'
   */
  rtb_Product_e = AutoVehicleControl_B.Switch3[2] - V_DesAnglef32s20;
  //RCLCPP_INFO(rclcpp::get_logger("control_node"), "rtb_Product_e:[%f] StC_SteeringAngleRequest:[%f] MC_Mode:[%d]", rtb_Product_e,StC_SteeringAngleRequest*0.0625,MC_Mode);
  //RCLCPP_INFO(rclcpp::get_logger("control_node"), "V_DesAcc:[%f] V_ObjSpd:[%f]", AutoVehicleControl_B.testpoint1,AutoVehicleControl_B.V_ObjSpdms);
  /*
  std::ofstream outFile;
  outFile.open("/home/liyanxi/software/src/Autocontroldata_20200603.txt",ios::binary | ios::app | ios::in | ios::out);//打开文件
	outFile<< (int)MC_Mode <<"  "<< rtb_Product_e <<"  "<<  AutoVehicleControl_B.Switch3[2] <<"  "<<  V_DesAnglef32s20 << "  " << V_VehPosXdou <<"  "<< V_VehPosYdou << "  " << o_traie_num_aim <<"\n";
	outFile.close(); //关闭文件
  */
  /* If: '<S77>/If' incorporates:
   *  Constant: '<S80>/Constant'
   *  Constant: '<S81>/Constant'
   *  Sum: '<S80>/Add'
   *  Sum: '<S81>/Add'
   */
  if (rtb_Product_e > 3.1415926535897931) {
    /* Outputs for IfAction SubSystem: '<S77>/If Action Subsystem' incorporates:
     *  ActionPort: '<S80>/Action Port'
     */
    rtb_Product_e -= 6.2831853071795862;

    /* End of Outputs for SubSystem: '<S77>/If Action Subsystem' */
  } else {
    if (rtb_Product_e < -3.1415926535897931) {
      /* Outputs for IfAction SubSystem: '<S77>/If Action Subsystem1' incorporates:
       *  ActionPort: '<S81>/Action Port'
       */
      rtb_Product_e += 6.2831853071795862;

      /* End of Outputs for SubSystem: '<S77>/If Action Subsystem1' */
    }
  }

  /* End of If: '<S77>/If' */

  /* Gain: '<S13>/P_SteerConP' */
  AutoVehicleControl_B.O_StreerAngReq = 13 * rtb_Product_e;  //13

  /* Switch: '<S78>/Switch2' incorporates:
   *  RelationalOperator: '<S78>/LowerRelop1'
   */
  if (AutoVehicleControl_B.O_StreerAngReq > (real_T)rtb_T_MaxSteerAng *
      4.76837158203125E-7) {
    rtb_Product_e = (real_T)rtb_T_MaxSteerAng * 4.76837158203125E-7;
  } else {
    /* Gain: '<S13>/Gain' incorporates:
     *  DataTypeConversion: '<S13>/Data Type Conversion'
     */
    rtb_Product_e = -((real_T)rtb_T_MaxSteerAng * 4.76837158203125E-7);

    /* Switch: '<S78>/Switch' incorporates:
     *  RelationalOperator: '<S78>/UpperRelop'
     */
    if (!(AutoVehicleControl_B.O_StreerAngReq < rtb_Product_e)) {
      rtb_Product_e = AutoVehicleControl_B.O_StreerAngReq;
    }

    /* End of Switch: '<S78>/Switch' */
  }

  /* End of Switch: '<S78>/Switch2' */

  /* Switch: '<S79>/Switch2' incorporates:
   *  DataTypeConversion: '<S13>/Data Type Conversion1'
   *  RelationalOperator: '<S79>/LowerRelop1'
   */
  if (rtb_Product_e > (real_T)rtb_T_MaxSteerAngRate * 4.76837158203125E-7) {
    rtb_Switch2_c = (real_T)rtb_T_MaxSteerAngRate * 4.76837158203125E-7;
  } else {
    /* Gain: '<S13>/Gain1' */
    rtb_MathFunction1 = -((real_T)rtb_T_MaxSteerAngRate * 4.76837158203125E-7);

    /* Switch: '<S79>/Switch' incorporates:
     *  RelationalOperator: '<S79>/UpperRelop'
     */
    if (rtb_Product_e < rtb_MathFunction1) {
      rtb_Switch2_c = rtb_MathFunction1;
    } else {
      rtb_Switch2_c = rtb_Product_e;
    }

    /* End of Switch: '<S79>/Switch' */
  }

  /* End of Switch: '<S79>/Switch2' */

  /* DataStoreRead: '<S1>/Data Store Read19' */
  rtb_V_APA_Abortfeedback = EPS_APA_Abortfeedback;

  /* DataStoreRead: '<S4>/Data Store Read13' */
  rtb_V_VehPosAng_f1 = RT1_L_Long_Rel;

  /* DataStoreRead: '<S4>/Data Store Read16' */
  rtb_DataStoreRead16 = RT1_V_Long_Rel;

  /* DataStoreRead: '<S4>/Data Store Read17' */
  rtb_DataStoreRead17 = RT1_Width_Rel;

  /* DataStoreRead: '<S2>/Data Store Read6' */
  rtb_V_TrajSpd = V_TrajeSpdf16s4;

  /* DataStoreRead: '<S2>/Data Store Read10' */
  rtb_Product_e = V_NearTrajeX;

  /* DataStoreRead: '<S2>/Data Store Read7' */
  rtb_MathFunction1 = V_NearTrajeY;

  /* DataStoreRead: '<S2>/Data Store Read8' */
  rtb_V_Traje1X = V_Traje1X;

  /* DataStoreRead: '<S2>/Data Store Read9' */
  rtb_V_Traje2X = V_Traje2X;

  /* DataStoreRead: '<S2>/Data Store Read22' */
  rtb_V_Traje3X = V_Traje3X;

  /* DataStoreRead: '<S2>/Data Store Read11' */
  rtb_V_Traje1Y = V_Traje1Y;

  /* DataStoreRead: '<S2>/Data Store Read13' */
  rtb_V_Traje2Y = V_Traje2Y;

  /* DataStoreRead: '<S2>/Data Store Read12' */
  rtb_V_Traje3Y = V_Traje3Y;

  /* Outputs for Enabled SubSystem: '<S7>/01_F_LaneChange' incorporates:
   *  EnablePort: '<S9>/Enable'
   */
  if (rtb_DataStoreRead4_g > 0) {
    if (!AutoVehicleControl_DWork.u_F_LaneChange_MODE) {
      /* InitializeConditions for UnitDelay: '<S9>/Unit Delay' */
      AutoVehicleControl_DWork.UnitDelay_DSTATE_p = 1.0;
      AutoVehicleControl_DWork.u_F_LaneChange_MODE = TRUE;
    }

    /* Outputs for Enabled SubSystem: '<S15>/01_F_MSeleTraj' incorporates:
     *  EnablePort: '<S18>/Enable'
     */
    /* RelationalOperator: '<S21>/Compare' incorporates:
     *  Constant: '<S21>/Constant'
     *  UnitDelay: '<S9>/Unit Delay'
     */
    if (AutoVehicleControl_DWork.UnitDelay_DSTATE_p == 1.0) {
      /* SignalConversion: '<S18>/TmpSignal ConversionAtDot Product2Inport1' incorporates:
       *  DataStoreRead: '<S4>/Data Store Read13'
       *  DataStoreRead: '<S4>/Data Store Read14'
       *  DataStoreRead: '<S4>/Data Store Read15'
       *  DataStoreRead: '<S4>/Data Store Read16'
       *  DataStoreRead: '<S4>/Data Store Read17'
       *  DataStoreRead: '<S4>/Data Store Read18'
       */
      rtb_TmpSignalConversionAtDotP_h[0] = RT1_L_Long_Rel;
      rtb_TmpSignalConversionAtDotP_h[1] = RT1_V_Long_Rel;
      rtb_TmpSignalConversionAtDotP_h[2] = RT1_Width_Rel;
      rtb_TmpSignalConversionAtDotP_h[3] = RT1_Class_Rel;
      rtb_TmpSignalConversionAtDotP_h[4] = RT1_L_Lat_Rel;
      rtb_TmpSignalConversionAtDotP_h[5] = RT1_V_Lat_Rel;

      /* DotProduct: '<S18>/Dot Product2' */
      rtb_Switch_n = 0.0;
      for (i = 0; i < 6; i++) {
        rtb_Switch_n += rtb_TmpSignalConversionAtDotP_h[i] *
          rtb_TmpSignalConversionAtDotP_h[i];
      }

      /* RelationalOperator: '<S27>/Compare' incorporates:
       *  Constant: '<S27>/Constant'
       *  DotProduct: '<S18>/Dot Product2'
       */
      rtb_Compare_ay = (rtb_Switch_n != 0.0);

      /* Logic: '<S18>/Logical Operator2' incorporates:
       *  Constant: '<S24>/Constant'
       *  DataStoreRead: '<S2>/Data Store Read4'
       *  DataStoreRead: '<S2>/Data Store Read6'
       *  Gain: '<S18>/P_TrajSpdFac'
       *  RelationalOperator: '<S18>/Relational Operator'
       *  RelationalOperator: '<S24>/Compare'
       */
      rtb_LogicalOperator2_g = (((real_T)rtb_V_VehSpdms * 4.76837158203125E-7 <=
        0.6 * V_TrajeSpdf16s4) && (v_road_typeu8 == 1));

      /* Logic: '<S18>/Logical Operator' incorporates:
       *  Constant: '<S25>/Constant'
       *  DataStoreRead: '<S2>/Data Store Read13'
       *  DataStoreRead: '<S2>/Data Store Read9'
       *  DotProduct: '<S18>/Dot Product'
       *  RelationalOperator: '<S25>/Compare'
       */
      AutoVehicleControl_B.Merge_h = (rtb_LogicalOperator2_g && rtb_Compare_ay &&
                                      (V_Traje2X * V_Traje2X + V_Traje2Y *
        V_Traje2Y != 0.0));

      /* Logic: '<S18>/Logical Operator1' incorporates:
       *  Constant: '<S26>/Constant'
       *  DataStoreRead: '<S2>/Data Store Read12'
       *  DataStoreRead: '<S2>/Data Store Read22'
       *  DotProduct: '<S18>/Dot Product1'
       *  RelationalOperator: '<S26>/Compare'
       */
      AutoVehicleControl_B.Merge1 = (rtb_LogicalOperator2_g && rtb_Compare_ay &&
                                     (V_Traje3X * V_Traje3X + V_Traje3Y *
        V_Traje3Y != 0.0));
    }

    /* End of RelationalOperator: '<S21>/Compare' */
    /* End of Outputs for SubSystem: '<S15>/01_F_MSeleTraj' */

    /* Outputs for Enabled SubSystem: '<S15>/02_F_LSeleTraj' incorporates:
     *  EnablePort: '<S19>/Enable'
     */
    /* RelationalOperator: '<S22>/Compare' incorporates:
     *  Constant: '<S19>/Constant'
     *  Constant: '<S22>/Constant'
     *  UnitDelay: '<S9>/Unit Delay'
     */
    if (AutoVehicleControl_DWork.UnitDelay_DSTATE_p == 2.0) {
      AutoVehicleControl_B.Merge_h = FALSE;

      /* Logic: '<S19>/Logical Operator2' incorporates:
       *  Constant: '<S19>/Constant'
       *  Constant: '<S28>/Constant'
       *  Constant: '<S29>/Constant'
       *  DataStoreRead: '<S2>/Data Store Read11'
       *  DataStoreRead: '<S2>/Data Store Read4'
       *  DataStoreRead: '<S2>/Data Store Read8'
       *  DotProduct: '<S19>/Dot Product1'
       *  RelationalOperator: '<S28>/Compare'
       *  RelationalOperator: '<S29>/Compare'
       */
      AutoVehicleControl_B.Merge1 = ((v_road_typeu8 == 1) && (V_Traje1X *
        V_Traje1X + V_Traje1Y * V_Traje1Y != 0.0));
    }

    /* End of RelationalOperator: '<S22>/Compare' */
    /* End of Outputs for SubSystem: '<S15>/02_F_LSeleTraj' */

    /* Outputs for Enabled SubSystem: '<S15>/03_F_RSeleTraj' incorporates:
     *  EnablePort: '<S20>/Enable'
     */
    /* RelationalOperator: '<S23>/Compare' incorporates:
     *  Constant: '<S20>/Constant'
     *  Constant: '<S23>/Constant'
     *  UnitDelay: '<S9>/Unit Delay'
     */
    if (AutoVehicleControl_DWork.UnitDelay_DSTATE_p == 3.0) {
      /* Logic: '<S20>/Logical Operator2' incorporates:
       *  Constant: '<S30>/Constant'
       *  Constant: '<S31>/Constant'
       *  DataStoreRead: '<S2>/Data Store Read11'
       *  DataStoreRead: '<S2>/Data Store Read4'
       *  DataStoreRead: '<S2>/Data Store Read8'
       *  DotProduct: '<S20>/Dot Product1'
       *  RelationalOperator: '<S30>/Compare'
       *  RelationalOperator: '<S31>/Compare'
       */
      AutoVehicleControl_B.Merge_h = ((v_road_typeu8 == 1) && (V_Traje1X *
        V_Traje1X + V_Traje1Y * V_Traje1Y != 0.0));
      AutoVehicleControl_B.Merge1 = FALSE;
    }

    /* End of RelationalOperator: '<S23>/Compare' */
    /* End of Outputs for SubSystem: '<S15>/03_F_RSeleTraj' */

    /* SignalConversion: '<S16>/TmpSignal ConversionAtDot ProductInport1' incorporates:
     *  DataStoreRead: '<S4>/Data Store Read10'
     *  DataStoreRead: '<S4>/Data Store Read2'
     *  DataStoreRead: '<S4>/Data Store Read3'
     *  DataStoreRead: '<S4>/Data Store Read4'
     *  DataStoreRead: '<S4>/Data Store Read5'
     *  DataStoreRead: '<S4>/Data Store Read6'
     */
    rtb_TmpSignalConversionAtDotP_h[0] = RT4_L_Long_Rel;
    rtb_TmpSignalConversionAtDotP_h[1] = RT4_V_Long_Rel;
    rtb_TmpSignalConversionAtDotP_h[2] = RT4_Width_Rel;
    rtb_TmpSignalConversionAtDotP_h[3] = RT4_Class_Rel;
    rtb_TmpSignalConversionAtDotP_h[4] = RT4_L_Lat_Rel;
    rtb_TmpSignalConversionAtDotP_h[5] = RT4_V_Lat_Rel;

    /* DotProduct: '<S16>/Dot Product' */
    rtb_Switch_n = 0.0;
    for (i = 0; i < 6; i++) {
      rtb_Switch_n += rtb_TmpSignalConversionAtDotP_h[i] *
        rtb_TmpSignalConversionAtDotP_h[i];
    }

    /* RelationalOperator: '<S40>/Compare' incorporates:
     *  Constant: '<S40>/Constant'
     *  DotProduct: '<S16>/Dot Product'
     */
    rtb_Compare_fh = (rtb_Switch_n == 0.0);

    /* SignalConversion: '<S16>/TmpSignal ConversionAtDot Product1Inport1' incorporates:
     *  DataStoreRead: '<S4>/Data Store Read31'
     *  DataStoreRead: '<S4>/Data Store Read32'
     *  DataStoreRead: '<S4>/Data Store Read33'
     *  DataStoreRead: '<S4>/Data Store Read34'
     *  DataStoreRead: '<S4>/Data Store Read35'
     *  DataStoreRead: '<S4>/Data Store Read36'
     */
    rtb_TmpSignalConversionAtDotP_h[0] = RT6_L_Long_Rel;
    rtb_TmpSignalConversionAtDotP_h[1] = RT6_V_Long_Rel;
    rtb_TmpSignalConversionAtDotP_h[2] = RT6_Width_Rel;
    rtb_TmpSignalConversionAtDotP_h[3] = RT6_Class_Rel;
    rtb_TmpSignalConversionAtDotP_h[4] = RT6_L_Lat_Rel;
    rtb_TmpSignalConversionAtDotP_h[5] = RT6_V_Lat_Rel;

    /* DotProduct: '<S16>/Dot Product1' */
    rtb_Add1_o = 0.0;
    for (i = 0; i < 6; i++) {
      rtb_Add1_o += rtb_TmpSignalConversionAtDotP_h[i] *
        rtb_TmpSignalConversionAtDotP_h[i];
    }

    /* RelationalOperator: '<S41>/Compare' incorporates:
     *  Constant: '<S41>/Constant'
     *  DotProduct: '<S16>/Dot Product1'
     */
    rtb_Compare_ay = (rtb_Add1_o == 0.0);

    /* RelationalOperator: '<S44>/Compare' incorporates:
     *  Constant: '<S44>/Constant'
     *  DotProduct: '<S16>/Dot Product'
     */
    rtb_LogicalOperator2_g = (rtb_Switch_n > 0.0);

    /* RelationalOperator: '<S45>/Compare' incorporates:
     *  Constant: '<S45>/Constant'
     *  DotProduct: '<S16>/Dot Product1'
     */
    rtb_Compare_o2 = (rtb_Add1_o > 0.0);

    /* Logic: '<S16>/Logical Operator' */
    rtb_LogicalOperator_h = (rtb_Compare_fh && rtb_Compare_ay);

    /* Logic: '<S16>/Logical Operator6' incorporates:
     *  Abs: '<S16>/Abs'
     *  Constant: '<S32>/Constant'
     *  Constant: '<S35>/Constant'
     *  DataStoreRead: '<S4>/Data Store Read2'
     *  DataStoreRead: '<S4>/Data Store Read5'
     *  Product: '<S16>/Divide'
     *  RelationalOperator: '<S32>/Compare'
     *  RelationalOperator: '<S35>/Compare'
     */
    rtb_Compare_ks = ((fabs(RT4_L_Long_Rel / RT4_V_Long_Rel) >= 5.0) &&
                      (RT4_L_Long_Rel >= 20.0));

    /* Logic: '<S16>/Logical Operator3' */
    rtb_LogicalOperator3 = (rtb_LogicalOperator2_g && rtb_Compare_ay &&
      rtb_Compare_ks);

    /* Logic: '<S16>/Logical Operator7' incorporates:
     *  Abs: '<S16>/Abs1'
     *  Constant: '<S33>/Constant'
     *  Constant: '<S34>/Constant'
     *  DataStoreRead: '<S4>/Data Store Read32'
     *  DataStoreRead: '<S4>/Data Store Read35'
     *  Product: '<S16>/Divide1'
     *  RelationalOperator: '<S33>/Compare'
     *  RelationalOperator: '<S34>/Compare'
     */
    rtb_Compare_ay = ((fabs(RT6_L_Long_Rel / RT6_V_Long_Rel) >= 5.0) &&
                      (RT6_L_Long_Rel <= -20.0));

    /* Logic: '<S16>/Logical Operator8' */
    rtb_LogicalOperator8 = (rtb_Compare_fh && rtb_Compare_o2 && rtb_Compare_ay);

    /* Logic: '<S16>/Logical Operator9' */
    rtb_LogicalOperator9 = (rtb_LogicalOperator2_g && rtb_Compare_o2 &&
      rtb_Compare_ks && rtb_Compare_ay);

    /* SignalConversion: '<S16>/TmpSignal ConversionAtDot Product2Inport1' incorporates:
     *  DataStoreRead: '<S4>/Data Store Read1'
     *  DataStoreRead: '<S4>/Data Store Read11'
     *  DataStoreRead: '<S4>/Data Store Read12'
     *  DataStoreRead: '<S4>/Data Store Read7'
     *  DataStoreRead: '<S4>/Data Store Read8'
     *  DataStoreRead: '<S4>/Data Store Read9'
     */
    rtb_TmpSignalConversionAtDotP_h[0] = RT3_L_Long_Rel;
    rtb_TmpSignalConversionAtDotP_h[1] = RT3_V_Long_Rel;
    rtb_TmpSignalConversionAtDotP_h[2] = RT3_Width_Rel;
    rtb_TmpSignalConversionAtDotP_h[3] = RT3_Class_Rel;
    rtb_TmpSignalConversionAtDotP_h[4] = RT3_L_Lat_Rel;
    rtb_TmpSignalConversionAtDotP_h[5] = RT3_V_Lat_Rel;

    /* DotProduct: '<S16>/Dot Product2' */
    rtb_Switch_n = 0.0;
    for (i = 0; i < 6; i++) {
      rtb_Switch_n += rtb_TmpSignalConversionAtDotP_h[i] *
        rtb_TmpSignalConversionAtDotP_h[i];
    }

    /* RelationalOperator: '<S42>/Compare' incorporates:
     *  Constant: '<S42>/Constant'
     *  DotProduct: '<S16>/Dot Product2'
     */
    rtb_Compare_ks = (rtb_Switch_n == 0.0);

    /* SignalConversion: '<S16>/TmpSignal ConversionAtDot Product3Inport1' incorporates:
     *  DataStoreRead: '<S4>/Data Store Read19'
     *  DataStoreRead: '<S4>/Data Store Read20'
     *  DataStoreRead: '<S4>/Data Store Read21'
     *  DataStoreRead: '<S4>/Data Store Read22'
     *  DataStoreRead: '<S4>/Data Store Read23'
     *  DataStoreRead: '<S4>/Data Store Read24'
     */
    rtb_TmpSignalConversionAtDotP_h[0] = RT2_L_Long_Rel;
    rtb_TmpSignalConversionAtDotP_h[1] = RT2_V_Long_Rel;
    rtb_TmpSignalConversionAtDotP_h[2] = RT2_Width_Rel;
    rtb_TmpSignalConversionAtDotP_h[3] = RT2_Class_Rel;
    rtb_TmpSignalConversionAtDotP_h[4] = RT2_L_Lat_Rel;
    rtb_TmpSignalConversionAtDotP_h[5] = RT2_V_Lat_Rel;

    /* DotProduct: '<S16>/Dot Product3' */
    rtb_Add1_o = 0.0;
    for (i = 0; i < 6; i++) {
      rtb_Add1_o += rtb_TmpSignalConversionAtDotP_h[i] *
        rtb_TmpSignalConversionAtDotP_h[i];
    }

    /* RelationalOperator: '<S43>/Compare' incorporates:
     *  Constant: '<S43>/Constant'
     *  DotProduct: '<S16>/Dot Product3'
     */
    rtb_Compare_o2 = (rtb_Add1_o == 0.0);

    /* RelationalOperator: '<S46>/Compare' incorporates:
     *  Constant: '<S46>/Constant'
     *  DotProduct: '<S16>/Dot Product2'
     */
    rtb_LogicalOperator2_g = (rtb_Switch_n > 0.0);

    /* RelationalOperator: '<S47>/Compare' incorporates:
     *  Constant: '<S47>/Constant'
     *  DotProduct: '<S16>/Dot Product3'
     */
    rtb_Compare_ay = (rtb_Add1_o > 0.0);

    /* Logic: '<S16>/Logical Operator10' incorporates:
     *  Abs: '<S16>/Abs3'
     *  Constant: '<S37>/Constant'
     *  Constant: '<S38>/Constant'
     *  DataStoreRead: '<S4>/Data Store Read20'
     *  DataStoreRead: '<S4>/Data Store Read23'
     *  Product: '<S16>/Divide3'
     *  RelationalOperator: '<S37>/Compare'
     *  RelationalOperator: '<S38>/Compare'
     */
    rtb_Compare_fh = ((fabs(RT2_L_Long_Rel / RT2_V_Long_Rel) >= 5.0) &&
                      (RT2_L_Long_Rel <= -20.0));

    /* Logic: '<S16>/Logical Operator4' incorporates:
     *  Abs: '<S16>/Abs2'
     *  Constant: '<S36>/Constant'
     *  Constant: '<S39>/Constant'
     *  DataStoreRead: '<S4>/Data Store Read1'
     *  DataStoreRead: '<S4>/Data Store Read7'
     *  Product: '<S16>/Divide2'
     *  RelationalOperator: '<S36>/Compare'
     *  RelationalOperator: '<S39>/Compare'
     */
    rtb_LogicalOperator4 = ((fabs(RT3_L_Long_Rel / RT3_V_Long_Rel) >= 5.0) &&
      (RT3_L_Long_Rel >= 20.0));

    /* Outputs for Enabled SubSystem: '<S9>/03_F_SeleTraj' incorporates:
     *  EnablePort: '<S17>/Enable'
     */
    if (rtb_DataStoreRead4_g > 0) {
      if (!AutoVehicleControl_DWork.u_F_SeleTraj_MODE) {
        /* InitializeConditions for Atomic SubSystem: '<S17>/02_F_LaneChAndTurnCorner' */
        /* InitializeConditions for Merge: '<S49>/Merge' */
        if (rtmIsFirstInitCond(AutoVehicleControl_M)) {
          AutoVehicleControl_B.Merge = 1.0;
        }

        /* End of InitializeConditions for Merge: '<S49>/Merge' */
        /* End of InitializeConditions for SubSystem: '<S17>/02_F_LaneChAndTurnCorner' */

        /* InitializeConditions for Chart: '<S17>/01_F_SeleTrajLCProcess' */
        AutoVehicleControl_DWork.is_active_c1_AutoVehicleControl = 0U;
        AutoVehicleControl_DWork.is_c1_AutoVehicleControl =
          AutoVehicleC_IN_NO_ACTIVE_CHILD;
        AutoVehicleControl_DWork.elapsedTicks_o = 0U;

        /* Enable for Chart: '<S17>/01_F_SeleTrajLCProcess' */
        AutoVehicleControl_DWork.presentTicks_d =
          AutoVehicleControl_M->Timing.clockTick0;
        AutoVehicleControl_DWork.previousTicks_e =
          AutoVehicleControl_DWork.presentTicks_d;
        AutoVehicleControl_DWork.u_F_SeleTraj_MODE = TRUE;
      }

      /* Logic: '<S17>/Logical Operator' incorporates:
       *  Logic: '<S16>/Logical Operator1'
       *  Logic: '<S16>/Logical Operator11'
       *  Logic: '<S16>/Logical Operator12'
       *  Logic: '<S16>/Logical Operator13'
       *  Logic: '<S16>/Logical Operator5'
       */
      rtb_LogicalOperator2_g = (AutoVehicleControl_B.Merge_h && ((rtb_Compare_ks
        && rtb_Compare_o2) || (rtb_LogicalOperator2_g && rtb_Compare_o2 &&
        rtb_LogicalOperator4) || (rtb_Compare_ay && rtb_Compare_ks &&
        rtb_Compare_fh) || (rtb_LogicalOperator2_g && rtb_Compare_ay &&
                            rtb_LogicalOperator4 && rtb_Compare_fh)));

      /* Logic: '<S17>/Logical Operator1' incorporates:
       *  Logic: '<S16>/Logical Operator2'
       */
      rtb_Compare_fh = (AutoVehicleControl_B.Merge1 && (rtb_LogicalOperator_h ||
        rtb_LogicalOperator3 || rtb_LogicalOperator8 || rtb_LogicalOperator9));

      /* Outputs for Atomic SubSystem: '<S17>/02_F_LaneChAndTurnCorner' */
      /* SignalConversion: '<S49>/TmpSignal ConversionAtDot ProductInport1' incorporates:
       *  Sum: '<S49>/Add2'
       *  Sum: '<S49>/Add3'
       */
      rtb_Add3_idx = AutoVehicleControl_B.Switch3[0] - rtb_Product_e;
      rtb_Add3_idx_0 = AutoVehicleControl_B.Switch3[1] - rtb_MathFunction1;

      /* DotProduct: '<S49>/Dot Product' */
      rtb_TmpSignalConversionAtDotP_0 = rtb_Add3_idx * rtb_Add3_idx +
        rtb_Add3_idx_0 * rtb_Add3_idx_0;

      /* SignalConversion: '<S49>/TmpSignal ConversionAtDot Product1Inport1' incorporates:
       *  Sum: '<S49>/Add1'
       *  Sum: '<S49>/Add4'
       */
      rtb_Add3_idx = AutoVehicleControl_B.Switch3[0] - rtb_V_Traje1X;
      rtb_Add3_idx_0 = AutoVehicleControl_B.Switch3[1] - rtb_V_Traje1Y;

      /* DotProduct: '<S49>/Dot Product1' */
      rtb_Add1_o = rtb_Add3_idx * rtb_Add3_idx + rtb_Add3_idx_0 * rtb_Add3_idx_0;

      /* SignalConversion: '<S49>/TmpSignal ConversionAtDot Product2Inport1' incorporates:
       *  Sum: '<S49>/Add5'
       *  Sum: '<S49>/Add6'
       */
      rtb_Add3_idx = AutoVehicleControl_B.Switch3[0] - rtb_V_Traje2X;
      rtb_Add3_idx_0 = AutoVehicleControl_B.Switch3[1] - rtb_V_Traje2Y;

      /* DotProduct: '<S49>/Dot Product2' */
      rtb_Switch_n = rtb_Add3_idx * rtb_Add3_idx + rtb_Add3_idx_0 *
        rtb_Add3_idx_0;

      /* SignalConversion: '<S49>/TmpSignal ConversionAtDot Product3Inport1' incorporates:
       *  Sum: '<S49>/Add7'
       *  Sum: '<S49>/Add8'
       */
      rtb_Add3_idx = AutoVehicleControl_B.Switch3[0] - rtb_V_Traje3X;
      rtb_Add3_idx_0 = AutoVehicleControl_B.Switch3[1] - rtb_V_Traje3Y;

      /* Sqrt: '<S49>/Sqrt1' incorporates:
       *  DotProduct: '<S49>/Dot Product1'
       */
      rtb_Add1_o = sqrt(rtb_Add1_o);

      /* Sqrt: '<S49>/Sqrt2' incorporates:
       *  DotProduct: '<S49>/Dot Product2'
       */
      rtb_Switch_n = sqrt(rtb_Switch_n);

      /* Sqrt: '<S49>/Sqrt3' incorporates:
       *  DotProduct: '<S49>/Dot Product3'
       */
      rtb_Product_e = sqrt(rtb_Add3_idx * rtb_Add3_idx + rtb_Add3_idx_0 *
                           rtb_Add3_idx_0);

      /* MinMax: '<S49>/MinMax' */
      if ((rtb_Add1_o <= rtb_Switch_n) || rtIsNaN(rtb_Switch_n)) {
        rtb_MathFunction1 = rtb_Add1_o;
      } else {
        rtb_MathFunction1 = rtb_Switch_n;
      }

      if (!((rtb_MathFunction1 <= rtb_Product_e) || rtIsNaN(rtb_Product_e))) {
        rtb_MathFunction1 = rtb_Product_e;
      }

      /* If: '<S49>/If' incorporates:
       *  Constant: '<S52>/Traj1'
       *  Constant: '<S53>/Traj2'
       *  Constant: '<S54>/Traj3'
       *  MinMax: '<S49>/MinMax'
       */
      if (rtb_MathFunction1 == rtb_Add1_o) {
        /* Outputs for IfAction SubSystem: '<S49>/If Action Subsystem' incorporates:
         *  ActionPort: '<S52>/Action Port'
         */
        AutoVehicleControl_B.Merge = 1.0;

        /* End of Outputs for SubSystem: '<S49>/If Action Subsystem' */
      } else if (rtb_MathFunction1 == rtb_Switch_n) {
        /* Outputs for IfAction SubSystem: '<S49>/If Action Subsystem1' incorporates:
         *  ActionPort: '<S53>/Action Port'
         */
        AutoVehicleControl_B.Merge = 2.0;

        /* End of Outputs for SubSystem: '<S49>/If Action Subsystem1' */
      } else {
        if (rtb_MathFunction1 == rtb_Product_e) {
          /* Outputs for IfAction SubSystem: '<S49>/If Action Subsystem2' incorporates:
           *  ActionPort: '<S54>/Action Port'
           */
          AutoVehicleControl_B.Merge = 3.0;

          /* End of Outputs for SubSystem: '<S49>/If Action Subsystem2' */
        }
      }

      /* End of If: '<S49>/If' */

      /* RelationalOperator: '<S49>/Relational Operator' incorporates:
       *  Constant: '<S49>/P_LaneChSucThr'
       *  DotProduct: '<S49>/Dot Product'
       *  Sqrt: '<S49>/Sqrt'
       */
      rtb_Compare_ay = (sqrt(rtb_TmpSignalConversionAtDotP_0) <= 0.6);

      /* End of Outputs for SubSystem: '<S17>/02_F_LaneChAndTurnCorner' */

      /* Chart: '<S17>/01_F_SeleTrajLCProcess' */
      AutoVehicleControl_DWork.presentTicks_d =
        AutoVehicleControl_M->Timing.clockTick0;
      AutoVehicleControl_DWork.elapsedTicks_o =
        AutoVehicleControl_DWork.presentTicks_d -
        AutoVehicleControl_DWork.previousTicks_e;
      AutoVehicleControl_DWork.previousTicks_e =
        AutoVehicleControl_DWork.presentTicks_d;
      AutoVehicleControl_DWork.temporalCounter_i1_cr +=
        AutoVehicleControl_DWork.elapsedTicks_o;

      /* Gateway: 07_Perception&DecisionPlanning/01_F_LaneChange/03_F_SeleTraj/01_F_SeleTrajLCProcess */
      /* During: 07_Perception&DecisionPlanning/01_F_LaneChange/03_F_SeleTraj/01_F_SeleTrajLCProcess */
      if (AutoVehicleControl_DWork.is_active_c1_AutoVehicleControl == 0U) {
        /* Entry: 07_Perception&DecisionPlanning/01_F_LaneChange/03_F_SeleTraj/01_F_SeleTrajLCProcess */
        AutoVehicleControl_DWork.is_active_c1_AutoVehicleControl = 1U;

        /* Entry Internal: 07_Perception&DecisionPlanning/01_F_LaneChange/03_F_SeleTraj/01_F_SeleTrajLCProcess */
        /* Transition: '<S48>:5' */
        AutoVehicleControl_DWork.is_c1_AutoVehicleControl =
          AutoVehicleControl_IN_TrajNum1;
        AutoVehicleControl_DWork.temporalCounter_i1_cr = 0U;

        /* Entry 'TrajNum1': '<S48>:53' */
        AutoVehicleControl_B.V_TrajNum = 1.0;
      } else {
        switch (AutoVehicleControl_DWork.is_c1_AutoVehicleControl) {
         case AutoVehicleControl_IN_Defult1:
          /* During 'Defult1': '<S48>:80' */
          if (rtb_LogicalOperator2_g) {
            /* Transition: '<S48>:54' */
            AutoVehicleControl_DWork.is_c1_AutoVehicleControl =
              AutoVehicleControl_IN_TrajNum2;
            AutoVehicleControl_DWork.temporalCounter_i1_cr = 0U;

            /* Entry 'TrajNum2': '<S48>:6' */
            AutoVehicleControl_B.V_TrajNum = 2.0;
            AutoVehicleControl_B.V_LCTurnLeft = 1.0;
            AutoVehicleControl_B.V_LCProcess = 1.0;
          }
          break;

         case AutoVehicleControl_IN_Defult2:
          /* During 'Defult2': '<S48>:75' */
          if (rtb_Compare_fh) {
            /* Transition: '<S48>:76' */
            AutoVehicleControl_B.V_LCTurnRight = 1.0;
            AutoVehicleControl_B.V_LCProcess = 1.0;
            AutoVehicleControl_DWork.is_c1_AutoVehicleControl =
              AutoVehicleControl_IN_TrajNum1;
            AutoVehicleControl_DWork.temporalCounter_i1_cr = 0U;

            /* Entry 'TrajNum1': '<S48>:53' */
            AutoVehicleControl_B.V_TrajNum = 1.0;
          }
          break;

         case AutoVehicleControl_IN_Defult3:
          /* During 'Defult3': '<S48>:81' */
          if (rtb_Compare_fh) {
            /* Transition: '<S48>:84' */
            AutoVehicleControl_DWork.is_c1_AutoVehicleControl =
              AutoVehicleControl_IN_TrajNum3;
            AutoVehicleControl_DWork.temporalCounter_i1_cr = 0U;

            /* Entry 'TrajNum3': '<S48>:35' */
            AutoVehicleControl_B.V_TrajNum = 3.0;
            AutoVehicleControl_B.V_LCTurnRight = 1.0;
            AutoVehicleControl_B.V_LCProcess = 1.0;
          }
          break;

         case AutoVehicleControl_IN_Defult4:
          /* During 'Defult4': '<S48>:77' */
          if (rtb_LogicalOperator2_g) {
            /* Transition: '<S48>:78' */
            AutoVehicleControl_B.V_LCTurnLeft = 1.0;
            AutoVehicleControl_B.V_LCProcess = 1.0;
            AutoVehicleControl_DWork.is_c1_AutoVehicleControl =
              AutoVehicleControl_IN_TrajNum1;
            AutoVehicleControl_DWork.temporalCounter_i1_cr = 0U;

            /* Entry 'TrajNum1': '<S48>:53' */
            AutoVehicleControl_B.V_TrajNum = 1.0;
          }
          break;

         case AutoVehicleControl_IN_TrajNum1:
          /* During 'TrajNum1': '<S48>:53' */
          if ((rtb_V_APA_Abortfeedback != 0) && (AutoVehicleControl_B.Merge ==
               2.0)) {
            /* Transition: '<S48>:91' */
            AutoVehicleControl_DWork.is_c1_AutoVehicleControl =
              AutoVehicleControl_IN_TrajNum2;
            AutoVehicleControl_DWork.temporalCounter_i1_cr = 0U;

            /* Entry 'TrajNum2': '<S48>:6' */
            AutoVehicleControl_B.V_TrajNum = 2.0;
            AutoVehicleControl_B.V_LCTurnLeft = 1.0;
            AutoVehicleControl_B.V_LCProcess = 1.0;
          } else if ((AutoVehicleControl_DWork.temporalCounter_i1_cr >= 800U) &&
                     rtb_Compare_ay) {
            /* Transition: '<S48>:82' */
            AutoVehicleControl_DWork.is_c1_AutoVehicleControl =
              AutoVehicleControl_IN_Defult1;

            /* Entry 'Defult1': '<S48>:80' */
            AutoVehicleControl_B.V_LCTurnRight = 0.0;
            AutoVehicleControl_B.V_LCProcess = 0.0;
          } else if ((rtb_V_APA_Abortfeedback != 0) &&
                     (AutoVehicleControl_B.Merge == 3.0)) {
            /* Transition: '<S48>:97' */
            AutoVehicleControl_DWork.is_c1_AutoVehicleControl =
              AutoVehicleControl_IN_TrajNum3;
            AutoVehicleControl_DWork.temporalCounter_i1_cr = 0U;

            /* Entry 'TrajNum3': '<S48>:35' */
            AutoVehicleControl_B.V_TrajNum = 3.0;
            AutoVehicleControl_B.V_LCTurnRight = 1.0;
            AutoVehicleControl_B.V_LCProcess = 1.0;
          } else {
            if ((AutoVehicleControl_DWork.temporalCounter_i1_cr >= 800U) &&
                rtb_Compare_ay) {
              /* Transition: '<S48>:83' */
              AutoVehicleControl_DWork.is_c1_AutoVehicleControl =
                AutoVehicleControl_IN_Defult3;

              /* Entry 'Defult3': '<S48>:81' */
              AutoVehicleControl_B.V_LCTurnLeft = 0.0;
              AutoVehicleControl_B.V_LCProcess = 0.0;
            }
          }
          break;

         case AutoVehicleControl_IN_TrajNum2:
          /* During 'TrajNum2': '<S48>:6' */
          if ((rtb_V_APA_Abortfeedback != 0) && (AutoVehicleControl_B.Merge ==
               1.0)) {
            /* Transition: '<S48>:96' */
            AutoVehicleControl_B.V_LCTurnLeft = 0.0;
            AutoVehicleControl_B.V_LCTurnRight = 1.0;

            /* Transition: '<S48>:100' */
            AutoVehicleControl_DWork.is_c1_AutoVehicleControl =
              AutoVehicleControl_IN_TrajNum1;
            AutoVehicleControl_DWork.temporalCounter_i1_cr = 0U;

            /* Entry 'TrajNum1': '<S48>:53' */
            AutoVehicleControl_B.V_TrajNum = 1.0;
          } else {
            if ((AutoVehicleControl_DWork.temporalCounter_i1_cr >= 800U) &&
                rtb_Compare_ay) {
              /* Transition: '<S48>:41' */
              AutoVehicleControl_DWork.is_c1_AutoVehicleControl =
                AutoVehicleControl_IN_Defult2;

              /* Entry 'Defult2': '<S48>:75' */
              AutoVehicleControl_B.V_LCTurnLeft = 0.0;
              AutoVehicleControl_B.V_LCProcess = 0.0;
            }
          }
          break;

         default:
          /* During 'TrajNum3': '<S48>:35' */
          if ((rtb_V_APA_Abortfeedback != 0) && (AutoVehicleControl_B.Merge ==
               1.0)) {
            /* Transition: '<S48>:98' */
            AutoVehicleControl_B.V_LCTurnLeft = 1.0;
            AutoVehicleControl_B.V_LCTurnRight = 0.0;

            /* Transition: '<S48>:100' */
            AutoVehicleControl_DWork.is_c1_AutoVehicleControl =
              AutoVehicleControl_IN_TrajNum1;
            AutoVehicleControl_DWork.temporalCounter_i1_cr = 0U;

            /* Entry 'TrajNum1': '<S48>:53' */
            AutoVehicleControl_B.V_TrajNum = 1.0;
          } else {
            if ((AutoVehicleControl_DWork.temporalCounter_i1_cr >= 800U) &&
                rtb_Compare_ay) {
              /* Transition: '<S48>:42' */
              AutoVehicleControl_DWork.is_c1_AutoVehicleControl =
                AutoVehicleControl_IN_Defult4;

              /* Entry 'Defult4': '<S48>:77' */
              AutoVehicleControl_B.V_LCTurnRight = 0.0;
              AutoVehicleControl_B.V_LCProcess = 0.0;
            }
          }
          break;
        }
      }

      /* End of Chart: '<S17>/01_F_SeleTrajLCProcess' */
    } else {
      if (AutoVehicleControl_DWork.u_F_SeleTraj_MODE) {
        /* Disable for Chart: '<S17>/01_F_SeleTrajLCProcess' */
        AutoVehicleControl_DWork.presentTicks_d =
          AutoVehicleControl_M->Timing.clockTick0;
        AutoVehicleControl_DWork.elapsedTicks_o =
          AutoVehicleControl_DWork.presentTicks_d -
          AutoVehicleControl_DWork.previousTicks_e;
        AutoVehicleControl_DWork.previousTicks_e =
          AutoVehicleControl_DWork.presentTicks_d;
        AutoVehicleControl_DWork.temporalCounter_i1_cr +=
          AutoVehicleControl_DWork.elapsedTicks_o;

        /* Disable for Outport: '<S17>/V_TrajNum[-]' */
        AutoVehicleControl_B.V_TrajNum = 1.0;

        /* Disable for Outport: '<S17>/V_LCProcess[-]' */
        AutoVehicleControl_B.V_LCProcess = 1.0;
        AutoVehicleControl_DWork.u_F_SeleTraj_MODE = FALSE;
      }
    }

    /* End of Outputs for SubSystem: '<S9>/03_F_SeleTraj' */
  } else {
    if (AutoVehicleControl_DWork.u_F_LaneChange_MODE) {
      /* Disable for Enabled SubSystem: '<S9>/03_F_SeleTraj' */
      if (AutoVehicleControl_DWork.u_F_SeleTraj_MODE) {
        /* Disable for Chart: '<S17>/01_F_SeleTrajLCProcess' */
        AutoVehicleControl_DWork.presentTicks_d =
          AutoVehicleControl_M->Timing.clockTick0;
        AutoVehicleControl_DWork.elapsedTicks_o =
          AutoVehicleControl_DWork.presentTicks_d -
          AutoVehicleControl_DWork.previousTicks_e;
        AutoVehicleControl_DWork.previousTicks_e =
          AutoVehicleControl_DWork.presentTicks_d;
        AutoVehicleControl_DWork.temporalCounter_i1_cr +=
          AutoVehicleControl_DWork.elapsedTicks_o;

        /* Disable for Outport: '<S17>/V_TrajNum[-]' */
        AutoVehicleControl_B.V_TrajNum = 1.0;

        /* Disable for Outport: '<S17>/V_LCProcess[-]' */
        AutoVehicleControl_B.V_LCProcess = 1.0;
        AutoVehicleControl_DWork.u_F_SeleTraj_MODE = FALSE;
      }

      /* End of Disable for SubSystem: '<S9>/03_F_SeleTraj' */

      /* Disable for Outport: '<S9>/V_LCProcess[-]' */
      AutoVehicleControl_B.V_LCProcess = 1.0;
      AutoVehicleControl_DWork.u_F_LaneChange_MODE = FALSE;
    }
  }

  /* End of Outputs for SubSystem: '<S7>/01_F_LaneChange' */

  /* Switch: '<S13>/Switch' */
  if (/*AutoVehicleControl_B.V_LCProcess > 0.0*/0) {
    /* Saturate: '<S13>/Saturation' */
    if (rtb_Switch2_c >= 0.87266462599716477) {
      AutoVehicleControl_B.O_DesSteerAng = 0.87266462599716477;
    } else if (rtb_Switch2_c <= -0.87266462599716477) {
      AutoVehicleControl_B.O_DesSteerAng = -0.87266462599716477;
    } else {
      AutoVehicleControl_B.O_DesSteerAng = rtb_Switch2_c;
    }

    /* End of Saturate: '<S13>/Saturation' */
  } else {
    AutoVehicleControl_B.O_DesSteerAng = rtb_Switch2_c;
  }

  /* End of Switch: '<S13>/Switch' */

  /* Chart: '<S83>/F_SteerAngTrans' */
  AutoVehicleControl_DWork.presentTicks_h =
    AutoVehicleControl_M->Timing.clockTick0;
  AutoVehicleControl_DWork.elapsedTicks_i =
    AutoVehicleControl_DWork.presentTicks_h -
    AutoVehicleControl_DWork.previousTicks_a;
  AutoVehicleControl_DWork.previousTicks_a =
    AutoVehicleControl_DWork.presentTicks_h;
  AutoVehicleControl_DWork.temporalCounter_i1_c +=
    AutoVehicleControl_DWork.elapsedTicks_i;

  /* Gateway: 08_EnableLogic/01_F_DesTrans/F_SteerAngTrans */
  /* During: 08_EnableLogic/01_F_DesTrans/F_SteerAngTrans */
  if (AutoVehicleControl_DWork.is_active_c5_AutoVehicleControl == 0U) {
    /* Entry: 08_EnableLogic/01_F_DesTrans/F_SteerAngTrans */
    AutoVehicleControl_DWork.is_active_c5_AutoVehicleControl = 1U;

    /* Entry Internal: 08_EnableLogic/01_F_DesTrans/F_SteerAngTrans */
    /* Transition: '<S85>:5' */
    AutoVehicleControl_DWork.is_c5_AutoVehicleControl =
      AutoVehicleControl_IN_DisEna;
  } else {
    switch (AutoVehicleControl_DWork.is_c5_AutoVehicleControl) {
     case AutoVehicleControl_IN_Defult2_j:
      /* During 'Defult2': '<S85>:25' */
      if (AutoVehicleControl_DWork.temporalCounter_i1_c >= 2000U) {
        /* Transition: '<S85>:23' */
        AutoVehicleControl_DWork.is_c5_AutoVehicleControl =
          AutoVehicleControl_IN_DisEna;
      }
      break;

     case AutoVehicleControl_IN_Defult3_i:
      /* During 'Defult3': '<S85>:35' */
      if (AutoVehicleControl_DWork.temporalCounter_i1_c >= 6U) {
        /* Transition: '<S85>:36' */
        AutoVehicleControl_DWork.is_c5_AutoVehicleControl =
          AutoVehicleControl_IN_Ena;
      }
      break;

     case AutoVehicleControl_IN_DisEna:
      /* During 'DisEna': '<S85>:4' */
      if (rtb_DataStoreRead4_b != 0) {
        /* Transition: '<S85>:7' */
        AutoVehicleControl_DWork.is_c5_AutoVehicleControl =
          AutoVehicleControl_IN_Defult3_i;
        AutoVehicleControl_DWork.temporalCounter_i1_c = 0U;
      } else {
        AutoVehicleControl_B.V_DesSteerAng = 0.0;
      }
      break;

     default:
      /* During 'Ena': '<S85>:6' */
      if ((rtb_DataStoreRead4_b != 0) && (rtb_V_APA_Abortfeedback != 0)) {
        /* Transition: '<S85>:9' */
        AutoVehicleControl_DWork.is_c5_AutoVehicleControl =
          AutoVehicleControl_IN_Defult2_j;
        AutoVehicleControl_DWork.temporalCounter_i1_c = 0U;

        /* Entry 'Defult2': '<S85>:25' */
        AutoVehicleControl_B.V_DesSteerAng = 0.0;
      } else if (!(rtb_DataStoreRead4_b != 0)) {
        /* Transition: '<S85>:29' */
        AutoVehicleControl_DWork.is_c5_AutoVehicleControl =
          AutoVehicleControl_IN_DisEna;
      } else {
        AutoVehicleControl_B.V_DesSteerAng = AutoVehicleControl_B.O_DesSteerAng;
      }
      break;
    }
  }

  /* End of Chart: '<S83>/F_SteerAngTrans' */

  /* Switch: '<S8>/Switch' incorporates:
   *  Constant: '<S8>/Constant'
   */
  if (rtb_Compare_n) {
    rtb_Switch_n = 0.0;
  } else {
    rtb_Switch_n = AutoVehicleControl_B.V_DesSteerAng;
  }

  /* End of Switch: '<S8>/Switch' */

  /* Sum: '<S8>/Add1' incorporates:
   *  Constant: '<S8>/Constant4'
   *  Product: '<S8>/Product'
   *  UnitDelay: '<S8>/Unit Delay2'
   */
  rtb_Switch_n = rtb_Switch_n * 57.295779513082323 -
    AutoVehicleControl_DWork.UnitDelay2_DSTATE;

  /* MinMax: '<S8>/MinMax' */
  if ((-8.0 >= rtb_Switch_n) || rtIsNaN(rtb_Switch_n)) {
    rtb_Switch_n = -8.0;
  }

  /* MinMax: '<S8>/MinMax1' incorporates:
   *  MinMax: '<S8>/MinMax'
   */
  if (!(rtb_Switch_n <= 8.0)) {
    rtb_Switch_n = 8.0;
  }

  /* Sum: '<S8>/Add2' incorporates:
   *  MinMax: '<S8>/MinMax'
   *  MinMax: '<S8>/MinMax1'
   *  UnitDelay: '<S8>/Unit Delay2'
   */
  rtb_V_Traje1X = rtb_Switch_n + AutoVehicleControl_DWork.UnitDelay2_DSTATE;

  /* Saturate: '<S8>/Saturation' */
  if (rtb_V_Traje1X >= 720.0) {
    rtb_Switch_n = 720.0;
  } else if (rtb_V_Traje1X <= -720.0) {
    rtb_Switch_n = -720.0;
  } else {
    rtb_Switch_n = rtb_V_Traje1X;//xiong
  }

  /* DataTypeConversion: '<S8>/Data Type Conversion4' incorporates:
   *  Saturate: '<S8>/Saturation'
   */
  rtb_Switch_n = floor(rtb_Switch_n * 16.0);
  if (rtIsNaN(rtb_Switch_n) || rtIsInf(rtb_Switch_n)) {
    rtb_Switch_n = 0.0;
  } else {
    rtb_Switch_n = fmod(rtb_Switch_n, 65536.0);
  }

  /* DataStoreWrite: '<S8>/Data Store Write' incorporates:
   *  DataTypeConversion: '<S8>/Data Type Conversion4'
   */
  StC_SteeringAngleRequest = (int16_T)(rtb_Switch_n < 0.0 ? (int16_T)-(int16_T)
    (uint16_T)-rtb_Switch_n : (int16_T)(uint16_T)rtb_Switch_n);

  /* Outputs for Enabled SubSystem: '<S7>/02_F_LongControl' incorporates:
   *  EnablePort: '<S10>/Enable'
   */
  if (rtb_DataStoreRead4_g > 0) {
    if (!AutoVehicleControl_DWork.u_F_LongControl_MODE) {
      /* InitializeConditions for Chart: '<S56>/03_F_DesState' */
      AutoVehicleControl_DWork.is_active_c3_AutoVehicleControl = 0U;
      AutoVehicleControl_DWork.is_c3_AutoVehicleControl =
        AutoVehicleC_IN_NO_ACTIVE_CHILD;

      /* InitializeConditions for Delay: '<S57>/Delay' */
      AutoVehicleControl_DWork.Delay_DSTATE = FALSE;

      /* InitializeConditions for DiscreteIntegrator: '<S71>/Discrete-Time Integrator' */
      AutoVehicleControl_DWork.DiscreteTimeIntegrator_DSTATE = 0.0;
      AutoVehicleControl_DWork.DiscreteTimeIntegrator_PrevRese = 0;

      /* InitializeConditions for Delay: '<S57>/Delay1' */
      AutoVehicleControl_DWork.Delay1_DSTATE = 0.0;

      /* InitializeConditions for Chart: '<S10>/04_F_TrafficLightStop' */
      AutoVehicleControl_DWork.is_active_c6_AutoVehicleControl = 0U;
      AutoVehicleControl_DWork.is_c6_AutoVehicleControl =
        AutoVehicleC_IN_NO_ACTIVE_CHILD;
      AutoVehicleControl_DWork.elapsedTicks_h = 0U;

      /* InitializeConditions for Chart: '<S10>/05_F_TrajEndStop' */
      AutoVehicleControl_DWork.is_active_c2_AutoVehicleControl = 0U;
      AutoVehicleControl_DWork.is_c2_AutoVehicleControl =
        AutoVehicleC_IN_NO_ACTIVE_CHILD;
      AutoVehicleControl_DWork.elapsedTicks_a = 0U;

      /* Enable for Chart: '<S10>/04_F_TrafficLightStop' */
      AutoVehicleControl_DWork.presentTicks_o =
        AutoVehicleControl_M->Timing.clockTick0;
      AutoVehicleControl_DWork.previousTicks_b =
        AutoVehicleControl_DWork.presentTicks_o;

      /* Enable for Chart: '<S10>/05_F_TrajEndStop' */
      AutoVehicleControl_DWork.presentTicks_b =
        AutoVehicleControl_M->Timing.clockTick0;
      AutoVehicleControl_DWork.previousTicks_o =
        AutoVehicleControl_DWork.presentTicks_b;
      AutoVehicleControl_DWork.u_F_LongControl_MODE = TRUE;
    }

    /* Sum: '<S55>/Add1' incorporates:
     *  DataTypeConversion: '<S10>/Data Type Conversion'
     */
    AutoVehicleControl_B.V_ObjSpdms = (real_T)rtb_V_VehSpdms *
      4.76837158203125E-7 + rtb_DataStoreRead16;

    /* Logic: '<S55>/Logical Operator1' incorporates:
     *  Constant: '<S62>/Constant'
     *  Constant: '<S63>/Constant'
     *  Constant: '<S64>/Constant'
     *  Logic: '<S55>/Logical Operator'
     *  RelationalOperator: '<S55>/Relational Operator1'
     *  RelationalOperator: '<S62>/Compare'
     *  RelationalOperator: '<S63>/Compare'
     *  RelationalOperator: '<S64>/Compare'
     */
    rtb_Compare_fh = (((rtb_DataStoreRead17 != 0.0) || (rtb_DataStoreRead16 !=
      0.0) || (rtb_V_VehPosAng_f1 != 0.0)) && (AutoVehicleControl_B.V_ObjSpdms <=
      rtb_V_TrajSpd));

    /* Sum: '<S65>/Add1' incorporates:
     *  Bias: '<S65>/P_StopDis'
     *  DataTypeConversion: '<S10>/Data Type Conversion'
     *  Gain: '<S65>/P_DisConDisCoef'
     *  Gain: '<S65>/P_DisConSpdCoef'
     *  Gain: '<S65>/P_HeadTime'
     *  Sum: '<S65>/Add'
     */
    rtb_MathFunction1 = (((real_T)rtb_V_VehSpdms * 4.76837158203125E-7 * 1.5 +
                          12) - rtb_V_VehPosAng_f1) * -0.4113 + 0.7071 *
      rtb_DataStoreRead16;

    /* Chart: '<S56>/03_F_DesState' */
    /* Gateway: 07_Perception&DecisionPlanning/02_F_LongControl/02_F_CalDesAcc/03_F_DesState */
    /* During: 07_Perception&DecisionPlanning/02_F_LongControl/02_F_CalDesAcc/03_F_DesState */
    if (AutoVehicleControl_DWork.is_active_c3_AutoVehicleControl == 0U) {
      /* Entry: 07_Perception&DecisionPlanning/02_F_LongControl/02_F_CalDesAcc/03_F_DesState */
      AutoVehicleControl_DWork.is_active_c3_AutoVehicleControl = 1U;

      /* Entry Internal: 07_Perception&DecisionPlanning/02_F_LongControl/02_F_CalDesAcc/03_F_DesState */
      /* Transition: '<S67>:5' */
      AutoVehicleControl_DWork.is_c3_AutoVehicleControl =
        AutoVehicleContro_IN_SpdControl;
    } else if (AutoVehicleControl_DWork.is_c3_AutoVehicleControl ==
               AutoVehicleContro_IN_DisControl) {
      /* During 'DisControl': '<S67>:6' */
      if ((rtb_Compare_fh == 0) || (rtb_MathFunction1 >= 0.0)) {
        /* Transition: '<S67>:9' */
        AutoVehicleControl_DWork.is_c3_AutoVehicleControl =
          AutoVehicleContro_IN_SpdControl;
      } else {
        AutoVehicleControl_B.V_DesState = 2.0;
      }
    } else {
      /* During 'SpdControl': '<S67>:4' */
      if ((rtb_Compare_fh == 1) && (rtb_MathFunction1 < 0.0)) {
        /* Transition: '<S67>:7' */
        AutoVehicleControl_DWork.is_c3_AutoVehicleControl =
          AutoVehicleContro_IN_DisControl;
      } else {
        AutoVehicleControl_B.V_DesState = 1.0;
      }
    }

    /* End of Chart: '<S56>/03_F_DesState' */

    /* Logic: '<S56>/Logical Operator' incorporates:
     *  Constant: '<S68>/Constant'
     *  RelationalOperator: '<S68>/Compare'
     */
    rtb_LogicalOperator_o = !(AutoVehicleControl_B.V_DesState == 2.0);

    /* Outputs for Enabled SubSystem: '<S56>/02_F_SpdControl' incorporates:
     *  EnablePort: '<S66>/Enable'
     */
    if (rtb_LogicalOperator_o) {
      if (!AutoVehicleControl_DWork.u_F_SpdControl_MODE) {
        /* InitializeConditions for DiscreteIntegrator: '<S66>/Discrete-Time Integrator' */
        AutoVehicleControl_DWork.DiscreteTimeIntegrator_IC_LOADI = 1U;
        AutoVehicleControl_DWork.DiscreteTimeIntegrator_PrevRe_m = 2;

        /* InitializeConditions for RateLimiter: '<S66>/P_DesAccSpd_RateLim' */
        AutoVehicleControl_DWork.PrevY = 0.0;
        AutoVehicleControl_DWork.u_F_SpdControl_MODE = TRUE;
      }

      /* DiscreteIntegrator: '<S66>/Discrete-Time Integrator' incorporates:
       *  DataTypeConversion: '<S10>/Data Type Conversion'
       */
      if (AutoVehicleControl_DWork.DiscreteTimeIntegrator_IC_LOADI != 0) {
        AutoVehicleControl_DWork.DiscreteTimeIntegrator_DSTATE_e = (real_T)
          rtb_V_VehSpdms * 4.76837158203125E-7;
      }

      if (rtb_LogicalOperator_o &&
          (AutoVehicleControl_DWork.DiscreteTimeIntegrator_PrevRe_m <= 0)) {
        AutoVehicleControl_DWork.DiscreteTimeIntegrator_DSTATE_e = (real_T)
          rtb_V_VehSpdms * 4.76837158203125E-7;
      }

      /* Gain: '<S66>/P_SpdConCoef' incorporates:
       *  DataTypeConversion: '<S10>/Data Type Conversion'
       *  DiscreteIntegrator: '<S66>/Discrete-Time Integrator'
       *  Sum: '<S66>/Add2'
       */
      rtb_Switch_n = (AutoVehicleControl_DWork.DiscreteTimeIntegrator_DSTATE_e -
                      (real_T)rtb_V_VehSpdms * 4.76837158203125E-7) * 1.3;

      /* RateLimiter: '<S66>/P_DesAccSpd_RateLim' */
      rtb_Product_e = rtb_Switch_n - AutoVehicleControl_DWork.PrevY;
      if (rtb_Product_e > 0.019) {
        rtb_Product_e = AutoVehicleControl_DWork.PrevY + 0.019;
      } else if (rtb_Product_e < -0.019) {
        rtb_Product_e = AutoVehicleControl_DWork.PrevY + -0.019;
      } else {
        rtb_Product_e = rtb_Switch_n;
      }

      AutoVehicleControl_DWork.PrevY = rtb_Product_e;

      /* End of RateLimiter: '<S66>/P_DesAccSpd_RateLim' */

      /* Switch: '<S66>/Switch1' */
      if (rtb_Switch_n >= 0.0) {
        /* Saturate: '<S66>/P_DesAccSpd_Satuation' */
        if (rtb_Product_e >= 1.0) {
          AutoVehicleControl_B.Switch1 = 1.0;
        } else if (rtb_Product_e <= 0.0) {
          AutoVehicleControl_B.Switch1 = 0.0;
        } else {
          AutoVehicleControl_B.Switch1 = rtb_Product_e;
        }

        /* End of Saturate: '<S66>/P_DesAccSpd_Satuation' */
      } else {
        AutoVehicleControl_B.Switch1 = rtb_Switch_n;
      }

      /* End of Switch: '<S66>/Switch1' */

      /* Update for DiscreteIntegrator: '<S66>/Discrete-Time Integrator' incorporates:
       *  DiscreteIntegrator: '<S66>/Discrete-Time Integrator'
       *  Sum: '<S66>/Add1'
       */
      AutoVehicleControl_DWork.DiscreteTimeIntegrator_IC_LOADI = 0U;
      AutoVehicleControl_DWork.DiscreteTimeIntegrator_DSTATE_e += (rtb_V_TrajSpd
        - AutoVehicleControl_DWork.DiscreteTimeIntegrator_DSTATE_e) * 0.01;
      if (rtb_LogicalOperator_o) {
        AutoVehicleControl_DWork.DiscreteTimeIntegrator_PrevRe_m = 1;
      } else {
        AutoVehicleControl_DWork.DiscreteTimeIntegrator_PrevRe_m = 0;
      }

      /* End of Update for DiscreteIntegrator: '<S66>/Discrete-Time Integrator' */
    } else {
      if (AutoVehicleControl_DWork.u_F_SpdControl_MODE) {
        AutoVehicleControl_DWork.u_F_SpdControl_MODE = FALSE;
      }
    }




    /* End of Outputs for SubSystem: '<S56>/02_F_SpdControl' */

    /* MultiPortSwitch: '<S56>/Multiport Switch' */
    switch ((int32_T)AutoVehicleControl_B.V_DesState) {
     case 1:
      AutoVehicleControl_B.testpoint1 = AutoVehicleControl_B.Switch1;
      break;

     case 2:
      AutoVehicleControl_B.testpoint1 = rtb_MathFunction1;
      break;

     default:
      AutoVehicleControl_B.testpoint1 = AutoVehicleControl_B.Switch1;
      break;
    }

    //RCLCPP_INFO(rclcpp::get_logger("control_node"), "V_DesAcc:[%f] V_ObjSpd:[%f]", AutoVehicleControl_B.testpoint1,AutoVehicleControl_B.V_ObjSpdms);

    /* End of MultiPortSwitch: '<S56>/Multiport Switch' */

    /* Lookup_n-D: '<S57>/T_RoadRes' incorporates:
     *  DataTypeConversion: '<S57>/Data Type Conversion1'
     */
    rtb_Switch_n = look1_linlxpw((real_T)rtb_V_VehSpdms * 4.76837158203125E-7,
                                 *(real_T (*)[6])
      AutoVehicleControl_ConstP.T_RoadRes_bp01Data, *(real_T (*)[6])
      AutoVehicleControl_ConstP.T_RoadRes_tableData, 5U);

    /* Logic: '<S71>/Logical Operator' incorporates:
     *  Delay: '<S57>/Delay'
     */
    rtb_LogicalOperator_l = AutoVehicleControl_DWork.Delay_DSTATE;

    /* DiscreteIntegrator: '<S71>/Discrete-Time Integrator' */
    if (rtb_LogicalOperator_l ||
        (AutoVehicleControl_DWork.DiscreteTimeIntegrator_PrevRese != 0)) {
      AutoVehicleControl_DWork.DiscreteTimeIntegrator_DSTATE = 0.0;
    }

    /* Sum: '<S57>/Add3' incorporates:
     *  DataStoreRead: '<S1>/Data Store Read3'
     */
    rtb_Add1_o = AutoVehicleControl_B.testpoint1 - (real_T)ESP_LongAccel *
      0.00390625;

    /* Sum: '<S57>/Add' incorporates:
     *  DataStoreRead: '<S2>/Data Store Read3'
     *  DiscreteIntegrator: '<S71>/Discrete-Time Integrator'
     *  Gain: '<S57>/P_AccPCoef'
     *  Gain: '<S57>/P_SlopeResisFactor[-]'
     *  Gain: '<S57>/P_VehMass'
     *  Sum: '<S57>/Add2'
     */
    
    //Proportional   0.3-->0.5-->1
    rtb_Switch_n = (((10 * rtb_Add1_o +
                      AutoVehicleControl_DWork.DiscreteTimeIntegrator_DSTATE) +
                     AutoVehicleControl_B.testpoint1) * 1720.0 + rtb_Switch_n) +
      0.95 * V_SlopeResisf32s20;  

    /* Lookup_n-D: '<S57>/T_TransSpdRaio' incorporates:
     *  DataStoreRead: '<S1>/Data Store Read8'
     *  DataTypeConversion: '<S57>/Data Type Conversion2'
     */
    rtb_Product_e = look1_linlxpw((real_T)AT_ActualGear, *(real_T (*)[6])
      AutoVehicleControl_ConstP.T_TransSpdRaio_bp01Da, *(real_T (*)[6])
      AutoVehicleControl_ConstP.T_TransSpdRaio_tableD, 5U);

    /* Product: '<S57>/Divide' incorporates:
     *  Constant: '<S57>/P_BaseTrq'
     *  DataStoreRead: '<S1>/Data Store Read11'
     *  Gain: '<S57>/Gain1'
     *  Gain: '<S57>/P_TyreRadius'
     *  Product: '<S57>/Product'
     */
    rtb_Product_e = 0.352 * rtb_Switch_n / rtb_Product_e / (164.0 * (real_T)
      EMS_MaxIndicatedTorq * 3.0517578125E-5 * 350.0);

    /* Switch: '<S72>/Switch2' incorporates:
     *  Constant: '<S70>/Constant3'
     *  Constant: '<S70>/Constant4'
     *  Delay: '<S57>/Delay1'
     *  RelationalOperator: '<S72>/LowerRelop1'
     *  RelationalOperator: '<S72>/UpperRelop'
     *  Switch: '<S72>/Switch'
     */
    if (AutoVehicleControl_DWork.Delay1_DSTATE > 1.0) {
      rtb_MathFunction1 = 1.0;
    } else if (AutoVehicleControl_DWork.Delay1_DSTATE < -1.0) {
      /* Switch: '<S72>/Switch' incorporates:
       *  Constant: '<S70>/Constant4'
       */
      rtb_MathFunction1 = -1.0;
    } else {
      rtb_MathFunction1 = AutoVehicleControl_DWork.Delay1_DSTATE;
    }

    /* End of Switch: '<S72>/Switch2' */

    /* Switch: '<S71>/Switch1' incorporates:
     *  Constant: '<S70>/Constant1'
     *  Constant: '<S70>/Constant2'
     *  DiscreteIntegrator: '<S71>/Discrete-Time Integrator'
     *  RelationalOperator: '<S71>/Relational Operator'
     *  RelationalOperator: '<S71>/Relational Operator1'
     *  Saturate: '<S71>/0...inf'
     *  Switch: '<S71>/Switch'
     */
    if (AutoVehicleControl_DWork.DiscreteTimeIntegrator_DSTATE >= 1.0) {
      /* Saturate: '<S71>/-inf...0' */
      if (!(rtb_MathFunction1 <= 0.0)) {
        rtb_MathFunction1 = 0.0;
      }

      /* End of Saturate: '<S71>/-inf...0' */
    } else {
      if ((AutoVehicleControl_DWork.DiscreteTimeIntegrator_DSTATE <= -1.0) &&
          (!(rtb_MathFunction1 >= 0.0))) {
        /* Switch: '<S71>/Switch' incorporates:
         *  Saturate: '<S71>/0...inf'
         */
        rtb_MathFunction1 = 0.0;
      }
    }

    /* End of Switch: '<S71>/Switch1' */

    /* Gain: '<S57>/P_VehMassInv' */
    rtb_Switch_n *= 0.00058139534883720929 * 0.1;

    /* Saturate: '<S57>/Saturation' */
    if (rtb_Switch_n >= 0.0) {
      AutoVehicleControl_B.O_DesBrk = 0.0;
    } else if (rtb_Switch_n <= -5.0) {
      AutoVehicleControl_B.O_DesBrk = -5.0;
    } else {
      AutoVehicleControl_B.O_DesBrk = rtb_Switch_n;
    }

    /* End of Saturate: '<S57>/Saturation' */

    /* Gain: '<S57>/Gain' */
    rtb_Switch_n = 100.0 * rtb_Product_e;

    /* Saturate: '<S57>/Saturation1' */
    if (rtb_Switch_n >= 100.0) {
      AutoVehicleControl_B.O_DesDrv = 100.0;
    } else if (rtb_Switch_n <= 0.0) {
      AutoVehicleControl_B.O_DesDrv = 0.0;
    } else {
      AutoVehicleControl_B.O_DesDrv = rtb_Switch_n;
    }

    /* End of Saturate: '<S57>/Saturation1' */

    /* Logic: '<S10>/Logical Operator' incorporates:
     *  Constant: '<S60>/Constant'
     *  DataStoreRead: '<S5>/Data Store Read1'
     *  RelationalOperator: '<S60>/Compare'
     */
    rtb_Compare_fh = ((V_TrafficLightDet != 0) && (rtb_V_RefPoint == 6646U));

    /* Chart: '<S10>/04_F_TrafficLightStop' */
    AutoVehicleControl_DWork.presentTicks_o =
      AutoVehicleControl_M->Timing.clockTick0;
    AutoVehicleControl_DWork.elapsedTicks_h =
      AutoVehicleControl_DWork.presentTicks_o -
      AutoVehicleControl_DWork.previousTicks_b;
    AutoVehicleControl_DWork.previousTicks_b =
      AutoVehicleControl_DWork.presentTicks_o;
    AutoVehicleControl_DWork.temporalCounter_i1_l +=
      AutoVehicleControl_DWork.elapsedTicks_h;

    /* Gateway: 07_Perception&DecisionPlanning/02_F_LongControl/04_F_TrafficLightStop */
    /* During: 07_Perception&DecisionPlanning/02_F_LongControl/04_F_TrafficLightStop */
    if (AutoVehicleControl_DWork.is_active_c6_AutoVehicleControl == 0U) {
      /* Entry: 07_Perception&DecisionPlanning/02_F_LongControl/04_F_TrafficLightStop */
      AutoVehicleControl_DWork.is_active_c6_AutoVehicleControl = 1U;

      /* Entry Internal: 07_Perception&DecisionPlanning/02_F_LongControl/04_F_TrafficLightStop */
      /* Transition: '<S58>:5' */
      AutoVehicleControl_DWork.is_c6_AutoVehicleControl =
        AutoVehicleControl_IN_NoStop;

      /* Entry 'NoStop': '<S58>:4' */
      AutoVehicleControl_B.B_TrafficLightStopEna = 0.0;
    } else if (AutoVehicleControl_DWork.is_c6_AutoVehicleControl ==
               AutoVehicleControl_IN_NoStop) {
      /* During 'NoStop': '<S58>:4' */
      if (rtb_Compare_fh == 1) {
        /* Transition: '<S58>:7' */
        AutoVehicleControl_DWork.is_c6_AutoVehicleControl =
          AutoVehicleControl_IN_Stop;
        AutoVehicleControl_DWork.temporalCounter_i1_l = 0U;

        /* Entry 'Stop': '<S58>:6' */
        AutoVehicleControl_B.B_TrafficLightStopEna = 1.0;
      }
    } else {
      /* During 'Stop': '<S58>:6' */
      if ((AutoVehicleControl_DWork.temporalCounter_i1_l >= 100U) &&
          (rtb_Compare_fh == 0)) {
        /* Transition: '<S58>:9' */
        AutoVehicleControl_DWork.is_c6_AutoVehicleControl =
          AutoVehicleControl_IN_NoStop;

        /* Entry 'NoStop': '<S58>:4' */
        AutoVehicleControl_B.B_TrafficLightStopEna = 0.0;
      }
    }

    /* End of Chart: '<S10>/04_F_TrafficLightStop' */

    /* Chart: '<S10>/05_F_TrajEndStop' incorporates:
     *  Constant: '<S61>/Constant'
     *  RelationalOperator: '<S61>/Compare'
     */
    AutoVehicleControl_DWork.presentTicks_b =
      AutoVehicleControl_M->Timing.clockTick0;
    AutoVehicleControl_DWork.elapsedTicks_a =
      AutoVehicleControl_DWork.presentTicks_b -
      AutoVehicleControl_DWork.previousTicks_o;
    AutoVehicleControl_DWork.previousTicks_o =
      AutoVehicleControl_DWork.presentTicks_b;
    AutoVehicleControl_DWork.temporalCounter_i1_f +=
      AutoVehicleControl_DWork.elapsedTicks_a;

    /* Gateway: 07_Perception&DecisionPlanning/02_F_LongControl/05_F_TrajEndStop */
    /* During: 07_Perception&DecisionPlanning/02_F_LongControl/05_F_TrajEndStop */
    if (AutoVehicleControl_DWork.is_active_c2_AutoVehicleControl == 0U) {
      /* Entry: 07_Perception&DecisionPlanning/02_F_LongControl/05_F_TrajEndStop */
      AutoVehicleControl_DWork.is_active_c2_AutoVehicleControl = 1U;

      /* Entry Internal: 07_Perception&DecisionPlanning/02_F_LongControl/05_F_TrajEndStop */
      /* Transition: '<S59>:5' */
      AutoVehicleControl_DWork.is_c2_AutoVehicleControl =
        AutoVehicleControl_IN_NoStop;

      /* Entry 'NoStop': '<S59>:4' */
      AutoVehicleControl_B.B_TrajEndEna = 0.0;
    } else if (AutoVehicleControl_DWork.is_c2_AutoVehicleControl ==
               AutoVehicleControl_IN_NoStop) {
      /* During 'NoStop': '<S59>:4' */
      //停车点设置 || vhicle_stop
      if (((rtb_V_RefPoint >= 14500U) == 1) || vhicle_stop ) {
        /* Transition: '<S59>:7' */
        cout<<"停车是不是11111111111111111"<<endl;
        AutoVehicleControl_DWork.is_c2_AutoVehicleControl =
          AutoVehicleControl_IN_Stop;
        AutoVehicleControl_DWork.temporalCounter_i1_f = 0U;

        /* Entry 'Stop': '<S59>:6' */
        AutoVehicleControl_B.B_TrajEndEna = 1.0;
      }
    } else {
      /* During 'Stop': '<S59>:6' */
      if (AutoVehicleControl_DWork.temporalCounter_i1_f >= 1000U) {
        /* Transition: '<S59>:9' */
        AutoVehicleControl_DWork.is_c2_AutoVehicleControl =
          AutoVehicleControl_IN_NoStop;

        /* Entry 'NoStop': '<S59>:4' */
        AutoVehicleControl_B.B_TrajEndEna = 0.0;
      }
    }

    /* End of Chart: '<S10>/05_F_TrajEndStop' */

    /* Switch: '<S10>/Switch2' incorporates:
     *  Constant: '<S10>/Stop1'
     *  Constant: '<S10>/Stop2'
     *  Logic: '<S10>/Logical Operator2'
     *  Switch: '<S10>/Switch3'
     */
    if ((AutoVehicleControl_B.B_TrafficLightStopEna != 0.0) ||
        (AutoVehicleControl_B.B_TrajEndEna != 0.0)) {
      AutoVehicleControl_B.Switch2 = -1.5;
      AutoVehicleControl_B.Switch3_p = 0.0;
    } else {
      AutoVehicleControl_B.Switch2 = AutoVehicleControl_B.O_DesBrk;
      AutoVehicleControl_B.Switch3_p = AutoVehicleControl_B.O_DesDrv;
    }

    /* End of Switch: '<S10>/Switch2' */

    /* Update for Delay: '<S57>/Delay' incorporates:
     *  RelationalOperator: '<S69>/Compare'
     */
    AutoVehicleControl_DWork.Delay_DSTATE = (rtb_V_VehSpdms != 0U);

    /* Update for DiscreteIntegrator: '<S71>/Discrete-Time Integrator' */
    if (!rtb_LogicalOperator_l) {
      AutoVehicleControl_DWork.DiscreteTimeIntegrator_DSTATE += 0.01 *
        rtb_MathFunction1;
    }

    if (rtb_LogicalOperator_l) {
      AutoVehicleControl_DWork.DiscreteTimeIntegrator_PrevRese = 1;
    } else {
      AutoVehicleControl_DWork.DiscreteTimeIntegrator_PrevRese = 0;
    }

    /* End of Update for DiscreteIntegrator: '<S71>/Discrete-Time Integrator' */

    /* Update for Delay: '<S57>/Delay1' incorporates:
     *  Gain: '<S57>/P_AccICoef'
     */
     //Integral    0.1-->0.3-->0.5
    AutoVehicleControl_DWork.Delay1_DSTATE = 0.001 * rtb_Add1_o;
  } else {
    if (AutoVehicleControl_DWork.u_F_LongControl_MODE) {
      /* Disable for Enabled SubSystem: '<S56>/02_F_SpdControl' */
      if (AutoVehicleControl_DWork.u_F_SpdControl_MODE) {
        AutoVehicleControl_DWork.u_F_SpdControl_MODE = FALSE;
      }

      /* End of Disable for SubSystem: '<S56>/02_F_SpdControl' */

      /* Disable for Chart: '<S10>/04_F_TrafficLightStop' */
      AutoVehicleControl_DWork.presentTicks_o =
        AutoVehicleControl_M->Timing.clockTick0;
      AutoVehicleControl_DWork.elapsedTicks_h =
        AutoVehicleControl_DWork.presentTicks_o -
        AutoVehicleControl_DWork.previousTicks_b;
      AutoVehicleControl_DWork.previousTicks_b =
        AutoVehicleControl_DWork.presentTicks_o;
      AutoVehicleControl_DWork.temporalCounter_i1_l +=
        AutoVehicleControl_DWork.elapsedTicks_h;

      /* Disable for Chart: '<S10>/05_F_TrajEndStop' */
      AutoVehicleControl_DWork.presentTicks_b =
        AutoVehicleControl_M->Timing.clockTick0;
      AutoVehicleControl_DWork.elapsedTicks_a =
        AutoVehicleControl_DWork.presentTicks_b -
        AutoVehicleControl_DWork.previousTicks_o;
      AutoVehicleControl_DWork.previousTicks_o =
        AutoVehicleControl_DWork.presentTicks_b;
      AutoVehicleControl_DWork.temporalCounter_i1_f +=
        AutoVehicleControl_DWork.elapsedTicks_a;

      /* Disable for Outport: '<S10>/V_DesBrk[m//s2]' */
      AutoVehicleControl_B.Switch2 = 0.0;

      /* Disable for Outport: '<S10>/V_DesDrv[%]' */
      AutoVehicleControl_B.Switch3_p = 0.0;
      AutoVehicleControl_DWork.u_F_LongControl_MODE = FALSE;
    }
  }

  /* End of Outputs for SubSystem: '<S7>/02_F_LongControl' */

  /* If: '<S83>/If1' incorporates:
   *  DataStoreRead: '<S1>/Data Store Read9'
   */
  if ((rtb_DataStoreRead4_b == 1) && (EMS_BrakePedalStatus == 0)) {
    /* Outputs for IfAction SubSystem: '<S83>/If Action Subsystem4' incorporates:
     *  ActionPort: '<S87>/Action Port'
     */
    /* DataTypeConversion: '<S83>/Data Type Conversion' incorporates:
     *  Inport: '<S87>/V_DesDrvIn[%]'
     */
    rtb_Switch_n = floor(AutoVehicleControl_B.Switch3_p * 256.0);

    /* End of Outputs for SubSystem: '<S83>/If Action Subsystem4' */
    if (rtIsNaN(rtb_Switch_n) || rtIsInf(rtb_Switch_n)) {
      rtb_Switch_n = 0.0;
    } else {
      rtb_Switch_n = fmod(rtb_Switch_n, 65536.0);
    }

    rtb_Add3_a = (uint16_T)(rtb_Switch_n < 0.0 ? (uint16_T)(int32_T)(int16_T)
      -(int16_T)(uint16_T)-rtb_Switch_n : (uint16_T)rtb_Switch_n);
  } else {
    /* DataTypeConversion: '<S83>/Data Type Conversion' */
    rtb_Add3_a = 0U;
  }

  /* End of If: '<S83>/If1' */

  /* DataStoreWrite: '<S8>/Data Store Write1' */
  TqR_AccTrqReq = rtb_Add3_a;

  /* If: '<S83>/If2' incorporates:
   *  DataStoreRead: '<S1>/Data Store Read9'
   */
  if ((rtb_DataStoreRead4_b == 1) && (EMS_BrakePedalStatus == 0)) {
    /* Outputs for IfAction SubSystem: '<S83>/If Action Subsystem6' incorporates:
     *  ActionPort: '<S89>/Action Port'
     */
    /* DataTypeConversion: '<S83>/Data Type Conversion1' incorporates:
     *  Inport: '<S89>/V_DesBrkIn[m//s2]'
     */
    rtb_Switch_n = floor(AutoVehicleControl_B.Switch2 * 16.0);

    /* End of Outputs for SubSystem: '<S83>/If Action Subsystem6' */
    if (rtIsNaN(rtb_Switch_n) || rtIsInf(rtb_Switch_n)) {
      rtb_Switch_n = 0.0;
    } else {
      rtb_Switch_n = fmod(rtb_Switch_n, 256.0);
    }

    rtb_DataTypeConversion1_o = (int8_T)(rtb_Switch_n < 0.0 ? (int8_T)-(int8_T)
      (uint8_T)-rtb_Switch_n : (int8_T)(uint8_T)rtb_Switch_n);
    
  } else {
    /* DataTypeConversion: '<S83>/Data Type Conversion1' */
    rtb_DataTypeConversion1_o = 0;
  }

  /* End of If: '<S83>/If2' */

  /* If: '<S84>/If3' incorporates:
   *  DataStoreRead: '<S1>/Data Store Read7'
   *  DataStoreRead: '<S1>/Data Store Read9'
   *  DataStoreWrite: '<S8>/Data Store Write2'
   *  DataTypeConversion: '<S84>/Data Type Conversion1'
   *  DataTypeConversion: '<S84>/Data Type Conversion4'
   *  DataTypeConversion: '<S84>/Data Type Conversion5'
   *  DataTypeConversion: '<S84>/Data Type Conversion6'
   */
  TqR_CDDAxEnable = (((int8_T)rtb_DataStoreRead4_b == 1) && ((int8_T)
    EMS_BrakePedalStatus == 0) && ((int8_T)TCU_GearShiftPositon == 3) &&
                     ((rtb_DataTypeConversion1_o >> 4) < 0));

  /* DataStoreWrite: '<S8>/Data Store Write3' */
  TqR_ACCTargetAccelerationReq = rtb_DataTypeConversion1_o;

  /* If: '<S84>/If2' incorporates:
   *  DataStoreRead: '<S1>/Data Store Read7'
   *  DataStoreRead: '<S1>/Data Store Read9'
   *  DataStoreWrite: '<S8>/Data Store Write4'
   *  DataTypeConversion: '<S84>/Data Type Conversion'
   */
  TqR_AccTrqReqEna = ((rtb_DataStoreRead4_b == 1) && (EMS_BrakePedalStatus == 0)
                      && (TCU_GearShiftPositon == 3) && ((rtb_Add3_a >> 8) > 0));

  /* Chart: '<S84>/F_SteerAngEna' incorporates:
   *  DataStoreRead: '<S1>/Data Store Read18'
   */
  AutoVehicleControl_DWork.presentTicks =
    AutoVehicleControl_M->Timing.clockTick0;
  AutoVehicleControl_DWork.elapsedTicks = AutoVehicleControl_DWork.presentTicks
    - AutoVehicleControl_DWork.previousTicks;
  AutoVehicleControl_DWork.previousTicks = AutoVehicleControl_DWork.presentTicks;
  AutoVehicleControl_DWork.temporalCounter_i1 +=
    AutoVehicleControl_DWork.elapsedTicks;

  /* Gateway: 08_EnableLogic/02_Enable/F_SteerAngEna */
  /* During: 08_EnableLogic/02_Enable/F_SteerAngEna */
  if (AutoVehicleControl_DWork.is_active_c4_AutoVehicleControl == 0U) {
    /* Entry: 08_EnableLogic/02_Enable/F_SteerAngEna */
    AutoVehicleControl_DWork.is_active_c4_AutoVehicleControl = 1U;

    /* Entry Internal: 08_EnableLogic/02_Enable/F_SteerAngEna */
    /* Transition: '<S90>:5' */
    AutoVehicleControl_DWork.is_c4_AutoVehicleControl =
      AutoVehicleControl_IN_DisEna;

    /* Entry 'DisEna': '<S90>:4' */
    AutoVehicleControl_B.V_DesSteerAngEna = 0.0;
  } else {
    switch (AutoVehicleControl_DWork.is_c4_AutoVehicleControl) {
     case AutoVehicleControl_IN_Defult2_j:
      /* During 'Defult2': '<S90>:25' */
      if (AutoVehicleControl_DWork.temporalCounter_i1 >= 2000U) {
        /* Transition: '<S90>:23' */
        AutoVehicleControl_DWork.is_c4_AutoVehicleControl =
          AutoVehicleControl_IN_DisEna;

        /* Entry 'DisEna': '<S90>:4' */
        AutoVehicleControl_B.V_DesSteerAngEna = 0.0;
      }
      break;

     case AutoVehicleControl_IN_Defult3_i:
      /* During 'Defult3': '<S90>:34' */
      if (AutoVehicleControl_DWork.temporalCounter_i1 >= 6U) {
        /* Transition: '<S90>:36' */
        AutoVehicleControl_DWork.is_c4_AutoVehicleControl =
          AutoVehicleControl_IN_Ena;
        AutoVehicleControl_DWork.is_Ena = AutoVehicleControl_IN_ESPConect;

        /* Entry 'ESPConect': '<S90>:39' */
        AutoVehicleControl_B.V_DesSteerAngEna = 1.0;
      }
      break;

     case AutoVehicleControl_IN_DisEna:
      /* During 'DisEna': '<S90>:4' */
      if (rtb_DataStoreRead4_b != 0) {
        /* Transition: '<S90>:7' */
        AutoVehicleControl_DWork.is_c4_AutoVehicleControl =
          AutoVehicleControl_IN_Defult3_i;
        AutoVehicleControl_DWork.temporalCounter_i1 = 0U;
      }
      break;

     default:
      /* During 'Ena': '<S90>:6' */
      if ((rtb_DataStoreRead4_b != 0) && (rtb_V_APA_Abortfeedback != 0)) {
        /* Transition: '<S90>:9' */
        /* Exit Internal 'Ena': '<S90>:6' */
        AutoVehicleControl_DWork.is_Ena = AutoVehicleC_IN_NO_ACTIVE_CHILD;
        AutoVehicleControl_DWork.is_c4_AutoVehicleControl =
          AutoVehicleControl_IN_Defult2_j;
        AutoVehicleControl_DWork.temporalCounter_i1 = 0U;
      } else if (!(rtb_DataStoreRead4_b != 0)) {
        /* Transition: '<S90>:29' */
        /* Exit Internal 'Ena': '<S90>:6' */
        AutoVehicleControl_DWork.is_Ena = AutoVehicleC_IN_NO_ACTIVE_CHILD;
        AutoVehicleControl_DWork.is_c4_AutoVehicleControl =
          AutoVehicleControl_IN_DisEna;

        /* Entry 'DisEna': '<S90>:4' */
        AutoVehicleControl_B.V_DesSteerAngEna = 0.0;
      } else {
        if ((AutoVehicleControl_DWork.is_Ena == AutoVehicleControl_IN_ESPConect)
            && (EPS_APA_ControlFeedback == 1)) {
          /* During 'ESPConect': '<S90>:39' */
          /* Transition: '<S90>:43' */
          AutoVehicleControl_DWork.is_Ena = AutoVehicleContro_IN_ESPConect1;

          /* Entry 'ESPConect1': '<S90>:42' */
          AutoVehicleControl_B.V_DesSteerAngEna = 2.0;
        }
      }
      break;
    }
  }

  /* End of Chart: '<S84>/F_SteerAngEna' */

  /* Switch: '<S8>/Switch1' */
  if (rtb_Compare_n) {
    /* DataStoreWrite: '<S8>/Data Store Write5' incorporates:
     *  Constant: '<S8>/Constant1'
     */
    StC_SteeringAngleReq = 0U;
  } else {
    /* DataTypeConversion: '<S8>/Data Type Conversion1' */
    rtb_Switch_n = floor(AutoVehicleControl_B.V_DesSteerAngEna);
    if (rtIsNaN(rtb_Switch_n) || rtIsInf(rtb_Switch_n)) {
      rtb_Switch_n = 0.0;
    } else {
      rtb_Switch_n = fmod(rtb_Switch_n, 256.0);
    }

    /* DataStoreWrite: '<S8>/Data Store Write5' incorporates:
     *  DataTypeConversion: '<S8>/Data Type Conversion1'
     */
    StC_SteeringAngleReq = (uint8_T)(rtb_Switch_n < 0.0 ? (uint8_T)(int32_T)
      (int8_T)-(int8_T)(uint8_T)-rtb_Switch_n : (uint8_T)rtb_Switch_n);
  }

  /* End of Switch: '<S8>/Switch1' */

  /* Update for Enabled SubSystem: '<S7>/01_F_LaneChange' incorporates:
   *  Update for EnablePort: '<S9>/Enable'
   */
  if (AutoVehicleControl_DWork.u_F_LaneChange_MODE) {
    /* Update for UnitDelay: '<S9>/Unit Delay' */
    AutoVehicleControl_DWork.UnitDelay_DSTATE_p = AutoVehicleControl_B.V_TrajNum;
  }

  /* End of Update for SubSystem: '<S7>/01_F_LaneChange' */

  /* Update for UnitDelay: '<S8>/Unit Delay2' */
  AutoVehicleControl_DWork.UnitDelay2_DSTATE = rtb_V_Traje1X;

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.01, which is the step size
   * of the task. Size of "clockTick0" ensures timer will not overflow during the
   * application lifespan selected.
   */
  AutoVehicleControl_M->Timing.clockTick0++;
}

/* Model initialize function */
void AutoVehicleControl_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)AutoVehicleControl_M, 0,
                sizeof(RT_MODEL_AutoVehicleControl));
  rtmSetFirstInitCond(AutoVehicleControl_M, 1);

  /* block I/O */
  (void) memset(((void *) &AutoVehicleControl_B), 0,
                sizeof(BlockIO_AutoVehicleControl));

  /* states (dwork) */
  (void) memset((void *)&AutoVehicleControl_DWork, 0,
                sizeof(D_Work_AutoVehicleControl));

  /* exported global states */
  V_VehPosYReqdou = 0.0;
  V_VehPosXReqdou = 0.0;
  RT3_Class_Rel = 0.0;
  RT3_Width_Rel = 0.0;
  RT3_V_Long_Rel = 0.0;
  RT4_Width_Rel = 0.0;
  RT4_V_Long_Rel = 0.0;
  RT4_V_Lat_Rel = 0.0;
  RT6_Width_Rel = 0.0;
  RT6_V_Long_Rel = 0.0;
  RT6_V_Lat_Rel = 0.0;
  RT6_L_Lat_Rel = 0.0;
  RT6_L_Long_Rel = 0.0;
  RT6_Class_Rel = 0.0;
  RT5_Width_Rel = 0.0;
  RT4_L_Lat_Rel = 0.0;
  RT5_V_Long_Rel = 0.0;
  RT5_V_Lat_Rel = 0.0;
  RT5_L_Lat_Rel = 0.0;
  RT5_L_Long_Rel = 0.0;
  RT5_Class_Rel = 0.0;
  RT2_Width_Rel = 0.0;
  RT2_V_Long_Rel = 0.0;
  RT2_V_Lat_Rel = 0.0;
  RT2_L_Lat_Rel = 0.0;
  RT2_L_Long_Rel = 0.0;
  RT4_L_Long_Rel = 0.0;
  RT2_Class_Rel = 0.0;
  RT1_Class_Rel = 0.0;
  RT1_Width_Rel = 0.0;
  RT1_V_Long_Rel = 0.0;
  RT1_V_Lat_Rel = 0.0;
  RT1_L_Lat_Rel = 0.0;
  RT1_L_Long_Rel = 0.0;
  RT3_V_Lat_Rel = 0.0;
  RT3_L_Lat_Rel = 0.0;
  RT4_Class_Rel = 0.0;
  RT3_L_Long_Rel = 0.0;
  GPS_Heading = 0.0;
  Latitude_B = 0.0;
  Longitude_L = 0.0;
  V_VehPosXdou = 0.0;
  V_VehPosYdou = 0.0;
  GPS_Pitch = 0.0;
  V_VehPosAngdou = 0.0;
  V_Traje2X = 0.0;
  V_Traje1X = 0.0;
  V_NearTrajeY = 0.0;
  V_TrajeSpdf16s4 = 0.0;
  V_LaneWidthf16s4 = 0.0;
  V_SlopeResisf32s20 = 0.0;
  V_Traje3X = 0.0;
  V_DesAnglef32s20 = 0.0;
  V_Traje2Y = 0.0;
  V_Traje3Y = 0.0;
  V_Traje1Y = 0.0;
  V_NearTrajeX = 0.0;
  V_RefPoint = 0U;
  Wheel_Speed_RR_Data = 0U;
  Wheel_Speed_RL_Data = 0U;
  ESP_VehicleSpeed = 0U;
  TqR_AccTrqReq = 0U;
  EMS_IndicatedRealEngTorq = 0U;
  ESP_LongAccel = 0;
  ESP_YawRate = 0;
  StC_SteeringAngleRequest = 0;
  SAS_SteeringAngle = 0;
  EMS_EngineSpeed = 0U;
  StC_SteeringAngleReq = 0U;
  MC_Mode = 0U;
  V_VehDesNum = 0U;
  V_TrafficLightDet = 0U;
  v_road_typeu8 = 0U;
  EMS_BrakePedalStatus = 0U;
  AT_ActualGear = 0U;
  TCU_GearShiftPositon = 0U;
  BCM_TurnLightSwitchSts = 0U;
  EPS_APA_Abortfeedback = 0U;
  TqR_AccTrqReqEna = FALSE;
  TqR_CDDAxEnable = FALSE;
  EPS_APA_EpasFAILED = FALSE;
  EPS_APA_ControlFeedback = FALSE;
  EMS_AccPedal = 0U;
  EMS_MinIndicatedTorq = 0U;
  EMS_MaxIndicatedTorq = 0U;
  TqR_ACCTargetAccelerationReq = 0;

  /* Start for Enabled SubSystem: '<S7>/01_F_LaneChange' */
  /* Start for Enabled SubSystem: '<S15>/03_F_RSeleTraj' */
  /* VirtualOutportStart for Outport: '<S20>/B_EnabLeft[-]' */
  AutoVehicleControl_B.Merge_h = TRUE;

  /* VirtualOutportStart for Outport: '<S20>/B_EnabRight[-]' */
  AutoVehicleControl_B.Merge1 = TRUE;

  /* End of Start for SubSystem: '<S15>/03_F_RSeleTraj' */

  /* Start for Enabled SubSystem: '<S9>/03_F_SeleTraj' */
  /* Start for Atomic SubSystem: '<S17>/02_F_LaneChAndTurnCorner' */
  /* Start for IfAction SubSystem: '<S49>/If Action Subsystem2' */
  /* VirtualOutportStart for Outport: '<S54>/To3[-]' */
  AutoVehicleControl_B.Merge = 3.0;

  /* End of Start for SubSystem: '<S49>/If Action Subsystem2' */
  /* End of Start for SubSystem: '<S17>/02_F_LaneChAndTurnCorner' */
  /* End of Start for SubSystem: '<S9>/03_F_SeleTraj' */

  /* InitializeConditions for Enabled SubSystem: '<S9>/03_F_SeleTraj' */
  /* InitializeConditions for Atomic SubSystem: '<S17>/02_F_LaneChAndTurnCorner' */
  /* InitializeConditions for Merge: '<S49>/Merge' */
  if (rtmIsFirstInitCond(AutoVehicleControl_M)) {
    AutoVehicleControl_B.Merge = 1.0;
  }

  /* End of InitializeConditions for Merge: '<S49>/Merge' */
  /* End of InitializeConditions for SubSystem: '<S17>/02_F_LaneChAndTurnCorner' */

  /* InitializeConditions for Chart: '<S17>/01_F_SeleTrajLCProcess' */
  AutoVehicleControl_DWork.is_active_c1_AutoVehicleControl = 0U;
  AutoVehicleControl_DWork.is_c1_AutoVehicleControl =
    AutoVehicleC_IN_NO_ACTIVE_CHILD;
  AutoVehicleControl_DWork.presentTicks_d = 0U;
  AutoVehicleControl_DWork.elapsedTicks_o = 0U;
  AutoVehicleControl_DWork.previousTicks_e = 0U;

  /* End of InitializeConditions for SubSystem: '<S9>/03_F_SeleTraj' */

  /* Start for Enabled SubSystem: '<S9>/03_F_SeleTraj' */
  /* VirtualOutportStart for Outport: '<S17>/V_TrajNum[-]' */
  AutoVehicleControl_B.V_TrajNum = 1.0;

  /* End of Start for SubSystem: '<S9>/03_F_SeleTraj' */
  /* End of Start for SubSystem: '<S7>/01_F_LaneChange' */

  /* InitializeConditions for Enabled SubSystem: '<S7>/01_F_LaneChange' */
  /* InitializeConditions for UnitDelay: '<S9>/Unit Delay' */
  AutoVehicleControl_DWork.UnitDelay_DSTATE_p = 1.0;

  /* End of InitializeConditions for SubSystem: '<S7>/01_F_LaneChange' */

  /* Start for Enabled SubSystem: '<S7>/01_F_LaneChange' */
  /* VirtualOutportStart for Outport: '<S9>/V_LCProcess[-]' */
  AutoVehicleControl_B.V_LCProcess = 1.0;

  /* End of Start for SubSystem: '<S7>/01_F_LaneChange' */

  /* Start for Enabled SubSystem: '<S7>/02_F_LongControl' */
  /* InitializeConditions for Enabled SubSystem: '<S56>/02_F_SpdControl' */
  /* InitializeConditions for DiscreteIntegrator: '<S66>/Discrete-Time Integrator' */
  AutoVehicleControl_DWork.DiscreteTimeIntegrator_IC_LOADI = 1U;
  AutoVehicleControl_DWork.DiscreteTimeIntegrator_PrevRe_m = 2;

  /* InitializeConditions for RateLimiter: '<S66>/P_DesAccSpd_RateLim' */
  AutoVehicleControl_DWork.PrevY = 0.0;

  /* End of InitializeConditions for SubSystem: '<S56>/02_F_SpdControl' */
  /* End of Start for SubSystem: '<S7>/02_F_LongControl' */

  /* InitializeConditions for Enabled SubSystem: '<S7>/02_F_LongControl' */
  /* InitializeConditions for Chart: '<S56>/03_F_DesState' */
  AutoVehicleControl_DWork.is_active_c3_AutoVehicleControl = 0U;
  AutoVehicleControl_DWork.is_c3_AutoVehicleControl =
    AutoVehicleC_IN_NO_ACTIVE_CHILD;

  /* InitializeConditions for Delay: '<S57>/Delay' */
  AutoVehicleControl_DWork.Delay_DSTATE = FALSE;

  /* InitializeConditions for DiscreteIntegrator: '<S71>/Discrete-Time Integrator' */
  AutoVehicleControl_DWork.DiscreteTimeIntegrator_DSTATE = 0.0;
  AutoVehicleControl_DWork.DiscreteTimeIntegrator_PrevRese = 0;

  /* InitializeConditions for Delay: '<S57>/Delay1' */
  AutoVehicleControl_DWork.Delay1_DSTATE = 0.0;

  /* InitializeConditions for Chart: '<S10>/04_F_TrafficLightStop' */
  AutoVehicleControl_DWork.is_active_c6_AutoVehicleControl = 0U;
  AutoVehicleControl_DWork.is_c6_AutoVehicleControl =
    AutoVehicleC_IN_NO_ACTIVE_CHILD;
  AutoVehicleControl_DWork.presentTicks_o = 0U;
  AutoVehicleControl_DWork.elapsedTicks_h = 0U;
  AutoVehicleControl_DWork.previousTicks_b = 0U;

  /* InitializeConditions for Chart: '<S10>/05_F_TrajEndStop' */
  AutoVehicleControl_DWork.is_active_c2_AutoVehicleControl = 0U;
  AutoVehicleControl_DWork.is_c2_AutoVehicleControl =
    AutoVehicleC_IN_NO_ACTIVE_CHILD;
  AutoVehicleControl_DWork.presentTicks_b = 0U;
  AutoVehicleControl_DWork.elapsedTicks_a = 0U;
  AutoVehicleControl_DWork.previousTicks_o = 0U;

  /* End of InitializeConditions for SubSystem: '<S7>/02_F_LongControl' */

  /* InitializeConditions for Atomic SubSystem: '<S7>/03_F_Position' */
  /* InitializeConditions for UnitDelay: '<S76>/Unit Delay' */
  AutoVehicleControl_DWork.UnitDelay_DSTATE[0] = -1.02;
  AutoVehicleControl_DWork.UnitDelay_DSTATE[1] = -0.4;
  AutoVehicleControl_DWork.UnitDelay_DSTATE[2] = 5.983;

  /* End of InitializeConditions for SubSystem: '<S7>/03_F_Position' */

  /* InitializeConditions for Chart: '<S83>/F_SteerAngTrans' */
  AutoVehicleControl_DWork.is_active_c5_AutoVehicleControl = 0U;
  AutoVehicleControl_DWork.is_c5_AutoVehicleControl =
    AutoVehicleC_IN_NO_ACTIVE_CHILD;
  AutoVehicleControl_DWork.presentTicks_h = 0U;
  AutoVehicleControl_DWork.elapsedTicks_i = 0U;
  AutoVehicleControl_DWork.previousTicks_a = 0U;

  /* InitializeConditions for Chart: '<S84>/F_SteerAngEna' */
  AutoVehicleControl_DWork.is_Ena = AutoVehicleC_IN_NO_ACTIVE_CHILD;
  AutoVehicleControl_DWork.is_active_c4_AutoVehicleControl = 0U;
  AutoVehicleControl_DWork.is_c4_AutoVehicleControl =
    AutoVehicleC_IN_NO_ACTIVE_CHILD;
  AutoVehicleControl_DWork.presentTicks = 0U;
  AutoVehicleControl_DWork.elapsedTicks = 0U;
  AutoVehicleControl_DWork.previousTicks = 0U;

  /* set "at time zero" to false */
  if (rtmIsFirstInitCond(AutoVehicleControl_M)) {
    rtmSetFirstInitCond(AutoVehicleControl_M, 0);
  }

  /* Enable for Chart: '<S83>/F_SteerAngTrans' */
  AutoVehicleControl_DWork.presentTicks_h =
    AutoVehicleControl_M->Timing.clockTick0;
  AutoVehicleControl_DWork.previousTicks_a =
    AutoVehicleControl_DWork.presentTicks_h;

  /* Enable for Chart: '<S84>/F_SteerAngEna' */
  AutoVehicleControl_DWork.presentTicks =
    AutoVehicleControl_M->Timing.clockTick0;
  AutoVehicleControl_DWork.previousTicks = AutoVehicleControl_DWork.presentTicks;
}

/* Model terminate function */
void AutoVehicleControl_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
