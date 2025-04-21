//
// File: RoutePlanning.h
//
// Code generated for Simulink model 'RoutePlanning'.
//
// Model version                  : 1.66
// Simulink Coder version         : 8.13 (R2017b) 24-Jul-2017
// C/C++ source code generated on : Mon Sep  6 16:10:21 2021
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->64-bit Embedded Processor (LP64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_RoutePlanning_h_
#define RTW_HEADER_RoutePlanning_h_
#ifndef RoutePlanning_COMMON_INCLUDES_
# define RoutePlanning_COMMON_INCLUDES_
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "can_com/rtwtypes.h"
#include "can_com/rt_nonfinite.h"
#include "can_com/rtGetInf.h"
#include "can_com/rtGetNaN.h"
#include "can_com/rt_defines.h"
#endif                                 /* RoutePlanning_COMMON_INCLUDES_ */

#include "can_com/RoutePlanning_types.h"


// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Block signals (auto storage)
typedef struct {
  real_T O_TraiedelX_h;                // '<S1>/Multiport Switch2'
  real_T O_TraiedelY_e;                // '<S1>/Multiport Switch3'
  real_T DataTypeConversion[2];        // '<S1>/Data Type Conversion'
  real_T O_TraiedelX_c;                // '<S9>/Index Vector'
  real_T O_TraiedelY_m;                // '<S9>/Index Vector7'
  uint32_T Saturation1;                // '<S2>/Saturation1'
  boolean_T O_IncRefPoint;             // '<S1>/Data Type Conversion7'
} BlockIO_RoutePlanning;

// Block states (auto storage) for system '<Root>'
typedef struct {
  real_T UnitDelay1_DSTATE;            // '<S1>/Unit Delay1'
  uint32_T UnitDelay_DSTATE_o;         // '<S1>/Unit Delay'
} D_Work_RoutePlanning;

// Real-time Model Data Structure
struct tag_RTM_RoutePlanning {
  const char_T * volatile errorStatus;
};

// Block signals (auto storage)
extern BlockIO_RoutePlanning RoutePlanning_B;

// Block states (auto storage)
extern D_Work_RoutePlanning RoutePlanning_DWork;

//
//  Exported Global Signals
//
//  Note: Exported global signals are block signals with an exported global
//  storage class designation.  Code generation will declare the memory for
//  these signals and export their symbols.
//

extern real_T o_traie_num_aim;           // '<S1>/Saturation5'
extern real_T o_traiedel_x;             // '<S8>/Index Vector'
extern real_T o_traiedel_y;             // '<S8>/Index Vector7'

//
//  Exported Global Parameters
//
//  Note: Exported global parameters are tunable parameters with an exported
//  global storage class designation.  Code generation will declare the memory for
//  these parameters and exports their symbols.
//

extern real_T C_LaneWidth[20000];       // Variable: C_LaneWidth
                                       //  Referenced by: '<S3>/T_LaneWidth'

extern real_T C_RoadTraj1Xtable_f32s23[20000];// Variable: C_RoadTraj1Xtable_f32s23
                                             //  Referenced by:
                                             //    '<S1>/Constant'
                                             //    '<S1>/Constant6'

extern real_T C_RoadTraj1Ytable_f32s23[20000];// Variable: C_RoadTraj1Ytable_f32s23
                                             //  Referenced by:
                                             //    '<S1>/Constant3'
                                             //    '<S1>/Constant9'

extern real_T C_RoadType[20000];        // Variable: C_RoadType
                                       //  Referenced by: '<S3>/T_RoadType'

extern real_T C_SlopeResis[20000];      // Variable: C_SlopeResis
                                       //  Referenced by: '<S3>/T_SlopeResis '

extern real_T C_TrajeSpd[20000];        // Variable: C_TrajeSpd
                                       //  Referenced by: '<S3>/T_TrajeSpd'

extern real_T C_VehicleWidth;          // Variable: C_VehicleWidth
                                       //  Referenced by: '<S10>/VehicleWidth'

extern int16_T C_PreDisMNf16s4;        // Variable: C_PreDisMNf16s4
                                       //  Referenced by: '<S1>/Constant15'

extern int16_T C_PreFac1f16s4;         // Variable: C_PreFac1f16s4
                                       //  Referenced by:
                                       //    '<S1>/Constant13'
                                       //    '<S1>/Constant14'


//
//  Exported States
//
//  Note: Exported states are block states with an exported global
//  storage class designation.  Code generation will declare the memory for these
//  states and exports their symbols.
//

//extern real_T V_LaneWidthf16s4;        // Simulink.Signal object 'V_LaneWidthf16s4' 
//extern real_T V_TrajeSpdf16s4;         // Simulink.Signal object 'V_TrajeSpdf16s4' 
//extern real_T V_SlopeResisf32s20;      // Simulink.Signal object 'V_SlopeResisf32s20' 
extern real_T v_obj_pointy1;            // Simulink.Signal object 'v_obj_pointy1'
extern real_T v_obj_pointx1;            // Simulink.Signal object 'v_obj_pointx1'
extern real_T v_obj_pointx2;            // Simulink.Signal object 'v_obj_pointx2'
extern real_T v_obj_pointy2;            // Simulink.Signal object 'v_obj_pointy2'
//extern real_T V_Traje1Y;               // Simulink.Signal object 'V_Traje1Y'
//extern real_T V_Traje2Y;               // Simulink.Signal object 'V_Traje2Y'
//extern real_T V_Traje3Y;               // Simulink.Signal object 'V_Traje3Y'
//extern real_T V_Traje3X;               // Simulink.Signal object 'V_Traje3X'
//extern real_T V_Traje2X;               // Simulink.Signal object 'V_Traje2X'
//extern real_T V_Traje1X;               // Simulink.Signal object 'V_Traje1X'
extern real_T V_Objwidth;              // Simulink.Signal object 'V_Objwidth'
//extern real_T V_DesAnglef32s20;        // Simulink.Signal object 'V_DesAnglef32s20' 
//extern real_T V_NearTrajeY;            // Simulink.Signal object 'V_NearTrajeY'
//extern real_T V_NearTrajeX;            // Simulink.Signal object 'V_NearTrajeX'
extern real_T V_VehSpd;                // Simulink.Signal object 'V_VehSpd'
extern real_T V_VehPosX;               // Simulink.Signal object 'V_VehPosX'
extern real_T V_VehPosY;               // Simulink.Signal object 'V_VehPosY'
//extern uint32_T V_RefPoint;            // Simulink.Signal object 'V_RefPoint'
extern uint16_T C_IniRefPointu16;      // Simulink.Signal object 'C_IniRefPointu16' 
//extern uint8_T v_road_typeu8;           // Simulink.Signal object 'v_road_typeu8'
extern uint8_T V_RoadTrajNumAim_u8;    // Simulink.Signal object 'V_RoadTrajNumAim_u8' 

#ifdef __cplusplus

extern "C" {

#endif


  // Model entry point functions
  extern void RoutePlanning_initialize(void);
  extern void RoutePlanning_step(void);
  extern void RoutePlanning_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_RoutePlanning *const RoutePlanning_M;

#ifdef __cplusplus

}
#endif

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'RoutePlanning'
//  '<S1>'   : 'RoutePlanning/01_RT_SelectTrajData'
//  '<S2>'   : 'RoutePlanning/02_F_IncRefPoint'
//  '<S3>'   : 'RoutePlanning/03_RT_TrajectType'
//  '<S4>'   : 'RoutePlanning/Compare To Constant1'
//  '<S5>'   : 'RoutePlanning/01_RT_SelectTrajData/Compare To Constant'
//  '<S6>'   : 'RoutePlanning/01_RT_SelectTrajData/Compare To Constant1'
//  '<S7>'   : 'RoutePlanning/01_RT_SelectTrajData/Compare To Zero'
//  '<S8>'   : 'RoutePlanning/01_RT_SelectTrajData/PlanPoit'
//  '<S9>'   : 'RoutePlanning/01_RT_SelectTrajData/PlanPoit1'
//  '<S10>'  : 'RoutePlanning/01_RT_SelectTrajData/PointLimit'
//  '<S11>'  : 'RoutePlanning/02_F_IncRefPoint/Compare To Constant'

#endif                                 // RTW_HEADER_RoutePlanning_h_

//
// File trailer for generated code.
//
// [EOF]
//
