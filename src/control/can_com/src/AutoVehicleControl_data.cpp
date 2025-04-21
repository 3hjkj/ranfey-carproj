/*
 * File: AutoVehicleControl_data.cpp
 *
 * Code generated for Simulink model 'AutoVehicleControl'.
 *
 * Model version                  : 1.3
 * Simulink Coder version         : 8.3 (R2012b) 20-Jul-2012
 * TLC version                    : 8.3 (Jul 21 2012)
 * C/C++ source code generated on : Tue Apr 18 14:14:53 2017
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Generic->64-bit Embedded Processor (LP64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "can_com/AutoVehicleControl.h"
#include "can_com/AutoVehicleControl_private.h"

/* Invariant block signals (auto storage) */
//const ConstBlockIO_AutoVehicleControl AutoVehicleControl_ConstB = {
 // 0
  /* '<S8>/Data Type Conversion2' */
//};

/* Constant parameters (auto storage) */
const ConstParam_AutoVehicleControl AutoVehicleControl_ConstP = {
  /* Expression: T_TransSpdRaioY
   * Referenced by: '<S57>/T_TransSpdRaio'
   */
  { 17.75, 10.33, 6.265, 4.808, 3.615, 2.541 },

  /* Expression: T_TransSpdRaioX
   * Referenced by: '<S57>/T_TransSpdRaio'
   */
  { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 },

  /* Expression: T_RoadResY
   * Referenced by: '<S57>/T_RoadRes'
   */
  { 200.0, 200.0, 260.0, 300.0, 350.0, 400.0 },

  /* Expression: T_RoadResX
   * Referenced by: '<S57>/T_RoadRes'
   */
  { 0.0, 30.0, 40.0, 50.0, 60.0, 70.0 },

  /* Pooled Parameter (Expression: [1 -1 0])
   * Referenced by:
   *   '<S74>/1-D Lookup Table'
   *   '<S75>/1-D Lookup Table'
   */
  { 1.0, -1.0, 0.0 },

  /* Pooled Parameter (Expression: [0 1 2])
   * Referenced by:
   *   '<S74>/1-D Lookup Table'
   *   '<S75>/1-D Lookup Table'
   */
  { 0.0, 1.0, 2.0 },

  /* Computed Parameter: T_MaxSteerAngRate_ta
   * Referenced by: '<S13>/T_MaxSteerAngRate'
   */
  { 24640883U, 21346684U, 20614640U, 14320441U, 6294199U, 4013121U, 2903315U,
    2281077U, 2098066U, 2098066U, 2098066U },

  /* Computed Parameter: T_MaxSteerAngRate_bp
   * Referenced by: '<S13>/T_MaxSteerAngRate'
   */
  { 0U, 5825422U, 11650844U, 14563556U, 17476267U, 20388978U, 23301689U,
    29127111U, 34952533U, 40777956U, 46603378U },

  /* Computed Parameter: T_MaxSteerAng_tableD
   * Referenced by: '<S13>/T_MaxSteerAng'
   */
  { 27451656U, 27451656U, 12627762U, 10980661U, 8941298U, 6294199U, 4305939U,
    4013121U, 2976519U, 2976519U, 1049033U, 875829U, 629420U, 283011U },

  /* Computed Parameter: T_MaxSteerAng_bp01Da
   * Referenced by: '<S13>/T_MaxSteerAng'
   */
  { 0U, 5825422U, 8738133U, 11650844U, 14563556U, 17476267U, 20388978U,
    23301689U, 26214400U, 29127111U, 32039822U, 34952533U, 40777956U, 46603378U
  }
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
