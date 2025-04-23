#ifndef __SLOWDATA_H__
#define __SLOWDATA_H__
#include <stdint.h>
#include "compile.h"


struct PlatInfo_S {
    uint32_t ppsStamp;
    float angle;
    PlatInfo_S()
    {
        ppsStamp = PPS_STAMP_DEFAULT;
        angle    =               0.0;
    }
};

struct SlowOption_S
{
    bool  pack;
    bool  pack_fast;
    bool  simulate;
    bool  scan_camera;
    bool  inclt_echo;
    bool  envir;
    bool  imu;
    bool  imu_lc100;
    bool  discard_lc100;
    bool  lfp_imu_lc100;
    bool  bfp_imu_lc100;
    bool  imu_ic100;
    bool  gps;
    bool  dmi;
    bool  pos2010;
    bool  gi510;
    bool  hg4930;
    bool  uimu_ic;
    bool  other;
    bool  synchron;
    bool  error_log;
    bool  imu_raw;
    bool  imu_raw_dat_01;
    bool  imu_raw_dat_02;
    bool  imu_raw_lc100_01;
    bool  imu_raw_lc100_02;
    bool  imu_raw_lc100;
    bool  imu_raw_dat;
    bool  imu_gwxt_dat;
    bool  imu_gwxt_lc100;
    bool  imuRawData_IE;
    bool  pps;
    bool plat_angel_v;//云台垂直角度
    bool plat_angel_h;//云台水平角度
    SlowOption_S()
    {
        pack = false;
        pack_fast = false;
        simulate = false;
        scan_camera = false;
        inclt_echo = false;
        envir = false;
        imu = false;
        imu_lc100 = false;
        discard_lc100 = false;
        lfp_imu_lc100 = false;
        bfp_imu_lc100 = false;
        imu_ic100 = false;
        gps = false;
        dmi = false;
        pos2010 = false;
        gi510 = false;
        gi510 = false;
        gi510 = false;
        hg4930 = false;
        uimu_ic = false;
        other = false;
        synchron = false;
        error_log = false;
        imu_raw = false;
        imu_raw_dat_01 = false;
        imu_raw_dat_02 = false;
        imu_raw_lc100_01 = false;
        imu_raw_lc100_02 = false;
        imu_raw_lc100 = false;
        imu_raw_dat = false;
        imu_gwxt_dat = false;
        imu_gwxt_lc100 = false;
        imuRawData_IE=false;
        pps=true;
        plat_angel_v=false;
        plat_angel_h=false;
    }
};
struct PackStream_S
{
  //具体实现
};

struct Pack_S
{
    PackStream_S*  pack;
    int            pack_size;
};


struct SlowOut_S//目前简写，实际有23类慢数据
{
    Pack_S  pack;
};



#endif // SLOWDATA_H
