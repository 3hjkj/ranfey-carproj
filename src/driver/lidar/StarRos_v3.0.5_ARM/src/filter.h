/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

#ifndef __FILTER_H
#define __FILTER_H
#include <string>
#include <string.h>
#include <vector>
using namespace std;

const int filerPtNum=10;
struct InputPara_S
{
    string device_ip;
    string device_name;
    string simu_filepath;
    string export_path;
    string display_mode;
    string filter_path;
    string isf_path;
    int dataport;
    int msgport;
    int heart_port;
    int scnSpeed;
    bool dual_echo;
    string cfg_path;
    bool read_once;
    int device_start;
    bool read_fast;
    bool save_xyz;
    bool use_gps;
    bool alg_flag;
    bool save_isf;
};
enum formant {FORMANT_VER4=1,FORMANT_VER5,FORMANT_VER6};

#pragma pack(1)
typedef struct  //256 Byte
{
  unsigned int pkgflag;
  unsigned int pkgnumber;
  unsigned char	gps_time[6];
  unsigned int maca;
  unsigned short macb;
  unsigned short dataport;
  unsigned short msgport;
  unsigned char motorspd;
  unsigned int deviceType;
  unsigned short phaseAngle;
  //according new heartbeat package
  unsigned char pack_format;
  unsigned char device_id;
  short temperature;
  unsigned int err_chksum;
  unsigned int point_freq;
  unsigned int dev_status;
  unsigned int low_sn;
  unsigned int high_sn;
  unsigned char padding[201];
}HEARTBEAT_S;
#pragma pack()

struct pointCrd
{
float x;
float y;
float z;
};

struct crdFilterPara_S
{
float time;
float num;
pointCrd pt[filerPtNum];
};

struct filterXYZ_S
{
  bool frameFlag;
  float frameTime;
  int ptNum;
  pointCrd ptMin[filerPtNum] ;
  pointCrd ptMax[filerPtNum] ;
};



#endif //__RFANS_IOAPI_H
