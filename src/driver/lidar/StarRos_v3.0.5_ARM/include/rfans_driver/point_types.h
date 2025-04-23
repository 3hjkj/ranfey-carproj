
/********************************************************************
 * $I
 * @Technic Support: <sdk@isurestar.com>
 * All right reserved, Sure-Star Coop.
 ********************************************************************/
#ifndef _POINT_TYPES_H_
#define _POINT_TYPES_H_
#pragma pack(1)
// 定义一个结构体，用于存储点云数据
typedef struct {
  float x,y,z ; // 点的坐标
  float intent; // 点的意图
  float vangle; // 点的垂直角度
  float hangle; // 点的水平角度
  float range; // 点的范围
  // int col;
  double timeflag; // 时间标志
  int laserid; // 激光器ID
//  int mirrorid; // 镜面ID

}TransClound_S;
#pragma pack()
#endif

