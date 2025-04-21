#pragma once
#include <cmath>
#include <cstdint>
#include <string>
#include <iostream>

/*=========================  公共宏  =========================*/
#define RETURN_EQ(val1, val2) \
  if ((val1) == (val2))       \
    return -1;
#define RETURN_NE(val1, val2) \
  if ((val1) != (val2))       \
    return -1;

#define BOOL_GE(val1, val2) ((val1) >= (val2))
#define BOOL_IN(val, lo, hi) (((val) >= (lo)) && ((val) <= (hi)))

/*=========================  常量  ============================*/
constexpr double kXGMIN = 166021.443081;
constexpr double kXGMAX = 833978.556908;
constexpr double kYGMIN = -8881585.81599;
constexpr double kYGMAX =  9328093.83056;

constexpr double kLONMIN = -180.0;
constexpr double kLONMAX =  180.0;
constexpr double kLATMIN =  -80.0;
constexpr double kLATMAX =   84.0;

constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / M_PI;

/*=========================  数据结构  ========================*/
struct PointGCCS
{
  double xg{0.0};
  double yg{0.0};
  double angle{0.0};

  bool Check() const
  {
    return BOOL_IN(xg, kXGMIN, kXGMAX) && BOOL_IN(yg, kYGMIN, kYGMAX);
  }
};

struct PointVCS
{
  double x{0.0};
  double y{0.0};
  double angle{0.0};
};

struct PointGICS
{
  long ug{0};
  long vg{0};

  bool Check() const { return BOOL_GE(ug, 0) && BOOL_GE(vg, 0); }
};

struct PointGPS
{
  double lon{0.0};
  double lat{0.0};
  double heading{0.0};

  bool Check() const { return BOOL_IN(lon, kLONMIN, kLONMAX) && BOOL_IN(lat, kLATMIN, kLATMAX); }
};

/*=====================  通用接口函数声明  =====================*/
int GeoTransform(const PointGCCS & pgccs, int utm_zone, PointGPS & pgps);
int GeoTransform(const PointGPS  & pgps,  PointGCCS & pgccs);

int GeoTransform(const PointGCCS & pgccs, double cell_size, PointGICS & pgics);
int GeoTransform(const PointGICS & pgics, double cell_size, PointGCCS & pgccs);

int GeoTransform(const PointGCCS & car_pgccs, const PointGCCS & tgt_pgccs,
                 PointVCS & output_vcs);
int GeoTransform(const PointGCCS & car_pgccs, const PointVCS  & tgt_vcs,
                 PointGCCS & output_pgccs);

/*===========================  GeoTool  =======================*/
class GeoTool
{
public:
  GeoTool() = default;
  GeoTool(int zone, double size) : cell_size_(size), utm_zone_(zone) {}

  bool SetCellSize(double size);
  bool SetUtmZone(int zone);
  int  GetUtmZone() const;
  int  GetLongZone(double longitude) const;

  /*—— 成员坐标转换 ——*/
  int GCCS2GPS (const PointGCCS & pgccs, PointGPS  & pgps)  const;
  int GPS2GCCS (const PointGPS  & pgps,  PointGCCS & pgccs) const;

  int GCCS2GICS(const PointGCCS & pgccs, PointGICS & pgics) const;
  int GICS2GCCS(const PointGICS & pgics, PointGCCS & pgccs) const;

  int GCCS2VCS (const PointGCCS & car_pgccs, const PointGCCS & tgt_pgccs,
                PointVCS & output_vcs) const;
  int VCS2GCCS (const PointGCCS & car_pgccs, const PointVCS & tgt_vcs,
                PointGCCS & output_pgccs) const;

  int GICS2VCS (const PointGCCS & car_pgccs, const PointGICS & tgt_pgics,
                PointVCS & output_vcs) const;
  int VCS2GICS (const PointGCCS & car_pgccs, const PointVCS  & tgt_vcs,
                PointGICS & output_pgics) const;

private:
  double cell_size_{0.05};  // 单元格尺寸 (m)
  int    utm_zone_{50};     // 默认 50 带（东经 120°‑126°）
};

/*=======================  辅助角度函数  ======================*/
double Angle2Heading(double angle_deg);   // 0° x‑正向，逆时针为正 → 方位角 (北 0° 顺时针)
double Heading2Angle(double heading_deg); // 反变换
