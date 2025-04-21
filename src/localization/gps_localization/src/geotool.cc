
// #include "geotool.h"
// #define ACCEPT_USE_OF_DEPRECATED_PROJ_API_H
// #include "proj_api.h"
// GeoTool geo;
// double Angle2Heading(double input)
// {
//   double output = 90 - input;
//   if (output < 0)
//     output += 360;
//   return output;
// }
// double Heading2Angle(double input)
// {
//   double output = 90 - input;
//   if (output < -180)
//     output += 360;
//   return output;
// }
// int GeoTransform(const PointGCCS &pgccs, const int utm_zone, PointGPS &pgps)
// {
//   RETURN_EQ(const_cast<PointGCCS &>(pgccs).Check(), 0);
//   double x = pgccs.xg, y = pgccs.yg;
//   double z = 0.;
//   std::string utmInit = "+proj=utm +zone=" + std::to_string(utm_zone) +
//                         "+ellps=WGS84 +datum=WGS84 +units=m +no_defs";
//   projPJ lonlat = pj_init_plus(" +proj=longlat +datum=WGS84 +no_defs");
//   projPJ utm = pj_init_plus(utmInit.c_str());
//   RETURN_NE(pj_transform(utm, lonlat, 1, 1, &x, &y, &z), 0);
//   x *= RAD_TO_DEG;
//   y *= RAD_TO_DEG;
//   pgps.lon = x;
//   pgps.lat = y;
//   pgps.heading = Angle2Heading(pgccs.angle);
//   if (lonlat)
//   {
//     pj_free(lonlat);
//   }
//   if (utm)
//   {
//     pj_free(utm);
//   }
//   return 0;
// }
// int GeoTransform(const PointGPS &pgps, PointGCCS &pgccs)
// {
//   RETURN_EQ(const_cast<PointGPS &>(pgps).Check(), 0);
//   double lon = pgps.lon, lat = pgps.lat;
//   double height = 0.;
//   std::string utmInit = "+proj=utm +zone=" + std::to_string(geo.GetLongZone(lon)) +
//                         "+ellps=WGS84 +datum=WGS84 +units=m +no_defs";
//   projPJ lonlat = pj_init_plus(" +proj=longlat +datum=WGS84 +no_defs");
//   projPJ utm = pj_init_plus(utmInit.c_str());
//   lon *= DEG_TO_RAD;
//   lat *= DEG_TO_RAD;
//   RETURN_NE(pj_transform(lonlat, utm, 1, 1, &lon, &lat, &height), 0);
//   pgccs.xg = lon;
//   pgccs.yg = lat;
//   pgccs.angle = pgps.heading;//Heading2Angle(pgps.heading);
//   if (lonlat)
//   {
//     pj_free(lonlat);
//   }
//   if (utm)
//   {
//     pj_free(utm);
//   }
//   return 0;
// }

// int GeoTransform(const PointGCCS &pgccs, const double cell_size,
//                  PointGICS &pgics)
// {
//   RETURN_EQ(const_cast<PointGCCS &>(pgccs).Check(), 0);
//   pgics.ug = static_cast<long>(pgccs.xg / cell_size);
//   pgics.vg = static_cast<long>(pgccs.yg / cell_size);
//   return 0;
// }
// int GeoTransform(const PointGICS &pgics, const double cell_size,
//                  PointGCCS &pgccs)
// {
//   RETURN_EQ(const_cast<PointGICS &>(pgics).Check(), 0);
//   pgccs.xg = static_cast<double>(pgics.ug * cell_size);
//   pgccs.yg = static_cast<double>(pgics.vg * cell_size);
//   return 0;
// }
// int GeoTransform(const PointGCCS &car_pgccs, const PointGCCS &target_pgccs,
//                  PointVCS &output_pvcs)
// {
//   RETURN_EQ(const_cast<PointGCCS &>(car_pgccs).Check(), 0);
//   RETURN_EQ(const_cast<PointGCCS &>(target_pgccs).Check(), 0);
//   double rad = car_pgccs.angle * DEG_TO_RAD;
//   double dx = target_pgccs.xg - car_pgccs.xg;
//   double dy = target_pgccs.yg - car_pgccs.yg;
//   output_pvcs.x = +dx * cos(rad) + dy * sin(rad);
//   output_pvcs.y = -dx * sin(rad) + dy * cos(rad);
//   output_pvcs.angle = target_pgccs.angle - car_pgccs.angle;
//   if (output_pvcs.angle >= 360)
//     output_pvcs.angle -= 360;
//   return 0;
// }

// int GeoTransform(const PointGCCS &car_pgccs, const PointVCS &target_pvcs,
//                  PointGCCS &output_pgccs)
// {
//   RETURN_EQ(const_cast<PointGCCS &>(car_pgccs).Check(), 0);
//   double rad = car_pgccs.angle * DEG_TO_RAD;
//   output_pgccs.xg = target_pvcs.x * cos(rad) - target_pvcs.y * sin(rad);
//   output_pgccs.yg = target_pvcs.x * sin(rad) + target_pvcs.y * cos(rad);
//   output_pgccs.xg += car_pgccs.xg;
//   output_pgccs.yg += car_pgccs.yg;
//   output_pgccs.angle = car_pgccs.angle + target_pvcs.angle;
//   if (output_pgccs.angle >= 360)
//     output_pgccs.angle -= 360;
//   return 0;
// }

// bool GeoTool::SetUtmZone(int zone)
// {
//   if (zone < 0 || zone > 60)
//     return 0;
//   else
//   {
//     utm_zone_ = zone;
//     return 1;
//   }
// }
// int GeoTool::GetUtmZone()
// {
//   return utm_zone_;
// }
// bool GeoTool::SetCellSize(double size)
// {
//   if (size <= 0)
//     return 0;
//   else
//   {
//     cell_size_ = size;
//     return 1;
//   }
// }
// int GeoTool::GetLongZone(double longitude)
// {
//   double longZone = 0.0;
//   if (longitude < 0.0)
//   {
//     longZone = ((180.0 + longitude) / 6.0) + 1;
//   }
//   else
//   {
//     longZone = (longitude / 6.0) + 31;
//   }
//   return static_cast<int>(longZone);
// }
// int GeoTool::GCCS2GPS(const PointGCCS &pgccs, PointGPS &pgps) const
// {
//   return GeoTransform(pgccs, utm_zone_, pgps);
// }
// int GeoTool::GPS2GCCS(const PointGPS &pgps, PointGCCS &pgccs) const
// {
//   return GeoTransform(pgps, pgccs);
// }

// int GeoTool::GCCS2GICS(const PointGCCS &pgccs, PointGICS &pgics) const
// {
//   return GeoTransform(pgccs, cell_size_, pgics);
// }
// int GeoTool::GICS2GCCS(const PointGICS &pgics, PointGCCS &pgccs) const
// {
//   return GeoTransform(pgics, cell_size_, pgccs);
// }

// int GeoTool::GCCS2VCS(const PointGCCS &car_pgccs, const PointGCCS &target_pgccs,
//                       PointVCS &output_pvcs) const
// {
//   return GeoTransform(car_pgccs, target_pgccs, output_pvcs);
// }
// int GeoTool::VCS2GCCS(const PointGCCS &car_pgccs, const PointVCS &target_pvcs,
//                       PointGCCS &output_pgccs) const
// {
//   return GeoTransform(car_pgccs, target_pvcs, output_pgccs);
// }

// int GeoTool::GICS2VCS(const PointGCCS &car_pgccs, const PointGICS &target_pgics,
//                       PointVCS &output_pvcs) const
// {
//   PointGCCS target_pgccs;
//   RETURN_EQ(GeoTransform(target_pgics, cell_size_, target_pgccs), -1);
//   return GeoTransform(car_pgccs, target_pgccs, output_pvcs);
// }
// int GeoTool::VCS2GICS(const PointGCCS &car_pgccs, const PointVCS &target_pvcs,
//                       PointGICS &output_pgics) const
// {
//   PointGCCS output_pgccs;
//   RETURN_EQ(GeoTransform(car_pgccs, target_pvcs, output_pgccs), -1);
//   return GeoTransform(output_pgccs, cell_size_, output_pgics);
// }

#include "geotool.hpp"
#include <stdexcept>
#include <proj.h>  

/*--------------------------------------------------
 * 简单角度工具
 *-------------------------------------------------*/
double Angle2Heading(double input)
{
  double output = 90.0 - input;
  return (output < 0.0) ? output + 360.0 : output;
}
double Heading2Angle(double input)
{
  double output = 90.0 - input;
  return (output < -180.0) ? output + 360.0 : output;
}

/*==================================================
 * 内部通用转换（PROJ 新 API 实现）
 *=================================================*/
namespace {

/// @brief 将一组坐标从 @p srcPJ 转到 @p dstPJ（经纬度采用度）。
/// 返回 0 成功，其他值为 PROJ 错误码。
int projTransform(PJ * srcPJ, PJ * dstPJ, double & x, double & y)
{
  if (!srcPJ || !dstPJ) return -1;

  PJ_COORD c;
  c.lpzt.lam = x * DEG_TO_RAD;
  c.lpzt.phi = y * DEG_TO_RAD;
  c.lpzt.z   = 0;
  c.lpzt.t   = 0;

  PJ_COORD out = proj_trans(srcPJ, PJ_FWD, c);
  if (proj_errno(srcPJ) != 0) return proj_errno(srcPJ);

  // out 为 (easting, northing, z, t) 或 (lon, lat, ...)
  if (dstPJ == srcPJ) { x = out.lp.lam * RAD_TO_DEG; y = out.lp.phi * RAD_TO_DEG; }
  else                { x = out.xy.x;               y = out.xy.y;               }
  return 0;
}

/// @brief 创建 UTM 与 WGS‑84 经纬度之间的 CRS 句柄
PJ * makeUtmCRS(PJ_CONTEXT * ctx, int zone, bool is_north=true)
{
  std::string utm_def = "+proj=utm +zone=" + std::to_string(zone)
                      + (is_north ? " +north" : " +south")
                      + " +datum=WGS84 +units=m +no_defs";
  return proj_create(ctx, utm_def.c_str());
}

} // anonymous namespace

/*--------------------------------------------------
 * GeoTool 成员实现
 *-------------------------------------------------*/
bool GeoTool::SetUtmZone(int zone)
{
  if (zone < 1 || zone > 60) return false;
  utm_zone_ = zone;
  return true;
}
int GeoTool::GetUtmZone() const { return utm_zone_; }

bool GeoTool::SetCellSize(double size)
{
  if (size <= 0.0) return false;
  cell_size_ = size;
  return true;
}

/// @brief 由经度求 UTM 带号（1‑60）
int GeoTool::GetLongZone(double lon) const
{
  double temp = (lon < 0.0) ? ((180.0 + lon) / 6.0) + 1
                            :  (lon        / 6.0) + 31;
  return static_cast<int>(temp);
}

/*==================== 基本坐标系互转 ====================*/
int GeoTool::GPS2GCCS(const PointGPS & gps, PointGCCS & gccs) const
{
  PJ_CONTEXT * ctx = proj_context_create();
  PJ * pj_ll  = proj_create(ctx, "+proj=longlat +datum=WGS84 +no_defs");
  PJ * pj_utm = makeUtmCRS(ctx, GetLongZone(gps.lon));

  double x = gps.lon, y = gps.lat;
  int err  = projTransform(pj_ll, pj_utm, x, y);
  if (err == 0) {
    gccs.xg   = x;
    gccs.yg   = y;
    gccs.angle= Heading2Angle(gps.heading);
  }
  proj_destroy(pj_ll); proj_destroy(pj_utm); proj_context_destroy(ctx);
  return err;
}

int GeoTool::GCCS2GPS(const PointGCCS & gccs, PointGPS & gps) const
{
  PJ_CONTEXT * ctx = proj_context_create();
  PJ * pj_ll  = proj_create(ctx, "+proj=longlat +datum=WGS84 +no_defs");
  PJ * pj_utm = makeUtmCRS(ctx, utm_zone_);

  double x = gccs.xg, y = gccs.yg;
  int err  = projTransform(pj_utm, pj_ll, x, y);
  if (err == 0) {
    gps.lon   = x;
    gps.lat   = y;
    gps.heading = Angle2Heading(gccs.angle);
  }
  proj_destroy(pj_ll); proj_destroy(pj_utm); proj_context_destroy(ctx);
  return err;
}

/*—— GCCS ↔ 以格网为基的栅格坐标 ——*/
int GeoTool::GCCS2GICS(const PointGCCS & gccs, PointGICS & gics) const
{
  gics.ug = static_cast<long>(std::lround(gccs.xg / cell_size_));
  gics.vg = static_cast<long>(std::lround(gccs.yg / cell_size_));
  return 0;
}
int GeoTool::GICS2GCCS(const PointGICS & gics, PointGCCS & gccs) const
{
  gccs.xg = static_cast<double>(gics.ug) * cell_size_;
  gccs.yg = static_cast<double>(gics.vg) * cell_size_;
  return 0;
}

/*—— 车体坐标 ↔ 全球笛卡尔 ——*/
int GeoTool::GCCS2VCS(const PointGCCS & car, const PointGCCS & tgt, PointVCS & vcs) const
{
  double rad = car.angle * DEG_TO_RAD;
  double dx  = tgt.xg - car.xg;
  double dy  = tgt.yg - car.yg;

  vcs.x     =  dx * cos(rad) + dy * sin(rad);
  vcs.y     = -dx * sin(rad) + dy * cos(rad);
  vcs.angle = tgt.angle - car.angle;
  if (vcs.angle >= 360.0) vcs.angle -= 360.0;
  return 0;
}

int GeoTool::VCS2GCCS(const PointGCCS & car, const PointVCS & vcs, PointGCCS & gccs) const
{
  double rad = car.angle * DEG_TO_RAD;
  gccs.xg = vcs.x * cos(rad) - vcs.y * sin(rad) + car.xg;
  gccs.yg = vcs.x * sin(rad) + vcs.y * cos(rad) + car.yg;

  gccs.angle = car.angle + vcs.angle;
  if (gccs.angle >= 360.0) gccs.angle -= 360.0;
  return 0;
}

/*—— 车体坐标 ↔ 栅格坐标 ——*/
int GeoTool::GICS2VCS(const PointGCCS & car, const PointGICS & gics, PointVCS & vcs) const
{
  PointGCCS tmp{};
  GICS2GCCS(gics, tmp);
  return GCCS2VCS(car, tmp, vcs);
}
int GeoTool::VCS2GICS(const PointGCCS & car, const PointVCS & vcs, PointGICS & gics) const
{
  PointGCCS tmp{};
  VCS2GCCS(car, vcs, tmp);
  return GCCS2GICS(tmp, gics);
}

