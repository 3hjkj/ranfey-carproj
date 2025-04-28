
#include "geotool.h"

#include <proj.h>
#include <cmath>          // std::sin / std::cos / M_PI
#include <string>
#include <cmath>
using std::cos;
using std::sin;

namespace {

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

/* 旧版工具函数保持原名，内部实现未变 */
inline double Angle2Heading(double a) {
  double h = 90.0 - a;
  if (h < 0.0) h += 360.0;
  return h;
}
inline double Heading2Angle(double h) {
  double a = 90.0 - h;
  if (a < -180.0) a += 360.0;
  return a;
}

/* 构造 “CRS→CRS” 变换对象的小工具。失败返回 nullptr */
PJ* createTransform(PJ_CONTEXT* C,
                    const std::string& src,
                    const std::string& dst)
{
  PJ* src_crs = proj_create(C, src.c_str());
  PJ* dst_crs = proj_create(C, dst.c_str());
  PJ* tr      = proj_create_crs_to_crs_from_pj(C, src_crs, dst_crs,
                                               /*area_of_interest*/nullptr,
                                               /*options*/nullptr);
  proj_destroy(src_crs);
  proj_destroy(dst_crs);
  return tr;
}

} // anonymous namespace
/* -------------------------------------------------------------------------- */

GeoTool geo;   // 全局实例与旧代码保持一致

/* ==================== GCCS → GPS ==================== */
int GeoTransform(const PointGCCS& pgccs, int utm_zone, PointGPS& pgps)
{
  RETURN_EQ(const_cast<PointGCCS&>(pgccs).Check(), 0);

  /* ---- 创建变换 ---- */
  PJ_CONTEXT* C = proj_context_create();
  std::string utm_def = "+proj=utm +zone=" + std::to_string(utm_zone) +
                        " +ellps=WGS84 +datum=WGS84 +units=m +no_defs";
  PJ* tr = createTransform(C, utm_def, "+proj=longlat +datum=WGS84 +no_defs");
  RETURN_NE(tr == nullptr, true);

  /* ---- 执行坐标转换 ---- */
  PJ_COORD in  = proj_coord(pgccs.xg, pgccs.yg, 0, 0);
  PJ_COORD out = proj_trans(tr, PJ_INV, in);   // UTM → LonLat

  pgps.lon     = proj_todeg(out.lp.lam);       // 弧度→度
  pgps.lat     = proj_todeg(out.lp.phi);
  pgps.heading = Angle2Heading(pgccs.angle);

  /* ---- 清理 ---- */
  proj_destroy(tr);
  proj_context_destroy(C);
  return 0;
}

/* ==================== GPS → GCCS ==================== */
int GeoTransform(const PointGPS& pgps, PointGCCS& pgccs)
{
  RETURN_EQ(const_cast<PointGPS&>(pgps).Check(), 0);

  int zone = geo.GetLongZone(pgps.lon);
  std::string utm_def = "+proj=utm +zone=" + std::to_string(zone) +
                        " +ellps=WGS84 +datum=WGS84 +units=m +no_defs";

  PJ_CONTEXT* C = proj_context_create();
  PJ* tr = createTransform(C, "+proj=longlat +datum=WGS84 +no_defs", utm_def);
  RETURN_NE(tr == nullptr, true);

  PJ_COORD in  = proj_coord(proj_torad(pgps.lon),
                            proj_torad(pgps.lat),
                            0, 0);
  PJ_COORD out = proj_trans(tr, PJ_FWD, in);   // LonLat → UTM

  pgccs.xg   = out.xy.x;
  pgccs.yg   = out.xy.y;
  pgccs.angle = Heading2Angle(pgps.heading);

  proj_destroy(tr);
  proj_context_destroy(C);
  return 0;
}

/* ==================== 纯几何变换：不依赖 PROJ ==================== */
int GeoTransform(const PointGCCS& pgccs, double cell_size, PointGICS& pgics)
{
  RETURN_EQ(const_cast<PointGCCS&>(pgccs).Check(), 0);
  pgics.ug = static_cast<long>(pgccs.xg / cell_size);
  pgics.vg = static_cast<long>(pgccs.yg / cell_size);
  return 0;
}

int GeoTransform(const PointGICS& pgics, double cell_size, PointGCCS& pgccs)
{
  RETURN_EQ(const_cast<PointGICS&>(pgics).Check(), 0);
  pgccs.xg = static_cast<double>(pgics.ug) * cell_size;
  pgccs.yg = static_cast<double>(pgics.vg) * cell_size;
  return 0;
}

int GeoTransform(const PointGCCS& car, const PointGCCS& target, PointVCS& pvcs)
{
  RETURN_EQ(const_cast<PointGCCS&>(car).Check(), 0);
  RETURN_EQ(const_cast<PointGCCS&>(target).Check(), 0);

  double rad = car.angle * DEG2RAD;
  double dx  = target.xg - car.xg;
  double dy  = target.yg - car.yg;

  pvcs.x     =  dx * std::cos(rad) + dy * std::sin(rad);
  pvcs.y     = -dx * std::sin(rad) + dy * std::cos(rad);
  pvcs.angle = target.angle - car.angle;
  if (pvcs.angle >= 360) pvcs.angle -= 360;
  return 0;
}

int GeoTransform(const PointGCCS& car, const PointVCS& pvcs, PointGCCS& out)
{
  RETURN_EQ(const_cast<PointGCCS&>(car).Check(), 0);

  double rad = car.angle * DEG2RAD;
  out.xg     = pvcs.x * std::cos(rad) - pvcs.y * std::sin(rad) + car.xg;
  out.yg     = pvcs.x * std::sin(rad) + pvcs.y * std::cos(rad) + car.yg;
  out.angle  = car.angle + pvcs.angle;
  if (out.angle >= 360) out.angle -= 360;
  return 0;
}

bool GeoTool::SetUtmZone(int zone)
{
  if (zone < 0 || zone > 60)
    return 0;
  else
  {
    utm_zone_ = zone;
    return 1;
  }
}
int GeoTool::GetUtmZone()
{
  return utm_zone_;
}
bool GeoTool::SetCellSize(double size)
{
  if (size <= 0)
    return 0;
  else
  {
    cell_size_ = size;
    return 1;
  }
}
int GeoTool::GetLongZone(double longitude)
{
  double longZone = 0.0;
  if (longitude < 0.0)
  {
    longZone = ((180.0 + longitude) / 6.0) + 1;
  }
  else
  {
    longZone = (longitude / 6.0) + 31;
  }
  return static_cast<int>(longZone);
}
int GeoTool::GCCS2GPS(const PointGCCS &pgccs, PointGPS &pgps) const
{
  return GeoTransform(pgccs, utm_zone_, pgps);
}
int GeoTool::GPS2GCCS(const PointGPS &pgps, PointGCCS &pgccs) const
{
  return GeoTransform(pgps, pgccs);
}

int GeoTool::GCCS2GICS(const PointGCCS &pgccs, PointGICS &pgics) const
{
  return GeoTransform(pgccs, cell_size_, pgics);
}
int GeoTool::GICS2GCCS(const PointGICS &pgics, PointGCCS &pgccs) const
{
  return GeoTransform(pgics, cell_size_, pgccs);
}

int GeoTool::GCCS2VCS(const PointGCCS &car_pgccs, const PointGCCS &target_pgccs,
                      PointVCS &output_pvcs) const
{
  return GeoTransform(car_pgccs, target_pgccs, output_pvcs);
}
int GeoTool::VCS2GCCS(const PointGCCS &car_pgccs, const PointVCS &target_pvcs,
                      PointGCCS &output_pgccs) const
{
  return GeoTransform(car_pgccs, target_pvcs, output_pgccs);
}

int GeoTool::GICS2VCS(const PointGCCS &car_pgccs, const PointGICS &target_pgics,
                      PointVCS &output_pvcs) const
{
  PointGCCS target_pgccs;
  RETURN_EQ(GeoTransform(target_pgics, cell_size_, target_pgccs), -1);
  return GeoTransform(car_pgccs, target_pgccs, output_pvcs);
}
int GeoTool::VCS2GICS(const PointGCCS &car_pgccs, const PointVCS &target_pvcs,
                      PointGICS &output_pgics) const
{
  PointGCCS output_pgccs;
  RETURN_EQ(GeoTransform(car_pgccs, target_pvcs, output_pgccs), -1);
  return GeoTransform(output_pgccs, cell_size_, output_pgics);
}
