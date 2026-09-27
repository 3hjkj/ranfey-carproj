#include "../include/wall_extract.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace perception
{
namespace
{

struct Pt2
{
    double x = 0.0;
    double y = 0.0;
};

double Cross(const Pt2& o, const Pt2& a, const Pt2& b)
{
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

/* Andrew monotone chain。入参 p 会被排序，凸包写进 out（逆时针，无重复首点）。

   叉积判据用 `<= 0` 而不是 `< 0`：共线点被剔掉，于是**完全共线**的点集会退化成
   2 个端点。这一点很重要 —— 实测本场景有一片 11.60 m 长、0.128 m 宽的纯墙，
   它的 XY 投影就是一条线。Python 那边 ConvexHull 对共线输入直接抛 QhullError、
   由调用方兜底；这里靠 `<= 0` 自然退化，两边在「2 个端点」这个结果上一致。 */
void ConvexHull(std::vector<Pt2>& p, std::vector<Pt2>& out)
{
    out.clear();
    if (p.size() < 3)
    {
        out = p;
        return;
    }
    std::sort(p.begin(), p.end(), [](const Pt2& a, const Pt2& b) {
        return a.x < b.x || (a.x == b.x && a.y < b.y);
    });
    p.erase(std::unique(p.begin(), p.end(), [](const Pt2& a, const Pt2& b) {
                return a.x == b.x && a.y == b.y;
            }),
            p.end());

    const int n = static_cast<int>(p.size());
    if (n < 3)
    {
        out = p;
        return;
    }

    std::vector<Pt2> h(2 * static_cast<size_t>(n));
    int k = 0;
    for (int i = 0; i < n; ++i)  // 下凸壳
    {
        while (k >= 2 && Cross(h[k - 2], h[k - 1], p[i]) <= 0.0)
            --k;
        h[k++] = p[i];
    }
    for (int i = n - 2, t = k + 1; i >= 0; --i)  // 上凸壳
    {
        while (k >= t && Cross(h[k - 2], h[k - 1], p[i]) <= 0.0)
            --k;
        h[k++] = p[i];
    }
    h.resize(static_cast<size_t>(k - 1));  // 末点与首点重复，去掉
    out.swap(h);
}

}  // namespace


void MinAreaRectXY(const std::vector<points>& pts,
                   const std::vector<int>&    sel,
                   double&                    span,
                   double&                    side)
{
    span = 0.0;
    side = 0.0;

    std::vector<Pt2> p;
    p.reserve(sel.size());
    for (int i : sel)
    {
        if (i < 0 || static_cast<size_t>(i) >= pts.size())
            continue;
        const double x = pts[i].x;
        const double y = pts[i].y;
        /* 非有限点和 dbscan.cpp 的 CheckNearPoints 一个口径：不参与几何计算，
           否则 NaN 会让 min/max 比较全部失效、算出一个假的矩形。 */
        if (!std::isfinite(x) || !std::isfinite(y))
            continue;
        p.push_back({x, y});
    }

    if (p.size() < 2)
        return;
    if (p.size() == 2)
    {
        span = std::hypot(p[0].x - p[1].x, p[0].y - p[1].y);
        return;  // side = 0，与 Python 版一致
    }

    std::vector<Pt2> h;
    ConvexHull(p, h);
    if (h.size() < 2)
        return;
    if (h.size() == 2)
    {
        span = std::hypot(h[0].x - h[1].x, h[0].y - h[1].y);
        return;  // 共线的片：退化成一条线段
    }

    const size_t m = h.size();
    double best_area = -1.0;
    for (size_t i = 0; i < m; ++i)
    {
        const Pt2& a = h[i];
        const Pt2& b = h[(i + 1) % m];
        const double ang = std::atan2(b.y - a.y, b.x - a.x);
        const double c = std::cos(-ang);
        const double s = std::sin(-ang);

        double x0 = std::numeric_limits<double>::max();
        double x1 = -x0;
        double y0 = x0;
        double y1 = -x0;
        for (size_t k = 0; k < m; ++k)
        {
            const double qx = h[k].x * c - h[k].y * s;
            const double qy = h[k].x * s + h[k].y * c;
            x0 = std::min(x0, qx);
            x1 = std::max(x1, qx);
            y0 = std::min(y0, qy);
            y1 = std::max(y1, qy);
        }
        const double w  = x1 - x0;
        const double hh = y1 - y0;
        if (best_area < 0.0 || w * hh < best_area)
        {
            best_area = w * hh;
            span      = std::max(w, hh);
            side      = std::min(w, hh);
        }
    }
}


std::vector<std::vector<int>> ExtractWalls(const std::vector<points>& data,
                                           const WallConfig&          cfg,
                                           std::vector<char>&         is_wall)
{
    std::vector<std::vector<int>> walls;
    is_wall.assign(data.size(), 0);
    if (!cfg.enable || data.empty() || cfg.min_points <= 0 || cfg.seg_eps <= 0.0)
        return walls;

    /* 1. 预分组。min_pts 用独立的 seg_min_pts 而不是主聚类的 dbscan_min_pts：
       这个门槛将来若被调大，预分组会跟着碎掉、墙就摘不出来了。 */
    DBSCAN seg(cfg.seg_eps, cfg.seg_min_pts);
    seg.SetVerbose(false);  // 这是内部辅助调用，它的耗时不是本帧的耗时，别混进日志
    seg.Clustering(data);
    const std::vector<std::vector<int>>& pieces = seg.GetClusterIndices();

    for (const auto& piece : pieces)
    {
        /* 2. 先过最便宜的两道门槛，再算凸包 —— 凸包是这里唯一有成本的一步。 */
        if (static_cast<int>(piece.size()) < cfg.min_points)
            continue;

        double z0 = std::numeric_limits<double>::max();
        double z1 = -z0;
        for (int i : piece)
        {
            if (i < 0 || static_cast<size_t>(i) >= data.size())
                continue;
            z0 = std::min(z0, data[i].z);
            z1 = std::max(z1, data[i].z);
        }
        if (z1 - z0 <= cfg.min_zspan)
            continue;

        double span = 0.0;
        double side = 0.0;
        MinAreaRectXY(data, piece, span, side);
        const double thin = (span > 1e-9) ? (side / span) : 1.0;

        /* 3. 判墙。两支：细长的单面墙走薄度；四面墙连成的环薄度很大（0.5~0.9），
              但它 XY 跨度必然超过 min_span，走第二支。 */
        if (!(thin < cfg.max_thin || span > cfg.min_span))
            continue;

        for (int i : piece)
            if (i >= 0 && static_cast<size_t>(i) < is_wall.size())
                is_wall[i] = 1;
        walls.emplace_back(piece);
    }
    return walls;
}

}  // namespace perception
