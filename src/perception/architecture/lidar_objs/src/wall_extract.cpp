#include "../include/wall_extract.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>

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


namespace
{

/* 均匀网格的格键：两个 int 格号拼成一个 64 位键。

   用哈希表而不是稠密二维数组，是为了让网格的开销只跟**点数**走、与片的尺寸和
   tau 都无关 —— tau 被配得很小时（比如 0.01），稠密数组的格数会直接爆掉。 */
inline uint64_t CellKey(int gx, int gy)
{
    return (static_cast<uint64_t>(static_cast<uint32_t>(gx)) << 32) |
           static_cast<uint64_t>(static_cast<uint32_t>(gy));
}

/* 片内局部密度掩码：keep[j] = 1 表示 piece[j] 在半径 tau 内至少有 k 个**其他**点。

   这与「到第 k 近邻的距离 < tau」（记作 dk < tau）是同一件事 ——
   dk < tau ⟺ 半径 tau 内至少有 k 个其他点 —— 所以**不需要真的去查 kNN**。
   这里用边长 tau 的均匀网格，只扫 3×3 邻域：3*tau 的边长完整包住半径 tau 的球，
   不会漏点；再对候选逐个算精确距离，凑够 k 个立刻早退。

   为什么不查 kNN：每帧要对每个预分组片都做一次，网格是 O(n) 且常数很小；
   而且「数个数」和「查第 k 近邻」在并列距离上的 tie-break 会给出不同答案，
   数个数才是判据真正想要的语义。

   非有限点（NaN/Inf）、越界下标、以及格号会溢出 int 的极端点一律 keep=0 判稀疏：
   它们本来就进不了几何计算（口径见 MinAreaRectXY 与 dbscan.cpp 的 CheckNearPoints）。 */
void DenseMask(const std::vector<points>& data,
               const std::vector<int>&    piece,
               double                     tau,
               int                        k,
               std::vector<char>&         keep)
{
    const size_t n = piece.size();
    keep.assign(n, 0);
    if (n == 0)
        return;
    if (k <= 0)
    {
        keep.assign(n, 1);  // k<=0 就是不过滤；ExtractWalls 已经拦过一道，这里兜底
        return;
    }

    std::vector<char> ok(n, 0);
    double x0 = std::numeric_limits<double>::max();  // 网格原点取各维最小值，
    double y0 = x0;                                  // 这样格号恒 >= 0，截断即 floor
    size_t good = 0;
    for (size_t j = 0; j < n; ++j)
    {
        const int i = piece[j];
        if (i < 0 || static_cast<size_t>(i) >= data.size())
            continue;
        const double x = data[i].x;
        const double y = data[i].y;
        if (!std::isfinite(x) || !std::isfinite(y))
            continue;
        ok[j] = 1;
        ++good;
        x0 = std::min(x0, x);
        y0 = std::min(y0, y);
    }
    /* 有限点连 k+1 个都凑不齐，每个点的邻居数必然 < k，整片都留不住。 */
    if (good < static_cast<size_t>(k) + 1)
        return;

    const double inv = 1.0 / tau;
    std::vector<int> gx(n, 0);
    std::vector<int> gy(n, 0);
    std::unordered_map<uint64_t, std::vector<int>> grid;
    grid.reserve(good * 2);
    for (size_t j = 0; j < n; ++j)
    {
        if (!ok[j])
            continue;
        const int i = piece[j];
        /* 格号按格距归一化。超过 2e9 格的极端点（tau 极小 + 离群很远）直接判稀疏，
           既避免 int 溢出这个 UB，语义上也对：离主群那么远当然没有邻居。 */
        const double rx = (data[i].x - x0) * inv;
        const double ry = (data[i].y - y0) * inv;
        if (!(rx >= 0.0 && rx < 2.0e9) || !(ry >= 0.0 && ry < 2.0e9))
        {
            ok[j] = 0;
            continue;
        }
        gx[j] = static_cast<int>(rx);  // x0/y0 是各维最小值，rx/ry >= 0 ⇒ 截断即 floor
        gy[j] = static_cast<int>(ry);
        grid[CellKey(gx[j], gy[j])].push_back(static_cast<int>(j));
    }

    const double tau2 = tau * tau;
    for (size_t j = 0; j < n; ++j)
    {
        if (!ok[j])
            continue;
        const int i = piece[j];
        const double xj = data[i].x;
        const double yj = data[i].y;
        int cnt = 0;
        for (int da = -1; da <= 1 && cnt < k; ++da)
        {
            for (int db = -1; db <= 1 && cnt < k; ++db)
            {
                const auto it = grid.find(CellKey(gx[j] + da, gy[j] + db));
                if (it == grid.end())
                    continue;
                for (int m : it->second)
                {
                    if (m == static_cast<int>(j))
                        continue;
                    const double dx = data[piece[m]].x - xj;
                    const double dy = data[piece[m]].y - yj;
                    if (dx * dx + dy * dy < tau2 && ++cnt >= k)
                        break;
                }
            }
        }
        keep[j] = (cnt >= k) ? 1 : 0;
    }
}

/* ───────────────────── 墙片拆成竖平面 ───────────────────── */

constexpr double kRad2Deg = 57.295779513082320876798154814105;

/* XY 上的 2×2 PCA：质心、主方向单位矢量、沿主方向的长、垂直方向的宽。 */
struct PcaXY
{
    double cx = 0.0, cy = 0.0;
    double ux = 1.0, uy = 0.0;
    double length = 0.0, width = 0.0;
};

PcaXY PcaXYOf(const std::vector<points>& data, const std::vector<int>& sel)
{
    PcaXY r;
    if (sel.empty())
        return r;
    const double n = static_cast<double>(sel.size());
    for (int i : sel) { r.cx += data[i].x; r.cy += data[i].y; }
    r.cx /= n;
    r.cy /= n;

    double a = 0.0, b = 0.0, c = 0.0;
    for (int i : sel)
    {
        const double dx = data[i].x - r.cx;
        const double dy = data[i].y - r.cy;
        a += dx * dx;
        b += dx * dy;
        c += dy * dy;
    }
    a /= n; b /= n; c /= n;

    /* 对称 2×2 矩阵的特征向量有解析解，不必上通用 eigensolver。
       大特征值对应的方向就是长轴，特征向量取 (b, l1 - a)（可直接验算 A·v = l1·v）。
       b 恰好为 0 时两个特征向量就是坐标轴，那时代入 (b, l1-a) 会得到零向量，
       所以单独分一支。 */
    const double l1 = 0.5 * (a + c) + std::sqrt(std::max(0.0, 0.25 * (a - c) * (a - c) + b * b));
    if (std::fabs(b) > 1e-12)
    {
        const double vx = b;
        const double vy = l1 - a;
        const double L = std::hypot(vx, vy);
        r.ux = vx / L;
        r.uy = vy / L;
    }
    else if (a >= c) { r.ux = 1.0; r.uy = 0.0; }
    else             { r.ux = 0.0; r.uy = 1.0; }

    double t0 = std::numeric_limits<double>::max(), t1 = -t0;
    double s0 = t0, s1 = t1;
    for (int i : sel)
    {
        const double dx = data[i].x - r.cx;
        const double dy = data[i].y - r.cy;
        const double t =  dx * r.ux + dy * r.uy;
        const double s = -dx * r.uy + dy * r.ux;
        t0 = std::min(t0, t); t1 = std::max(t1, t);
        s0 = std::min(s0, s); s1 = std::max(s1, s);
    }
    r.length = t1 - t0;
    r.width  = s1 - s0;
    return r;
}

int FindRoot(std::vector<int>& par, int x)
{
    while (par[x] != x)
    {
        par[x] = par[par[x]];  // 路径减半
        x = par[x];
    }
    return x;
}

/* 把 sel 里 mask 为真的位置按 XY 距离 <= link 连成块，返回**最大那块**。
   结果写进 out，内容是 sel 里的位置（不是 data 的下标）。

   为什么要有这一步：RANSAC 找的是平面，而平面上可以躺着两块互不相接的东西
   （同一面墙被走廊口断开、或两个物体共面）。不切开的话它们会被当成一张面。 */
void LargestXYComponent(const std::vector<points>& data,
                        const std::vector<int>&    sel,
                        const std::vector<char>&   mask,
                        double                     link,
                        std::vector<int>&          out)
{
    out.clear();
    if (link <= 0.0)
        return;
    std::vector<int> pos;
    for (size_t k = 0; k < sel.size() && k < mask.size(); ++k)
    {
        if (!mask[k])
            continue;
        const points& p = data[sel[k]];
        if (!std::isfinite(p.x) || !std::isfinite(p.y))
            continue;  // 非有限点算不出格号，本来也进不了内点集，这里只是兜底
        pos.push_back(static_cast<int>(k));
    }
    const int m = static_cast<int>(pos.size());
    if (m == 0)
        return;

    std::vector<int> par(m);
    for (int i = 0; i < m; ++i)
        par[i] = i;

    const double inv = 1.0 / link;
    std::unordered_map<uint64_t, std::vector<int>> grid;
    grid.reserve(static_cast<size_t>(m) * 2);
    for (int i = 0; i < m; ++i)
    {
        const points& p = data[sel[pos[i]]];
        grid[CellKey(static_cast<int>(std::floor(p.x * inv)),
                     static_cast<int>(std::floor(p.y * inv)))]
            .push_back(i);
    }

    const double l2 = link * link;
    for (int i = 0; i < m; ++i)
    {
        const points& p = data[sel[pos[i]]];
        const int gx = static_cast<int>(std::floor(p.x * inv));
        const int gy = static_cast<int>(std::floor(p.y * inv));
        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                const auto it = grid.find(CellKey(gx + dx, gy + dy));
                if (it == grid.end())
                    continue;
                for (int j : it->second)
                {
                    if (j <= i)
                        continue;
                    const points& q = data[sel[pos[j]]];
                    const double ex = p.x - q.x;
                    const double ey = p.y - q.y;
                    if (ex * ex + ey * ey > l2)
                        continue;
                    const int ra = FindRoot(par, i);
                    const int rb = FindRoot(par, j);
                    if (ra != rb)
                        par[ra] = rb;
                }
            }
        }
    }

    /* 取最大的分量。并列时取根号小的那个，让结果与遍历顺序无关。 */
    std::unordered_map<int, int> cnt;
    int best_root = -1, best_n = 0;
    for (int i = 0; i < m; ++i)
    {
        const int r = FindRoot(par, i);
        const int c = ++cnt[r];
        if (c > best_n || (c == best_n && r < best_root))
        {
            best_n = c;
            best_root = r;
        }
    }
    out.reserve(static_cast<size_t>(best_n));
    for (int i = 0; i < m; ++i)
        if (FindRoot(par, i) == best_root)
            out.push_back(pos[i]);
}

/* 3D RANSAC 找一张近竖直的平面。inl[k] = 1 表示 sel[k] 在面上。

   **抽样必须用固定种子**：同一片墙逐帧的点几乎一样，固定种子就得到几乎一样的
   抽样序列，拆出来的面才逐帧稳（实测 15/15 帧稳定 2 面）。拿随机设备播种会让
   面数帧间跳，下游 tracker 跟着一会儿多一个目标一会儿少一个。 */
void FitVerticalPlane(const std::vector<points>& data,
                      const std::vector<int>&    sel,
                      const WallConfig&          cfg,
                      std::vector<char>&         inl)
{
    const int n = static_cast<int>(sel.size());
    inl.assign(static_cast<size_t>(n), 0);
    if (n < 3 || cfg.face_iter <= 0)
        return;

    uint64_t s = 88172645463325252ull;  // xorshift64，固定种子
    auto rnd = [&s]() {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        return s;
    };

    std::vector<char> cur(static_cast<size_t>(n), 0);
    int best = 0;
    const int stop = static_cast<int>(static_cast<double>(n) * 0.95);
    for (int it = 0; it < cfg.face_iter; ++it)
    {
        const int i0 = static_cast<int>(rnd() % static_cast<uint64_t>(n));
        const int i1 = static_cast<int>(rnd() % static_cast<uint64_t>(n));
        const int i2 = static_cast<int>(rnd() % static_cast<uint64_t>(n));
        if (i0 == i1 || i1 == i2 || i0 == i2)
            continue;

        const points& A = data[sel[i0]];
        const points& B = data[sel[i1]];
        const points& C = data[sel[i2]];
        const double ux = B.x - A.x, uy = B.y - A.y, uz = B.z - A.z;
        const double vx = C.x - A.x, vy = C.y - A.y, vz = C.z - A.z;
        double nx = uy * vz - uz * vy;
        double ny = uz * vx - ux * vz;
        double nz = ux * vy - uy * vx;
        const double L = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (L < 1e-9)
            continue;  // 三点共线，法向无意义
        nx /= L; ny /= L; nz /= L;
        if (std::fabs(nz) > cfg.face_max_tilt)
            continue;  // 墙是竖的：水平面（地面、桌面）在这里就被挡掉了

        int cnt = 0;
        for (int k = 0; k < n; ++k)
        {
            const points& p = data[sel[k]];
            const double d = (p.x - A.x) * nx + (p.y - A.y) * ny + (p.z - A.z) * nz;
            const char hit = (std::fabs(d) < cfg.face_tol) ? 1 : 0;
            cur[static_cast<size_t>(k)] = hit;
            cnt += hit;
        }
        if (cnt > best)
        {
            best = cnt;
            inl = cur;
            if (best >= stop)
                break;  // 已经几乎全在面上，再抽也不会更好
        }
    }
}

/* 两个点集之间最近的 XY 距离，超过 cut 就提前返回（先用包围盒挡一道）。 */
double MinXYDist(const std::vector<points>& data,
                 const std::vector<int>&    a,
                 const std::vector<int>&    b,
                 double                     cut)
{
    if (a.empty() || b.empty())
        return std::numeric_limits<double>::max();
    double ax0 = std::numeric_limits<double>::max(), ax1 = -ax0, ay0 = ax0, ay1 = -ax0;
    for (int i : a)
    {
        ax0 = std::min(ax0, data[i].x); ax1 = std::max(ax1, data[i].x);
        ay0 = std::min(ay0, data[i].y); ay1 = std::max(ay1, data[i].y);
    }
    double bx0 = std::numeric_limits<double>::max(), bx1 = -bx0, by0 = bx0, by1 = -bx0;
    for (int i : b)
    {
        bx0 = std::min(bx0, data[i].x); bx1 = std::max(bx1, data[i].x);
        by0 = std::min(by0, data[i].y); by1 = std::max(by1, data[i].y);
    }
    const double gx = std::max(0.0, std::max(bx0 - ax1, ax0 - bx1));
    const double gy = std::max(0.0, std::max(by0 - ay1, ay0 - by1));
    if (std::hypot(gx, gy) > cut)
        return std::numeric_limits<double>::max();

    double best = std::numeric_limits<double>::max();
    for (int i : a)
    {
        for (int j : b)
        {
            const double dx = data[i].x - data[j].x;
            const double dy = data[i].y - data[j].y;
            best = std::min(best, dx * dx + dy * dy);
            if (best <= 0.0)
                return 0.0;
        }
    }
    return std::sqrt(best);
}

/* 方向相近、端点相接、且**合起来仍是薄面**的两张面并成一张。

   前两条不够：真正相邻的两面墙（比如直角墙的两条臂）方向差 90° 不会被并，
   但一条略微弯折的墙会被 RANSAC 切成两段（实测切在 4.6 m + 3.0 m 处，两段方向差
   16°），不并回去面数就在 3 ↔ 4 之间逐帧跳。最后那条「并完还是薄面」是防它把
   两面真正相邻的墙糊成一块方疙瘩。 */
void MergeFaces(const std::vector<points>& data,
                std::vector<std::vector<int>>& faces,
                const WallConfig&          cfg)
{
    bool changed = true;
    while (changed && faces.size() > 1)
    {
        changed = false;
        for (size_t i = 0; i < faces.size() && !changed; ++i)
        {
            for (size_t j = i + 1; j < faces.size() && !changed; ++j)
            {
                const PcaXY pa = PcaXYOf(data, faces[i]);
                const PcaXY pb = PcaXYOf(data, faces[j]);
                /* 直线的夹角取 [0°, 90°]：方向矢量是没有正负的 */
                const double cross = std::fabs(pa.ux * pb.uy - pa.uy * pb.ux);
                const double deg = std::asin(std::min(1.0, cross)) * kRad2Deg;
                if (deg > cfg.face_merge_deg)
                    continue;
                if (MinXYDist(data, faces[i], faces[j], cfg.face_merge_gap) > cfg.face_merge_gap)
                    continue;

                std::vector<int> cat = faces[i];
                cat.insert(cat.end(), faces[j].begin(), faces[j].end());
                const PcaXY pc = PcaXYOf(data, cat);
                if (pc.length > 1e-9 && pc.width / pc.length > cfg.face_merge_thin)
                    continue;

                faces[i].swap(cat);
                faces.erase(faces.begin() + static_cast<long>(j));
                changed = true;
            }
        }
    }
}

/* 没归到任何面的点并到最近的那张面上 —— **一个点都不丢**。

   实测剩余 88~136 点、到最近面距离中位 0.30 m、最大 1.10 m、局部 3D 短轴 > 0.34
   的有 0 个：它们是墙面的壳（墙面本身略有弯曲 / 掠射角下的毛边），不是物体。
   丢掉会让墙看起来变薄、变碎；并进最近的面只让那张面宽一点。 */
void AttachNearest(const std::vector<points>&    data,
                   std::vector<std::vector<int>>& faces,
                   const std::vector<int>&       rem)
{
    for (int i : rem)
    {
        double best_d = std::numeric_limits<double>::max();
        size_t best_f = 0;
        for (size_t f = 0; f < faces.size(); ++f)
        {
            double d = std::numeric_limits<double>::max();
            for (int j : faces[f])
            {
                const double dx = data[i].x - data[j].x;
                const double dy = data[i].y - data[j].y;
                const double dz = data[i].z - data[j].z;
                d = std::min(d, dx * dx + dy * dy + dz * dz);
                if (d <= 0.0)
                    break;
            }
            if (d < best_d)
            {
                best_d = d;
                best_f = f;
            }
        }
        faces[best_f].push_back(i);
    }
}

}  // namespace


std::vector<std::vector<int>> SplitWallFaces(const std::vector<points>& data,
                                             const std::vector<int>&    piece,
                                             const WallConfig&          cfg)
{
    std::vector<std::vector<int>> whole;
    whole.push_back(piece);
    if (!cfg.face_split || cfg.face_min_pts <= 0 || cfg.face_iter <= 0 ||
        static_cast<int>(piece.size()) < 2 * cfg.face_min_pts)
        return whole;

    /* 1. 反复找面、切走，剩下的继续找。 */
    std::vector<std::vector<int>> faces;
    std::vector<int> rem = piece;
    while (static_cast<int>(rem.size()) >= cfg.face_min_pts)
    {
        std::vector<char> inl;
        FitVerticalPlane(data, rem, cfg, inl);

        std::vector<int> take;  // rem 里的**位置**，不是 data 的下标
        LargestXYComponent(data, rem, inl, cfg.face_link, take);
        /* 凑不够一张面就收手。剩下的点会在第 4 步并到已有的面上，不会丢。 */
        if (static_cast<int>(take.size()) < cfg.face_min_pts)
            break;

        std::vector<int> f;
        f.reserve(take.size());
        for (int k : take)
            if (k >= 0 && k < static_cast<int>(rem.size()))
                f.push_back(rem[static_cast<size_t>(k)]);
        std::sort(f.begin(), f.end());
        faces.push_back(std::move(f));

        std::vector<char> gone(rem.size(), 0);
        for (int k : take)
            if (k >= 0 && k < static_cast<int>(rem.size()))
                gone[static_cast<size_t>(k)] = 1;
        std::vector<int> left;
        left.reserve(rem.size());
        for (size_t k = 0; k < rem.size(); ++k)
            if (!gone[k])
                left.push_back(rem[k]);
        rem.swap(left);
    }

    /* 一张面都没拆出来 / 只拆出一张：返回整片。这是这个函数的**安全性来源** ——
       普通直墙（实测 370 点那片 11.6 × 0.14 m）逐位与不拆时相同。 */
    if (faces.size() < 2)
        return whole;

    /* 2. 把同一面墙上多切的那一刀并回去。 */
    MergeFaces(data, faces, cfg);
    if (faces.size() < 2)
        return whole;

    /* 3. 剩余的点并到最近的面（一个不丢）。 */
    AttachNearest(data, faces, rem);

    /* 4. 按点数降序，让 idx 的分配顺序逐帧稳定。 */
    std::sort(faces.begin(), faces.end(),
              [](const std::vector<int>& a, const std::vector<int>& b) {
                  return a.size() > b.size();
              });
    return faces;
}


std::vector<std::vector<int>> ExtractWalls(const std::vector<points>& data,
                                           const WallConfig&          cfg,
                                           std::vector<char>&         is_wall,
                                           std::vector<char>&         is_sparse)
{
    std::vector<std::vector<int>> walls;
    is_wall.assign(data.size(), 0);
    is_sparse.assign(data.size(), 0);
    if (!cfg.enable || data.empty() || cfg.min_points <= 0 || cfg.seg_eps <= 0.0)
        return walls;

    /* 1. 预分组。min_pts 用独立的 seg_min_pts 而不是主聚类的 dbscan_min_pts：
       这个门槛将来若被调大，预分组会跟着碎掉、墙就摘不出来了。 */
    DBSCAN seg(cfg.seg_eps, cfg.seg_min_pts);
    seg.SetVerbose(false);  // 这是内部辅助调用，它的耗时不是本帧的耗时，别混进日志
    seg.Clustering(data);
    const std::vector<std::vector<int>>& pieces = seg.GetClusterIndices();

    /* 密度过滤整条路径的开关。density_tau/density_k 任一非正就退化成纯几何判据，
       与加这个模块之前逐点相同 —— A/B 时靠的就是这一点。 */
    const bool dense_on = (cfg.density_tau > 0.0 && cfg.density_k > 0 &&
                           cfg.density_min_piece > 0 && cfg.density_min_pts > 0);

    /* 2. 候选片：真正拿去判墙的点集。密度过滤关着时就是预分组片本身；开着时是
          「片内实体点重连之后的小片」。下面第 5 步的判墙两支对两者一视同仁。 */
    std::vector<std::vector<int>> cands;
    cands.reserve(pieces.size());

    if (!dense_on)
    {
        cands.assign(pieces.begin(), pieces.end());
    }
    else
    {
        /* 片内重连用的 DBSCAN，跨片复用一个对象 —— Clustering 会 clear + resize
           全部内部状态（含 adj_points_count），复用与每次新建逐点等价。 */
        DBSCAN sub(cfg.seg_eps, cfg.seg_min_pts);
        sub.SetVerbose(false);

        std::vector<char>   keep;
        std::vector<points> sub_pts;
        std::vector<int>    sub_orig;  // sub_pts[k] 在 data 里的原下标

        for (const auto& piece : pieces)
        {
            /* 太小的片完全不碰：它不可能是墙（判墙要 min_points，通常 30），
               也不该因为几十个点里凑不出 density_min_pts 个实体点就丢点。 */
            if (static_cast<int>(piece.size()) < cfg.density_min_piece)
                continue;

            DenseMask(data, piece, cfg.density_tau, cfg.density_k, keep);

            int dense_n = 0;
            for (char c : keep)
                if (c) ++dense_n;
            /* 整片都不够密：它不是墙，**也不丢点** —— 丢的依据是「片内有实体、
               这些点是实体之外的散点」，整片都散就没有这个依据。 */
            if (dense_n < cfg.density_min_pts)
                continue;

            /* 丢点。注意这一步与下面有没有识别出墙**无关**：一个片只要够大够密，
               它的散点就一律丢，否则它们会流回主聚类重新当桥（实测非墙最大簇
               从 123 涨到 207）。 */
            if (cfg.density_drop)
            {
                for (size_t j = 0; j < piece.size(); ++j)
                {
                    const int i = piece[j];
                    if (!keep[j] && i >= 0 && static_cast<size_t>(i) < is_sparse.size())
                        is_sparse[i] = 1;
                }
            }

            sub_pts.clear();
            sub_orig.clear();
            for (size_t j = 0; j < piece.size(); ++j)
            {
                if (!keep[j])
                    continue;
                sub_pts.push_back(data[piece[j]]);
                sub_orig.push_back(piece[j]);
            }

            sub.Clustering(sub_pts);  // 返回值丢掉，只要簇下标表
            const std::vector<std::vector<int>>& sub_idx = sub.GetClusterIndices();
            for (const auto& sp : sub_idx)
            {
                if (sp.size() < 2)
                    continue;
                std::vector<int> c;
                c.reserve(sp.size());
                for (int t : sp)
                    if (t >= 0 && static_cast<size_t>(t) < sub_orig.size())
                        c.push_back(sub_orig[t]);
                /* GetClusterIndices 已经是升序（dbscan.cpp:64-70 是按 i 升序 push 的），
                   这里再排一次只是为了让「返回的原下标升序」这条约定不依赖实现细节。 */
                std::sort(c.begin(), c.end());
                cands.emplace_back(std::move(c));
            }
        }
    }

    /* 3. 判墙。 */
    for (const auto& c : cands)
    {
        /* 4. 先过最便宜的两道门槛，再算凸包 —— 凸包是这里唯一有成本的一步。 */
        if (static_cast<int>(c.size()) < cfg.min_points)
            continue;

        double z0 = std::numeric_limits<double>::max();
        double z1 = -z0;
        for (int i : c)
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
        MinAreaRectXY(data, c, span, side);
        const double thin = (span > 1e-9) ? (side / span) : 1.0;

        /* 5. 判墙。两支：细长的单面墙走薄度；四面墙连成的环薄度很大（0.5~0.9），
              但它 XY 跨度必然超过 min_span，走第二支。 */
        if (!(thin < cfg.max_thin || span > cfg.min_span))
            continue;

        for (int i : c)
            if (i >= 0 && static_cast<size_t>(i) < is_wall.size())
                is_wall[i] = 1;

        /* 6. 拆面。上面两步判的是「它是不是墙」——点数、z 跨度、薄度，判不出
              「它是不是**一面**墙」。一面 L 形的直角墙在 DBSCAN 眼里是一张连续
              曲面，天然连成一片，整片发出去就是 MinRotateRect 给的那个
              6.35 × 11.39 m 的大框：中心落在房间当中，框横跨整个转角，站在墙边的
              物体全被圈在里面（实测 111 点那个有 67/111 个点在这个框内），看起来
              就是「墙连带旁边的物品聚成一团」。拆成竖平面之后每张面各发一个目标，
              框变成约 8.0 × 0.3 m 与 7.7 × 0.9 m，贴着墙走。

              拆不出第二张面时 SplitWallFaces 返回的就是这整片 c，与不拆逐位相同 ——
              这是这个改动唯一的安全性来源，普通直墙不受影响。 */
        std::vector<std::vector<int>> fs = SplitWallFaces(data, c, cfg);
        for (auto& f : fs)
            walls.emplace_back(std::move(f));
    }
    return walls;
}

}  // namespace perception
