#ifndef _DBSCAN_H__
#define _DBSCAN_H__

#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>        // ← 原来是 <ros/ros.h>，仅此一处改动
// #include "glog/include/logging.h"

namespace perception
{
    struct points
    {
        double x;
        double y;
        double z;
    };

    using namespace std;

    const int DB_NOISE          = -2;
    const int DB_NOT_CLASSIFIED = -1;

    class DbscanType
    {
    public:
        DbscanType()  = default;
        ~DbscanType() = default;

        int    id               = 0;
        double x                = 0.0;
        double y                = 0.0;
        double z                = 0.0;      // ← 原来没有这一维，见 GetDist 的注释
        int    adj_points_count = 0;
        int    cluster_idx      = DB_NOT_CLASSIFIED;

        /* 三维欧氏距离。原来这里只算 x、y（DbscanType 也只有这两个坐标），
           于是一整面竖直墙在聚类眼里被压成一条曲线：墙面上同一方位、不同高度的
           点 x-y 几乎重合，距离≈0，整圈墙必然连成一块。实测同一帧同一批点，
           二维时最大簇 927 点、跨 12.0×9.2×3.3 m；补上 z 并收到 eps=0.3 后
           最大簇 137 点、跨 3.6×0.8×2.7 m。
           注意光是补 z 不够：eps>=0.4 时墙在三维里本来就真连通（相邻扫描线的
           垂直间距小于 0.4 m），照样成块，必须同时收 eps。

           顺带去掉原来那句 `if (fabs(x-ot.x) > 0.5 || fabs(y-ot.y) > 0.5) return 10;`。
           那个 0.5 硬上限只在 eps>0.5 时才会卡人（eps<=0.5 时真正的约束是
           `GetDist(...) <= eps_`，等价于半径 eps 的正圆），但它会让 eps 写成 0.6
           实际得到"半宽 0.5 的方形、四角按 0.6 切掉"，跟字面意思对不上，
           而且不报错。去掉后 eps 写多少就是多少。 */
        double GetDist(const DbscanType& ot) const
        {
            const double dx = x - ot.x;
            const double dy = y - ot.y;
            const double dz = z - ot.z;
            return sqrt(dx * dx + dy * dy + dz * dz);
        }
    };

    class DBSCAN
    {
    private:
        vector<DbscanType> points_;
        vector<vector<int>> cluster_;     // 存储聚类结果
        vector<vector<int>> adj_points_;  // 存储邻接表

        double eps_;
        int    min_points_num_;
        int    size_;
        int    cluster_idx_;
        bool   verbose_ = true;

        void ClearBuffer();
        void DepthFirstSearch(int now, int c);
        void CheckNearPoints();
        bool IsCoreObject(int idx);
        vector<vector<points>> GetResult(const vector<points>& data_in,
                                         vector<vector<int>>& indices);

    public:
        DBSCAN(double eps, int min_points_num);
        ~DBSCAN();

        /* 配置入口。eps_ / min_points_num_ 是 private 且原先只在构造函数里赋值，
           外面改不了，所以门槛只能写死在 lidar_cluster.cpp 的字面量里。
           现在 eps 按字面生效（三维欧氏）。历史坑已修：GetDist 原来只看 x、y，
           且带一个 `>0.5 return 10` 的提前返回 —— 那个上限只在 eps>0.5 时才会
           卡人（eps<=0.5 时真正的约束是 `GetDist(...) <= eps_`，等价于半径 eps
           的正圆），但它让 SetEps(0.6) 实际得到"半宽 0.5 的方形、四角按 0.6 切掉"，
           跟字面意思对不上还不报错。 */
        void SetEps(double eps)         { eps_ = eps; }
        void SetMinPoints(int min_pts)  { min_points_num_ = min_pts; }

        /* Clustering() 每帧会往 stdout 打两行耗时（" time check near:" /
           " cluster:"）。摘墙的预分组是**每帧第二次**调用 Clustering()，
           它的耗时是内部辅助开销、不是本帧的聚类耗时，混进日志会把那两行的
           含义搞乱，所以那里关掉。默认 true，生产主聚类的行为逐字不变。 */
        void SetVerbose(bool v)         { verbose_ = v; }

        void Init(const vector<points>& data_in);
        vector<vector<points>> Clustering(const vector<points>& data_in);
        vector<int>            ClusteringSingle(const vector<points>& data_in);

        /* 上一次 Clustering() 的簇索引表：cluster_[c] 存的是第 c 簇的各个点
           在输入 data_in 里的下标。它与 Clustering() 的返回值**逐项一一对应**
           （见 dbscan.cpp 的 GetResult：按下标顺序搬运、不跳过空簇），所以
           外部可以借它把"每个输入点落在哪个簇"完整还原出来。
           注意 ClusteringSingle() 也会写 cluster_，但它只返回最大那簇。 */
        const vector<vector<int>>& GetClusterIndices() const { return cluster_; }
    };
}

#endif  // _DBSCAN_H__
