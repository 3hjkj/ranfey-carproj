#include "../include/lidar_cluster.h"
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>        // std::max，见类型判定里的墙
#include <cmath>            // std::sqrt，见最小点数的距离缩放
#include <cstring>          // memcpy，见 PublishLabels



namespace perception
{

/* ───────── 构造 / 析构 ───────── */
PointsCluster::PointsCluster(const rclcpp::Node::SharedPtr& node)
: node_(node)
{
  /* 若需要发布调试点云，可取消以下两行注释
  pub_no_ground_points_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          topic_name1, 10);
  */

  pub_labels_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      topic_labels, 10);

  /* 默认值与 Configure 的成员默认值一致；Init() 读完 objs.json 会再覆盖一次 */
  dbscan_          = std::make_shared<DBSCAN>(dbscan_eps_, dbscan_min_pts_);
  min_rotate_rect_ = std::make_shared<min_rotate_rect>();
  point_tmp.reset(new pcl::PointCloud<pcl::PointXYZ>);
}


/* ───────── 注入聚类门槛 ───────── */
void PointsCluster::Configure(const lidar_objs::objs_config& cfg)
{
  dbscan_eps_         = cfg.dbscan_eps;
  dbscan_min_pts_     = cfg.dbscan_min_pts;
  min_cluster_points_ = cfg.min_cluster_points;
  min_obj_height_     = cfg.min_obj_height;
  min_pts_near_       = cfg.min_pts_near;
  min_pts_rmax_       = cfg.min_pts_rmax;
  wall_size_m_        = cfg.wall_size_m;

  /* ← 这两行原来漏了。漏掉的后果不是"少一个旋钮"，而是**切分永远按头文件默认值
     （split_size_m_=6.0 / split_eps_=0.3）在跑**，objs.json 里写的 split_size_m
     根本不进代码。实测：主 DBSCAN(eps=1.2,3) 本来把整间房聚成 13 簇（与真值的
     13~17 吻合），但因为 3 个跨 >6m 的巨簇（墙）被固定 0.3 的切分器再切一遍，
     最终发出 45 个目标 —— 逐字复刻 Pub 的离线程序算出的也是 45，与节点一致。
     这也解释了为什么 dbscan_eps 从 0.6 翻到 1.2 指标纹丝不动：两个 eps 都产出
     同样的 3 个巨簇，再被同一个 0.3 切碎，结果自然一样。 */
  split_eps_          = cfg.split_eps;
  split_size_m_       = cfg.split_size_m;

  /* 摘墙。seg_eps 在这里算出来，不写死绝对值 —— 理由见 wall_extract.h：
     实测 eps 恰好等于体素格距时面邻接会在浮点边界上断掉，这个门槛和体素绑死。 */
  wall_cfg_.enable      = cfg.wall_extract;
  wall_cfg_.seg_eps     = static_cast<double>(cfg.cluster_voxel_size) *
                          cfg.wall_seg_eps_ratio;
  wall_cfg_.seg_min_pts = cfg.wall_seg_min_pts;
  wall_cfg_.min_points  = cfg.wall_min_points;
  wall_cfg_.min_zspan   = cfg.wall_min_zspan;
  wall_cfg_.max_thin    = cfg.wall_max_thin;
  wall_cfg_.min_span    = cfg.wall_min_span;

  /* 片内局部密度过滤。只滤不丢是不管用的那个变体（实测非墙最大簇 123 → 207），
     所以 drop 这个键也要打日志、也要盯。 */
  wall_cfg_.density_drop      = cfg.wall_density_drop;
  wall_cfg_.density_tau       = cfg.wall_density_tau;
  wall_cfg_.density_k         = cfg.wall_density_k;
  wall_cfg_.density_min_piece = cfg.wall_density_min_piece;
  wall_cfg_.density_min_pts   = cfg.wall_density_min_pts;

  /* 墙片拆竖平面。密度过滤只解决「两面墙被桥点焊住」，解决不了「一面直角墙
     算一个目标还是两个」—— 拆完每张面各发一个目标，那个横跨房间的
     6.35 × 11.39 m 的大框才会变成贴着墙的 8.0 × 0.3 m 与 7.7 × 0.9 m。
     实测数据见 wall_extract.h 的「直角墙」一节。 */
  wall_cfg_.face_split      = cfg.wall_face_split;
  wall_cfg_.face_min_pts    = cfg.wall_face_min_pts;
  wall_cfg_.face_iter       = cfg.wall_face_iter;
  wall_cfg_.face_tol        = cfg.wall_face_tol;
  wall_cfg_.face_max_tilt   = cfg.wall_face_max_tilt;
  wall_cfg_.face_link       = cfg.wall_face_link;
  wall_cfg_.face_merge_deg  = cfg.wall_face_merge_deg;
  wall_cfg_.face_merge_gap  = cfg.wall_face_merge_gap;
  wall_cfg_.face_merge_thin = cfg.wall_face_merge_thin;

  if (dbscan_) {
    dbscan_->SetEps(dbscan_eps_);
    dbscan_->SetMinPoints(dbscan_min_pts_);
  }

  /* 这个 INFO 是 fmt 风格（{}），不是 printf 的 %f/%d —— 写错了不会报错，
     只会把格式串原样打出来，等于没打日志。
     **每个旋钮都必须出现在这行里**：split_* 就是因为没打，才能在被漏掉的情况下
     静默按默认值跑了这么久。 */
  INFO("cluster cfg: eps={} min_pts={} min_cluster_points={} min_h={} "
       "min_pts_near={} rmax={} wall={} split_eps={} split_size_m={}",
       dbscan_eps_, dbscan_min_pts_, min_cluster_points_, min_obj_height_,
       min_pts_near_, min_pts_rmax_, wall_size_m_, split_eps_, split_size_m_);
  INFO("wall cfg: enable={} seg_eps={} seg_min_pts={} min_points={} "
       "min_zspan={} max_thin={} min_span={}",
       wall_cfg_.enable, wall_cfg_.seg_eps, wall_cfg_.seg_min_pts,
       wall_cfg_.min_points, wall_cfg_.min_zspan, wall_cfg_.max_thin,
       wall_cfg_.min_span);
  INFO("wall density cfg: drop={} tau={} k={} min_piece={} min_pts={}",
       wall_cfg_.density_drop, wall_cfg_.density_tau, wall_cfg_.density_k,
       wall_cfg_.density_min_piece, wall_cfg_.density_min_pts);
  INFO("wall face cfg: split={} min_pts={} iter={} tol={} max_tilt={} link={} "
       "merge_deg={} merge_gap={} merge_thin={}",
       wall_cfg_.face_split, wall_cfg_.face_min_pts, wall_cfg_.face_iter,
       wall_cfg_.face_tol, wall_cfg_.face_max_tilt, wall_cfg_.face_link,
       wall_cfg_.face_merge_deg, wall_cfg_.face_merge_gap,
       wall_cfg_.face_merge_thin);
}


/* 最小点数随距离缩放：近处要得严（碎片多），远处放宽（行人本来点数就少） */
int PointsCluster::MinPointsAt(double r) const
{
  if (min_pts_rmax_ <= 0.0) return min_cluster_points_;
  double t = r / min_pts_rmax_;
  if (t <= 0.0) t = 0.0;
  if (t >= 1.0) return min_cluster_points_;
  return static_cast<int>(min_pts_near_ +
                          (min_cluster_points_ - min_pts_near_) * t + 0.5);
}


/* ───────── 对外接口：聚类并返回 Objects ───────── */
lidar_msgs::msg::Objects
PointsCluster::Pub(const pcl::PointCloud<pcl::PointXYZ>::Ptr& data_in)
{
  /* 1. PCL → std::vector<points>（与旧版相同） */
  std::vector<points> data = CloudToPoints(data_in);

  /* 2. 摘墙。墙和物体在几何上都是连续曲面，距离判据分不开它们（判据、三次失败
        尝试、实测数据见 wall_extract.h）。所以先把墙点整片分出去，让墙不再充当
        「物体之间的桥」，剩下的点再走主聚类。
     is_sparse 是片内密度过滤剔出来的散点，**同样不喂主聚类**：留着它们会以
        seg_eps 为跳板把隔着一米多的两片墙重新连起来（实测非墙最大簇 123 → 207）。
     keep_idx[k] = 第 k 个保留点在 data 里的原下标。主聚类跑在 data_objs 上，
        所以后面每个簇的下标都要经它映回 data —— 标签写错下标会标到别的点上。 */
  std::vector<char>             is_wall;
  std::vector<char>             is_sparse;
  std::vector<std::vector<int>> wall_pieces = ExtractWalls(data, wall_cfg_, is_wall, is_sparse);

  std::vector<int>    keep_idx;
  std::vector<points> data_objs;
  keep_idx.reserve(data.size());
  data_objs.reserve(data.size());
  /* 什么都没被摘掉时这一圈就是恒等映射 —— 与没有这个模块时逐位相同，
     所以不需要再分一条「没摘墙」的快路径出来。 */
  for (size_t i = 0; i < data.size(); ++i)
    if (!is_wall[i] && !is_sparse[i]) { keep_idx.push_back(static_cast<int>(i)); data_objs.push_back(data[i]); }

  /* 兜底：摘完剩下的点太少就整帧不摘。宁可退化成旧行为（墙和物体合并），
     也不能因为判据在某帧抽风把点云清空、整帧不发目标 —— 那对下游是静默失效。
     连密度过滤丢掉的散点也一并放回来：这一支的语义就是「这一帧当作没做过过滤」。 */
  if (!wall_pieces.empty() && (data_objs.size() < 50 || data_objs.size() * 10 < data.size()))
  {
    ERROR("wall extract skipped: keep {} / {} points", data_objs.size(), data.size());
    keep_idx.clear();
    data_objs.clear();
    wall_pieces.clear();
    is_sparse.assign(data.size(), 0);
    for (size_t i = 0; i < data.size(); ++i) { keep_idx.push_back(static_cast<int>(i)); data_objs.push_back(data[i]); }
  }

  /* 3. 主聚类，用**大半径** dbscan_eps_。这里刻意不为了拆墙而把半径调小：
     半径一小，远处物体的扫描线之间本来隔得就宽，会连物体一起切碎（实测
     一个 344 点的物体在 0.25 m 下裂成 17 瓣）。 */
  std::vector<std::vector<points>> indices = dbscan_->Clustering(data_objs);

  lidar_msgs::msg::Objects objs;

  /* 主聚类一簇都没找到（全是噪点）时 indices 为空。**这里不能直接 return**：
     摘墙开着的时候墙点根本不在 indices 里，直接返回等于墙点既没进主聚类、
     也没被发成目标 —— 静默丢掉一整圈墙。所以只用一个标志跳过主循环。 */
  const bool has_clusters = !indices.empty();

  /* DBSCAN 的簇索引表：cluster_idx[c] 是第 c 簇的点在 **data_objs** 里的下标
     （主聚类吃的是 data_objs），与上面的 indices 逐项对应。下面被 continue 掉的
     簇（点太少、太矮、切完还是碎片）算噪点，标签留 -1。 */
  const std::vector<std::vector<int>>& cluster_idx = dbscan_->GetClusterIndices();
  std::vector<int32_t> labels(data.size(), -1);

  /* 3a 用的切分器。只处理跨度超限的巨簇，与主聚类共用 min_pts。 */
  DBSCAN splitter(split_eps_, dbscan_min_pts_);

  int idx = 0;
  for (size_t ci = 0; has_clusters && ci < indices.size(); ++ci)
  {
    const auto& cluster = indices[ci];

    /* src 在下面**恒为 data 的原下标**。主聚类跑在 data_objs 上，所以这里要经
       keep_idx 映回去；不摘墙时 keep_idx 是恒等映射，src == cluster_idx[ci]，
       与没有这个模块时逐位相同。3a 的切分和 3b 的直发都用这一个映射。 */
    std::vector<int> src_idx;
    if (ci < cluster_idx.size())
    {
      const auto& raw = cluster_idx[ci];
      src_idx.reserve(raw.size());
      for (int k : raw)
        if (k >= 0 && static_cast<size_t>(k) < keep_idx.size())
          src_idx.push_back(keep_idx[k]);
    }
    const std::vector<int>* src = &src_idx;

    /* 3a. 巨簇事后切分：一整圈墙在三维里是真连通的曲面，任何"能把车连成一体"
           的半径都大到足以让它连成一块，实测 eps 在 0.30 与 0.33 之间是一跳，
           中间没有可用档位。所以反过来做：**大半径保真物体完整，只把跨度超限
           的簇用小半径再切一遍**。真物体小，永远进不了这个分支。
           只切一层不递归 —— eps=split_eps_ 下最大簇约 4 m，不会再超限。

           摘墙开着的时候这个分支基本不会触发（墙已经被摘走，剩下的簇跨不到
           6 m），但**不能改坏**：split_size_m_ 一开它就是唯一的巨簇出路。 */
    if (split_size_m_ > 0.0 && !cluster.empty())
    {
      double x0 = cluster[0].x, x1 = x0;
      double y0 = cluster[0].y, y1 = y0;
      double z0 = cluster[0].z, z1 = z0;
      for (const auto& p : cluster)
      {
        x0 = std::min(x0, p.x); x1 = std::max(x1, p.x);
        y0 = std::min(y0, p.y); y1 = std::max(y1, p.y);
        z0 = std::min(z0, p.z); z1 = std::max(z1, p.z);
      }
      const double span = std::max(std::max(x1 - x0, y1 - y0), z1 - z0);

      if (span > split_size_m_)
      {
        std::vector<std::vector<points>> subs = splitter.Clustering(cluster);
        const std::vector<std::vector<int>>& sub_idx = splitter.GetClusterIndices();
        for (size_t si = 0; si < subs.size(); ++si)
        {
          /* sub_idx 的下标是相对于 cluster 的，要经 src 映射回 data 的下标，
             否则标签会写到别的点上去。 */
          std::vector<int> orig;
          if (src && si < sub_idx.size())
          {
            orig.reserve(sub_idx[si].size());
            for (int k : sub_idx[si])
              if (k >= 0 && static_cast<size_t>(k) < src->size())
                orig.push_back((*src)[k]);
          }
          EmitObject(subs[si], orig, idx, labels, objs);
        }
        continue;
      }
    }

    /* 3b. 常规路径：跨度没超限，整簇就是一个目标。 */
    std::vector<int> orig;
    if (src) orig = *src;
    EmitObject(cluster, orig, idx, labels, objs);
  }

  /* 3c. 墙片单独发，强制 type=5。放在主循环之后，日志与调试输出里对照方便。
         墙片**不进** 3a 的切分器 —— 一整面墙被切成几段没有意义，而且判据已经
         用比 3a 更强的信息（薄度 / z 跨度 / 连通片）判过它是墙了。
         piece 里存的就是 data 的原下标，可以直接当 src_idx 用。 */
  for (const auto& piece : wall_pieces)
  {
    std::vector<points> wp;
    wp.reserve(piece.size());
    for (int i : piece)
      if (i >= 0 && static_cast<size_t>(i) < data.size())
        wp.push_back(data[i]);
    EmitObject(wp, piece, idx, labels, objs, /*force_wall=*/true);
  }

  INFO("cluster_objs: size={} (wall_pieces={})", objs.objs.size(), wall_pieces.size());
  PublishLabels(data_in, labels);
  return objs;
}


/* ───────── 把一个簇落成一个 Object ───────── */
void PointsCluster::EmitObject(const std::vector<points>& cluster,
                               const std::vector<int>& src_idx,
                               int& idx,
                               std::vector<int32_t>& labels,
                               lidar_msgs::msg::Objects& objs,
                               bool force_wall)
{
  if (cluster.empty()) return;

  /* 最小点数门槛随距离缩放，所以先算这簇离传感器多远 */
  double cx = 0.0, cy = 0.0;
  for (const auto& p : cluster) { cx += p.x; cy += p.y; }
  const double range = std::sqrt(cx * cx + cy * cy) /
                       static_cast<double>(cluster.size());
  if (static_cast<int>(cluster.size()) < MinPointsAt(range)) return;

  std::vector<double> rect = min_rotate_rect_->MinRotateRect(cluster);
  /* 过滤过低目标。rect 下标：0=x 1=y 2=length 3=width 4=height
     5=heading 6=z中心，见 min_rotate_rect.cpp 的注释 */
  if (rect.size() < 7 || rect[4] < min_obj_height_) return;

  lidar_msgs::msg::Object obj;
  obj.rel_x  = rect[0];
  obj.rel_y  = rect[1];
  /* rel_z 是目标在传感器坐标系下的 z 中心，不是 height。
     原来这里写的是 rect[4]（就是 height），于是 rel_z 与 height 恒等，
     下游拿 rel_z 判断目标高度时看到的其实是"高度"本身，位置信息全丢。 */
  obj.rel_z  = rect[6];
  obj.length = rect[2];
  obj.width  = rect[3];
  obj.height = rect[4];
  obj.rel_heading = rect[5] * arc2degree;
  obj.idx = idx++;

  /* 把这个目标的点标上同一个簇号。标签值直接用 obj.idx（从 0 起），
     于是 label 与 /perception/lidar_objs 里的 objs[i].idx 一一对应，
     评估时不用再去猜哪个标签对应哪个框。 */
  for (int pi : src_idx)
    if (pi >= 0 && static_cast<size_t>(pi) < labels.size())
      labels[pi] = obj.idx;

  /* 类型判定。5=墙/建筑，1=卡车，2=车，3=人，4=骑行者，0=未知。
     原来的四分支没有"墙"这一类、也没有尺寸上界，只要求 l>0.4 && w>0.4
     && h>0.8，于是一面十几米宽的墙落进"车"。所以墙这一支必须**排在最前**。

     判据用 max(length, width) 而不是 length：length/width 会因为最小面积
     搜索的平局处理逐帧互换（实测同一物体在 1.88×4.28 和 4.28×1.88 之间
     反复翻），单看 length 会让一个竖着的墙某几帧漏成"车"，max 对互换不变。
     实测三个墙簇的 max 是 13.71 / 11.78 / 10.55 m，车最大 2.23 m，6 m 分得干净。
     巨簇切分之后，墙片的 max 会降到 4 m 上下 —— 这时靠的是下一支的
     "薄而高"判据，不是尺寸。 */
  const double thin = std::min(obj.length, obj.width);
  /* force_wall：摘墙模块已经把这一片判成墙了，而且用的是比这里更强的信息
     （连通片点数、z 跨度、XY 薄度 / 跨度）。下面几支只看尺寸，任何大东西都能
     满足「max > 6.0」；一片被 3a 切过、或者本来就窄的墙落到这儿会被判成车。
     所以墙片直接给定 type=5，不再走判定。 */
  if (force_wall)
    obj.type = 5;
  else if (std::max(obj.length, obj.width) > wall_size_m_ && obj.height > 0.8)
    obj.type = 5;
  else if (obj.height > 1.2 && thin < 0.35 && std::max(obj.length, obj.width) > 1.2)
    obj.type = 5;                        // 巨簇切剩下的墙片：薄、高、长
  else if (obj.length < 0.4 && obj.width < 0.4 && obj.height > 0.8)
    obj.type = 3;
  else if (((obj.length < 3 && obj.length > 0.4 && obj.width < 0.4) ||
            (obj.length < 0.4 && obj.width > 0.4 && obj.width < 3)) &&
           obj.height > 0.8)
    obj.type = 4;
  else if (obj.length > 0.4 && obj.width > 0.4 && obj.height > 0.8)
    obj.type = 2;
  else
    obj.type = 0;

  objs.objs.emplace_back(obj);
}

/* ───────── 发逐点聚类标签（评估用） ───────── */
void PointsCluster::PublishLabels(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                  const std::vector<int32_t>& labels)
{
  if (!pub_labels_) return;

  const size_t n = cloud->points.size();
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp    = node_->get_clock()->now();
  msg.header.frame_id = "rflink";      // 与驱动 launch 里的 frame_id 一致
  msg.height       = 1;
  msg.width        = n;
  msg.is_bigendian = false;
  msg.point_step   = 16;               // 3 × float32 + 1 × int32
  msg.row_step     = msg.point_step * n;
  msg.is_dense     = false;

  static const char*    names[4] = {"x", "y", "z", "label"};
  static const uint8_t  types[4] = {sensor_msgs::msg::PointField::FLOAT32,
                                    sensor_msgs::msg::PointField::FLOAT32,
                                    sensor_msgs::msg::PointField::FLOAT32,
                                    sensor_msgs::msg::PointField::INT32};
  msg.fields.resize(4);
  for (int f = 0; f < 4; ++f)
  {
    msg.fields[f].name     = names[f];
    msg.fields[f].offset   = f * 4;
    msg.fields[f].datatype = types[f];
    msg.fields[f].count    = 1;
  }

  msg.data.resize(msg.row_step);
  for (size_t i = 0; i < n; ++i)
  {
    uint8_t* p = msg.data.data() + i * 16;
    const float xyz[3] = {cloud->points[i].x,
                          cloud->points[i].y,
                          cloud->points[i].z};
    std::memcpy(p, xyz, sizeof(xyz));
    const int32_t lb = (i < labels.size()) ? labels[i] : -1;
    std::memcpy(p + 12, &lb, sizeof(lb));
  }
  pub_labels_->publish(msg);
}

/* ───────── PCL 点云 → 自定义 points 数组 ───────── */
std::vector<points>
PointsCluster::CloudToPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  std::vector<points> data_out;
  if (cloud->points.empty()) return data_out;

  data_out.reserve(cloud->points.size());
  for (const auto& p : cloud->points)
    data_out.push_back({p.x, p.y, p.z});

  return data_out;
}

} // namespace perception
