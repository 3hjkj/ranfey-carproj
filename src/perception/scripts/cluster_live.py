#!/usr/bin/env python3
"""实时聚类成像：把 perception 的聚类结果连续推到浏览器里看。

和 cluster_view3d.py（单帧静态页）的关系
----------------------------------------
页面、渲染器、相机交互全部复用 cluster_view3d.py 里的 HTML_TEMPLATE ——
只有"数据怎么来、颜色怎么定"两处不同。这里用**字符串替换**改造那份模板，
每处替换都断言锚点存在（见 patch()），将来改了 cluster_view3d.py 会当场报错，
而不是悄悄生成一个坏页面。

为什么用 SSE 而不是 WebSocket
-----------------------------
这台机器上 aiohttp / websockets / flask 一个都没装，而数据是**单向**的
（服务端推、浏览器只负责画），用标准库 http.server 发 text/event-stream
就够了，不用为了一个只读的可视化去装依赖。

颜色为什么不能按点数排名发
--------------------------
静态页是单帧，按点数排名上色没问题。实时页要是也这么干，帧间目标一多一少、
大小次序一变，颜色就整体重排，满屏闪 —— 根本没法看。所以这里做**跨帧跟踪**：
按质心最近邻把这一帧的簇匹配到上一帧的目标上，颜色跟着目标走（见 Tracker）。
这也是 dataviz 那条"颜色跟着实体走，不跟着排名走"。

跑法
----
    # 1) 另起一份 perception 接实时雷达（别动现有那份回放的，见下面）
    # 2) python3 cluster_live.py --labels /live/lidar_cluster_labels \
    #        --objs /live/lidar_objs --bg /rfans_points
    # 3) 浏览器开 http://127.0.0.1:8765/

为什么 perception 要跑第二份：当前那份的 lidar_topic_name0 指的是
/rfans_points_replay，是给 cluster_eval 做真值对比用的，不能动。第二份
接实时话题、把输出重映射到 /live/* 命名空间，两边互不干扰。命令见 --help。
"""

import argparse
import base64
import json
import os
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from lidar_msgs.msg import Objects

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cluster_eval import LABEL_DTYPE          # noqa: E402
from cluster_view3d import (                  # noqa: E402
    HTML_TEMPLATE, RAW_DTYPE, DARK_SERIES, OTHER_DARK, TYPE_NAME, b64_i16)


# ───────────────────────── 跨帧跟踪 ─────────────────────────

class Tracker:
    """按质心最近邻把每帧的簇接成跨帧的"目标"，让颜色跟着目标而不是排名。

    没有它的话，实时页面每帧都在重新按点数排名发色，目标一多一少颜色就整体
    重排，看起来满屏闪。有了它，同一面墙从头到尾是同一个颜色、同一个编号。

    槽位用满 8 个之后新目标一律发灰（dataviz：分类色板不循环使用，第 9 个
    以后折叠成 Other）。目标消失超过 ttl 秒就释放槽位。
    """

    def __init__(self, match_dist=2.0, ttl=1.5, max_slots=len(DARK_SERIES)):
        self.match_dist = match_dist
        self.ttl = ttl
        self.max_slots = max_slots
        self.tracks = {}          # tid -> {'x','y','t','color'}
        self.slot = {}            # color -> tid（占用的槽位）
        self.next_id = 0

    def _free_color(self):
        for c in DARK_SERIES:
            if c not in self.slot:
                return c
        return OTHER_DARK

    def update(self, clusters, now):
        """clusters: [(本帧簇号, 质心 np.array)]，按点数降序。返回 {簇号: 目标号}。"""
        # 过期目标先释放槽位
        for tid in [t for t, v in self.tracks.items() if now - v['t'] > self.ttl]:
            self.slot.pop(self.tracks[tid]['color'], None)
            del self.tracks[tid]

        out, claimed = {}, set()
        for cid, cen in clusters:
            best, bd = None, self.match_dist
            for tid, v in self.tracks.items():
                if tid in claimed:
                    continue
                d = float(np.hypot(v['x'] - cen[0], v['y'] - cen[1]))
                if d < bd:
                    best, bd = tid, d
            if best is None:                       # 新目标
                best = self.next_id
                self.next_id += 1
                col = self._free_color()
                self.slot[col] = best
                self.tracks[best] = {'x': cen[0], 'y': cen[1], 't': now, 'color': col}
            else:
                self.tracks[best].update(x=cen[0], y=cen[1], t=now)
            claimed.add(best)
            out[cid] = best
        return out


# ─────────────────── 背景累积（治密度闪烁） ───────────────────

class BgAccum:
    """按「激光号 + 方位角」把最近 N 圈的背景点叠起来，专治背景一闪一闪。

    为什么需要：实测相邻两圈**几何**是对得上的 —— 同一个 (方位角, 激光号) 的回波
    下一圈还有 97.5%，两圈点对点最近距中位 0.0086 m。晃的不是几何，是**密度**：
    每圈点数在 21334~53304 之间（额定 45056 的 47%~118%），圈间变化中位 7.3%、
    最大 84.5%。丢的又都是弱回波（丢了的中位 intent 49 / range 3.55 m，还在的
    82 / 4.89 m），所以不是成片死区，是全画面均匀地"少一层"。画面看着就像在抖。

    为什么按射线号而不是坐标体素：晃的是"这一圈这根射线有没有回波"，不是它打在哪。
    按射线存最后一次命中，等于把 N 圈的命中取并集 —— 密度填平，而**上限只是
    激光数 × 方位格数**（32 × 1500），不会越堆越多。坐标体素化反而会在物体边缘
    糊出一层壳，键的稳定性也依赖体素对齐。

    代价（必须知道）：**动的东西会拖尾** N 圈。这是 rviz decay time 的同一个取舍，
    不是 bug —— 静态的墙、地面正是它要治的对象。要让动目标干净，把 N 调小。

    格宽怎么定的（实测，`/tmp/accum_sweep.py`）——**结论和直觉相反，格宽要偏粗**：
    方位角并不是固定量化网格，跨圈是**带抖动**的（同一根射线两圈之间偏 中位
    0.0480°、90 分位 0.1120°），而真实相邻间距是 1 分位 0.0730°、5 分位 0.1150°、
    中位 0.2550° —— **两个分布重叠**，没有哪个格宽能干净地分开它们。用真数据扫：

        格宽°   槽位数   稳态点数   圈间变化中位   圈间变化最大
        0.03    384000    199790      3.94%        9.61%
        0.05    230400    157216      3.82%        9.82%
        0.08    144000    110423      1.63%        5.63%
        0.12     96000     76518      0.07%        1.75%
        0.24     48000     38722      0.02%        0.09%   ← 默认
        0.30     38400     31080      0.04%        0.09%
        （对照：不累积时单圈中位 36178 点，圈间变化中位 12.8%、最大 70.7%）

    格宽**越细越糟**：0.03° 时同一根射线每圈都落进新的格，并集一路涨到 20 万点还
    稳不住（末圈/中位 1.11，即还在长），抖动反而更大。0.24° 因为把抖动（90 分位
    0.112°）整个吃进一个格，并集很快饱和：38722 点、圈间变化 0.02%、不再增长，
    而且这个点数和今天单圈全量（中位 36178）同量级，**SSE 体积不变**。
    再往粗到 0.30° 就越过标称步进 0.2550° 了，稳态 31080 点已经**低于**单圈中位，
    画面比现在更稀 —— 那是反向的失败。

    代价是会把约 17% 的相邻射线对并进同一格。被并的那两点只差 ~0.12°，在 10 m
    处相当于 2 cm —— 对一块给人看的背景没有任何可见差别，换来的是密度稳住。
    """

    def __init__(self, n_laser, bin_deg, keep):
        self.n_laser, self.bin_deg, self.keep = n_laser, bin_deg, keep
        self.nbin = max(1, int(round(360.0 / bin_deg)))
        self.n = n_laser * self.nbin
        self.xyz = np.zeros((self.n, 3), np.float32)
        self.last = np.full(self.n, -1, dtype=np.int64)
        self.k = 0
        self.n_bad_laser = 0        # laserid 越界的点数，供 /stats 自省

    def add(self, s):
        """把一圈并进来。k 是圈号（每次调用 +1），'多久算过期'按圈数算而不是按秒，
        因为要的语义是"最近 N 圈" —— 用秒的话雷达一掉速累积的圈数就变了。"""
        self.k += 1
        if len(s) == 0:
            return
        lid = s['laserid'].astype(np.int64)
        ok = (lid >= 0) & (lid < self.n_laser)
        if not ok.all():
            self.n_bad_laser += int((~ok).sum())
            lid, s = lid[ok], s[ok]
            if len(lid) == 0:
                return
        b = np.rint(s['h_angle'] / self.bin_deg).astype(np.int64) % self.nbin
        key = lid * self.nbin + b
        # 同一格在一圈里撞了多次就后写覆盖：同一根射线本来只有一次回波，真撞上是
        # 设备量化抖动，取哪个都一样。
        self.xyz[key] = np.stack([s['x'], s['y'], s['z']], 1).astype(np.float32)
        self.last[key] = self.k

    def points(self):
        """当前该画的背景（最近 keep 圈里出现过的射线）。

        必须同时卡 `last >= 0`：`last` 的初值是 -1，光用 `last > k - keep` 的话，
        在攒满 keep 圈之前（k < keep）阈值是负的，**从没被写过的空槽也会被判成
        "最近见过"** —— 于是把 xyz 里那片全零当成背景发出去，画面正中多出一坨
        原点。实测就是这样：前 6 帧背景点数恒等于上界 48000（32 线 × 1500 格），
        而一圈只有 3.6 万个点，物理上填不满，一眼就能看出是空槽混进来了。
        """
        return self.xyz[(self.last >= 0) & (self.last > self.k - self.keep)] \
            .astype(np.float64)


# ───────────────────────── ROS 侧 ─────────────────────────

class Live(Node):
    def __init__(self, args, state):
        super().__init__('cluster_live')
        self.args, self.state = args, state
        self.tracker = Tracker(args.track_dist, args.track_ttl)
        self.lock = threading.Lock()
        self.lab = self.objs = self.raw = None
        self._last_lab = None
        # 有没有"还没配上框"的新标签，见 _objs 里的说明
        self.lab_fresh = False
        # 背景降频发（见 _build）：缓存上次编好的 base64 和它的点数
        self._bg_cache = None
        self._bg_t = 0.0
        # 背景累积（见 BgAccum）。--bg-accum 0 就退回"只画最新一圈"的老行为
        self.acc = (BgAccum(args.bg_lasers, args.bg_accum_bin, args.bg_accum)
                    if args.bg_accum > 0 else None)
        if self.acc is not None:
            self.get_logger().info(
                '背景累积：最近 %d 圈，方位格宽 %.3f°（%d 格 × %d 线）'
                % (args.bg_accum, args.bg_accum_bin, self.acc.nbin, args.bg_lasers))

        # BEST_EFFORT：实时图要的是低延迟，别让可靠传输的重传把画面拖住
        q = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2, args.labels, self._lab, q)
        self.create_subscription(Objects, args.objs, self._objs, q)
        # 背景这条**必须**跟发布者一样用 RELIABLE。rfans_relay 是 RELIABLE，
        # 按 DDS 的规则 RELIABLE 发布者配 BEST_EFFORT 订阅者是"兼容"的，所以
        # 匹配得上、不报错 —— 但实测就是这么订阅**一帧都收不到**，而同一时刻
        # 按 RELIABLE 订 8 秒收 40 帧。更坑的是它时好时坏：同一份代码十分钟前
        # 还收得到 4.4 万点，重启一次就变成 0，页面背景整片消失，看起来像渲染
        # 坏了。别赌兼容性，跟源头保持一致。
        qb = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                        history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2, args.bg, self._raw, qb)
        self.t0, self.n_sent = time.time(), 0

    def _lab(self, m):
        with self.lock:
            self.lab = np.frombuffer(m.data, dtype=LABEL_DTYPE)
            self.lab_fresh = True

    def _objs(self, m):
        with self.lock:
            self.objs = m
            # 在这边建帧，不在标签回调里建。
            #
            # 为什么：perception 里标签是**先**发的（PointsCluster::Process 末尾
            # 的 PublishLabels，lidar_cluster.cpp:159），框是 Process 返回之后才由
            # RosBridge::Publish 发的（rosbridge.cpp:177）。所以标签(N) 到的时候，
            # self.objs 里躺的必然是**上一拍**的框 —— 拿它配标签(N)，框就会被画到
            # 别的簇上。实测 14 个框里 6 个的中心离自己标的簇 4~17 m，正好落在**别的
            # 簇**的质心上（差 0.13~0.58 m）。
            #
            # 框永远在标签之后到，所以在框回调里建帧，配上的就是同一拍的标签。
            # 用 lab_fresh 当闸门：没有新的标签就不建（重复的框不该重复推帧）。
            if not self.lab_fresh:
                return
            self.lab_fresh = False
            self._maybe_emit()

    def _raw(self, m):
        with self.lock:
            s = np.frombuffer(m.data, dtype=RAW_DTYPE)
            self.raw = s
            if self.acc is not None:
                # 累积必须在**这里**做，不能挪到 _build 里：_build 只在建帧时跑
                # （跟着标签走），而背景是降频刷新的 —— 放那边就会漏掉中间几圈，
                # 攒出来的是"每隔几圈抽一圈"的并集，不是真正的最近 N 圈。
                self.acc.add(s)

    def _maybe_emit(self):
        """发一帧（一次完整的聚类结果），带上框和背景。

        调用时已经持有 self.lock。
        """
        if self.lab is None or self.objs is None:
            return
        self._last_lab = self.lab                 # 建帧失败也算数，别反复重试同一帧
        try:
            payload = self._build()
        except Exception as e:                     # 单帧出错不该把整个流打断
            print(f'建帧失败，跳过：{type(e).__name__}: {e}')
            return
        blob = json.dumps(payload, separators=(',', ':'))
        self.state.push(blob)
        with self.state.cond:
            self.state.last_bg_bytes = len(payload.get('bg') or '')
            self.state.bg_hz = self.args.bg_hz
            self.state.bg_n = payload.get('bgN', 0)
            self.state.pt_n = payload.get('ptN', 0)
            self.state.bg_accum = self.args.bg_accum
            self.state.bg_acc_n = self.acc.k if self.acc is not None else -1
        self.n_sent += 1

    def _build(self):
        lab_arr = self.lab
        xyz = np.stack([lab_arr['x'], lab_arr['y'], lab_arr['z']],
                       1).astype(np.float64)
        lbl = lab_arr['label'].astype(np.int64)

        uniq, cnt = np.unique(lbl[lbl >= 0], return_counts=True)
        order = uniq[np.argsort(-cnt)]
        cen = {int(c): xyz[lbl == int(c)].mean(0) for c in uniq}

        tid_of = self.tracker.update([(int(c), cen[int(c)]) for c in order],
                                     time.time())

        # 标签换成目标号，渲染器就能直接按目标号取色（不用改一行 JS）
        new_lbl = np.full(len(lbl), -1, dtype=np.int64)
        for c in uniq:
            new_lbl[lbl == int(c)] = tid_of[int(c)]

        n_of = {tid_of[int(c)]: int(cnt[i]) for i, c in enumerate(uniq)}
        cmap = {tid_of[int(c)]: self.tracker.tracks[tid_of[int(c)]]['color']
                for c in uniq}

        boxes = []
        for o in self.objs.objs:
            tid = tid_of.get(int(o.idx))
            if tid is None:                        # 这一帧被过滤掉的框
                continue
            c = cen.get(int(o.idx))
            dx = dy = dz = None
            if c is not None:
                dx, dy, dz = (float(o.rel_x - c[0]), float(o.rel_y - c[1]),
                              float(o.rel_z - c[2]))
            boxes.append({
                'idx': tid, 'x': float(o.rel_x), 'y': float(o.rel_y),
                'z': float(o.rel_z), 'l': float(o.length), 'w': float(o.width),
                'h': float(o.height), 'hd': float(o.rel_heading),
                'type': int(o.type), 'score': float(o.score),
                'n': n_of.get(tid, 0), 'dx': dx, 'dy': dy, 'dz': dz,
                'cls': int(o.idx),
            })

        # 背景抽稀：等间隔取样是确定性的，帧间同样位置取到同样的点，不会闪
        # 背景默认**不抽稀**，整帧 4.4 万点都画 —— 抽稀过一版（默认 10000，即
        # 1:4），页面里只有 1.2 万点、是原始云的 29%，比 rviz 看着空一大截。
        # 但那不是数据的问题，是这个参数。
        #
        # 但全量背景每帧重发太贵：单帧 SSE 从 24 KB 涨到 372 KB，浏览器每帧还
        # 要重解码、重画 4.4 万个点。实测把这台机器上 perception 的标签发布从
        # 5.0 Hz 拖到 1.9 Hz —— 拖慢的正是它自己要显示的那条流水线。
        # 背景本来就基本不动，所以按 --bg-hz 降频刷新，簇照旧每帧发；
        # 没带背景的帧，浏览器沿用上一张（见 setFrame 的补丁）。
        now = time.time()
        with self.state.cond:
            forced, self.state.force_bg = self.state.force_bg, False
        refresh_bg = (forced or self.args.bg_hz <= 0
                      or self._bg_cache is None
                      or now - self._bg_t >= 1.0 / self.args.bg_hz)
        if refresh_bg:
            bg = np.empty((0, 3))
            if self.acc is not None:
                # 累积结果：最近 N 圈里出现过回波的射线，每根只留一个位置。
                # 点数上界 = 激光数 × 方位格数（32 × 1500），跟单圈同量级，
                # 所以累积不会把这一条再撑胖 —— 见 BgAccum 的说明。
                bg = self.acc.points()
            elif self.raw is not None and len(self.raw):
                s = self.raw
                bg = np.stack([s['x'], s['y'], s['z']], 1).astype(np.float64)
            if self.args.max_bg and len(bg) > self.args.max_bg:
                bg = bg[::max(1, len(bg) // self.args.max_bg)]
            self._bg_cache = (b64_i16(bg.reshape(-1)), int(len(bg)))
            self._bg_t = now
            # 相机只在第一帧定一次位（以整帧的包围盒为准），之后跟着用户走
            if not self.state.cam:
                if len(bg):
                    lo, hi = bg.min(0), bg.max(0)
                else:
                    lo, hi = xyz.min(0), xyz.max(0)
                mid = (lo + hi) / 2
                self.state.cam = {
                    'tx': float(mid[0]), 'ty': float(mid[1]), 'tz': float(mid[2]),
                    'dist': float(max(8.0, np.linalg.norm(hi - lo) * 1.3))}

        bg_b64, bg_n = self._bg_cache

        return {
            # 没刷新就发空串，浏览器沿用上一张（省下每次 350 KB 的 JSON 解析）
            'bg': bg_b64 if refresh_bg else '', 'bgN': bg_n,
            'pt': b64_i16(xyz.reshape(-1)), 'ptN': int(len(xyz)),
            'lab': base64.b64encode(new_lbl.astype('<i2').tobytes()).decode(),
            'cmap': {str(k): v for k, v in cmap.items()},
            'boxes': boxes, 'noise': int((lbl < 0).sum()),
            'mount': self.args.mount_height, 'cam': self.state.cam,
            'frames': self.n_sent,
            'secs': round(time.time() - self.t0, 1),
            'ntrack': len(self.tracker.tracks),
        }


# ───────────────────────── HTTP / SSE 侧 ─────────────────────────

class State:
    def __init__(self):
        self.cond = threading.Condition()
        self.frame = None
        self.seq = 0
        self.cam = None
        # 给 /stats 用的自省量
        self.t0 = time.time()
        self.clients = 0
        self.last_bytes = 0
        self.last_bg_bytes = 0
        self.bg_n = 0
        self.pt_n = 0
        # 新客户端一接入就置位，让下一帧一定带上背景。
        # 不带的话，新页面第一帧就拿不到背景，画面是空的。
        self.force_bg = False
        self.bg_hz = 0.0
        # 背景累积的自省量（--bg-accum 0 时为 0 / -1）
        self.bg_accum = 0
        self.bg_acc_n = -1

    def push(self, payload):
        with self.cond:
            self.frame = payload
            self.seq += 1
            self.last_bytes = len(payload)
            self.cond.notify_all()


SELFTEST = """<!DOCTYPE html><html lang="zh"><head><meta charset="utf-8">
<title>等待第一帧…</title></head><body style="font:14px monospace;background:#111;color:#eee">
<div id="o">连接中…</div>
<script>
let n=0;
const es=new EventSource('/stream');
es.onmessage=e=>{
  n++;
  const d=JSON.parse(e.data);
  document.title='SSE OK n='+n;
  document.getElementById('o').textContent=
    '收到 '+n+' 帧｜本帧 '+e.data.length+' 字节｜聚类点 '+d.ptN+
    '｜目标 '+Object.keys(d.cmap).length+'｜检测框 '+d.boxes.length+
    '｜服务端已推 '+d.frames+' 帧';
  if(n>=3) es.close();        // 收够就断，让 --dump-dom 能返回
};
es.onerror=()=>{document.title='SSE ERR';};
</script></body></html>"""


def make_handler(state, page_bytes, snapshot_bytes, page_ver=''):
    class H(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path.startswith('/stream'):
                self._stream()
            elif self.path.startswith('/selftest'):
                self._selftest()
            elif self.path.startswith('/snapshot'):
                body = snapshot_bytes
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path.startswith('/stats'):
                # 让服务自己报数。用 ros2 topic hz 去量是不准的 —— 那个命令
                # 自己就是一个新订阅者，量 5 Hz 的话题时同时给它加了负载。
                now = time.time()
                with state.cond:
                    body = json.dumps({
                        'frames': state.seq,
                        'secs': round(now - state.t0, 1),
                        'fps': round(state.seq / max(1e-6, now - state.t0), 2),
                        'clients': state.clients,
                        'last_frame_bytes': state.last_bytes,
                        'last_bg_bytes': state.last_bg_bytes,
                        'bgN': state.bg_n, 'ptN': state.pt_n,
                        'bg_hz': state.bg_hz,
                        'bg_accum': state.bg_accum,
                        'bg_acc_n': state.bg_acc_n,
                        'page_ver': page_ver,
                    }).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path in ('/', '/index.html'):
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                # 不许缓存：页面里带着数据流和版本号，浏览器拿旧的就没意义了。
                # 更不能让"按了 F5 还是老样子"这种事发生 —— 那会让人以为修复
                # 没生效，而去查错的方向。
                self.send_header('Cache-Control', 'no-store, must-revalidate')
                self.send_header('Content-Length', str(len(page_bytes)))
                self.end_headers()
                self.wfile.write(page_bytes)
            else:
                self.send_error(404)

        def _selftest(self):
            """自检页：收满 3 帧就主动断开，把结果写进 DOM。

            专门为了能无头验证 —— 主页面挂着一条不关的 SSE，Chrome 的
            --virtual-time-budget 会一直等这条未完成的请求，--dump-dom 永远
            不返回（实测超时 60 s、DOM 拿到 0 字节）。这里收够就 close()，
            挂起的请求结束，虚拟时间才走得下去。
            """
            body = SELFTEST.encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _stream(self):
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
            self.send_header('Cache-Control', 'no-cache')
            # 不发 Content-Length，靠 chunked 之外的"连接挂着"来流式推
            self.send_header('X-Accel-Buffering', 'no')
            self.end_headers()
            last = -1
            with state.cond:
                state.clients += 1
                state.force_bg = True   # 新页面要先拿到一张背景
            try:
                while True:
                    with state.cond:
                        while state.seq == last:
                            state.cond.wait(timeout=2.0)
                        last, data = state.seq, state.frame
                    if data is None:
                        continue
                    self.wfile.write(b'data: ' + data.encode() + b'\n\n')
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass          # 浏览器关页面了，正常
            finally:
                with state.cond:
                    state.clients -= 1

    return H


# ───────────────────────── 改造静态模板 ─────────────────────────

def patch(html, old, new, n=1):
    got = html.count(old)
    if got != n:
        raise SystemExit(
            f'HTML_TEMPLATE 的锚点对不上：期望 {n} 处，实际 {got} 处\n'
            f'  锚点: {old[:70]!r}\n'
            f'cluster_view3d.py 改了的话，这里的替换要跟着改。')
    return html.replace(old, new)


def build_page(snapshot_after=0):
    """snapshot_after>0 时生成"快照页"：收够这么多帧就自己断开，便于截图。"""
    h = HTML_TEMPLATE
    h = patch(h, 'perception 聚类（单帧）', 'perception 聚类（实时）')
    h = patch(h, 'const P = __PAYLOAD__;', 'let P = __PAYLOAD__;')
    # 初始化成**空的合法数组**，不是 undefined。背景是降频发的，新开的页面
    # 第一帧很可能恰好是"没带背景"的那种；这时候 bgv 是 undefined，
    # drawPoints 一取长度就抛异常 —— 而它在 render() 里排在画聚类点和框之前，
    # 一抛就整帧全空，只剩地面网格。看到的像是"聚类没做出来"，其实只是背景
    # 没到。后端那边也会对新连上的客户端补发一帧背景（见 /stream），这里是兜底。
    h = patch(h, 'const bgv=dec16(P.bg), ptv=dec16(P.pt), lbv=decI16(P.lab);',
              "let bgv=dec16(''),ptv=dec16(''),lbv=decI16('');")
    h = patch(h,
              'const CMAP={}; for(const k in P.cmap) CMAP[+k]=hex2rgb(P.cmap[k]);',
              '''let CMAP={};
function setFrame(Q){        // 每来一帧就换掉数据，相机和图例状态都保留
  P=Q;
  // 背景是降频发的（后端 --bg-hz）。这一帧没带 bg 就沿用上一张，
  // 不能解码空串 —— 解出来是 0 个点，画面会整片闪没。
  if(Q.bg && Q.bg.length) bgv=dec16(Q.bg);
  ptv=dec16(P.pt); lbv=decI16(P.lab);
  CMAP={}; for(const k in P.cmap) CMAP[+k]=hex2rgb(P.cmap[k]);
}
setFrame(P);''')
    # 侧栏文案：这里显示的是跨帧跟踪的目标，不是单帧的簇
    h = patch(h, '簇（按点数）· 点一下只看它', '目标（跨帧跟踪）· 点一下只看它')
    # 把一次性写死的副标题改成每帧刷新
    h = patch(h, """document.getElementById('sub').innerHTML=
  `${P.ptN} 个聚类点　${Object.keys(P.cmap).length} 个簇　`+
  `${P.boxes.length} 个检测框<br>地面参考网格按架高 ${P.mount.toFixed(2)} m 画在 z=-${P.mount.toFixed(2)}`;""",
              """function updateSub(){
  const fps=(P.secs>0?(P.frames/P.secs):0).toFixed(1);
  document.getElementById('sub').innerHTML=
    `${P.ptN} 个聚类点　${Object.keys(P.cmap).length} 个目标　`+
    `${P.boxes.length} 个检测框<br>`+
    `实时 ${fps} 帧/秒　已收 ${P.frames} 帧　在跟 ${P.ntrack} 个目标<br>`+
    `地面网格按架高 ${P.mount.toFixed(2)} m 画在 z=-${P.mount.toFixed(2)}`;
}
updateSub();""")
    # 追加 SSE 客户端
    h = patch(h, 'buildLegend(); loop();', """buildLegend(); loop();

// ---- 实时流 ----
let legendKey=null, camInit=false;
const PAGE_VER='__PAGE_VER__';
const es=new EventSource('/stream');
es.onmessage=ev=>{
  const Q=JSON.parse(ev.data);
  setFrame(Q);
  // 第一帧才把相机对准场景：出生帧里没有任何点，框不出边界
  if(!camInit){camInit=true;Object.assign(HOME,Q.cam);Object.assign(cam,Q.cam);}
  // 图例只在"目标集合变了"时重建 —— 每帧重建会把 solo 选中态和滚动位置清掉
  const key=Object.keys(Q.cmap).join(',');
  if(key!==legendKey){legendKey=key; buildLegend();} else refreshLegend();
  updateSub();
  need();
};
es.onerror=()=>{document.getElementById('stat').textContent='⚠ 与后端断开，正在重连…';};

// 后端改的是页面里的 JS，浏览器这边是加载时定死的。改了代码之后，
// 还开着的旧标签页会一直跑旧逻辑（比如旧的 setFrame 不认识"本帧没带
// 背景"，会画成一张没有背景的空图），看起来就像修复没生效。
// 这里对一下版本号，变了就自己刷新。
setInterval(()=>{
  fetch('/stats').then(r=>r.json()).then(d=>{
    if(d.page_ver && d.page_ver!==PAGE_VER) location.reload();
  }).catch(()=>{});
}, 5000);""")
    if snapshot_after:
        # 快照模式：真页面、真渲染，只是收够 N 帧就把流断掉。
        # 不这么干的话页面永远挂着一条未完成的 SSE，Chrome 的
        # --virtual-time-budget 会一直等它，--screenshot / --dump-dom 都不返回
        # （见 memory: headless-chrome-sse-virtual-time-hang）。
        h = patch(h, 'es.onerror=()=>{',
                  f'let _n=0; const _orig=es.onmessage; es.onmessage=ev=>{{'
                  f'_orig(ev); if(++_n>={snapshot_after}) es.close(); }};\n'
                  f'es.onerror=()=>{{')
    return h


# 出生帧：还没收到任何数据时先塞一个合法的空帧，免得脚本一上来就 null 崩掉。
# 相机放在一个中性的远处，第一帧真数据到了会立刻重定位（见上面的 camInit）。
EMPTY_FRAME = {
    'bg': '', 'bgN': 0, 'pt': '', 'ptN': 0, 'lab': '', 'cmap': {},
    'boxes': [], 'noise': 0, 'mount': 1.76,
    'cam': {'tx': 0.0, 'ty': 0.0, 'tz': 0.0, 'dist': 25.0},
    'frames': 0, 'secs': 0, 'ntrack': 0,
}


# ───────────────────────── main ─────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description='实时聚类成像（SSE 推到浏览器）',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""先用另一份 perception 接实时雷达，把输出改到 /live 命名空间：

  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 run perception perception --ros-args \\
    -p lidar_topic_name0:=/rfans_points \\
    -p cell_config:=install/perception/share/perception/config_json/cell.json \\
    -p obj_config:=install/perception/share/perception/config_json/objs.json \\
    -p preprocess_config:=install/perception/share/perception/config_json/preprocess.json \\
    -p log_path:=/tmp/perception_live.log \\
    -r /perception/lidar_objs:=/live/lidar_objs \\
    -r /perception/lidar_cluster_labels:=/live/lidar_cluster_labels \\
    -r /perception/lidar_cells:=/live/lidar_cells \\
    -r /perception/no_ground_points:=/live/no_ground_points

（现有那份 perception 的 lidar_topic_name0 指的是 /rfans_points_replay，
  是给 cluster_eval 做真值对比用的，别去动它。）
""")
    ap.add_argument('--labels', default='/live/lidar_cluster_labels')
    ap.add_argument('--objs', default='/live/lidar_objs')
    ap.add_argument('--bg', default='/rfans_points')
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--mount-height', type=float, default=1.76)
    ap.add_argument('--max-bg', type=int, default=0,
                    help='每帧背景点上限，0 = 全画（默认）。雷达一帧 4.4 万点，'
                         '只有全画才和 rviz 看到的一样全；嫌卡再往小调')
    ap.add_argument('--bg-hz', type=float, default=2.0,
                    help='背景点云的刷新频率，0 = 每帧都发（默认 2.0）。'
                         '背景基本不动，全量 4.4 万点每帧重发会把整条流水线拖慢；'
                         '降频之后浏览器在两次刷新之间沿用上一张，画面看不出差别')
    ap.add_argument('--bg-accum', type=int, default=8,
                    help='背景累积最近 N 圈（默认 8，0 = 关闭）。实测每圈点数在额定的 '
                         '55%%~118%% 之间跳（圈间变化中位 12.8%%、最大 70.7%%），背景因此'
                         '一闪一闪；按「激光号+方位角」把 N 圈的命中取并集，实测圈间变化'
                         '降到 0.02%%（中位）。上限是 32 线 × 1500 格，不会越堆越多。'
                         '代价：动的东西拖尾 N 圈，要干净就把它调小')
    ap.add_argument('--bg-accum-bin', type=float, default=0.24,
                    help='方位角量化格宽（度，默认 0.24）。**要偏粗，不是偏细**：方位角'
                         '跨圈带抖动（中位 0.048°、90 分位 0.112°），格宽太细会让同一根'
                         '射线每圈落进新的格、并集一路涨（0.03° 时涨到 20 万点还稳不住）；'
                         '太粗又越过标称步进 0.2550° 把画面搞稀（0.30° 时比不累积还少）。'
                         '实测 0.24° 是稳态 38722 点、圈间变化 0.02%')
    ap.add_argument('--bg-lasers', type=int, default=32,
                    help='激光线数，用来给 laserid 定界（默认 32）')
    ap.add_argument('--track-dist', type=float, default=2.0,
                    help='跨帧匹配的质心距离阈值（米）')
    ap.add_argument('--track-ttl', type=float, default=1.5,
                    help='目标消失多久后释放颜色槽位（秒）')
    ap.add_argument('--dump-page', metavar='FILE',
                    help='只生成页面不连 ROS，用来查渲染问题')
    args = ap.parse_args()

    seed = dict(EMPTY_FRAME, mount=args.mount_height)
    import hashlib
    page_ver = hashlib.sha1(build_page(1).encode()).hexdigest()[:8]
    mk = lambda n: (build_page(n)                                  # noqa: E731
                    .replace('__PAYLOAD__', json.dumps(seed))
                    .replace('__PAGE_VER__', page_ver))
    page = mk(0)
    # 快照页收满 12 帧就断流：够走完至少一次背景刷新（--bg-hz 2.0、雷达 5 Hz
    # 时每 2~3 帧一张背景），截出来的图背景是齐的，不是空的。
    snapshot = mk(12)
    if args.dump_page:
        with open(args.dump_page, 'w') as f:
            f.write(page)
        print(f'页面写到 {args.dump_page}')
        return 0

    state = State()
    rclpy.init()

    # rclpy.init() 会把 SIGINT/SIGTERM 换成它自己的处理函数，而那个函数内部要调
    # rclpy.shutdown() —— 这台机器上它会挂死。后果是 Ctrl-C 既不抛 KeyboardInterrupt
    # （下面的主循环永不退出），进程也再收不了任何信号：主线程卡在信号处理函数里
    # 等 DDS 关停，只有 kill -9 能收，而 kill -9 会留僵尸读者拖死发布者。
    # 所以在 init() 之后把两个信号抢回来，只置一个事件位，不碰 rclpy。
    stop = threading.Event()
    for _s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(_s, lambda *_: stop.set())

    node = Live(args, state)

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    srv = ThreadingHTTPServer((args.host, args.port),
                              make_handler(state, page.encode(),
                                           snapshot.encode(), page_ver))
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()

    url = f'http://{args.host}:{args.port}/'
    print(f'实时聚类成像: {url}')
    print(f'  标签 {args.labels}\n  检测框 {args.objs}\n  背景 {args.bg}')
    print('浏览器打开上面的地址；Ctrl-C 退出。')
    try:
        while not stop.wait(2.0):
            print(f'  已推 {node.n_sent} 帧，'
                  f'{node.n_sent / max(1e-9, time.time() - node.t0):.1f} 帧/秒，'
                  f'在跟 {len(node.tracker.tracks)} 个目标')
    finally:
        print('\n退出')
        # 关停本身也可能挂在同一个 rclpy.shutdown 上。挂住就由看门狗强退 ——
        # 宁可非正常退出，也不要再变回那个杀不掉的僵尸。
        threading.Timer(6.0, lambda: os._exit(0)).start()
        srv.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
