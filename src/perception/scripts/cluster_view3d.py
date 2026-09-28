#!/usr/bin/env python3
"""把 perception 的聚类结果导出成一个**能转着看的 3D 页面**。

和 cluster_view.py 的区别
-------------------------
cluster_view.py 是"perception vs 真值"的静态对照图，回答"错在哪"。
这个脚本回答的是更直接的问题：**"你聚出来的这些簇，到底是不是东西？"**
所以它不画真值，就画 perception 自己的输出：

    背景    一帧原始点云（浅灰，只做参照）
    前景    聚类吃到的点，按簇上色（点太少的簇统一灰）
    框      每个簇拟合的最小外接矩形（线框）+ 类型标签
    地面    按架高在 z = -1.76 处画一层参考网格

页面是自包含的（点数据以 base64 int16 毫米编码嵌在 HTML 里，无外部依赖、
不用起服务器），鼠标拖拽转视角、滚轮缩放、右键平移，右边图例点一下可以
只看某一个簇。

为什么要画检测框
----------------
`/perception/lidar_objs` 里的 rel_x/rel_y 直接来自 min_rotate_rect 的
GetFinalShape，而那个函数的 (x, y) 是

    x = (p0.x + p1.x) / 2      // p0=(xmin,ymin) p1=(xmax,ymin)
    y = (p0.y + p2.y) / 2      // p2=(xmin,ymax)

也就是**矩形两条相邻边的中点**，不是矩形中心。所以框会整体偏出去半个
身位。这个脚本会把"框中心"和"簇质心"的差值一起报出来，画面上也一眼能
看出来 —— 这正是要转着看才能确认的东西。

用法
----
    source /opt/ros/humble/setup.bash
    source ~/carProj/install/setup.bash
    ./cluster_view3d.py --out /tmp/cluster3d.html
"""

import argparse
import base64
import json
import os
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from lidar_msgs.msg import Objects

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cluster_eval import LABEL_DTYPE, LABEL_TOPIC  # noqa: E402

RAW_TOPIC = '/rfans_points_replay'
OBJ_TOPIC = '/perception/lidar_objs'

RAW_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'),
                      ('v_angle', '<f4'), ('h_angle', '<f4'), ('range', '<f4'),
                      ('timestamp', '<f8'), ('laserid', '<i4')])

# dataviz/palette.md 分类色板的**暗色列**（画布是深色的，要用暗色档）
DARK_SERIES = ['#3987e5', '#d95926', '#199e70', '#c98500',
               '#d55181', '#008300', '#9085e9', '#e66767']
OTHER_DARK = '#7c7a74'

TYPE_NAME = {0: '未知', 1: '卡车', 2: '车', 3: '人', 4: '骑行者', 5: '墙'}


class Capture(Node):
    """按**到达时刻**配对标签帧和检测框。

    obj.idx 是**每帧重新编的**簇序号（lidar_cluster.cpp 里 idx++），不是稳定
    ID。所以拿第 N 帧的标签去对第 N±1 帧的框，序号一错位，算出来的"框中心
    偏离簇质心"就会是十几米的鬼数 —— 实测过，最大 16.9 m，纯属张冠李戴。
    两个话题是同一帧里前后脚发的，按接收时间取最近的一对就对了。
    """

    def __init__(self, keep=40):
        super().__init__('cluster_view3d')
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST)
        self.labs, self.raws, self.objs = [], [], []
        self.keep = keep
        self.create_subscription(PointCloud2, LABEL_TOPIC, self._lab, q)
        self.create_subscription(PointCloud2, RAW_TOPIC, self._raw, q)
        self.create_subscription(Objects, OBJ_TOPIC, self._obj, q)

    @staticmethod
    def _push(buf, item, keep):
        buf.append(item)
        if len(buf) > keep:
            buf.pop(0)

    def _lab(self, m):
        self._push(self.labs, (time.time(),
                               np.frombuffer(m.data, dtype=LABEL_DTYPE).copy()),
                   self.keep)

    def _raw(self, m):
        self._push(self.raws, (time.time(),
                               np.frombuffer(m.data, dtype=RAW_DTYPE).copy()),
                   self.keep)

    def _obj(self, m):
        self._push(self.objs, (time.time(), m), self.keep)

    def done(self):
        return len(self.labs) >= 3 and self.objs and self.raws

    def closest(self):
        """标签帧和检测框里接收时刻最接近的一对，外加离标签最近的一帧原始云。"""
        if not (self.labs and self.objs):
            return None, None, None
        t_l, lab = self.labs[-1]
        t_o, obj = min(self.objs, key=lambda kv: abs(kv[0] - t_l))
        t_r, raw = min(self.raws, key=lambda kv: abs(kv[0] - t_l))
        if abs(t_o - t_l) > 0.5:
            self.get_logger().warn(f'标签与检测框差 {abs(t_o-t_l)*1000:.0f} ms，'
                                   f'配对可能不可靠')
        return lab, raw, obj


def b64_i16(arr):
    """int16 毫米编码 —— 场景 ±30m 装得下，比 JSON 数字省一个数量级。"""
    return base64.b64encode(
        np.round(np.asarray(arr, dtype=np.float64) * 1000.0)
        .astype('<i2').tobytes()).decode()


def main():
    ap = argparse.ArgumentParser(description='聚类结果 3D 交互页面')
    ap.add_argument('--out', default='/tmp/cluster3d.html')
    ap.add_argument('--mount-height', type=float, default=1.76,
                    help='激光雷达离地高度（米），用来画地面参考网格')
    ap.add_argument('--max-bg', type=int, default=30000, help='背景点云上限')
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = Capture()
    print(f'等待一帧标签 + 原始云 + 检测框 ...')
    while rclpy.ok() and not node.done():
        rclpy.spin_once(node, timeout_sec=0.5)
    # 再多收一小会儿，让标签帧后面那几帧检测框也进缓冲，配对有余量
    for _ in range(12):
        if not rclpy.ok():
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.done():
        print('没收齐。perception 和回放节点都在跑吗？')
        return 1

    arr, raw, objs = node.closest()
    if arr is None or objs is None:
        print('标签或检测框没收到。')
        return 1
    node.destroy_node()
    rclpy.try_shutdown()

    xyz = np.stack([arr['x'], arr['y'], arr['z']], 1).astype(np.float64)
    lab = arr['label'].astype(np.int64)
    print(f'聚类点 {len(xyz)}，簇 {len(np.unique(lab[lab >= 0]))} 个；'
          f'背景帧 {len(raw)} 点；检测框 {len(objs.objs)} 个')

    # 背景抽稀：按距离均匀取，别让近处糊成一团
    if len(raw) > args.max_bg:
        sel = np.random.default_rng(0).choice(len(raw), args.max_bg, replace=False)
        sel.sort()
    else:
        sel = np.arange(len(raw))
    bg = np.stack([raw['x'][sel], raw['y'][sel], raw['z'][sel]], 1).astype(np.float64)

    # 按簇点数排序发色，超过八槽折叠成灰（和 cluster_view.py 同一套规则）
    uniq, cnt = np.unique(lab[lab >= 0], return_counts=True)
    order = uniq[np.argsort(-cnt)]
    cmap = {int(c): (DARK_SERIES[i] if i < len(DARK_SERIES) else OTHER_DARK)
            for i, c in enumerate(order)}
    noise = int((lab < 0).sum())

    # 每个簇的质心，用来和检测框中心对账
    cen = {int(c): xyz[lab == c].mean(0) for c in uniq}

    box_list = []
    offs = []
    for o in objs.objs:
        c = cen.get(int(o.idx))
        dx = dy = dz = None
        if c is not None:
            dx, dy, dz = (float(o.rel_x - c[0]), float(o.rel_y - c[1]),
                          float(o.rel_z - c[2]))
            offs.append((np.hypot(dx, dy), int(o.idx)))
        box_list.append({
            'idx': int(o.idx), 'x': float(o.rel_x), 'y': float(o.rel_y),
            'z': float(o.rel_z), 'l': float(o.length), 'w': float(o.width),
            'h': float(o.height), 'hd': float(o.rel_heading),
            'type': int(o.type), 'score': float(o.score),
            'n': int(cnt[list(uniq).index(o.idx)]) if int(o.idx) in list(uniq) else 0,
            'dx': dx, 'dy': dy, 'dz': dz,
        })

    payload = {
        'bg': b64_i16(bg.reshape(-1)),
        'bgN': int(len(bg)),
        'pt': b64_i16(xyz.reshape(-1)),
        'ptN': int(len(xyz)),
        'lab': base64.b64encode(lab.astype('<i2').tobytes()).decode(),
        'cmap': {str(k): v for k, v in cmap.items()},
        'boxes': box_list,
        'noise': noise,
        'mount': args.mount_height,
        # 初始视角对准聚类点的质心，距离按场景尺寸给 —— 固定写死的话换个
        # 场景就要手动拖半天才找得到东西
        'cam': {'tx': float(xyz[:, 0].mean()), 'ty': float(xyz[:, 1].mean()),
                'tz': float(xyz[:, 2].mean()),
                'dist': float(np.linalg.norm(xyz.max(0) - xyz.min(0)) * 1.15)},
    }

    html = HTML_TEMPLATE.replace('__PAYLOAD__', json.dumps(payload))
    with open(args.out, 'w') as f:
        f.write(html)
    print(f'页面写到 {args.out}  ({os.path.getsize(args.out)/1024:.0f} KB)')

    if offs:
        offs.sort(reverse=True)
        print(f'\n检测框中心 rel_x/rel_y 与簇质心的水平偏差（米）：')
        print('  ' + '  '.join(f'#{i}:{d:.2f}' for d, i in offs[:10]))
        print(f'  中位 {np.median([d for d, _ in offs]):.3f} m   '
              f'最大 {max(d for d, _ in offs):.3f} m')
        print('  （框边长是 length/width，偏差如果是它们的量级，说明 rel_x/rel_y '
              '不是矩形的中心而是一条边的中点）')
    return 0


HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="zh"><head><meta charset="utf-8">
<title>perception 聚类 3D</title>
<style>
 :root{--surface:#1a1a19;--page:#0d0d0d;--ink:#fff;--ink2:#c3c2b7;
       --muted:#898781;--grid:#2c2c2a;--line:#383835;}
 *{box-sizing:border-box}
 html,body{margin:0;height:100%;background:var(--page);color:var(--ink);
   font:13px/1.5 system-ui,-apple-system,"Segoe UI",sans-serif;overflow:hidden}
 #wrap{display:flex;height:100%}
 /* canvas 必须绝对定位撑满 stage：它的 width 属性会变成 flex item 的
    min-content 宽度，和"按 CSS 尺寸设后备缓冲"打架，结果是 canvas 越画越宽、
    右边栏被挤成 0 宽。min-width:0 + 绝对定位把这个反馈环断开。 */
 #stage{position:relative;flex:1;min-width:0;overflow:hidden}
 #cv{position:absolute;inset:0;width:100%;height:100%;display:block;cursor:grab}
 #cv.drag{cursor:grabbing}
 #side{width:310px;flex:none;background:var(--surface);border-left:1px solid var(--line);
   display:flex;flex-direction:column;overflow:hidden}
 #side h1{font-size:13px;font-weight:600;margin:0;padding:12px 14px 8px}
 #side .sub{color:var(--muted);font-size:11px;padding:0 14px 10px;
   border-bottom:1px solid var(--line)}
 #toggles{padding:10px 14px;border-bottom:1px solid var(--line);font-size:12px;
   color:var(--ink2);display:flex;flex-wrap:wrap;gap:10px}
 #toggles label{display:flex;align-items:center;gap:4px;cursor:pointer}
 #legend{overflow-y:auto;flex:1;padding:6px 0}
 .row{display:flex;align-items:center;gap:8px;padding:5px 14px;cursor:pointer;
   border-left:3px solid transparent}
 .row:hover{background:#221f1c}
 .row.on{border-left-color:var(--ink)}
 .row .sw{width:10px;height:10px;border-radius:2px;flex:none}
 .row .nm{flex:1;font-variant-numeric:tabular-nums}
 .row .meta{color:var(--muted);font-size:11px;font-variant-numeric:tabular-nums}
 .lg-title{color:var(--muted);font-size:10px;letter-spacing:.06em;
   padding:8px 14px 4px;text-transform:uppercase}
 #hint{position:absolute;left:14px;bottom:12px;color:var(--muted);font-size:11px;
   background:rgba(26,26,25,.82);padding:6px 10px;border-radius:5px}
 #tip{position:absolute;padding:5px 9px;background:rgba(13,13,13,.94);
   border:1px solid var(--line);border-radius:5px;font-size:11px;color:var(--ink2);
   pointer-events:none;display:none;white-space:nowrap;font-variant-numeric:tabular-nums}
 #stat{position:absolute;left:14px;top:12px;color:var(--muted);font-size:11px;
   background:rgba(26,26,25,.82);padding:6px 10px;border-radius:5px;
   font-variant-numeric:tabular-nums}
</style></head><body>
<div id="wrap">
  <div id="stage">
    <canvas id="cv"></canvas>
    <div id="stat"></div><div id="hint">拖拽转视角　滚轮缩放　右键平移　点图例只看某个簇　Esc 取消　R 复位</div>
    <div id="tip"></div>
  </div>
  <div id="side">
    <h1>perception 聚类（单帧）</h1>
    <div class="sub" id="sub"></div>
    <div id="toggles">
      <label><input type="checkbox" id="tBg" checked>背景点云</label>
      <label><input type="checkbox" id="tBox" checked>检测框</label>
      <label><input type="checkbox" id="tGnd" checked>地面网格</label>
    </div>
    <div id="legend"></div>
  </div>
</div>
<script>
// 出错了别静默成白屏 —— 写到标题上，--dump-dom 一眼能看到
addEventListener('error',e=>{document.title='ERR: '+e.message;});
const P = __PAYLOAD__;
function dec16(b64){const s=atob(b64),n=s.length,u=new Uint8Array(n);
  for(let i=0;i<n;i++)u[i]=s.charCodeAt(i);return new Int16Array(u.buffer);}
function decI16(b64){return dec16(b64);}
const bgv=dec16(P.bg), ptv=dec16(P.pt), lbv=decI16(P.lab);

// ---- 相机：绕目标点的球坐标 ----
const HOME={yaw:-0.9,pitch:0.45,dist:P.cam.dist,tx:P.cam.tx,ty:P.cam.ty,tz:P.cam.tz};
const cam=Object.assign({},HOME);
let solo=null, hover=null;

const cv=document.getElementById('cv'), ctx=cv.getContext('2d');
let W=0,H=0,img=null,zb=null;
// 画布后备缓冲跟着 CSS 尺寸走。必须在每帧开头查一次 —— 脚本跑的时候布局
// 可能还没发生（getBoundingClientRect 返回 0），那时候建的 1×1 缓冲会一直
// 用下去，页面看着就是一片空白。
function ensureSize(){
  const r=cv.getBoundingClientRect();
  const w=Math.max(1,r.width|0), h=Math.max(1,r.height|0);
  if(w!==W||h!==H){W=w;H=h;cv.width=W;cv.height=H;
    img=ctx.createImageData(W,H);zb=new Float32Array(W*H);}
}
addEventListener('resize',()=>{W=0;need();});

function camBasis(){
  const cy=Math.cos(cam.yaw),sy=Math.sin(cam.yaw);
  const cp=Math.cos(cam.pitch),sp=Math.sin(cam.pitch);
  // 世界 z 朝上：eye 在 target + dist*(...) 处
  const dir=[cp*cy,cp*sy,sp];
  const eye=[cam.tx+dir[0]*cam.dist,cam.ty+dir[1]*cam.dist,cam.tz+dir[2]*cam.dist];
  const fwd=[-dir[0],-dir[1],-dir[2]];
  let up=[0,0,1];
  let right=[fwd[1]*up[2]-fwd[2]*up[1],fwd[2]*up[0]-fwd[0]*up[2],fwd[0]*up[1]-fwd[1]*up[0]];
  const rl=Math.hypot(...right)||1; right=right.map(v=>v/rl);
  up=[right[1]*fwd[2]-right[2]*fwd[1],right[2]*fwd[0]-right[0]*fwd[2],right[0]*fwd[1]-right[1]*fwd[0]];
  return {eye,fwd,right,up};
}
let B=null, focal=1;
function updateBasis(){B=camBasis();focal=(H*0.5)/Math.tan(0.42);}
function project(p){
  const dx=p[0]-B.eye[0],dy=p[1]-B.eye[1],dz=p[2]-B.eye[2];
  const z=dx*B.fwd[0]+dy*B.fwd[1]+dz*B.fwd[2];
  if(z<=0.15)return null;
  const x=dx*B.right[0]+dy*B.right[1]+dz*B.right[2];
  const y=dx*B.up[0]+dy*B.up[1]+dz*B.up[2];
  return [W*0.5+x*focal/z, H*0.5-y*focal/z, z];
}
function put(px,py,z,r,g,b){
  if(px<0||py<0||px>=W||py>=H)return;
  const i=py*W+px;
  if(z<zb[i]){zb[i]=z;const o=i*4;img.data[o]=r;img.data[o+1]=g;img.data[o+2]=b;img.data[o+3]=255;}
}
function hex2rgb(h){return [parseInt(h.slice(1,3),16),parseInt(h.slice(3,5),16),parseInt(h.slice(5,7),16)];}
const CMAP={}; for(const k in P.cmap) CMAP[+k]=hex2rgb(P.cmap[k]);
const GRAY=[124,122,116], BG=[70,68,64], NOISE=[230,230,230];

function drawPoints(){
  const showBg=document.getElementById('tBg').checked;
  if(showBg){
    for(let i=0;i<P.bgN;i++){
      const p=[bgv[i*3]/1000,bgv[i*3+1]/1000,bgv[i*3+2]/1000];
      const s=project(p); if(!s)continue;
      const px=s[0]|0,py=s[1]|0;
      put(px,py,s[2],BG[0],BG[1],BG[2]);
    }
  }
  for(let i=0;i<P.ptN;i++){
    const c=lbv[i];
    if(c<0){const p=[ptv[i*3]/1000,ptv[i*3+1]/1000,ptv[i*3+2]/1000];
      const s=project(p); if(s)put(s[0]|0,s[1]|0,s[2],NOISE[0],NOISE[1],NOISE[2]); continue;}
    if(solo!==null&&c!==solo)continue;
    const col=CMAP[c]||GRAY;
    const dim=(hover!==null&&c!==hover)?0.45:1;
    const p=[ptv[i*3]/1000,ptv[i*3+1]/1000,ptv[i*3+2]/1000];
    const s=project(p); if(!s)continue;
    const px=s[0]|0,py=s[1]|0, big=(s[2]<12)?1:0;
    for(let a=-big;a<=big;a++)for(let b=-big;b<=big;b++)
      put(px+a,py+b,s[2],col[0]*dim, col[1]*dim, col[2]*dim);
  }
}
function line3(a,b,r,g,bl,front){
  const A=project(a),Bp=project(b); if(!A||!Bp)return;
  const z=front?-1:(A[2]+Bp[2])*0.5;   // 框画在最前；地面网格要老老实实被遮挡
  const n=Math.max(2,(Math.hypot(Bp[0]-A[0],Bp[1]-A[1])|0));
  for(let i=0;i<=n;i++){
    const t=i/n, px=(A[0]+(Bp[0]-A[0])*t)|0, py=(A[1]+(Bp[1]-A[1])*t)|0;
    put(px,py,z,r,g,bl);
  }
}
const labels=[];
function drawBoxes(){
  labels.length=0;
  if(!document.getElementById('tBox').checked)return;
  for(const o of P.boxes){
    if(solo!==null&&o.idx!==solo)continue;
    const col=CMAP[o.idx]||GRAY;
    const dim=(hover!==null&&o.idx!==hover)?0.4:1;
    const ch=Math.cos(o.hd), sh=Math.sin(o.hd);
    const hl=o.l/2, hw=o.w/2, hz=o.h/2;
    // min_rotate_rect: length 沿 heading 方向，width 垂直
    const c=[[hl,hw],[-hl,hw],[-hl,-hw],[hl,-hw]].map(([u,v])=>
      [o.x+u*ch-v*sh, o.y+u*sh+v*ch]);
    const z0=o.z-hz, z1=o.z+hz;
    const bot=c.map(p=>[p[0],p[1],z0]), top=c.map(p=>[p[0],p[1],z1]);
    for(let i=0;i<4;i++){
      line3(bot[i],bot[(i+1)%4],col[0]*dim,col[1]*dim,col[2]*dim,true);
      line3(top[i],top[(i+1)%4],col[0]*dim,col[1]*dim,col[2]*dim,true);
      line3(bot[i],top[i],col[0]*dim,col[1]*dim,col[2]*dim,true);
    }
    const s=project([o.x,o.y,o.z]);
    if(s)labels.push({x:s[0],y:s[1],t:`#${o.idx} ${TYPE_CN(o.type)} ${o.l.toFixed(1)}×${o.w.toFixed(1)}×${o.h.toFixed(1)}`,
                      on:(hover===null||hover===o.idx)});
  }
}
function TYPE_CN(t){return ['未知','卡车','车','人','骑行者','墙'][t]||'未知';}
function drawGround(){
  if(!document.getElementById('tGnd').checked)return;
  const z=-P.mount, R=22, step=2;
  const g=[44,44,42];
  for(let i=-R;i<=R;i+=step){
    line3([i,-R,z],[i,R,z],g[0],g[1],g[2],false);
    line3([-R,i,z],[R,i,z],g[0],g[1],g[2],false);
  }
  // 传感器原点
  const s=project([0,0,0]);
  if(s)labels.push({x:s[0],y:s[1],t:'雷达原点',on:true,strong:true});
}
function render(){
  ensureSize();
  updateBasis();
  img.data.fill(0); zb.fill(1e9);
  drawGround(); drawPoints(); drawBoxes();
  ctx.putImageData(img,0,0);
  ctx.font='11px system-ui,-apple-system,"Segoe UI",sans-serif';
  ctx.textBaseline='middle';
  for(const L of labels){
    if(!L.on)continue;
    ctx.fillStyle='rgba(13,13,13,.72)';
    const w=ctx.measureText(L.t).width+10;
    ctx.fillRect(L.x+8,L.y-9,w,18);
    ctx.fillStyle=L.strong?'#898781':'#c3c2b7';
    ctx.fillText(L.t,L.x+13,L.y);
  }
  document.getElementById('stat').textContent=
    `视角 ${(cam.yaw*57.3).toFixed(0)}° / ${(cam.pitch*57.3).toFixed(0)}°　距离 ${cam.dist.toFixed(1)} m`;
}
// 按需重绘：常驻 rAF 每帧清 4MB 的 ImageData 是白烧 CPU，改脏标记
let dirty=true;
const need=()=>{dirty=true;};
function loop(){if(dirty){dirty=false;render();}requestAnimationFrame(loop);}

// ---- 交互 ----
let drag=null;
cv.addEventListener('contextmenu',e=>e.preventDefault());
cv.addEventListener('mousedown',e=>{drag={x:e.clientX,y:e.clientY,btn:e.button};cv.classList.add('drag');});
addEventListener('mouseup',()=>{drag=null;cv.classList.remove('drag');});
addEventListener('mousemove',e=>{
  if(drag){
    const dx=e.clientX-drag.x, dy=e.clientY-drag.y;
    drag.x=e.clientX;drag.y=e.clientY;
    need();
    if(drag.btn===2){                       // 平移：沿相机的 right/up 反向推 target
      const k=cam.dist/focal;
      cam.tx-=(B.right[0]*dx - B.up[0]*dy)*k;
      cam.ty-=(B.right[1]*dx - B.up[1]*dy)*k;
      cam.tz-=(B.right[2]*dx - B.up[2]*dy)*k;
    }else{
      cam.yaw+=dx*0.008;
      cam.pitch=Math.max(-1.45,Math.min(1.45,cam.pitch+dy*0.008));
    }
    return;
  }
  // 悬停高亮：找最近的框
  const r=cv.getBoundingClientRect();
  const mx=e.clientX-r.left,my=e.clientY-r.top;
  let best=null,bd=1e9;
  for(const o of P.boxes){
    const s=project([o.x,o.y,o.z]); if(!s)continue;
    const d=Math.hypot(s[0]-mx,s[1]-my);
    if(d<40&&d<bd){bd=d;best=o.idx;}
  }
  if(best!==hover)need();
  hover=best;
  const tip=document.getElementById('tip');
  if(best!==null){
    const o=P.boxes.find(q=>q.idx===best);
    tip.style.display='block';
    tip.style.left=(mx+16)+'px'; tip.style.top=(my+14)+'px';
    tip.innerHTML=`#${o.idx} ${TYPE_CN(o.type)}　${o.n} 点<br>`+
      `${o.l.toFixed(2)}×${o.w.toFixed(2)}×${o.h.toFixed(2)} m　朝向 ${(o.hd*57.3).toFixed(0)}°<br>`+
      `中心(${o.x.toFixed(2)}, ${o.y.toFixed(2)}, ${o.z.toFixed(2)})`+
      (o.dx!==null?`<br>与簇质心差 (${o.dx.toFixed(2)}, ${o.dy.toFixed(2)}, ${o.dz.toFixed(2)})`:'');
  } else tip.style.display='none';
});
cv.addEventListener('wheel',e=>{e.preventDefault();need();
  cam.dist=Math.max(2,Math.min(120,cam.dist*Math.exp(e.deltaY*0.0011)));},{passive:false});
for(const id of ['tBg','tBox','tGnd'])
  document.getElementById(id).addEventListener('change',need);

// ---- 图例 ----
function buildLegend(){
  const el=document.getElementById('legend'); el.innerHTML='';
  const rows=Object.keys(P.cmap).map(Number)
    .sort((a,b)=>(P.boxes.find(o=>o.idx===b)?.n||0)-(P.boxes.find(o=>o.idx===a)?.n||0));
  const t=document.createElement('div');
  t.className='lg-title'; t.textContent=`簇（按点数）· 点一下只看它`;
  el.appendChild(t);
  for(const c of rows){
    const b=P.boxes.find(o=>o.idx===c);
    const d=document.createElement('div'); d.className='row';
    d.innerHTML=`<span class="sw" style="background:${P.cmap[c]}"></span>`+
      `<span class="nm">#${c}</span>`+
      `<span class="meta">${b?b.n:0} 点　${b?b.l.toFixed(1)+'×'+b.w.toFixed(1)+'×'+b.h.toFixed(1):''}　${b?TYPE_CN(b.type):''}</span>`;
    d.onclick=()=>{solo=(solo===c)?null:c; refreshLegend(); need();};
    d.onmouseenter=()=>{hover=c; need();};
    d.onmouseleave=()=>{hover=null; need();};
    d.dataset.c=c; el.appendChild(d);
  }
  if(P.noise>0){
    const n=document.createElement('div'); n.className='row';
    n.innerHTML=`<span class="sw" style="background:rgb(230,230,230)"></span>`+
      `<span class="nm">噪点</span><span class="meta">${P.noise} 点（未成簇）</span>`;
    el.appendChild(n);
  }
  refreshLegend();
}
function refreshLegend(){
  for(const d of document.querySelectorAll('.row'))
    d.classList.toggle('on', solo!==null && +d.dataset.c===solo);
}
addEventListener('keydown',e=>{
  if(e.key==='Escape'){solo=null;refreshLegend();need();}
  if(e.key==='r'||e.key==='R'){Object.assign(cam,HOME);need();}
});

document.getElementById('sub').innerHTML=
  `${P.ptN} 个聚类点　${Object.keys(P.cmap).length} 个簇　`+
  `${P.boxes.length} 个检测框<br>地面参考网格按架高 ${P.mount.toFixed(2)} m 画在 z=-${P.mount.toFixed(2)}`;

buildLegend(); loop();
</script></body></html>
"""


if __name__ == '__main__':
    sys.exit(main())
