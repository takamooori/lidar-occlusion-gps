#!/usr/bin/env python3
"""
export_skymap_html.py - 遮蔽スカイマップ・ブラウザ（実データ版）生成
=====================================================================
GLIM dump の全フレームについて遮蔽率とレイ単位のヒット状態を計算し、
データを埋め込んだ「自己完結の単一HTML」を書き出す。

生成された HTML は ROS2 も Jupyter も不要・オフラインで開ける。
スライダー/番号でフレーム指定、グリッドで複数フレーム一覧、
遮蔽率タイムラインで全フレームを見比べできる。

依存（同じディレクトリに置く）:
  occlusion_core.py, data_source.py

使い方:
  python3 export_skymap_html.py ~/ros2_ws/dump/nakaniwa_0522
  python3 export_skymap_html.py <dump_dir> --out skymap.html --stride 2
  python3 export_skymap_html.py <dump_dir> --north-az 37      # 真北合わせ
  python3 export_skymap_html.py <dump_dir> --max-frames 40    # 先頭40フレームのみ

点群データの渡し方:
  入力は GLIM dump ディレクトリ。中に 6桁フォルダ(000000, 000001, ...)があり、
  各フォルダに data.txt(stamp + T_world_lidar) と points_compact.bin(float32 N×3)。
  座標系(world/lidar)は data_source 側で自動判定して lidar 座標に正規化される。
  → ツールが触るのは data_source.make_source だけなので、将来 rosbag 生点群へ
    移行する場合も data_source に RosbagSource を実装すれば本ファイルは無変更。
"""

import os
import sys
import json
import argparse
import datetime
import numpy as np

from data_source import make_source
from occlusion_core import (
    compute_occlusion, fibonacci_hemisphere, azimuth_elevation,
    DEFAULT_N_RAYS, DEFAULT_MAX_DIST, DEFAULT_MIN_DIST,
    DEFAULT_EL_MIN_DEG, DEFAULT_ANGLE_DEG,
)


def build_data(dump_dir, n_rays, max_dist, min_dist, el_min, angle,
               north_az, stride, max_frames, verbose=True):
    """全フレームを走査して HTML 埋め込み用の data 辞書を返す。"""
    src = make_source("dump", dump_dir=dump_dir)
    rays = fibonacci_hemisphere(n_rays)
    az, el = azimuth_elevation(rays)            # az: +X=0,CCW / el: 仰角
    az_plot = az - north_az                     # north_az を図の上(N)へ
    R = len(rays)

    folders = src.list_frames()
    if stride > 1:
        folders = folders[::stride]
    if max_frames:
        folders = folders[:max_frames]
    if not folders:
        raise RuntimeError(f"dump にフレームがありません: {dump_dir}")

    frames = []
    for i, folder in enumerate(folders):
        fd = src.get_frame(folder)
        if fd is None:
            continue
        res = compute_occlusion(fd.points_lidar, rays=rays, max_dist=max_dist,
                                min_dist=min_dist, el_min_deg=el_min,
                                angle_deg=angle, track_hits=False)
        # レイ単位のステータス: 0=開空, 1=遮蔽, 2=mask外(レイ仰角<el_min)
        masked = el < el_min
        status = np.where(masked, 2, np.where(res.ray_hit_mask, 1, 0)).astype(np.uint8)
        nhit = int((status == 1).sum())
        pos = fd.position  # world座標 (x, y, z)
        frames.append({
            "label": folder,
            "stamp": (round(fd.stamp, 2) if fd.stamp else None),
            "occ": round(float(nhit) / R, 4),
            "nhit": nhit,
            "x": round(float(pos[0]), 3),
            "y": round(float(pos[1]), 3),
            "status": "".join(chr(48 + int(s)) for s in status),  # 長さR の '0/1/2' 文字列
        })
        if verbose:
            print(f"\r  {i+1}/{len(folders)}  {folder} occ={frames[-1]['occ']:.3f}",
                  end="", flush=True)
    if verbose:
        print()
    if not frames:
        raise RuntimeError("有効なフレームが0件でした（data.txt / points_compact.bin を確認）")

    return {
        "meta": {
            "dump": os.path.basename(os.path.abspath(dump_dir.rstrip("/"))),
            "n_rays": R, "el_min": el_min, "north_az": north_az,
            "max_dist": max_dist, "min_dist": min_dist, "angle": angle,
            "stride": stride,
            "generated": datetime.datetime.now().strftime("%Y-%m-%d %H:%M"),
        },
        "az": [round(float(a), 2) for a in az_plot],
        "el": [round(float(e), 2) for e in el],
        "frames": frames,
    }


# ======================================================================
#  HTML テンプレート（__DATA__ に JSON を差し込む。デモと同じ落ち着いた白基調）
# ======================================================================
TEMPLATE = r'''<meta charset="utf-8">
<title>遮蔽スカイマップ・ブラウザ</title>
<style>
  :root{
    --bg:#ffffff; --bg2:#f7f8fa; --panel:#f6f7f9; --line:#e4e7ec;
    --text:#333842; --muted:#7b828d; --dim:#a6acb6;
    --accent:#3f8e83; --accent2:#b88a3e;
    --hit:#d15a52; --miss:#5277ac; --mask:#b6bcc6;
    --sans:ui-sans-serif,system-ui,"Hiragino Sans","Noto Sans JP",sans-serif;
    --mono:ui-monospace,"SFMono-Regular",Menlo,Consolas,"Noto Sans JP",monospace;
  }
  *{box-sizing:border-box}
  html,body{margin:0;background:linear-gradient(180deg,#fbfcfd,#ffffff);color:var(--text);font-family:var(--sans);}
  .wrap{max-width:1120px;margin:0 auto;padding:22px 22px 40px;}
  header{display:flex;align-items:baseline;gap:14px;flex-wrap:wrap;border-bottom:1px solid var(--line);padding-bottom:14px;margin-bottom:18px;}
  h1{font-size:18px;font-weight:600;letter-spacing:.02em;margin:0;}
  .tag{font-family:var(--mono);font-size:11px;color:#2f6b62;background:#edf4f2;border:1px solid #d4e6e2;padding:2px 8px;border-radius:4px;font-weight:600;letter-spacing:.06em;}
  .sub{color:var(--muted);font-size:12.5px;font-family:var(--mono);}
  .grid{display:grid;grid-template-columns:1.05fr 0.95fr 300px;gap:18px;align-items:start;}
  @media(max-width:1180px){.grid{grid-template-columns:1fr 1fr;}}
  @media(max-width:820px){.grid{grid-template-columns:1fr;}}
  .stage{background:linear-gradient(180deg,#ffffff,#fafbfc);border:1px solid var(--line);border-radius:14px;padding:10px;position:relative;}
  svg{display:block;width:100%;height:auto;}
  .panel{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:16px 16px 18px;}
  .panel + .panel{margin-top:16px;}
  .readout{display:grid;grid-template-columns:1fr 1fr;gap:10px 14px;margin-bottom:4px;}
  .kv{display:flex;flex-direction:column;gap:3px;}
  .kv .k{font-size:10.5px;color:var(--muted);letter-spacing:.05em;text-transform:uppercase;}
  .kv .v{font-family:var(--mono);font-size:19px;font-weight:600;color:var(--text);}
  .occbig .v{font-size:30px;color:var(--accent);}
  .occbar{height:7px;border-radius:4px;background:#e9ecf1;overflow:hidden;margin-top:6px;}
  .occbar > i{display:block;height:100%;background:linear-gradient(90deg,var(--miss),var(--hit));}
  .ctl{margin-top:6px;}
  .row{display:flex;align-items:center;gap:8px;margin-top:12px;}
  label.lbl{font-size:11px;color:var(--muted);letter-spacing:.04em;min-width:54px;}
  input[type=range]{flex:1;accent-color:var(--accent);height:4px;}
  input[type=number]{width:84px;background:#fff;border:1px solid var(--line);color:var(--text);font-family:var(--mono);font-size:14px;border-radius:7px;padding:6px 8px;}
  button{background:#fff;border:1px solid var(--line);color:var(--text);font-family:var(--mono);font-size:13px;padding:7px 11px;border-radius:8px;cursor:pointer;transition:.15s;}
  button:hover{border-color:var(--accent);color:var(--accent);}
  button.primary{background:var(--accent);color:#fff;border-color:var(--accent);font-weight:600;}
  button.primary:hover{filter:brightness(1.06);color:#fff;}
  .legend{display:flex;gap:16px;flex-wrap:wrap;font-size:12px;color:var(--muted);margin-top:2px;}
  .legend span{display:inline-flex;align-items:center;gap:6px;}
  .dot{width:10px;height:10px;border-radius:50%;display:inline-block;}
  .timeline-h{display:flex;justify-content:space-between;align-items:center;margin-bottom:6px;}
  .timeline-h .k{font-size:10.5px;color:var(--muted);letter-spacing:.05em;text-transform:uppercase;}
  .hint{font-size:11.5px;color:var(--dim);margin-top:8px;line-height:1.6;}
  .gridview{display:none;grid-template-columns:repeat(auto-fill,minmax(118px,1fr));gap:10px;}
  .gridview.on{display:grid;}
  .stage.hidden{display:none;}
  .mini{background:var(--bg2);border:1px solid var(--line);border-radius:9px;padding:6px;cursor:pointer;text-align:center;transition:.15s;}
  .mini:hover{border-color:var(--accent);}
  .mini.cur{border-color:var(--accent2);box-shadow:0 0 0 1px var(--accent2) inset;}
  .mini canvas{width:100%;height:auto;display:block;}
  .mini .ml{font-family:var(--mono);font-size:10.5px;color:var(--muted);margin-top:4px;}
  .mini .mo{color:var(--accent);}
  .seg{display:inline-flex;border:1px solid var(--line);border-radius:8px;overflow:hidden;}
  .seg button{border:none;border-radius:0;background:#fff;}
  .seg button.on{background:var(--accent);color:#fff;font-weight:600;}
</style>

<div class="wrap">
  <header>
    <h1>遮蔽スカイマップ・ブラウザ</h1>
    <span class="tag" id="dumptag">–</span>
    <span class="sub">中心=天頂 · 外周=地平線 · 真上から見た上半球投影</span>
  </header>
  <div class="grid">
    <div>
      <div class="stage" id="stage"><svg id="sky" viewBox="0 0 560 560"></svg></div>
      <div class="gridview" id="gridview"></div>
    </div>
    <div>
      <div class="panel" style="padding:14px">
        <div class="timeline-h"><span class="k">キーフレーム位置（GLIM軌跡 · 遮蔽率カラー）</span><span class="sub" id="mapcoord">–</span></div>
        <svg id="mapview" viewBox="0 0 360 360" style="background:#fff;border:1px solid var(--line);border-radius:8px;"></svg>
        <div class="legend" style="margin-top:10px;justify-content:space-between">
          <span><i class="dot" style="background:#5277ac"></i>低 occ</span>
          <span style="font-size:10.5px;color:var(--dim)">クリックでフレーム切替</span>
          <span><i class="dot" style="background:#d15a52"></i>高 occ</span>
        </div>
        <div class="hint" id="maphint">XY: world座標 [m] · 現在フレームは <span style="color:var(--accent2)">●</span></div>
      </div>
    </div>
    <div>
      <div class="panel">
        <div class="readout">
          <div class="kv occbig"><span class="k">遮蔽率 occ</span><span class="v" id="occv">–</span></div>
          <div class="kv"><span class="k">ヒット / 全レイ</span><span class="v" id="hitv">–</span></div>
        </div>
        <div class="occbar"><i id="occbar"></i></div>
        <div class="readout" style="margin-top:14px">
          <div class="kv"><span class="k">フレーム</span><span class="v" id="framev">–</span></div>
          <div class="kv"><span class="k">stamp</span><span class="v" id="stampv" style="font-size:14px">–</span></div>
        </div>
        <div class="seg" style="margin-top:16px">
          <button id="modeSingle" class="on">単一フレーム</button>
          <button id="modeGrid">グリッド（全フレーム）</button>
        </div>
        <div class="ctl" id="singleCtl">
          <div class="row"><label class="lbl">フレーム</label>
            <input type="range" id="slider" min="0" value="0">
            <input type="number" id="num" min="0" value="0"></div>
          <div class="row">
            <button id="prev">◀ 前</button>
            <button id="play" class="primary">▶ 再生</button>
            <button id="next">次 ▶</button>
            <button id="peak">最大遮蔽へ</button></div>
        </div>
        <div class="legend" style="margin-top:16px">
          <span><i class="dot" style="background:var(--hit)"></i>遮蔽</span>
          <span><i class="dot" style="background:var(--miss)"></i>開空</span>
          <span><i class="dot" style="background:var(--mask)"></i>mask外(仰角&lt;<span id="elminlab">15</span>°)</span>
        </div>
      </div>
      <div class="panel">
        <div class="timeline-h"><span class="k">遮蔽率タイムライン</span><span class="sub" id="rangev"></span></div>
        <svg id="timeline" viewBox="0 0 288 70"></svg>
        <div class="hint" id="prov">クリックでそのフレームへジャンプ。</div>
      </div>
    </div>
  </div>
</div>

<script>
const DATA = __DATA__;
const D2R=Math.PI/180;
const RAYS = DATA.az.map((a,i)=>({az:a, el:DATA.el[i]}));
const N_RAYS = RAYS.length, EL_MIN = DATA.meta.el_min, N_FRAMES = DATA.frames.length;
const FRAMES = DATA.frames;
function st(f,i){return FRAMES[f].status.charCodeAt(i)-48;}
const occVals=FRAMES.map(f=>f.occ);
const occMax=Math.max.apply(null,occVals), occMin=Math.min.apply(null,occVals);
const peakIdx=occVals.indexOf(occMax);

const CX=280,CY=280,RAD=232;
function proj(az,el){const r=(90-el)/90*RAD, a=(90+az)*D2R; return [CX+r*Math.cos(a), CY-r*Math.sin(a)];}

const sky=document.getElementById('sky'); const NS='http://www.w3.org/2000/svg';
function el2(tag,attrs){const e=document.createElementNS(NS,tag);for(const k in attrs)e.setAttribute(k,attrs[k]);return e;}
[0,30,60].forEach(elev=>sky.appendChild(el2('circle',{cx:CX,cy:CY,r:(90-elev)/90*RAD,fill:'none',stroke:'#e8eaef','stroke-width':1})));
sky.appendChild(el2('circle',{cx:CX,cy:CY,r:(90-EL_MIN)/90*RAD,fill:'none',stroke:'#c2c7d0','stroke-width':1.4,'stroke-dasharray':'5 5'}));
[[CX,CY-RAD,CX,CY+RAD],[CX-RAD,CY,CX+RAD,CY]].forEach(p=>sky.appendChild(el2('line',{x1:p[0],y1:p[1],x2:p[2],y2:p[3],stroke:'#eef0f3','stroke-width':1})));
[['N',CX,CY-RAD-14],['E',CX+RAD+14,CY],['S',CX,CY+RAD+18],['W',CX-RAD-14,CY]].forEach(c=>{const t=el2('text',{x:c[1],y:c[2],fill:'#4a505c','font-size':16,'font-weight':600,'text-anchor':'middle','dominant-baseline':'middle','font-family':'ui-monospace,monospace'});t.textContent=c[0];sky.appendChild(t);});
const ml=el2('text',{x:CX-(RAD*0.74),y:CY+(RAD*0.66),fill:'#9aa0ab','font-size':10,'text-anchor':'middle'});ml.textContent='mask角='+EL_MIN+'°';sky.appendChild(ml);

const CIR=[], layer=el2('g',{}); sky.appendChild(layer);
for(let i=0;i<N_RAYS;i++){const p=proj(RAYS[i].az,RAYS[i].el);const c=el2('circle',{cx:p[0].toFixed(1),cy:p[1].toFixed(1),r:2.3});layer.appendChild(c);CIR.push(c);}
const FILL=['#5277ac','#d15a52','#b6bcc6'], RAD_R=['2.3','2.9','2.0'];

let cur=0, playing=false, timer=null;
function setFrame(f){
  cur=Math.max(0,Math.min(N_FRAMES-1,f|0));
  const fr=FRAMES[cur];
  for(let i=0;i<N_RAYS;i++){const h=st(cur,i);CIR[i].setAttribute('fill',FILL[h]);CIR[i].setAttribute('r',RAD_R[h]);CIR[i].setAttribute('opacity',h===2?0.55:0.9);}
  document.getElementById('occv').textContent=fr.occ.toFixed(3);
  document.getElementById('hitv').textContent=fr.nhit+' / '+N_RAYS;
  document.getElementById('framev').textContent=fr.label;
  document.getElementById('stampv').textContent=(fr.stamp!=null?fr.stamp:'–');
  document.getElementById('occbar').style.width=(fr.occ*100).toFixed(1)+'%';
  document.getElementById('slider').value=cur; document.getElementById('num').value=cur;
  updateMarker();
  updateMapMarker();
  document.querySelectorAll('.mini').forEach((m,i)=>m.classList.toggle('cur',i===cur));
}

// ====== 位置パネル（GLIM軌跡 + 遮蔽率カラー） ======
const mapview=document.getElementById('mapview');
const MW=360, MH=360, MPAD=28;
const XS=FRAMES.map(f=>f.x), YS=FRAMES.map(f=>f.y);
const xMin=Math.min.apply(null,XS), xMax=Math.max.apply(null,XS);
const yMin=Math.min.apply(null,YS), yMax=Math.max.apply(null,YS);
const xRng=Math.max(xMax-xMin,0.1), yRng=Math.max(yMax-yMin,0.1);
const dataAspect=xRng/yRng, plotW=MW-2*MPAD, plotH=MH-2*MPAD;
let mx0, mx1, my0, my1;
// アスペクト比を保ってプロット領域に収める
if(dataAspect > plotW/plotH){
  const usedH = plotW/dataAspect;
  mx0=MPAD; mx1=MW-MPAD; my0=MPAD+(plotH-usedH)/2; my1=my0+usedH;
}else{
  const usedW = plotH*dataAspect;
  my0=MPAD; my1=MH-MPAD; mx0=MPAD+(plotW-usedW)/2; mx1=mx0+usedW;
}
function mx(x){return mx0+(x-xMin)/xRng*(mx1-mx0);}
function my(y){return my1-(y-yMin)/yRng*(my1-my0);}  // Y軸は上向き
// occを 0..1 に正規化してカラーマップ
function occColor(o){
  const t=(o-occMin)/((occMax-occMin)||1);
  // 青(#5277ac) → 灰 → 赤(#d15a52)
  const r=Math.round(82 + t*(209-82));
  const g=Math.round(119 + t*(90-119));
  const b=Math.round(172 + t*(82-172));
  return 'rgb('+r+','+g+','+b+')';
}
// 枠
mapview.appendChild(el2('rect',{x:mx0-4,y:my0-4,width:(mx1-mx0)+8,height:(my1-my0)+8,fill:'#fafbfc',stroke:'#eef0f3','stroke-width':1,rx:4}));
// 軌跡線（細い灰色）
let pd=''; FRAMES.forEach((f,i)=>{ pd += (i?'L':'M')+mx(f.x).toFixed(1)+' '+my(f.y).toFixed(1)+' '; });
mapview.appendChild(el2('path',{d:pd,fill:'none',stroke:'#c2c7d0','stroke-width':1.2}));
// 各フレームのドット
const MDOT=[];
FRAMES.forEach((f,i)=>{
  const c=el2('circle',{cx:mx(f.x).toFixed(1),cy:my(f.y).toFixed(1),r:3.4,fill:occColor(f.occ),
                        stroke:'#fff','stroke-width':0.6,cursor:'pointer'});
  c.addEventListener('click',()=>{stop();setFrame(i);});
  // ツールチップ用 title
  const t=el2('title',{}); t.textContent='frame '+f.label+' / occ='+f.occ.toFixed(3)+' / xy=('+f.x+', '+f.y+')';
  c.appendChild(t);
  mapview.appendChild(c); MDOT.push(c);
});
// 始点（S）・終点（E）マーカー
function labelDot(x,y,txt,col){
  mapview.appendChild(el2('circle',{cx:x,cy:y,r:6,fill:'none',stroke:col,'stroke-width':1.6}));
  const tt=el2('text',{x:x+9,y:y+4,fill:col,'font-size':10.5,'font-family':'ui-monospace,monospace'});
  tt.textContent=txt; mapview.appendChild(tt);
}
labelDot(mx(XS[0]),my(YS[0]),'S','#4a505c');
labelDot(mx(XS[N_FRAMES-1]),my(YS[N_FRAMES-1]),'E','#4a505c');
// 軸ラベル（XY範囲のみ簡易表示）
function axText(x,y,txt,anchor){
  const t=el2('text',{x:x,y:y,fill:'#9aa0ab','font-size':9.5,'text-anchor':anchor||'middle','font-family':'ui-monospace,monospace'});
  t.textContent=txt; mapview.appendChild(t);
}
axText(mx0, my1+14, xMin.toFixed(1)+' m','start');
axText(mx1, my1+14, xMax.toFixed(1)+' m','end');
axText(mx0-6, my1+4, yMin.toFixed(1)+'','end');
axText(mx0-6, my0+8, yMax.toFixed(1)+'','end');
axText((mx0+mx1)/2, my1+14, 'X','middle');
axText(mx0-18, (my0+my1)/2, 'Y','middle');
// 現在フレームのハイライト（金色リング）
const curRing=el2('circle',{cx:mx(XS[0]),cy:my(YS[0]),r:7,fill:'none',stroke:'#b88a3e','stroke-width':2.0});
mapview.appendChild(curRing);
function updateMapMarker(){
  const f=FRAMES[cur];
  curRing.setAttribute('cx',mx(f.x).toFixed(1));
  curRing.setAttribute('cy',my(f.y).toFixed(1));
  document.getElementById('mapcoord').textContent='x='+f.x+', y='+f.y+' m';
}

const tl=document.getElementById('timeline'); const TW=288,TH=70,PAD=8;
function tlx(i){return N_FRAMES<2?TW/2:PAD+i/(N_FRAMES-1)*(TW-2*PAD);}
function tly(o){const lo=Math.max(0,occMin-0.05),hi=occMax+0.05;return TH-PAD-(o-lo)/((hi-lo)||1)*(TH-2*PAD);}
(function(){let d='';FRAMES.forEach((f,i)=>{d+=(i?'L':'M')+tlx(i).toFixed(1)+' '+tly(f.occ).toFixed(1)+' ';});
  tl.appendChild(el2('path',{d:d+'L'+tlx(N_FRAMES-1)+' '+(TH-PAD)+' L'+tlx(0)+' '+(TH-PAD)+' Z',fill:'rgba(63,142,131,0.12)'}));
  tl.appendChild(el2('path',{d,fill:'none',stroke:'#3f8e83','stroke-width':1.6}));
  FRAMES.forEach((f,i)=>{const c=el2('circle',{cx:tlx(i),cy:tly(f.occ),r:2.4,fill:'#3f8e83',cursor:'pointer'});c.addEventListener('click',()=>{stop();setFrame(i);});tl.appendChild(c);});
  tl.appendChild(el2('line',{id:'tlmark',x1:0,y1:PAD,x2:0,y2:TH-PAD,stroke:'#b88a3e','stroke-width':1.4}));})();
function updateMarker(){const m=document.getElementById('tlmark');const x=tlx(cur);m.setAttribute('x1',x);m.setAttribute('x2',x);}

const gv=document.getElementById('gridview');
function buildGrid(){gv.innerHTML='';
  FRAMES.forEach((fr,idx)=>{
    const cell=document.createElement('div');cell.className='mini'+(idx===cur?' cur':'');
    const cv=document.createElement('canvas');cv.width=200;cv.height=200;
    const ctx=cv.getContext('2d'),cx=100,cy=100,rad=88;
    ctx.fillStyle='#f7f8fa';ctx.fillRect(0,0,200,200);
    ctx.strokeStyle='#c2c7d0';ctx.setLineDash([3,3]);ctx.beginPath();ctx.arc(cx,cy,(90-EL_MIN)/90*rad,0,7);ctx.stroke();ctx.setLineDash([]);
    for(let i=0;i<N_RAYS;i+=2){const h=st(idx,i);const r=(90-RAYS[i].el)/90*rad,a=(90+RAYS[i].az)*D2R;
      ctx.fillStyle=h===1?'#d15a52':h===2?'#c8cdd5':'#5277ac';ctx.beginPath();ctx.arc(cx+r*Math.cos(a),cy-r*Math.sin(a),h===1?1.6:1.2,0,7);ctx.fill();}
    cell.appendChild(cv);
    const lab=document.createElement('div');lab.className='ml';lab.innerHTML=fr.label+' · <span class="mo">'+fr.occ.toFixed(2)+'</span>';cell.appendChild(lab);
    cell.addEventListener('click',()=>{setMode('single');setFrame(idx);});gv.appendChild(cell);});
}
function setMode(m){const g=m==='grid';
  document.getElementById('stage').classList.toggle('hidden',g);
  document.getElementById('gridview').classList.toggle('on',g);
  document.getElementById('singleCtl').style.display=g?'none':'block';
  document.getElementById('modeSingle').classList.toggle('on',!g);
  document.getElementById('modeGrid').classList.toggle('on',g);
  if(g){stop();buildGrid();}}
document.getElementById('modeSingle').onclick=()=>setMode('single');
document.getElementById('modeGrid').onclick=()=>setMode('grid');

const slider=document.getElementById('slider'),num=document.getElementById('num');
slider.max=N_FRAMES-1;num.max=N_FRAMES-1;
slider.addEventListener('input',()=>setFrame(+slider.value));
num.addEventListener('input',()=>setFrame(+num.value));
document.getElementById('prev').onclick=()=>{stop();setFrame(cur-1);};
document.getElementById('next').onclick=()=>{stop();setFrame(cur+1);};
document.getElementById('peak').onclick=()=>{stop();setFrame(peakIdx);};
function play(){playing=true;document.getElementById('play').textContent='⏸ 停止';timer=setInterval(()=>setFrame(cur>=N_FRAMES-1?0:cur+1),420);}
function stop(){playing=false;document.getElementById('play').textContent='▶ 再生';clearInterval(timer);}
document.getElementById('play').onclick=()=>playing?stop():play();

const M=DATA.meta;
document.getElementById('dumptag').textContent=M.dump+' / '+N_FRAMES+'フレーム';
document.getElementById('elminlab').textContent=M.el_min;
document.getElementById('rangev').textContent='occ '+occMin.toFixed(2)+'–'+occMax.toFixed(2);
document.getElementById('prov').textContent='クリックでジャンプ。N_rays='+M.n_rays+', dist='+M.min_dist+'–'+M.max_dist+'m, angle='+M.angle+'°, mask='+M.el_min+'°, north_az='+M.north_az+'°  ('+M.generated+' 生成)';
setFrame(0);
</script>
'''


def main():
    p = argparse.ArgumentParser(description="遮蔽スカイマップ・ブラウザHTML生成")
    p.add_argument("dump_dir")
    p.add_argument("--out", default=None, help="出力HTML（省略時 skymap_<dump>.html）")
    p.add_argument("--north-az", type=float, default=0.0,
                   help="LiDAR座標で北を指す方位角[deg]。図の上(N)へ回す")
    p.add_argument("--stride", type=int, default=1, help="フレーム間引き(例 2で半分)")
    p.add_argument("--max-frames", type=int, default=0, help="先頭Nフレームに制限(0=全部)")
    p.add_argument("--n-rays", type=int, default=DEFAULT_N_RAYS)
    p.add_argument("--max-dist", type=float, default=DEFAULT_MAX_DIST)
    p.add_argument("--min-dist", type=float, default=DEFAULT_MIN_DIST)
    p.add_argument("--el-min", type=float, default=DEFAULT_EL_MIN_DEG)
    p.add_argument("--angle", type=float, default=DEFAULT_ANGLE_DEG)
    args = p.parse_args()

    print(f"dump を走査して遮蔽データを計算中... ({args.dump_dir})")
    data = build_data(args.dump_dir, n_rays=args.n_rays, max_dist=args.max_dist,
                      min_dist=args.min_dist, el_min=args.el_min, angle=args.angle,
                      north_az=args.north_az, stride=args.stride,
                      max_frames=(args.max_frames or None))

    out = args.out or f"skymap_{data['meta']['dump']}.html"
    html = TEMPLATE.replace("__DATA__", json.dumps(data, ensure_ascii=False, separators=(",", ":")))
    with open(out, "w", encoding="utf-8") as f:
        f.write(html)

    occs = [fr["occ"] for fr in data["frames"]]
    size_kb = os.path.getsize(out) / 1024
    print(f"\n生成: {out}  ({size_kb:.0f} KB, {len(data['frames'])}フレーム)")
    print(f"  遮蔽率レンジ: {min(occs):.3f}–{max(occs):.3f}")
    print(f"  ブラウザで開く: xdg-open {out}  （ヘッドレスならホストにコピーして開く）")


if __name__ == "__main__":
    main()
