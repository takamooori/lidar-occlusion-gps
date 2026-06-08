#!/usr/bin/env python3
"""
skymap_viz.py - 遮蔽スカイマップ エンジン（真上から見た半球投影）
=================================================================
3次元の上半球レイを「真上から見た2次元の円板」に投影し、
遮蔽認定レイを赤・開空を青・mask外(仰角<el_min)を灰で描く。
 
設計方針（このファイルはエンジン。色やラベルの微調整はノートブック側で）:
  - 見た目の調整は すべて SkymapStyle 1つに集約。build_skymap(frame, result, style)。
  - フレーム選択・遮蔽計算・描画関数を提供。重い処理（全フレームスキャン）と
    描画を分離してあるので、ノートブックでは「読み込み1回 → スタイル変更して
    描画セルだけ再実行」で即確認できる。
 
データの流れ（既存資産を再利用）:
  data_source.make_source → get_frame → occlusion_core.compute_occlusion
  → OcclusionResult(rays, ray_hit_mask, ...) を build_skymap で描画。
 
CLI 例:
  python3 skymap_viz.py <dump_dir>                 # 最大遮蔽フレームを自動選択
  python3 skymap_viz.py <dump_dir> --target 0.6    # occ≈0.6 に最も近いフレーム
  python3 skymap_viz.py <dump_dir> --frame 000087  # 直接指定（スキャン省略で高速）
  python3 skymap_viz.py <dump_dir> --rank-only --top 15
"""
 
import os
import sys
import argparse
from dataclasses import dataclass, field
from typing import Optional, Tuple
 
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
 
from data_source import make_source
from occlusion_core import (
    compute_occlusion, fibonacci_hemisphere, azimuth_elevation,
)
from jp_font import setup_japanese_font, L
 
setup_japanese_font()
 
 
# ======================================================================
#  スタイル設定 — 見た目の調整はここだけ触れば良い
# ======================================================================
@dataclass
class SkymapStyle:
    """スカイマップの見た目を一元管理する。build_skymap に渡す。"""
 
    # --- 配色（参照イメージ(b)の凡例に合わせた既定）---
    c_hit:    str = "#d83a3a"   # 遮蔽（ヒット）
    c_miss:   str = "#3f6fc4"   # 開空（ミス）
    c_mask:   str = "#bdbdbd"   # mask外（仰角<el_min）
    c_ring:   str = "#777777"   # 地平線・グリッド
    c_accent: str = "#2e8b57"   # サマリボックスの縁
    bg:       str = "white"     # 背景
 
    # --- 点サイズ・透明度 ---
    s_hit:  float = 22.0
    s_miss: float = 12.0
    s_mask: float = 9.0
    a_hit:  float = 0.92
    a_miss: float = 0.70
    a_mask: float = 0.55
    hit_edge: str = "white"     # 遮蔽点の縁取り（""で無し）
 
    # --- 図サイズ ---
    figsize: Tuple[float, float] = (7.6, 8.2)
    dpi: int = 220
 
    # --- 向き ---
    north_az_deg: float = 0.0       # この方位(LiDAR座標)を図の上(N)へ回転
    theta_direction: int = 1        # 1=CCW(+Y左/−Y右の鳥瞰) / -1=左右反転
    # コンパスラベル（上→右→下→左 の順）。前右後左にするなら ("前","右","後","左")
    compass_labels: Tuple[str, str, str, str] = ("N", "E", "S", "W")
 
    # --- 表示トグル ---
    show_compass:   bool = True
    show_mask_ring: bool = True
    show_el_labels: bool = True     # 仰角の同心円ラベル(0/30/60/90°)
    show_legend:    bool = True
    show_summary:   bool = True
 
    # --- 文言（None=自動生成 / 文字列で上書き）---
    title:           Optional[str] = None
    mask_ring_label: Optional[str] = None   # None→「mask角=○°」
    legend_hit:      Optional[str] = None
    legend_miss:     Optional[str] = None
    legend_mask:     Optional[str] = None
 
    # --- フォントサイズ ---
    title_size:   float = 13.5
    legend_size:  float = 8.5
    compass_size: float = 13.0
    summary_size: float = 10.0
 
 
# ======================================================================
#  フレーム選択
# ======================================================================
def rank_frames(source, n_rays=1000, max_dist=30.0, min_dist=1.5,
                el_min_deg=15.0, angle_deg=5.0, verbose=True):
    """全フレームの遮蔽率を計算し (folder, occ, stamp) を降順で返す。"""
    rays = fibonacci_hemisphere(n_rays)
    folders = source.list_frames()
    rows = []
    for i, folder in enumerate(folders):
        frame = source.get_frame(folder)
        if frame is None:
            continue
        res = compute_occlusion(frame.points_lidar, rays=rays,
                                max_dist=max_dist, min_dist=min_dist,
                                el_min_deg=el_min_deg, angle_deg=angle_deg,
                                track_hits=False)
        rows.append((folder, res.occlusion_rate, frame.stamp))
        if verbose:
            print(f"\r  スキャン {i+1}/{len(folders)} "
                  f"({folder}: occ={res.occlusion_rate:.3f})",
                  end="", flush=True)
    if verbose:
        print()
    rows.sort(key=lambda r: r[1], reverse=True)
    return rows
 
 
def select_frame(ranking, target=None):
    """target=None なら最大遮蔽、指定なら occ が最も近いフレームを選ぶ。"""
    if not ranking:
        return None
    if target is None:
        return ranking[0][0]
    return min(ranking, key=lambda r: abs(r[1] - target))[0]
 
 
# ======================================================================
#  描画
# ======================================================================
def build_skymap(frame, result, style: Optional[SkymapStyle] = None):
    """
    真上から見た半球投影の単独Figureを返す。
      中心 = 天頂(仰角90°)、外周 = 地平線(仰角0°)、半径 = 90 − 仰角
      遮蔽=赤 / 開空=青 / mask外(仰角<el_min)=灰
    見た目は style(SkymapStyle) で制御。None なら既定スタイル。
    """
    if style is None:
        style = SkymapStyle()
    s = style
 
    az, el = azimuth_elevation(result.rays)        # az: +X=0,CCW / el: 仰角
    r = 90.0 - el                                  # 天頂角(中心0, 地平線90)
    theta = np.deg2rad(az - s.north_az_deg)        # north_az を上(N)へ
 
    el_min = result.el_min_deg
    masked = el < el_min
    hit = result.ray_hit_mask & ~masked
    miss = (~result.ray_hit_mask) & ~masked
 
    fig = plt.figure(figsize=s.figsize)
    fig.patch.set_facecolor(s.bg)
    ax = fig.add_subplot(111, projection="polar")
    ax.set_facecolor(s.bg)
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(s.theta_direction)
    ax.set_ylim(0, 90)
 
    # --- 同心円グリッド ---
    ax.set_rgrids([30, 60], labels=["", ""])
    ax.grid(color=s.c_ring, alpha=0.25, linewidth=0.7)
    if s.show_el_labels:
        for el_lab, rr in [(90, 0), (60, 30), (30, 60), (0, 90)]:
            ax.text(np.deg2rad(33), rr, f"{el_lab}°",
                    color="#999999", fontsize=7, ha="center", va="center")
 
    # --- mask角リング ---
    if s.show_mask_ring:
        ring = np.linspace(0, 2 * np.pi, 200)
        ax.plot(ring, np.full_like(ring, 90 - el_min),
                color=s.c_mask, linestyle="--", linewidth=1.3,
                alpha=0.9, zorder=2)
        ring_lab = s.mask_ring_label or L(f"mask角={el_min:.0f}°",
                                          f"mask={el_min:.0f}°")
        ax.text(np.deg2rad(135), 90 - el_min + 1.5, ring_lab,
                color="#888888", fontsize=8, ha="center")
 
    # --- 点群（mask外 → 開空 → 遮蔽 の順に重ねる）---
    lab_mask = s.legend_mask or L(f"mask外 仰角<{el_min:.0f}° ({masked.sum()})",
                                  f"masked <{el_min:.0f}° ({masked.sum()})")
    lab_miss = s.legend_miss or L(f"開空 ({miss.sum()})", f"open ({miss.sum()})")
    lab_hit  = s.legend_hit  or L(f"遮蔽 ({hit.sum()})", f"occluded ({hit.sum()})")
    if masked.any():
        ax.scatter(theta[masked], r[masked], s=s.s_mask, c=s.c_mask,
                   alpha=s.a_mask, edgecolors="none", zorder=3, label=lab_mask)
    if miss.any():
        ax.scatter(theta[miss], r[miss], s=s.s_miss, c=s.c_miss,
                   alpha=s.a_miss, edgecolors="none", zorder=4, label=lab_miss)
    if hit.any():
        ax.scatter(theta[hit], r[hit], s=s.s_hit, c=s.c_hit, alpha=s.a_hit,
                   edgecolors=(s.hit_edge or "none"),
                   linewidths=0.3 if s.hit_edge else 0,
                   zorder=5, label=lab_hit)
 
    # --- コンパスラベル（上→右→下→左 = 角度 0/270/180/90）---
    if s.show_compass:
        for lab, deg in zip(s.compass_labels, (0, 270, 180, 90)):
            ax.text(np.deg2rad(deg), 98, lab, color="#333333",
                    fontsize=s.compass_size, fontweight="bold",
                    ha="center", va="center")
 
    ax.set_xticklabels([])
    ax.tick_params(colors="#999999", labelsize=7)
    for spine in ax.spines.values():
        spine.set_color(s.c_ring)
        spine.set_alpha(0.5)
 
    # --- タイトル ---
    title = s.title or L("遮蔽スカイマップ（中心=天頂 / 外周=地平線）",
                         "Occlusion sky map (center=zenith / edge=horizon)")
    fig.suptitle(title, fontsize=s.title_size, fontweight="bold", y=0.985)
 
    # --- サマリ ---
    if s.show_summary:
        occ = result.occlusion_rate
        stamp_s = f"  stamp={frame.stamp:.2f}" if frame.stamp else ""
        if abs(s.north_az_deg) < 1e-6:
            north_note = L("基準: +X=ロボット前方=N",
                           "ref: +X=robot front=N")
        else:
            north_note = L(f"基準: north-az={s.north_az_deg:.0f}° を N に回転",
                           f"ref: north-az={s.north_az_deg:.0f} rotated to N")
        summary = (
            L(f"フレーム {frame.folder}{stamp_s}",
              f"frame {frame.folder}{stamp_s}") + "\n"
            + L(f"遮蔽率 occ = {occ:.3f}   ({result.n_hit} / {result.n_rays} レイ)",
                f"occ = {occ:.3f}   ({result.n_hit} / {result.n_rays} rays)")
            + "\n" + north_note
        )
        fig.text(0.5, 0.045, summary, ha="center", va="bottom",
                 fontsize=s.summary_size, linespacing=1.5,
                 bbox=dict(facecolor="#f4faf4", edgecolor=s.c_accent,
                           linewidth=1.2, boxstyle="round,pad=0.5"))
 
    if s.show_legend:
        ax.legend(loc="upper left", bbox_to_anchor=(-0.10, 1.0),
                  fontsize=s.legend_size, frameon=True, facecolor="white",
                  edgecolor="#cccccc", framealpha=0.95)
 
    fig.subplots_adjust(top=0.90, bottom=0.13, left=0.06, right=0.94)
    return fig
 
 
# ======================================================================
#  まとめて使うヘルパ（ノートブックから呼ぶと便利）
# ======================================================================
def load_frame_result(dump_dir, frame=None, target=None,
                      n_rays=1000, max_dist=30.0, min_dist=1.5,
                      el_min_deg=15.0, angle_deg=5.0, verbose=True):
    """
    フレームを選んで (frame, result, ranking) を返す。重い処理はここだけ。
      frame  指定 → スキャンせず直接そのフレーム
      target 指定 → occ がそれに最も近いフレーム
      どちらも None → 最大遮蔽フレーム
    """
    src = make_source("dump", dump_dir=dump_dir)
    kw = dict(n_rays=n_rays, max_dist=max_dist, min_dist=min_dist,
              el_min_deg=el_min_deg, angle_deg=angle_deg)
    ranking = None
    if frame is None:
        if verbose:
            print("全フレームをスキャン中...")
        ranking = rank_frames(src, verbose=verbose, **kw)
        if not ranking:
            raise RuntimeError("有効なフレームがありません")
        frame = select_frame(ranking, target=target)
    fd = src.get_frame(frame)
    if fd is None:
        raise RuntimeError(f"フレーム {frame} を読めません")
    res = compute_occlusion(fd.points_lidar, **kw)
    return fd, res, ranking
 
 
# ======================================================================
#  CLI
# ======================================================================
def main():
    p = argparse.ArgumentParser(description="遮蔽スカイマップ可視化")
    p.add_argument("dump_dir")
    p.add_argument("--frame", default=None, help="フレーム直接指定（高速）")
    p.add_argument("--target", type=float, default=None,
                   help="この遮蔽率に最も近いフレームを選ぶ（例 0.6）")
    p.add_argument("--top", type=int, default=10, help="ランキング表示件数")
    p.add_argument("--rank-only", action="store_true")
    p.add_argument("--north-az", type=float, default=0.0,
                   help="LiDAR座標で北を指す方位角[deg]。図の上(N)へ回す")
    p.add_argument("--n-rays", type=int, default=1000)
    p.add_argument("--max-dist", type=float, default=30.0)
    p.add_argument("--min-dist", type=float, default=1.5)
    p.add_argument("--el-min", type=float, default=15.0)
    p.add_argument("--angle", type=float, default=5.0)
    p.add_argument("--save", default=None)
    args = p.parse_args()
 
    src = make_source("dump", dump_dir=args.dump_dir)
    kw = dict(n_rays=args.n_rays, max_dist=args.max_dist, min_dist=args.min_dist,
              el_min_deg=args.el_min, angle_deg=args.angle)
 
    if args.frame is not None:
        target_folder = args.frame
    else:
        print("全フレームをスキャンして遮蔽率を評価中...")
        ranking = rank_frames(src, **kw)
        if not ranking:
            print("有効なフレームがありません。"); sys.exit(1)
        print(f"\n===== 遮蔽率ランキング 上位{args.top} =====")
        print(f"{'順位':>4} {'フレーム':>8} {'遮蔽率':>8} {'stamp':>14}")
        for i, (f, occ, st) in enumerate(ranking[:args.top]):
            st_s = f"{st:.2f}" if st else "-"
            print(f"{i+1:>4} {f:>8} {occ:>8.3f} {st_s:>14}")
        target_folder = select_frame(ranking, target=args.target)
        sel_occ = dict((f, o) for f, o, _ in ranking)[target_folder]
        tag = "最大遮蔽" if args.target is None else f"occ≈{args.target} に最近接"
        print(f"\n選択フレーム: {target_folder}  (occ={sel_occ:.3f}, {tag})")
 
    if args.rank_only:
        return
 
    frame = src.get_frame(target_folder)
    if frame is None:
        print(f"フレーム {target_folder} を読めません。"); sys.exit(1)
    result = compute_occlusion(frame.points_lidar, **kw)
    print(f"遮蔽率: {result.occlusion_rate:.4f} "
          f"({result.n_hit}/{result.n_rays})  座標系: {frame.coord_note}")
 
    style = SkymapStyle(north_az_deg=args.north_az)
    fig = build_skymap(frame, result, style=style)
    out = args.save or os.path.join(
        os.path.dirname(os.path.abspath(args.dump_dir.rstrip("/"))) or ".",
        f"skymap_{target_folder}.png")
    fig.savefig(out, dpi=style.dpi, facecolor=fig.get_facecolor(),
                bbox_inches="tight")
    print(f"保存: {out}")
 
 
if __name__ == "__main__":
    main()

