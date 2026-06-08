#!/usr/bin/env python3
"""
verify_viz.py - 遮蔽検証ビジュアライザ
========================================
「どのレイが遮蔽認定され、それが点群全体のどこに当たったか」を
全体と対応づけて確認するための可視化。
 
従来 visualize_frame.py との違い:
  - 上半球点を「遮蔽に寄与した点 / していない点」で色分けして全体表示。
  - レイと、そのレイが当たった点を線で結んで対応を明示。
  - スカイマップ（方位-仰角の極座標）で遮蔽方向の分布を俯瞰。
  これにより「認識された部分だけでなく全体を比較した上で確認」できる。
 
単体実行:
  python3 verify_viz.py <dump_dir> <frame_folder> [--max-dist 30] [--save out.png]
"""
 
import os
import sys
import argparse
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa
 
from data_source import make_source
from occlusion_core import compute_occlusion, azimuth_elevation
 
 
def build_verification_figure(frame, result, title_suffix=""):
    """
    4パネル構成の検証Figureを返す。
      左上: 上半球点群全体（遮蔽寄与点を強調色 / 非寄与点を淡色）
      右上: レイキャスト結果（遮蔽レイ / 開放レイ）+ 当たり点への対応線
      左下: スカイマップ（遮蔽方向の方位-仰角分布）
      右下: 距離ヒストグラム + サマリテキスト
    """
    fig = plt.figure(figsize=(16, 10))
    fig.patch.set_facecolor('#1e1e1e')
 
    pts_upper = result.pts_upper
    pts_lower = result.pts_lower
    in_range = result.pts_upper_in_range
    hit_idx = result.hit_point_indices  # 遮蔽に寄与した上半球点index
 
    contrib_mask = np.zeros(len(pts_upper), dtype=bool)
    if len(hit_idx) > 0:
        contrib_mask[hit_idx] = True
 
    # ---------- 左上: 点群全体 + 遮蔽寄与強調 ----------
    ax1 = fig.add_subplot(221, projection='3d')
    ax1.set_facecolor('#2d2d2d')
    # 非寄与の上半球点（淡いグレー）
    non = pts_upper[~contrib_mask]
    if len(non) > 0:
        s = max(1, len(non) // 3000)
        ax1.scatter(non[::s, 0], non[::s, 1], non[::s, 2],
                    c='#5a6a7a', s=1, alpha=0.4, label=f'上半球 非遮蔽({len(non)})')
    # 遮蔽に寄与した点（赤系・強調）
    con = pts_upper[contrib_mask]
    if len(con) > 0:
        ax1.scatter(con[:, 0], con[:, 1], con[:, 2],
                    c='#ff5050', s=8, alpha=0.9, label=f'遮蔽寄与点({len(con)})')
    # 下半球（参考）
    if len(pts_lower) > 0:
        s2 = max(1, len(pts_lower) // 2000)
        ax1.scatter(pts_lower[::s2, 0], pts_lower[::s2, 1], pts_lower[::s2, 2],
                    c='#3a3a3a', s=0.5, alpha=0.2)
    ax1.scatter(0, 0, 0, c='yellow', s=60, marker='^', label='LiDAR')
    ax1.set_title('点群全体と遮蔽寄与箇所', color='white', fontsize=10)
    _style_3d(ax1)
 
    # ---------- 右上: レイ + 当たり点対応線 ----------
    ax2 = fig.add_subplot(222, projection='3d')
    ax2.set_facecolor('#2d2d2d')
    scale = min(result.max_dist, 15.0)
    hit_rays = result.hit_rays
    miss_rays = result.miss_rays
    if len(miss_rays) > 0:
        mr = miss_rays[::3]
        ax2.quiver(0, 0, 0, mr[:, 0]*scale, mr[:, 1]*scale, mr[:, 2]*scale,
                   color='#4a90d0', alpha=0.15, linewidth=0.5,
                   label=f'開放({len(miss_rays)})')
    if len(hit_rays) > 0:
        hr = hit_rays[::2]
        ax2.quiver(0, 0, 0, hr[:, 0]*scale, hr[:, 1]*scale, hr[:, 2]*scale,
                   color='#ff5050', alpha=0.5, linewidth=0.7,
                   label=f'遮蔽({len(hit_rays)})')
    # 当たり点への対応線（間引いて描画）
    sample_rays = np.where(result.ray_hit_mask)[0][::8]
    for r_idx in sample_rays:
        p_idx = result.ray_hit_point_idx[r_idx]
        if p_idx >= 0:
            p = pts_upper[p_idx]
            ax2.plot([0, p[0]], [0, p[1]], [0, p[2]],
                     color='#ffd050', alpha=0.4, linewidth=0.6)
    ax2.scatter(0, 0, 0, c='yellow', s=60, marker='^')
    ax2.set_title('レイ遮蔽判定と当たり点対応', color='white', fontsize=10)
    _style_3d(ax2)
 
    # ---------- 左下: スカイマップ ----------
    ax3 = fig.add_subplot(223, projection='polar')
    ax3.set_facecolor('#2d2d2d')
    az_all, el_all = azimuth_elevation(result.rays)
    # 極座標: 角度=方位, 半径=天頂角(90-仰角) 中心が天頂
    r_all = 90 - el_all
    theta_all = np.deg2rad(az_all)
    hit = result.ray_hit_mask
    ax3.scatter(theta_all[~hit], r_all[~hit], c='#4a90d0', s=6, alpha=0.5, label='開放')
    ax3.scatter(theta_all[hit], r_all[hit], c='#ff5050', s=12, alpha=0.8, label='遮蔽')
    ax3.set_ylim(0, 90)
    ax3.set_title('スカイマップ（中心=天頂, 外周=地平線）',
                  color='white', fontsize=10, pad=15)
    ax3.tick_params(colors='white', labelsize=7)
    ax3.legend(loc='lower right', fontsize=7, facecolor='#2d2d2d', labelcolor='white')
 
    # ---------- 右下: 距離ヒストグラム + サマリ ----------
    ax4 = fig.add_subplot(224)
    ax4.set_facecolor('#2d2d2d')
    if len(pts_upper) > 0:
        d = np.linalg.norm(pts_upper, axis=1)
        ax4.hist(d, bins=40, color='#5a9ad0', alpha=0.7, edgecolor='#1e1e1e')
        ax4.axvline(result.max_dist, color='#ff5050', linestyle='--',
                    linewidth=1.5, label=f'MAX_DIST={result.max_dist:.0f}m')
        ax4.set_xlabel('距離 [m]', color='white', fontsize=9)
        ax4.set_ylabel('上半球点数', color='white', fontsize=9)
        ax4.legend(fontsize=8, facecolor='#2d2d2d', labelcolor='white')
    ax4.tick_params(colors='white', labelsize=8)
    ax4.set_title('上半球点の距離分布', color='white', fontsize=10)
    for spine in ax4.spines.values():
        spine.set_color('#555555')
 
    # サマリテキスト
    summary = (
        f"フレーム: {frame.folder}   stamp: "
        f"{frame.stamp:.2f}\n" if frame.stamp else f"フレーム: {frame.folder}\n"
    )
    summary += (
        f"遮蔽率: {result.occlusion_rate:.3f}  "
        f"({result.n_hit}/{result.n_rays} レイ)\n"
        f"上半球点: {len(pts_upper)}  "
        f"うち閾値内: {int(in_range.sum())}  "
        f"遮蔽寄与: {len(hit_idx)}\n"
        f"座標系: {frame.coord_note}"
    )
    fig.text(0.5, 0.04, summary, color='#dddddd', fontsize=9,
             ha='center', va='bottom', family='monospace',
             bbox=dict(facecolor='#2d2d2d', edgecolor='#555555', pad=8))
 
    fig.suptitle(f'遮蔽検証 {frame.folder} {title_suffix}',
                 color='white', fontsize=14, y=0.98)
    fig.subplots_adjust(top=0.92, bottom=0.12, hspace=0.25, wspace=0.2)
    return fig
 
 
def _style_3d(ax):
    ax.set_xlabel('X', color='white', fontsize=7)
    ax.set_ylabel('Y', color='white', fontsize=7)
    ax.set_zlabel('Z', color='white', fontsize=7)
    ax.tick_params(colors='white', labelsize=6)
    ax.legend(fontsize=6, facecolor='#2d2d2d', labelcolor='white', loc='upper left')
 
 
def main():
    parser = argparse.ArgumentParser(description="遮蔽検証ビジュアライザ")
    parser.add_argument('dump_dir')
    parser.add_argument('frame', help='フレームフォルダ名 (例: 000050)')
    parser.add_argument('--max-dist', type=float, default=30.0)
    parser.add_argument('--n-rays', type=int, default=1000)
    parser.add_argument('--save', default=None)
    args = parser.parse_args()
 
    src = make_source("dump", dump_dir=args.dump_dir)
    frame = src.get_frame(args.frame)
    if frame is None:
        print(f"フレーム {args.frame} を読めません")
        sys.exit(1)
 
    result = compute_occlusion(frame.points_lidar,
                               n_rays=args.n_rays, max_dist=args.max_dist)
    print(f"遮蔽率: {result.occlusion_rate:.4f} ({result.n_hit}/{result.n_rays})")
    print(f"座標系メモ: {frame.coord_note}")
 
    fig = build_verification_figure(frame, result)
    if args.save:
        fig.savefig(args.save, dpi=150, facecolor=fig.get_facecolor())
        print(f"保存: {args.save}")
    else:
        plt.show()
 
 
if __name__ == '__main__':
    main()

