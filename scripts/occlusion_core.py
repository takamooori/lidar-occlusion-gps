#!/usr/bin/env python3
"""
occlusion_core.py - 遮蔽率計算コアエンジン
=============================================
フィボナッチ格子レイキャスティングによる上半球遮蔽率を計算する。

従来コードとの違い:
  - 「どのレイがどの点群に当たったか」を OcclusionResult として保持する。
    これにより可視化で「遮蔽認定された部分が点群のどこか」を追跡できる。
  - データソースから渡される points_lidar が LiDARローカル座標であることを前提。
  - 距離閾値・角度閾値・仰角閾値・近傍除去をパラメータ化。

アルゴリズム（案A）:
  上半球(Z>0)の各点を方向ベクトル化し、フィボナッチ格子の各レイに対して
  「角度閾値以内 かつ 有効距離帯 かつ 仰角閾値以上」の点が1つでもあれば、
  そのレイは遮蔽と認定。
  遮蔽率 = 遮蔽レイ数 / 全レイ数。

  有効距離帯  : MIN_DIST < dist < MAX_DIST （近傍除去＋遠方カット）
  仰角閾値    : elevation > EL_MIN_DEG （GPS mask角に対応）

  MIN_DIST    : 1.5m が標準。近距離の通行人・自己反射を遮蔽物として誤認識しない
  EL_MIN_DEG  : 15° が標準。GPS受信機の典型的なマスク角(10〜15°)に対応。
                受信機が使わない低仰角の衛星方向の遮蔽を計算しても意味がないため。
"""

import numpy as np
from dataclasses import dataclass


# デフォルトパラメータ
DEFAULT_N_RAYS     = 1000
DEFAULT_MAX_DIST   = 30.0
DEFAULT_MIN_DIST   = 1.5     # 近傍除去[m]（人間・自己反射対策）
DEFAULT_EL_MIN_DEG = 15.0    # 仰角カットオフ[度]（GPS mask角に対応）
DEFAULT_ANGLE_DEG  = 5.0     # レイと点の許容角度（cos換算で判定）


def fibonacci_hemisphere(n):
    """フィボナッチ格子で上半球にレイ方向ベクトルを生成。"""
    golden = (1 + np.sqrt(5)) / 2
    rays = []
    for i in range(n):
        theta = np.arccos(1 - (i + 0.5) / n)
        phi = 2 * np.pi * i / golden
        x = np.sin(theta) * np.cos(phi)
        y = np.sin(theta) * np.sin(phi)
        z = np.cos(theta)
        if z >= 0:
            rays.append([x, y, z])
    return np.array(rays)


@dataclass
class OcclusionResult:
    """遮蔽率計算の全結果。可視化・検証のために中間情報も保持。"""
    occlusion_rate: float
    rays: np.ndarray                 # (R, 3) 全レイ方向
    ray_hit_mask: np.ndarray         # (R,) bool: 各レイが遮蔽されたか
    ray_hit_point_idx: np.ndarray    # (R,) int: 各レイに当たった代表点のindex(-1=miss)
    pts_upper: np.ndarray            # (Nu, 3) 上半球点（Z>0、lidar座標、表示用文脈）
    pts_lower: np.ndarray            # (Nl, 3) 下半球点（表示用文脈）
    pts_upper_in_range: np.ndarray   # (Nu,) bool: 距離帯+仰角フィルタを通過したか（遮蔽計算対象）
    n_rays: int
    max_dist: float
    min_dist: float                  # 新規
    el_min_deg: float                # 新規
    angle_deg: float

    @property
    def hit_rays(self):
        return self.rays[self.ray_hit_mask]

    @property
    def miss_rays(self):
        return self.rays[~self.ray_hit_mask]

    @property
    def n_hit(self):
        return int(self.ray_hit_mask.sum())

    @property
    def hit_point_indices(self):
        """実際にレイに当たった上半球点のindex集合（重複除去）。"""
        idx = self.ray_hit_point_idx[self.ray_hit_mask]
        return np.unique(idx[idx >= 0])


def compute_occlusion(points_lidar, rays=None,
                      n_rays=DEFAULT_N_RAYS,
                      max_dist=DEFAULT_MAX_DIST,
                      min_dist=DEFAULT_MIN_DIST,
                      el_min_deg=DEFAULT_EL_MIN_DEG,
                      angle_deg=DEFAULT_ANGLE_DEG,
                      track_hits=True):
    """
    遮蔽率を計算して OcclusionResult を返す。

    points_lidar : (N,3) LiDARローカル座標の点群
    rays         : 事前生成したレイ（省略時は内部生成）
    max_dist     : 距離上限[m]。これより遠い点は遮蔽計算から除外
    min_dist     : 距離下限[m]。これより近い点は人間/自己反射として遮蔽計算から除外
    el_min_deg   : 仰角下限[度]。これより低い点は GPS mask角相当として遮蔽計算から除外
    angle_deg    : レイと点の許容角度[度]
    track_hits   : True なら各レイの当たり点indexを記録（可視化用）
    """
    if rays is None:
        rays = fibonacci_hemisphere(n_rays)
    R = len(rays)
    cos_thresh = np.cos(np.deg2rad(angle_deg))
    sin_el_thresh = np.sin(np.deg2rad(el_min_deg))

    pts = np.asarray(points_lidar, dtype=np.float64)
    # pts_upper / pts_lower は「表示用」の文脈点群（Z>0 / Z<=0 で分けるだけ）
    # 遮蔽計算で使うのは pts_upper のうち in_range を満たすもの
    upper_mask = pts[:, 2] > 0
    pts_upper = pts[upper_mask]
    pts_lower = pts[~upper_mask]

    ray_hit_mask = np.zeros(R, dtype=bool)
    ray_hit_point_idx = np.full(R, -1, dtype=int)

    if len(pts_upper) == 0:
        in_range = np.zeros(0, dtype=bool)
        return OcclusionResult(
            occlusion_rate=0.0, rays=rays, ray_hit_mask=ray_hit_mask,
            ray_hit_point_idx=ray_hit_point_idx, pts_upper=pts_upper,
            pts_lower=pts_lower, pts_upper_in_range=in_range,
            n_rays=R, max_dist=max_dist, min_dist=min_dist,
            el_min_deg=el_min_deg, angle_deg=angle_deg,
        )

    dists = np.linalg.norm(pts_upper, axis=1)
    safe_dists = np.maximum(dists, 1e-6)   # 原点に近すぎる点でのゼロ割回避

    # 遮蔽計算の有効点マスク
    #   距離帯  : min_dist < dist < max_dist
    #   仰角    : Z / dist > sin(el_min_deg)
    dist_mask = (dists > min_dist) & (dists < max_dist)
    el_mask   = (pts_upper[:, 2] / safe_dists) > sin_el_thresh
    in_range  = dist_mask & el_mask    # 遮蔽計算対象マスク

    dirs = pts_upper / safe_dists[:, np.newaxis]

    for r_idx, ray in enumerate(rays):
        dot = dirs @ ray
        mask = (dot > cos_thresh) & in_range
        if np.any(mask):
            ray_hit_mask[r_idx] = True
            if track_hits:
                # 最も近い当たり点を代表として記録
                cand = np.where(mask)[0]
                ray_hit_point_idx[r_idx] = cand[np.argmin(dists[cand])]

    rate = ray_hit_mask.sum() / R
    return OcclusionResult(
        occlusion_rate=float(rate), rays=rays, ray_hit_mask=ray_hit_mask,
        ray_hit_point_idx=ray_hit_point_idx, pts_upper=pts_upper,
        pts_lower=pts_lower, pts_upper_in_range=in_range,
        n_rays=R, max_dist=max_dist, min_dist=min_dist,
        el_min_deg=el_min_deg, angle_deg=angle_deg,
    )


def azimuth_elevation(vec):
    """方向ベクトル -> (方位角[deg, 0=+X 反時計回り], 仰角[deg])。スカイマップ用。"""
    x, y, z = vec[..., 0], vec[..., 1], vec[..., 2]
    az = np.degrees(np.arctan2(y, x))
    el = np.degrees(np.arcsin(np.clip(z, -1, 1)))
    return az, el
