#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sector_occlusion.py
====================
スカラー1個だった遮蔽率を「仰角バンド × 方位セクター」の2次元グリッドに拡張する。

設計方針
--------
- occlusion_core.py と同じレイ（フィボナッチ格子 N=1000）・同じ5ステップフィルタを
  そのまま再現。グローバル遮蔽率(scalar)は従来値と一致する。
- 各レイがどのセル(仰角バンド,方位セクター)に属するかは「レイ方向」だけで決まる
  （格子は固定なので一度だけ計算）。セル遮蔽率 = そのセル内でヒットしたレイの割合。
- エンジン層（本ファイル）と表示層（sector_analysis.ipynb）を分離。
  重い計算は1回だけ実行→CSVへ永続化、グラフ調整は notebook 側で高速反復する。

出力
----
- <out>.csv         : 1行=1フレーム。列 = frame, t, occ_scalar, occ_e{b}_a{s} ...
- <out>.grid.json   : グリッド定義・セルごとのレイ本数・ラベル（CSVを読むためのメタ情報）

使い方
------
  # 自己テスト（彼らのデータ無しで動作確認）
  python sector_occlusion.py --self-test

  # 実データ（dump から全フレーム計算）
  python sector_occlusion.py \
      --dump ~/ros2_ws/dump/nakaniwa_0522 \
      --out  ~/ros2_ws/dump/nakaniwa_0522/analysis/sector_occlusion.csv
"""
from __future__ import annotations

import argparse
import json
import os
from dataclasses import dataclass, field

import numpy as np

# ----------------------------------------------------------------------------
# パラメータ（occlusion_core.py と同一に保つこと）
# ----------------------------------------------------------------------------
N_RAYS = 1000          # フィボナッチ格子の本数
R_MIN = 1.5            # 点群の最小距離 [m]
R_MAX = 30.0           # 点群の最大距離 [m]
ELEV_MIN_DEG = 15.0    # 点群の仰角ゲート [deg]（これ未満の点は障害物とみなさない）
HIT_ANG_DEG = 5.0      # ヒット判定の角度しきい値 [deg]

GOLDEN = (1.0 + 5 ** 0.5) / 2.0  # 黄金比

# 8セクターの標準コンパスラベル
_COMPASS_8 = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
_COMPASS_4 = ["N", "E", "S", "W"]
_COMPASS_16 = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
               "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]


# ----------------------------------------------------------------------------
# レイ（フィボナッチ格子・上半球）
# ----------------------------------------------------------------------------
def fibonacci_hemisphere(n: int = N_RAYS) -> np.ndarray:
    """上半球の単位ベクトル [n,3]。立体角が均一になるよう z を等間隔に取る。
    occlusion_core.py の格子定義と同じ。"""
    i = np.arange(n)
    z = 1.0 - (i + 0.5) / n                 # z = cos(天頂角) = sin(仰角), (0,1]
    r = np.sqrt(np.clip(1.0 - z * z, 0.0, 1.0))
    phi = 2.0 * np.pi * i / GOLDEN           # 方位（格子上の基準系）
    x = r * np.cos(phi)
    y = r * np.sin(phi)
    return np.stack([x, y, z], axis=1)


# ----------------------------------------------------------------------------
# グリッド定義
# ----------------------------------------------------------------------------
@dataclass
class SectorGrid:
    """仰角バンド × 方位セクターのビン定義。

    elev_edges_deg : 仰角(地平=0, 天頂=90)の境界。例 [15,30,45,60,90] → 4バンド。
    n_azimuth      : 方位セクター数（4/8/16 推奨）。
    azimuth_offset_deg, clockwise:
        レイ方位→コンパス方位の対応。既定は +X=東 / +Y=北 (ENU) を仮定。
        skymap_viz.py の NESW 表示と回転がズレる場合はここで調整する。
    """
    elev_edges_deg: tuple = (15.0, 30.0, 45.0, 60.0, 90.0)
    n_azimuth: int = 8
    azimuth_offset_deg: float = 0.0
    clockwise: bool = True

    # 内部キャッシュ
    _ray_dirs: np.ndarray = field(default=None, repr=False)
    _band_idx: np.ndarray = field(default=None, repr=False)
    _sect_idx: np.ndarray = field(default=None, repr=False)
    _valid: np.ndarray = field(default=None, repr=False)

    # --- レイ角度 -----------------------------------------------------------
    def _bearings_deg(self, dirs: np.ndarray) -> np.ndarray:
        """単位ベクトル→コンパス方位[deg]（0=N, 時計回り）。"""
        ux, uy = dirs[:, 0], dirs[:, 1]
        math_az = np.degrees(np.arctan2(uy, ux))      # +X基準の反時計回り
        bearing = 90.0 - math_az                       # ENU→コンパス(北=0,時計回り)
        if not self.clockwise:
            bearing = -bearing
        bearing = (bearing + self.azimuth_offset_deg) % 360.0
        return bearing

    def assign(self, dirs: np.ndarray):
        """各レイをセル(band,sect)へ割り当て。範囲外は valid=False。"""
        elev = np.degrees(np.arcsin(np.clip(dirs[:, 2], -1.0, 1.0)))  # 仰角
        edges = np.asarray(self.elev_edges_deg, dtype=float)
        band = np.digitize(elev, edges) - 1                 # 0..len(edges)-2
        valid = (band >= 0) & (band < len(edges) - 1)

        bearing = self._bearings_deg(dirs)
        step = 360.0 / self.n_azimuth
        sect = np.floor((bearing + step / 2.0) / step).astype(int) % self.n_azimuth
        band = np.clip(band, 0, len(edges) - 2)
        return band, sect, valid

    def bind(self, dirs: np.ndarray):
        """レイ方向を固定して割り当てをキャッシュ（格子は不変なので1回でよい）。"""
        self._ray_dirs = dirs
        self._band_idx, self._sect_idx, self._valid = self.assign(dirs)
        return self

    # --- 形状・ラベル -------------------------------------------------------
    @property
    def n_bands(self) -> int:
        return len(self.elev_edges_deg) - 1

    @property
    def shape(self) -> tuple:
        return (self.n_bands, self.n_azimuth)

    def sector_labels(self) -> list:
        if self.n_azimuth == 8:
            return list(_COMPASS_8)
        if self.n_azimuth == 4:
            return list(_COMPASS_4)
        if self.n_azimuth == 16:
            return list(_COMPASS_16)
        return [f"A{k}" for k in range(self.n_azimuth)]

    def band_labels(self) -> list:
        e = self.elev_edges_deg
        return [f"E{int(e[b])}-{int(e[b + 1])}" for b in range(self.n_bands)]

    def cell_counts(self) -> np.ndarray:
        """セルごとのレイ本数 [n_bands, n_azimuth]。"""
        cnt = np.zeros(self.shape, dtype=int)
        m = self._valid
        np.add.at(cnt, (self._band_idx[m], self._sect_idx[m]), 1)
        return cnt

    def column_names(self) -> list:
        return [f"occ_e{b}_a{s}"
                for b in range(self.n_bands) for s in range(self.n_azimuth)]

    def to_meta(self) -> dict:
        return {
            "elev_edges_deg": list(self.elev_edges_deg),
            "n_azimuth": self.n_azimuth,
            "azimuth_offset_deg": self.azimuth_offset_deg,
            "clockwise": self.clockwise,
            "n_bands": self.n_bands,
            "band_labels": self.band_labels(),
            "sector_labels": self.sector_labels(),
            "cell_counts": self.cell_counts().tolist(),
            "columns": self.column_names(),
            "params": {"N_RAYS": N_RAYS, "R_MIN": R_MIN, "R_MAX": R_MAX,
                       "ELEV_MIN_DEG": ELEV_MIN_DEG, "HIT_ANG_DEG": HIT_ANG_DEG},
            "note": "occ_e{b}_a{s}: band b は elev_edges[b]..[b+1] deg, sector s は sector_labels[s].",
        }


# ----------------------------------------------------------------------------
# レイキャスト（occlusion_core.py の 5ステップフィルタを再現）
# ----------------------------------------------------------------------------
def raycast_hits(points: np.ndarray, ray_dirs: np.ndarray,
                 r_min: float = R_MIN, r_max: float = R_MAX,
                 elev_min_deg: float = ELEV_MIN_DEG,
                 hit_ang_deg: float = HIT_ANG_DEG,
                 chunk: int = 4000) -> np.ndarray:
    """各レイがヒット(=遮蔽)かを返す bool[N_rays]。

    points : センサ座標系の生スキャン [M,3]（+Z=上）。
    手順:  (1)z>0 (2)1.5<‖p‖<30 (3)仰角>15° (4)レイと点が5°以内ならヒット。
    """
    p = np.asarray(points, dtype=float)
    if p.size == 0:
        return np.zeros(len(ray_dirs), dtype=bool)
    norm = np.linalg.norm(p, axis=1)
    with np.errstate(invalid="ignore", divide="ignore"):
        elev = np.degrees(np.arcsin(np.clip(p[:, 2] / np.where(norm > 0, norm, 1), -1, 1)))
    keep = (p[:, 2] > 0) & (norm > r_min) & (norm < r_max) & (elev > elev_min_deg)
    p = p[keep]
    norm = norm[keep]
    if p.shape[0] == 0:
        return np.zeros(len(ray_dirs), dtype=bool)
    d = p / norm[:, None]                       # 点の単位方向 [M,3]

    cos_thr = np.cos(np.radians(hit_ang_deg))
    hits = np.zeros(len(ray_dirs), dtype=bool)
    # 点をチャンク分割して [N_rays, chunk] の内積を取る（メモリ節約）
    for s in range(0, d.shape[0], chunk):
        block = d[s:s + chunk]                  # [c,3]
        cos_sim = ray_dirs @ block.T            # [N_rays, c]
        hits |= (cos_sim > cos_thr).any(axis=1)
    return hits


@dataclass
class FrameResult:
    grid_occ: np.ndarray     # [n_bands, n_azimuth]  セル遮蔽率（NaN=レイ無し）
    scalar: float            # 全レイ遮蔽率（従来の occlusion_core と一致）
    grid_scalar: float       # グリッド対象レイ(=elev_edges範囲内)のみの遮蔽率


def compute_frame(points: np.ndarray, grid: SectorGrid) -> FrameResult:
    """1フレーム分のセクター遮蔽率を計算。grid は bind() 済みであること。"""
    hits = raycast_hits(points, grid._ray_dirs)
    scalar = float(hits.mean())

    nb, na = grid.shape
    hit_cnt = np.zeros((nb, na))
    tot_cnt = np.zeros((nb, na))
    m = grid._valid
    np.add.at(tot_cnt, (grid._band_idx[m], grid._sect_idx[m]), 1)
    np.add.at(hit_cnt, (grid._band_idx[m], grid._sect_idx[m]), hits[m].astype(float))
    with np.errstate(invalid="ignore", divide="ignore"):
        occ = np.where(tot_cnt > 0, hit_cnt / tot_cnt, np.nan)
    grid_scalar = float(hits[m].mean()) if m.any() else float("nan")
    return FrameResult(grid_occ=occ, scalar=scalar, grid_scalar=grid_scalar)


# ----------------------------------------------------------------------------
# CSV 書き出し
# ----------------------------------------------------------------------------
def write_outputs(rows: list, grid: SectorGrid, out_csv: str):
    """rows = [{'frame':int,'t':float,'result':FrameResult}, ...] を CSV+JSON に。"""
    import csv
    os.makedirs(os.path.dirname(os.path.abspath(out_csv)), exist_ok=True)
    cols = grid.column_names()
    header = ["frame", "t", "occ_scalar", "occ_grid_scalar"] + cols
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        for r in rows:
            res: FrameResult = r["result"]
            flat = res.grid_occ.reshape(-1)
            vals = ["" if np.isnan(v) else f"{v:.6f}" for v in flat]
            w.writerow([r["frame"], f"{r['t']:.6f}",
                        f"{res.scalar:.6f}", f"{res.grid_scalar:.6f}"] + vals)
    meta_path = os.path.splitext(out_csv)[0] + ".grid.json"
    with open(meta_path, "w") as f:
        json.dump(grid.to_meta(), f, ensure_ascii=False, indent=2)
    return out_csv, meta_path


# ----------------------------------------------------------------------------
# データソース連携（data_source.GlimDumpSource の実APIに合わせ済み）
# ----------------------------------------------------------------------------
def iter_frames(dump_dir: str):
    """dump から (frame_id, stamp, points_lidar[M,3]) を順に yield する。

    data_source.GlimDumpSource の API に対応:
      - list_frames() で folder名一覧（6桁ゼロ埋め）を取得
      - get_frame(folder) で FrameData を取得
      - points_lidar は LiDARローカル座標(+Z=上)に正規化済み
        （dump が world座標で入っていても Source 側で T^-1 を掛けて lidar座標に
         落とした上で返してくれる）

    frame_id は既存CSV(correlation.csv等)との結合キーに合わせて int で返す。
    """
    import data_source  # 既存モジュール
    src = data_source.GlimDumpSource(dump_dir)
    for folder in src.list_frames():
        fr = src.get_frame(folder)
        if fr is None or fr.points_lidar is None or len(fr.points_lidar) == 0:
            continue
        stamp = float(fr.stamp) if fr.stamp is not None else 0.0
        yield int(fr.folder), stamp, np.asarray(fr.points_lidar, dtype=float)


def run_dump(dump_dir: str, out_csv: str, grid: SectorGrid) -> None:
    grid.bind(fibonacci_hemisphere())
    rows = []
    for frame_id, t, pts in iter_frames(dump_dir):
        rows.append({"frame": frame_id, "t": t, "result": compute_frame(pts, grid)})
    csv_path, meta_path = write_outputs(rows, grid, out_csv)
    print(f"[done] {len(rows)} frames -> {csv_path}")
    print(f"[meta] {meta_path}")


# ----------------------------------------------------------------------------
# 自己テスト（合成データ：南東の低空に「建物」を置いて遮蔽が出るか確認）
# ----------------------------------------------------------------------------
def _bearing_points(n, bear_lo, bear_hi, el_lo, el_hi, r_lo, r_hi, rng):
    """コンパス方位[deg]・仰角[deg]の範囲に点群を生成（+X=東,+Y=北）。"""
    bear = np.radians(rng.uniform(bear_lo, bear_hi, n))
    el = np.radians(rng.uniform(el_lo, el_hi, n))
    r = rng.uniform(r_lo, r_hi, n)
    east = r * np.cos(el) * np.sin(bear)
    north = r * np.cos(el) * np.cos(bear)
    up = r * np.sin(el)
    return np.stack([east, north, up], axis=1)


def _synthetic_scan(seed=0) -> np.ndarray:
    """現実的な疎なスキャン：開空はリターンが少ない。
    地面クラッタ(大半は15°ゲートで除外) + 低空のまばらな障害物 + SEの密な建物。"""
    rng = np.random.default_rng(seed)
    ground = _bearing_points(150, 0, 360, 1, 14, 3, 25, rng)          # ほぼ除外される
    sparse = _bearing_points(120, 0, 360, 15, 35, 4, 20, rng)         # 弱いベースライン
    building = _bearing_points(1500, 120, 150, 20, 50, 8, 15, rng)    # SE の建物（密）
    return np.vstack([ground, sparse, building])


def _self_test():
    print("=== self-test ===")
    grid = SectorGrid().bind(fibonacci_hemisphere())
    cnt = grid.cell_counts()
    print("grid shape (bands x sectors):", grid.shape)
    print("bands :", grid.band_labels())
    print("sects :", grid.sector_labels())
    print("rays/cell min/mean/max:", cnt.min(), round(cnt.mean(), 1), cnt.max())
    assert cnt.min() > 0, "空セルがある→格子本数 or ビン設定を見直す"

    pts = _synthetic_scan()
    res = compute_frame(pts, grid)
    print(f"scalar occ        = {res.scalar:.3f}")
    print(f"grid-only scalar  = {res.grid_scalar:.3f}")
    print("grid_occ (bands rows x sectors cols):")
    labs = grid.sector_labels()
    print("        " + "  ".join(f"{l:>4}" for l in labs))
    for b, bl in enumerate(grid.band_labels()):
        cells = "  ".join("  -- " if np.isnan(v) else f"{v:4.2f}" for v in res.grid_occ[b])
        print(f"{bl:>7} {cells}")

    # 検証：建物方向(SE,中仰)が反対方向(NW)より明確に高いこと
    se = labs.index("SE")
    nw = labs.index("NW")
    band_mid = 1  # E30-45
    se_val = res.grid_occ[band_mid, se]
    nw_val = res.grid_occ[band_mid, nw]
    print(f"\nSE@{grid.band_labels()[band_mid]} = {se_val:.2f}  vs NW = {nw_val:.2f}")
    assert se_val > nw_val + 0.3, "建物方向のセルが突出していない→ロジック要確認"

    out = "/tmp/sector_selftest.csv"
    rows = [{"frame": k, "t": float(k) * 0.1,
             "result": compute_frame(_synthetic_scan(seed=k), grid)} for k in range(5)]
    write_outputs(rows, grid, out)
    print(f"\nwrote demo CSV: {out}")
    print("OK ✅  ロジック健全。実データは --dump で実行。")


# ----------------------------------------------------------------------------
def _parse_edges(s: str) -> tuple:
    return tuple(float(x) for x in s.split(","))


def main():
    ap = argparse.ArgumentParser(description="セクター別(仰角×方位)遮蔽率の計算")
    ap.add_argument("--dump", help="GLIM dump ディレクトリ")
    ap.add_argument("--out", help="出力CSVパス")
    ap.add_argument("--n-azimuth", type=int, default=8)
    ap.add_argument("--elev-edges", type=_parse_edges, default=(15.0, 30.0, 45.0, 60.0, 90.0),
                    help="例: 15,30,45,60,90")
    ap.add_argument("--azimuth-offset", type=float, default=0.0)
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    if args.self_test:
        _self_test()
        return
    if not args.dump or not args.out:
        ap.error("--dump と --out を指定してください（または --self-test）")
    grid = SectorGrid(elev_edges_deg=args.elev_edges,
                      n_azimuth=args.n_azimuth,
                      azimuth_offset_deg=args.azimuth_offset)
    run_dump(args.dump, args.out, grid)


if __name__ == "__main__":
    main()
