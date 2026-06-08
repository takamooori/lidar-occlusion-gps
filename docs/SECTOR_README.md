# セクター別 遮蔽率 — 使い方メモ

スカラー1個だった遮蔽率を **仰角バンド × 方位セクター** で見られるようにする。
新規ファイルは3つだけ。既存パイプラインには触らない（並走する）。

---

## 仕組み（30秒）

レイキャストは既存と同じ（フィボナッチ格子 N=1000・5ステップフィルタ）。
**変えたのは集計だけ**：各レイを「どの仰角バンド・どの方位セクターか」で仕分けし、
セルごとに `ヒット数 / レイ数` を出す。

```
        従来              新
   占有率 = 0.28   →   仰角\方位  N   NE  E  SE  S ...
   （1個）              E15-30   .70 .70 .77 .93 ...   ← 低空ほど塞がる
                        E30-45   .24 .25 .40 1.0 ...
                        E45-60   .00 .00 .00 .62 ...
                        E60-90   .00 ...               ← 天頂(センサ死角)
```

グローバル占有率 `occ_scalar` は **従来値と一致**（後方互換）。

---

## ファイル連携図

```
                  GLIM dump (~/ros2_ws/maps/dump_xxx/)
                          │
        ┌─────────────────┴─────────────────┐
        │ 既存パイプライン                  │ ★新規（並走）
        ▼                                   ▼
  analysis_pipeline.py              sector_occlusion.py
        │ → correlation.csv                 │ → sector_occlusion.csv
        ▼                                   │   (+ .grid.json)
  compare_trajectory.py                     │
        │ → 整合済み軌跡誤差                │
        ▼                                   │
  fix_correlation_csv.py                    │
        │ → correlation.csv (誤差を上書き)  │
        ▼                                   │
  correlation.csv  ─────────────────────────┤
        │                                   │ frame をキーに結合
        ▼                                   ▼
  plot_notebook.ipynb              sector_analysis.ipynb
    (Fig 1–3 / scalar)               (Fig A–D / 方向別)
```

- 新規パイプラインは既存に **手を加えずに並走**
- `sector_analysis.ipynb` が `sector_occlusion.csv` と `correlation.csv` を `frame` で結合
- 統合は将来の TODO（→「あとで」）

---

## 新規ファイル

| ファイル | 役割 |
|---|---|
| `sector_occlusion.py` | エンジン＋CLI。dump → セクター占有率CSV |
| `sector_analysis.ipynb` | 表示＋評価（Fig A–D） |
| `SECTOR_README.md` | このメモ |

配置場所：`~/ros2_ws/maps/`（`data_source.py` と同じディレクトリ）。

---

## 実行（最短ルート）

```bash
cd ~/ros2_ws/maps

# 0) 動作確認（データ不要・数秒）
python sector_occlusion.py --self-test

# 1) dump から全フレーム計算 → CSV
python sector_occlusion.py \
    --dump ~/ros2_ws/maps/dump_nakaniwa_0522 \
    --out  ~/ros2_ws/maps/analysis_nakaniwa_0522/sector_occlusion.csv
# → sector_occlusion.csv ＋ sector_occlusion.grid.json

# 2) Fig C/D には correlation.csv の GPS誤差が必要。
#    既存パイプライン未実行なら先に：
python analysis_pipeline.py
python compare_trajectory.py
python fix_correlation_csv.py

# 3) 解析notebook（VS Code / Jupyter）
jupyter notebook sector_analysis.ipynb
# 先頭セルで sector_occlusion.csv と correlation.csv のパスを指定
```

### よく変える引数

| 引数 | 既定 | 意味 |
|---|---|---|
| `--n-azimuth` | 8 | 方位分割数（4 / 8 / 16） |
| `--elev-edges` | `15,30,45,60,90` | 仰角境界 [deg]（地平=0, 天頂=90） |
| `--azimuth-offset` | 0 | 方位の北合わせ [deg] |

### 出力CSV の列

1行 = 1フレーム。

| 列 | 意味 |
|---|---|
| `frame` | フレームID（dumpフォルダ名 `000042` → `42`） |
| `t` | UNIX時刻 [秒] |
| `occ_scalar` | 全レイの遮蔽率（**従来値と一致 = 後方互換**） |
| `occ_grid_scalar` | グリッド対象レイ（仰角範囲内）のみの遮蔽率 |
| `occ_e{b}_a{s}` | 仰角バンド b × 方位セクター s の遮蔽率 |

`b` / `s` と仰角範囲 / 方位ラベルの対応は **同名 `.grid.json`** に入っている。

---

## 注意点

- **方位はセンサ相対**：N/E/S/W はセンサ X/Y 軸基準。ロボットが回転すると絶対方位とはズレるので、現状は「ロボット前後左右」ぐらいに解釈。Fig A の高遮蔽方向 ↔ 現場写真で照合し、必要なら `--azimuth-offset` を調整。
- **E60–90 はほぼ常に 0**：MID360 は仰角 ≳52° が死角。意味があるのは E15–30 / E30–45 / E45–60。
- **座標系は自動正規化**：`GlimDumpSource` が world→lidar の変換をやってくれるので意識不要。

---

## 評価のしかた

| 観点 | 図 | 言えること |
|---|---|---|
| **妥当性** | Fig A ↔ 現場写真 | 遮蔽率が物理的に正しい（中間①対応） |
| **誤差の方向依存** | Fig C：セル別 occ–GPS誤差 相関マップ | **どの方向・高さ** の遮蔽が誤差を生むか |
| **スカラー超え** | Fig D：scalar vs 方向特徴 の相関/R² | 方向分解の価値を定量化 |
| **R行列駆動** | `R_GPS = R₀·exp(α·occ_dir)` | メイン提案に直結 |
| **(発展) 衛星整合** | 衛星 az/el 方向の occ ↔ FIX率/CN0/残差 | NLOS 衛星を直接予測（最強の検証） |

**狙い**：スカラーは弱相関 (r≈0.16)。GPS誤差は NLOS/マルチパスで起きるので
**全天の塞がり量より「衛星が来る方向（特に低〜中仰角）の塞がり」** の方が効くはず。
Fig C/D で確かめ、効く方向だけを R 行列の driver にする。

**注意（誠実に）**
- セル単位で見るのが診断的（バンド平均は方向性を薄める）
- セル数 ≫ フレーム数だと重回帰 R² は過大評価 → **低次元の方向特徴** で比較
- 中庭は遮蔽レンジが狭い (0.15–0.65) → **高遮蔽環境の追加データ** で検出力UP

---

## 全ファイル早見表（覚えきれない用）

### エンジン
| ファイル | 役割 |
|---|---|
| `occlusion_core.py` | レイキャスト本体 |
| `data_source.py` | データ抽象化（`GlimDumpSource` / `RosbagSource`） |
| `sector_occlusion.py` | 🆕 セクター占有率＋CLI |

### 既存解析パイプライン（実行順）
| # | ファイル | 入力 → 出力 |
|---|---|---|
| 1 | `analysis_pipeline.py` | dump → `correlation.csv`, `occlusion.csv` |
| 2 | `compare_trajectory.py` | GPS + GLIM軌跡 → 整合済み軌跡誤差 |
| 3 | `fix_correlation_csv.py` | (2) の結果 → `correlation.csv` の error列を上書き |
| 4 | `plot_notebook.ipynb` | `correlation.csv` → Fig 1–3 |
| 補 | `gps_correlation.py` | 相関 / ANOVA / R 行列設計 |

### 可視化
| ファイル | 役割 |
|---|---|
| `skymap_viz.py` ほか | スカイマップ系（方位等距離投影） |
| `sector_analysis.ipynb` | 🆕 セクター解析・評価（Fig A–D） |
| `jp_font.py` | 日本語フォント自動検出 |

---

## あとで（任意）

`sector_occlusion.py` の `compute_frame()` を `analysis_pipeline.py` のフレームループに
組み込めば、`correlation.csv` に `occ_e*_a*` 列を直接足せて **別CSV＋結合が不要** になる
（既存 TODO の alignment 統合＋4→3ステップ化と合流）。
まずは別CSV版で運用 → 効果が見えたら統合、を推奨。
