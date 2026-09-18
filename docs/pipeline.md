# 処理の流れ（データ取得 → 解析）

最終更新: 2026-09-16 / 対象: `kitakan_0731` で確認済み

---

## 0. 全体像

```
[走行]  bag録画
   │
   ├─ /livox/lidar2 ──┐
   │  /imu/spresense  │
   │                  ▼
   │            [GLIM] SLAM       →  dump/<dataset>/   ── 参照軌跡 + 点群
   │                                      │
   │                                      ▼
   │                            [occ算出] 上半球レイキャスト
   │                                      │
   └─ /gps_raw ──[GGA/HDTパース]──┐       │
      /gnss/ppp_status            ▼       ▼
                            gps_raw.csv  occlusion_rate.csv
                                    └──┬──┘
                                       ▼
                              correlation.csv   ← 時刻同期 + SE(2)アライメント
                                       ▼
                              plot_notebook.ipynb
```

---

## 1. データ取得（走行時）

| トピック | 型 | 用途 |
|---|---|---|
| `/livox/lidar2` | CustomMsg | **上向きLiDAR** → occ算出・GLIM入力 |
| `/livox/imu2` | Imu | 上向きLiDAR内蔵IMU |
| `/livox/lidar` | CustomMsg | 前向きLiDAR（現状未使用） |
| `/imu/spresense` | Imu | **GLIMのIMU入力** |
| `/gps_raw` | String | GGA + HDT の生文字列 |
| `/gnss/ppp_status` | PppNav | σ・pos_type（自己申告） |
| `/odom` | Odometry | 車輪オドメトリ |

**録り漏らすと後から復元できない。** 2025年つくばbagは上向きLiDARと`/gps_raw`が無く使用不能になった。

---

## 2. GLIM 実行（参照軌跡の生成）

### 設定（`install/glim/share/glim/config/`）

| 項目 | 値 | 理由 |
|---|---|---|
| `config.json` | `*_cpu.json` ×3 | GTX 1650（VRAM 4GB）ではGPU構成がOOMで落ちる |
| `config_ros.json` `points_topic` | `/converted_pointcloud2_lidar2` | 上向きLiDARの変換後 |
| `config_ros.json` `imu_topic` | `/imu/spresense` | |

`install/` 側を編集する。`src/` を直しても反映されない（symlinkビルドではないため）。

### コマンド（3ターミナル）

```bash
# T1
ros2 run glim_ros glim_rosnode --ros-args -p use_sim_time:=true

# T2  CustomMsg → PointCloud2（lidar / lidar2 の2ノードが立つ）
ros2 launch livox_to_pointcloud2 livox_to_pointcloud2.launch.py

# T3  remapは不要（config側が既にlidar2を向いている）
cd ~/ros2_ws/bag && ros2 bag play 0731 --clock -r 0.5
```

`-r 0.5` は点群ドロップ防止。等速だと数秒の欠落が出てIMU積分が切れる。

### 保存

Ctrl+C で自動的に `/tmp/dump` へ出力される（`dump_on_unload` の設定は不要）。

```bash
mv /tmp/dump ~/ros2_ws/dump/kitakan_0731
```

`data.txt` は各キーフレームのフォルダ内にある（`stamp` と `T_world_lidar`）。

---

## 3. 遮蔽率の算出（`analysis_pipeline.py` Step 1）

キーフレームごとに、上半球を覆う遮蔽の割合を出す。

1. `points_compact.bin` を読む（float32 / shape(-1,3) / **LiDARローカル座標**）
2. 上半球にフィボナッチ格子で仮想光線を **N=1000** 本配置
3. 仰角 **15°以上** の光線だけを評価対象にする（GNSSの仰角マスク相当）
4. 距離 **1.5〜30m** の点だけを使う（近傍の自己反射と遠方ノイズを除去）
5. 光線と点のなす角が **5°以内** なら「その光線は遮蔽された」
6. 遮蔽された光線の割合 = `occ`

出力 `occlusion_rate.csv` : `stamp, occlusion_rate, folder, n_upper, n_in_range`

---

## 4. GPS の抽出（`extract_gps_raw.py`）

`/gps_raw` は HDT と GGA の repr を連結した1本の文字列なので、正規表現で両方を切り出す。

| 処理 | 内容 |
|---|---|
| GGAパース | 緯度経度・quality・衛星数・HDOP |
| 座標変換 | EPSG:6668（JGD2011緯度経度）→ **EPSG:6677**（平面直角第IX系）。東京もつくばも第IX系 |
| 方位 | HDT（真北基準・時計回り）→ ENU yaw（東基準・反時計回り）。**さらに -90° の補正**（2アンテナが機体左右配置のため、基線方向 ≠ 機体前方） |
| PPP結合 | `/gnss/ppp_status` を stamp 最近傍で突合。`valid=false` のとき σ は null |

出力 `gps_raw.csv` : `stamp, x, y, yaw, quality, pos_type, lat_sd, lon_sd, sol_age, ...`

---

## 5. 誤差の算出（`analysis_pipeline.py` Step 2）

GLIMの軌跡は初期位置を原点とするローカル座標なので、GPS座標系に載せ替える。

1. 最初のGPSエポックの `(x0, y0, yaw0)` を基準にする
2. GLIM軌跡を SE(2) 変換（回転 `yaw0` + 並進 `x0, y0`）
3. occ の stamp ごとに、GLIM と GPS の最近傍点を探す（許容 **1.0秒**）
4. 誤差 = 2点間のユークリッド距離

出力 `correlation.csv` : `stamp, occlusion_rate, gps_error, x_glim, y_glim, x_gps, y_gps`

**yaw0 の正しさが誤差を支配する。** 1°のズレが走行100mで約1.7mの誤差を生む。

---

## 6. コマンド早見表

```bash
cd ~/ros2_ws/src/lidar_occlusion_gps
source ~/ros2_ws/install/setup.bash

D="D=kitakan_0731 BAG_DIR=$HOME/ros2_ws/bag/0731"

make show    $D                      # パス確認（実行前推奨）
make gps     $D                      # /gps_raw → gps_raw.csv
make analyze $D GPS_CSV=$HOME/ros2_ws/dump/kitakan_0731/analysis/gps_raw.csv
make notebook D=kitakan_0731         # プロット
```

| 変数 | 既定値 | 用途 |
|---|---|---|
| `D` | `nakaniwa_0522` | データセット名 |
| `BAG_DIR` | `bag/<MMDD>/<dataset>_bag` | 命名規則から外れる場合に指定 |
| `GPS_CSV` | （空） | 指定すると bag ではなく CSV から GPS を読む |
| `PPP_PKG` | `orange_msgs_0723` | 録画時の PppNav 定義 |
| `HDT_OFFSET` | `-90` | アンテナ基線 → 機体前方の補正 |

---

## 7. 既知の落とし穴

| 症状 | 原因 | 対処 |
|---|---|---|
| `cudaErrorIllegalAddress` で落ちる | GPU構成 + VRAM 4GB | `config.json` を `*_cpu.json` に |
| 点群が数秒欠落する | 処理が追いつかない | `-r 0.5`（足りなければ `0.3`） |
| `/livox/lidar` が20Hz | remapで前向きと上向きが混在 | remapしない |
| `Fast CDR exception` | PppNav の定義バージョン違い | `--ppp-pkg orange_msgs_0723` |
| 誤差が数十mになる | HDTオフセット未適用 | `--hdt-offset -90` |
| `make all` が失敗 | `compare_trajectory.py` が `/odom/UM982` を読む | `make analyze` 単体で実行 |
| 相関値が過去と一致しすぎる | ノートブックが旧データセットを読んでいる | セル2の `DATASET` を確認 |
| colcon で `source directory does not exist` | 旧パスのCMakeキャッシュ | `rm -rf build/<pkg> install/<pkg>` |

---

## 8. 診断：誤差が大きいときの切り分け

軌跡の形が合っているのに誤差が大きい場合、原因は回転ズレ。

```bash
python3 - <<'EOF'
import numpy as np, pandas as pd
d = pd.read_csv("/home/ubuntu/ros2_ws/dump/kitakan_0731/analysis/correlation.csv")
G = d[["x_glim","y_glim"]].to_numpy(); P = d[["x_gps","y_gps"]].to_numpy()
t = (d["stamp"] - d["stamp"].iloc[0]).to_numpy(); e = d["gps_error"].to_numpy()
print(f"corr(time, error)      = {np.corrcoef(t, e)[0,1]:+.3f}")
print(f"corr(dist, error)      = {np.corrcoef(np.hypot(*(P-P[0]).T), e)[0,1]:+.3f}")
Gc, Pc = G-G.mean(0), P-P.mean(0)
U,S,Vt = np.linalg.svd(Gc.T@Pc); R=(U@Vt).T
if np.linalg.det(R)<0: Vt[-1]*=-1; R=(U@Vt).T
e2 = np.hypot(*(P-(Gc@R.T+P.mean(0))).T)
print(f"additional rotation    = {np.degrees(np.arctan2(R[1,0],R[0,0])):+.2f} deg")
print(f"RMS before / after     = {np.sqrt((e**2).mean()):.2f} -> {np.sqrt((e2**2).mean()):.2f} m")
EOF
```

- `corr(dist, error)` が +1 に近い → 回転ズレ。`additional rotation` の分だけ `HDT_OFFSET` を直す
- `corr(time, error)` が高い → GLIM のドリフト
- 回転を当ててもRMSが大きい → GLIM軌跡そのものの破綻
