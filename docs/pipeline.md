# lidar-occlusion-gps

LiDAR遮蔽率を用いたGNSS測位信頼性の動的推定とEKFへの適用（修士論文研究）

## ディレクトリ構成

```
~/ros2_ws/
├── src/lidar_occlusion_gps/   ← このリポジトリ
│   ├── scripts/                # Python実行ファイル
│   ├── notebooks/              # Jupyter notebook
│   └── docs/                   # 設計メモ
├── bag/                        # rosbag（Git管理外）
└── dump/<dataset_name>/        # GLIM出力 + analysis/（Git管理外）
```

## データセット

| 名前 | 場所 | 日付 | フレーム数 |
|---|---|---|---|
| nakaniwa_0522 | 中庭 | 2026-05-22 | 119 |

## 実行パイプライン

3ステップに集約（`analysis_pipeline.py` が SE(2) アライメントを内蔵）：

```bash
cd ~/ros2_ws/src/lidar_occlusion_gps

# ① 遮蔽率計算 + GPS誤差算出（SE(2)アライメント込み）→ CSV 2つ出力
python scripts/analysis_pipeline.py

# ② 軌跡可視化（補助スクリプト：trajectory_compare.png / error_over_time.png）
python scripts/compare_trajectory.py

# ③ notebooks/plot_notebook.ipynb で可視化（遮蔽率 vs GPS誤差の散布図など）
```

**出力ファイル**

| ファイル | 生成元 | 内容 |
|---|---|---|
| `occlusion_rate.csv` | ① | stamp, occlusion_rate, folder, n_upper, n_in_range |
| `correlation.csv` | ① | stamp, occlusion_rate, gps_error, x_glim, y_glim, x_gps, y_gps（x_glim/y_glim は SE(2)アライメント済み） |
| `trajectory_compare.png` | ② | XY平面の軌跡重ね描き |
| `error_over_time.png` | ② | 時系列誤差プロット |
| `trajectory_compare.csv` | ② | アライメント済み軌跡データ |

> 旧 `fix_correlation_csv.py` は `analysis_pipeline.py` に統合済みのため削除。
