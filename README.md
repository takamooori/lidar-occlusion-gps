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

```bash
cd ~/ros2_ws/src/lidar_occlusion_gps
python scripts/analysis_pipeline.py
python scripts/compare_trajectory.py
python scripts/fix_correlation_csv.py
# notebooks/plot_notebook.ipynb で可視化
```
