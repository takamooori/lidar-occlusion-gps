# lidar-occlusion-gps

LiDAR遮蔽率を用いたGNSS測位信頼性の動的推定とEKFへの適用（修士論文研究 / Kobayashi Lab）

LiDAR上半球の遮蔽率を二重指標として利用し、GPS観測ノイズ共分散 R 行列を動的に更新する EKF を提案。

## ディレクトリ構成

```
~/ros2_ws/
├── src/lidar_occlusion_gps/   ← このリポジトリ
│   ├── scripts/                # Python 実行ファイル
│   ├── notebooks/              # Jupyter notebook
│   └── docs/                   # 設計メモ・詳細ドキュメント
├── bag/                        # rosbag（Git管理外）
└── dump/<dataset_name>/        # GLIM出力 + analysis/（Git管理外）
```

## クイックスタート

```bash
cd ~/ros2_ws/src/lidar_occlusion_gps
python scripts/analysis_pipeline.py     # ① 遮蔽率 + GPS誤差（SE(2)アライメント込み）
python scripts/compare_trajectory.py    # ② 軌跡可視化（補助）
# ③ notebooks/plot_notebook.ipynb で可視化
```

詳細なパイプライン仕様・出力ファイル一覧は [`docs/pipeline.md`](docs/pipeline.md) を参照。

## データセット

| 名前 | 場所 | 日付 | フレーム数 |
|---|---|---|---|
| nakaniwa_0522 | 中庭 | 2026-05-22 | 119 |
