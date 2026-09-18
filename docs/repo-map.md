# lidar-occlusion-gps リポジトリマップ

更新: 2026-09-16。AIに読み込ませる前提で簡潔に書く。詳細な手順・落とし穴は docs/pipeline.md を参照。

## 流れ（1行）

bag → GLIM（参照軌跡）→ occ算出 → GPS抽出 → 結合CSV → notebookで図

## scripts/ — 数字を作る側（ターミナルで実行、出力はCSV）

| ファイル | 実行方法 | 入力 → 出力 |
|---|---|---|
| analysis_pipeline.py | make analyze | dump + gps_raw.csv → occlusion_rate.csv, correlation.csv |
| extract_gps_raw.py | make gps | bag(/gps_raw, /gnss/ppp_status) → gps_raw.csv（位置・quality・σ） |
| sector_occlusion.py | 直接実行 | dump → セクター別CSV（方位×仰角バンド） |
| occlusion_core.py | ライブラリ（単体実行しない） | レイキャスト本体。N=1000, 仰角15°+, 1.5-30m, 5°判定 |
| data_source.py | ライブラリ（単体実行しない） | GLIMダンプ読み込み（points_compact.bin, data.txt） |
| workflow/workflow.sh | `workflow.sh save <場所>` | /tmp/dump → ~/ros2_ws/dump/<場所>_<MMDD_HHMM>/ |

以下は単発ツール。日常の流れでは使わない（削除せず残置）:
skymap_viz.py + jp_font.py（上半球正射影＝中間発表Fig.4）, verify_viz.py（光線配置検証＝Fig.2）,
export_skymap_html.py（スカイマップHTML版）, glim_inspector.py（dump確認）,
compare_trajectory.py（軌跡PNG。/odom/UM982 依存のため0731以降は不可、notebookで代替）

## notebooks/ — 数字を見る側（VS Codeで実行、入力はCSV）

### plot_notebook.ipynb（日常用）
- セル2: DATASET を1行で切替（実行前に必ず確認）
- セル3-5: occ時系列 / 誤差時系列 / 相関散布図（中間発表Fig.5-6と同型）
- セル6: 300dpi PNG一括保存
- セル7-12: σ vs occ 比較（説明力・ラグ・NONE区間・4象限）。
  **検討中の解析であり、内容は未消化のまま残置。日常では実行不要。**

### sector_analysis.ipynb（仰角バンド解析用）
- 入力: sector_occlusion.py のセクターCSV
- Fig A: セクター遮蔽率マップ / Fig B: 仰角バンドタイムライン
- Fig C: どの方向・高さが誤差を説明するか / Fig D: スカラーvs方向特徴の重回帰
- Fig Y: スカラー vs E45-60（中間発表Fig.8・Table 4の生成元）

## コマンド早見

```bash
# GLIM（3ターミナル）
ros2 run glim_ros glim_rosnode --ros-args -p use_sim_time:=true   # T1（短縮方法は検討中）
ros2 launch livox_to_pointcloud2 livox_to_pointcloud2.launch.py   # T2
cd ~/ros2_ws/bag && ros2 bag play <bag> --clock -r 0.5            # T3
# 終了: T1をCtrl+C → /tmp/dump に自動保存
./scripts/workflow/workflow.sh save <場所>

# 解析
cd ~/ros2_ws/src/lidar_occlusion_gps && source ~/ros2_ws/install/setup.bash
make gps     D=<dataset> BAG_DIR=<bagパス>
make analyze D=<dataset> BAG_DIR=<bagパス> GPS_CSV=<gps_raw.csvパス>
make notebook D=<dataset>
```

## 前提・注意（要約）

- GLIMはCPU構成（install/側 config.json）。GPU構成はVRAM 4GBで落ちる
- bag再生はremap不要（config_ros.jsonが lidar2 系を向いている）
- PppNav は録画時定義 orange_msgs_0723 で読む（現行定義は非互換）
- HDTは -90°補正が必要（2アンテナ左右配置のため）
- make all は使わない（compare_trajectory.py が /odom/UM982 依存）
