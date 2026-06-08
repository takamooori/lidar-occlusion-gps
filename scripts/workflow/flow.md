# データ取得ワークフロー

LiDAR遮蔽率研究におけるデータ取得・GLIM処理・dump整理の手順書。

## 全体像

```
[Phase 1] raw bag 録画
   ↓
[Phase 2] GLIM処理（replay + 追加bag録画）
   ↓
[Phase 3] dump整理（workflow.sh save）
```

## 配置

| 種別 | パス |
|---|---|
| スクリプト | `~/ros2_ws/src/lidar_occlusion_gps/scripts/workflow/workflow.sh` |
| raw bag | `~/ros2_ws/bag/<MMDD>/<location>_<MMDD>_bag/` |
| GLIM dump | `~/ros2_ws/dump/<location>_<MMDD_HHMM>/` |
| 追加 bag | `~/ros2_ws/bag/<MMDD>/glim_corrected_<MMDD_HHMM>/` |

## 命名規則

- raw bag: `<location>_<MMDD>` （日単位）
- dump / 追加bag: `<location>_<MMDD_HHMM>` （分単位 = 同日複数取得可）

## 事前準備（初回のみ）

```bash
chmod +x ~/ros2_ws/src/lidar_occlusion_gps/scripts/workflow/workflow.sh
```

エイリアス登録すると便利:

```bash
echo 'alias wf="~/ros2_ws/src/lidar_occlusion_gps/scripts/workflow/workflow.sh"' >> ~/.bashrc
source ~/.bashrc
```

---

## Phase 1: 生データ取得

各センサーを立ち上げた状態で、raw bag を録画する。

```bash
mkdir -p ~/ros2_ws/bag/$(date +%m%d)
cd ~/ros2_ws/bag/$(date +%m%d)

ros2 bag record \
    -o nakaniwa_$(date +%m%d)_bag \
    /livox/lidar2 \
    /odom/UM982 \
    /imu/spresense \
    /livox/imu2 \
    /odom
# 取得終了したら Ctrl+C
```

---

## Phase 2: GLIM処理（オフライン解析用 dump 生成）

### Terminal A: GLIM起動

```bash
ros2 launch glim_ros glim_rosbag.launch.xml
# 初期化完了・待機状態になるまで待つ
```

### Terminal B: replay + 追加bag録画

```bash
cd ~/ros2_ws/bag/$(date +%m%d)

# 追加bag録画開始（バックグラウンド）
ros2 bag record -o glim_corrected_$(date +%m%d_%H%M) /glim_ros/odom_corrected &
REC_PID=$!

# raw bag を replay
ros2 bag play ~/ros2_ws/bag/<MMDD>/<name>_bag

# replay完了後、追加bag録画を停止
kill $REC_PID
```

### Terminal A: GLIM 停止

```bash
# Ctrl+C で停止
# → /tmp/dump にdumpが出力されている状態
```

---

## Phase 3: dump 整理

GLIM 停止後、以下を1回実行するだけ。

```bash
~/ros2_ws/src/lidar_occlusion_gps/scripts/workflow/workflow.sh save nakaniwa
# エイリアス登録済みなら:  wf save nakaniwa
```

実行結果:

```
[INFO] 移動中: /tmp/dump -> /home/<user>/ros2_ws/dump/nakaniwa_0608_1530
[OK] 保存完了
     データセット名: nakaniwa_0608_1530
     保存先:        /home/<user>/ros2_ws/dump/nakaniwa_0608_1530
```

`/tmp/dump` は空ディレクトリとして自動再作成されるので、次のGLIM実行に向けてそのまま使える。

---

## 注意事項

| 項目 | 内容 |
|---|---|
| GLIM 停止確認 | save 実行前に必ず GLIM を停止（書き込み中だとファイル破損） |
| save 忘れ防止 | 次の GLIM 起動前に `ls /tmp/dump` で空確認推奨 |
| stamp破損対策 | `/odom/combine_glim` は録画しない（get_clock().now() 起因のstamp破損あり） |
| 二重実行 | 同分内の `save` 二重実行はエラーになる仕様 |

## トラブルシューティング

| 症状 | 対処 |
|---|---|
| `/tmp/dump が空です` | GLIM の launch 設定で dump 出力先 = /tmp/dump になっているか確認 |
| `既に存在` エラー | 同分内の二重実行。1分待ってから再実行、または手動リネーム |
| dump が混在している（前回分残り） | `~/ros2_ws/dump/_backup/unsaved_<timestamp>/` に手動退避 |
