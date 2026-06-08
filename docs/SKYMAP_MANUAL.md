# 遮蔽スカイマップ ツール マニュアル

上半球の遮蔽方向を「真上から見た円板」に投影して可視化する。
中心=天頂、外周=地平線、**遮蔽=赤・開空=青・mask外(仰角<15°)=灰**。

## 構成

| 用途 | ツール | 出力 |
|---|---|---|
| 多数フレームを高速ブラウズ | `export_skymap_html.py` | 自己完結HTML |
| 発表用に1枚作り込む | `skymap_viz.py` | 高解像PNG |

共通エンジン: `occlusion_core.py`, `data_source.py`, `jp_font.py`

## 1. 多数フレーム閲覧（HTML）

```bash
python3 export_skymap_html.py ~/ros2_ws/maps/dump_nakaniwa_0522
# → skymap_dump_nakaniwa_0522.html
```

主なオプション:

| オプション | 用途 |
|---|---|
| `--out <file>` | 出力ファイル名 |
| `--stride 2` | フレーム間引き |
| `--max-frames 40` | 先頭40フレームのみ |
| `--north-az 37` | 真北合わせ |

HTML側でできること: スライダーでフレーム指定、◀▶送り、▶再生、最大遮蔽へジャンプ、グリッドで全フレーム一覧。

## 2. 発表用1枚（PNG）

```bash
# 最大遮蔽フレームを自動選択
python3 skymap_viz.py ~/ros2_ws/maps/dump_nakaniwa_0522

# occ ≈ 0.6 に最も近いフレームを選択
python3 skymap_viz.py <dump> --target 0.6

# フレーム直接指定（スキャン省略で高速）
python3 skymap_viz.py <dump> --frame 000087

# 遮蔽率ランキングだけ表示
python3 skymap_viz.py <dump> --rank-only
```

## パラメータ（既定値）

| 引数 | 既定 | 意味 |
|---|---|---|
| `--n-rays` | 1000 | フィボナッチ格子のレイ数 |
| `--max-dist` | 30 | 距離上限 [m] |
| `--min-dist` | 1.5 | 近傍除去 [m] |
| `--el-min` | 15 | 仰角カット [°]（GPS mask角対応） |
| `--angle` | 5 | レイ許容角度 [°] |
| `--north-az` | 0 | 北方向 [°] を図の上(N)へ回す |

## 方位について

既定は **LiDAR +X = ロボット前方 = 図の上(N)**。
GLIM world座標は yaw が初期姿勢依存。真北合わせは `--north-az` に「LiDAR座標で北を指す方位角[°]」を指定。

## 入力データ

GLIM dump ディレクトリ:

```
dump_xxx/
  000000/  data.txt   points_compact.bin
  000001/  data.txt   points_compact.bin
  ...
```

- `data.txt`: `stamp:` と `T_world_lidar:`
- `points_compact.bin`: float32 (N,3)

## トラブル対応

| 症状 | 確認 |
|---|---|
| `dump_dir` が見つからない | パス確認（`~` は展開される） |
| フレーム0件 | 各フォルダに `data.txt` と `points_compact.bin` があるか |
| occ が全部0/異常 | `--min-dist/--max-dist/--el-min` が厳しすぎないか |
| 日本語が豆腐 | `jp_font.py` が同じディレクトリにあるか |
| HTMLが重い | `--stride` で間引く |
