#!/usr/bin/env python3
"""
analysis_pipeline.py  -  遮蔽率 × GPS誤差 計算パイプライン
==============================================================
GLIMダンプの遮蔽率を計算し、rosbagからGPS-GLIM誤差を計算してCSVへ保存。
グラフ作成は plot_notebook.ipynb 側で行う設計（責務分離）。

使い方:
  # ① 全部まとめて実行（遮蔽率計算 + GPS誤差 → 2つのCSV出力）
python3 analysis_pipeline.py \
  --dump ~/ros2_ws/dump/nakaniwa_0522 \
  --bag  ~/ros2_ws/bag/0522/nakaniwa_0522_glim_bag/rosbag2_2026_06_02-04_46_43_0.db3 \
  --out  ~/ros2_ws/dump/nakaniwa_0522/analysis/

  # ② 遮蔽率CSVが既にある場合（計算スキップ・GPS処理だけ）
  python3 analysis_pipeline.py \\
      --occ-csv ~/ros2_ws/dump/nakaniwa_0522/analysis/occlusion_rate.csv \\
      --bag  ... \\
      --out  ...

  # ③ 遮蔽率だけ計算してCSV保存（rosbagなし環境）
  python3 analysis_pipeline.py \\
      --dump ~/ros2_ws/dump/nakaniwa_0522 \\
      --out  ~/ros2_ws/dump/nakaniwa_0522/analysis/ \\
      --no-gps

出力ファイル:
  occlusion_rate.csv  : stamp, occlusion_rate, folder, n_upper, n_in_range
  correlation.csv     : stamp, occlusion_rate, gps_error, x_glim, y_glim, x_gps, y_gps

→ これらのCSVを plot_notebook.ipynb で読み込んでグラフを作成する。
"""

import os
import sys
import csv
import argparse
import numpy as np

# ──────────────────────────────────────────────
#  パラメータ（ここだけ変える）
# ──────────────────────────────────────────────
N_RAYS      = 1000
MAX_DIST    = 30.0    # 距離上限 [m]
MIN_DIST    = 1.5     # 近傍除去 [m]（人間・自己反射対策）
EL_MIN_DEG  = 15.0    # 仰角カットオフ [°]（GPS mask角相当）
ANGLE_DEG   = 5.0     # レイ許容角 [°]

TOPIC_GPS   = "/odom/UM982"
TOPIC_GLIM  = "/odom/combine_glim"
SYNC_TOL    = 1.0     # 時刻同期許容幅 [秒]


# ──────────────────────────────────────────────
#  Step 1: 遮蔽率計算
# ──────────────────────────────────────────────
def compute_all_occlusion(dump_dir):
    """
    GLIMダンプの全フレームについて遮蔽率を計算する。
    既存の data_source / occlusion_core モジュールを使用。
    Returns: list of dict(folder, stamp, occlusion_rate, n_upper, n_in_range)
    """
    _add_script_dir_to_path()
    from data_source import GlimDumpSource
    from occlusion_core import compute_occlusion, fibonacci_hemisphere

    source  = GlimDumpSource(dump_dir)
    folders = source.list_frames()
    rays    = fibonacci_hemisphere(N_RAYS)
    results = []

    print(f"[Step 1] 遮蔽率計算: {len(folders)} フレーム")
    print(f"  パラメータ: N_RAYS={N_RAYS}, MIN={MIN_DIST}m, MAX={MAX_DIST}m, "
          f"EL_MIN={EL_MIN_DEG}°, ANGLE={ANGLE_DEG}°")

    for i, folder in enumerate(folders):
        frame = source.get_frame(folder)
        if frame is None:
            print(f"  SKIP {folder}")
            continue
        res = compute_occlusion(
            frame.points_lidar, rays=rays,
            max_dist=MAX_DIST, min_dist=MIN_DIST,
            el_min_deg=EL_MIN_DEG, angle_deg=ANGLE_DEG,
            track_hits=False,
        )
        results.append(dict(
            folder=folder,
            stamp=frame.stamp,
            occlusion_rate=res.occlusion_rate,
            n_upper=len(res.pts_upper),
            n_in_range=int(res.pts_upper_in_range.sum()),
        ))
        if (i + 1) % 20 == 0 or (i + 1) == len(folders):
            print(f"  {i+1}/{len(folders)}  last occ={res.occlusion_rate:.3f}")

    return results


def save_occlusion_csv(results, path):
    """遮蔽率CSVを保存。stamp と occlusion_rate に加えてメタ情報も保持。"""
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["stamp", "occlusion_rate", "folder", "n_upper", "n_in_range"])
        for r in results:
            if r["stamp"] is not None:
                w.writerow([
                    r["stamp"], r["occlusion_rate"],
                    r["folder"], r["n_upper"], r["n_in_range"],
                ])
    print(f"  -> 保存: {path}")


def load_occlusion_csv(path):
    """既存の遮蔽率CSVを読み込む。folder情報は無くてもよい。"""
    rows = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            d = dict(
                stamp=float(row["stamp"]),
                occlusion_rate=float(row["occlusion_rate"]),
            )
            if "folder" in row:
                d["folder"] = row["folder"]
            rows.append(d)
    return rows


# ──────────────────────────────────────────────
#  Step 2: GPS-GLIM 誤差計算
# ──────────────────────────────────────────────
def read_odom_from_bag(bag_path, topic_name):
    """rosbag2 から (stamp, x, y) のリストを取得。ROS2環境必須。"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry

    storage   = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic_name]))

    records = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg   = deserialize_message(data, Odometry)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        records.append((stamp, msg.pose.pose.position.x, msg.pose.pose.position.y))
    return records
    
def read_glim_traj_from_dump(dump_dir):
    """GLIMダンプの data.txt から (stamp, x, y) のリストを取得。
    T_world_lidar の並進成分は odom_corrected と同等。"""
    _add_script_dir_to_path()
    from data_source import GlimDumpSource
    source = GlimDumpSource(dump_dir)
    records = []
    for folder in source.list_frames():
        fd = source.get_frame(folder)
        if fd is None or fd.stamp is None:
            continue
        pos = fd.position  # T_world_lidar[:3,3]
        records.append((float(fd.stamp), float(pos[0]), float(pos[1])))
    return records


def _nearest(query, records, tol):
    """time stamp が最も近いレコードを返す。tol超過で None。"""
    if not records:
        return None
    stamps = np.array([r[0] for r in records])
    idx    = int(np.argmin(np.abs(stamps - query)))
    return records[idx] if abs(stamps[idx] - query) <= tol else None


def build_correlation_table(occ_rows, glim_records, gps_records):
    """
    遮蔽率リスト × GLIM odom × GPS odom を時刻同期して誤差を算出。
    Returns: list of dict(stamp, occlusion_rate, gps_error, x_glim, y_glim, x_gps, y_gps)
    """
    results, skipped = [], 0
    for row in occ_rows:
        stamp = row["stamp"]
        occ   = row["occlusion_rate"]
        g = _nearest(stamp, glim_records, SYNC_TOL)
        p = _nearest(stamp, gps_records,  SYNC_TOL)
        if g is None or p is None:
            skipped += 1
            continue
        err = float(np.hypot(p[1] - g[1], p[2] - g[2]))
        results.append(dict(
            stamp=stamp, occlusion_rate=occ, gps_error=err,
            x_glim=g[1], y_glim=g[2], x_gps=p[1], y_gps=p[2],
        ))
    print(f"  時刻同期: {len(results)} 件マッチ / {len(occ_rows)} 件  "
          f"(スキップ {skipped})")
    return results


def save_correlation_csv(results, path):
    if not results:
        print("  [WARN] 相関データが空のため保存スキップ")
        return
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(results[0].keys()))
        w.writeheader()
        w.writerows(results)
    print(f"  -> 保存: {path}")


# ──────────────────────────────────────────────
#  ユーティリティ
# ──────────────────────────────────────────────
def _add_script_dir_to_path():
    """このスクリプトと同じディレクトリをsys.pathに追加（モジュール import 用）。"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    if script_dir not in sys.path:
        sys.path.insert(0, script_dir)


# ──────────────────────────────────────────────
#  メイン
# ──────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="遮蔽率 × GPS誤差 計算パイプライン（CSV出力まで）",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument("--dump",    default="~/ros2_ws/dump/nakaniwa_0522",
                        help="GLIMダンプディレクトリ")
    parser.add_argument("--bag",     default="~/ros2_ws/bag/0522/nakaniwa_0522_bag/"
                        "rosbag2_2026_05_22-10_12_14_0.db3",
                        help="rosbag2 パス")
    parser.add_argument("--occ-csv", default=None,
                        help="既存の遮蔽率CSV（指定時は計算をスキップ）")
    parser.add_argument("--out",     default="~/ros2_ws/dump/nakaniwa_0522/analysis/",
                        help="出力ディレクトリ")
    parser.add_argument("--no-gps",  action="store_true",
                        help="GPS誤差計算をスキップ（遮蔽率CSVのみ生成）")
    parser.add_argument("--glim-topic", default=TOPIC_GLIM,
                        help=f"GLIMのodometryトピック名 (default: {TOPIC_GLIM})")
    parser.add_argument("--gps-topic",  default=TOPIC_GPS,
                        help=f"GPSのodometryトピック名 (default: {TOPIC_GPS})")
    args = parser.parse_args()

    dump_dir = os.path.expanduser(args.dump)
    bag_path = os.path.expanduser(args.bag)
    out_dir  = os.path.expanduser(args.out)
    os.makedirs(out_dir, exist_ok=True)

    # ── Step 1: 遮蔽率 ─────────────────────────
    occ_csv = os.path.join(out_dir, "occlusion_rate.csv")
    if args.occ_csv:
        print(f"[Step 1] 既存の遮蔽率CSVを使用: {args.occ_csv}")
        occ_rows = load_occlusion_csv(os.path.expanduser(args.occ_csv))
    else:
        raw_results = compute_all_occlusion(dump_dir)
        save_occlusion_csv(raw_results, occ_csv)
        occ_rows = [dict(stamp=r["stamp"], occlusion_rate=r["occlusion_rate"],
                         folder=r["folder"])
                    for r in raw_results if r["stamp"] is not None]

    print(f"  遮蔽率サンプル数: {len(occ_rows)}")

    if args.no_gps:
        print("\n--no-gps 指定のため GPS関連処理をスキップします。")
        print(f"\n========== 完了 ==========")
        print(f"出力先: {out_dir}")
        print(f"  occlusion_rate.csv")
        print(f"\nグラフ作成は plot_notebook.ipynb を VS Code で開いて実行してください。")
        return

    # ── Step 2: GPS誤差 ─────────────────────────
    print(f"\n[Step 2] GLIM軌跡=dump, GPS=rosbag から読み出し")
    print(f"  bag: {bag_path}")
    try:
        glim_records = read_glim_traj_from_dump(dump_dir)
        gps_records  = read_odom_from_bag(bag_path, args.gps_topic)
        print(f"  GLIM={len(glim_records)} サンプル(dump), "
              f"GPS={len(gps_records)} サンプル(bag)")
    except Exception as e:
        print(f"[ERROR] rosbag 読み出し失敗: {e}")
        print("  ROS2環境でソースして再実行するか、--no-gps を指定してください。")
        sys.exit(1)

    corr_rows = build_correlation_table(occ_rows, glim_records, gps_records)
    if not corr_rows:
        print("[ERROR] 時刻同期結果が0件。stampやトピック名を確認してください。")
        print(f"  GLIM topic: {args.glim_topic}")
        print(f"  GPS  topic: {args.gps_topic}")
        print(f"  SYNC_TOL: {SYNC_TOL} 秒")
        sys.exit(1)

    save_correlation_csv(corr_rows, os.path.join(out_dir, "correlation.csv"))

    print(f"\n========== 完了 ==========")
    print(f"出力先: {out_dir}")
    print(f"  occlusion_rate.csv")
    print(f"  correlation.csv")
    print(f"\nグラフ作成は plot_notebook.ipynb を VS Code で開いて実行してください。")


if __name__ == "__main__":
    main()
