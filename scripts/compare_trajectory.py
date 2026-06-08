#!/usr/bin/env python3
"""
compare_trajectory.py
=====================
UM982 (GPS) と GLIMダンプ（combine_glim相当のアライメント済み）の軌跡を
同じ座標系で重ねて可視化する。

combine_glim と同等の SE(2) アライメント:
  最初のUM982の (x0, y0, yaw0) を使って
    aligned_x = cos(yaw0)*gx - sin(yaw0)*gy + x0
    aligned_y = sin(yaw0)*gx + cos(yaw0)*gy + y0

使い方:
    python3 compare_trajectory.py \\
        --dump ~/ros2_ws/dump/nakaniwa_0522 \\
        --bag  ~/ros2_ws/bag/0522/nakaniwa_0522_bag/rosbag2_2026_05_22-10_12_14_0.db3 \\
        --out  ~/ros2_ws/dump/nakaniwa_0522/analysis/

出力:
    trajectory_compare.png  : XY平面の軌跡重ね描き
    error_over_time.png     : 時系列誤差プロット
    trajectory_compare.csv  : (stamp, x_gps, y_gps, x_glim_aligned, y_glim_aligned, error)
"""

import os
import sys
import csv
import math
import argparse
import numpy as np

# 同ディレクトリのモジュールを読めるように
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


# ===== パラメータ =====
TOPIC_GPS = "/odom/UM982"
SYNC_TOL  = 1.0   # 時刻同期許容 [秒]


# ──────────────────────────────────────────
# rosbag から UM982 を (stamp, x, y, yaw) で読む
# ──────────────────────────────────────────
def read_um982(bag_path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry

    storage   = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC_GPS]))

    records = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, Odometry)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qx, qy, qz, qw = (msg.pose.pose.orientation.x,
                          msg.pose.pose.orientation.y,
                          msg.pose.pose.orientation.z,
                          msg.pose.pose.orientation.w)
        t3 = 2.0 * (qw * qz + qx * qy)
        t4 = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(t3, t4)
        records.append((stamp, x, y, yaw))
    records.sort(key=lambda r: r[0])
    return records


# ──────────────────────────────────────────
# GLIMダンプから (stamp, x, y) を読んでアライメント
# ──────────────────────────────────────────
def read_glim_aligned(dump_dir, x0, y0, yaw0):
    from data_source import GlimDumpSource

    cos_y, sin_y = math.cos(yaw0), math.sin(yaw0)
    source = GlimDumpSource(dump_dir)

    records = []
    for folder in source.list_frames():
        fd = source.get_frame(folder)
        if fd is None or fd.stamp is None:
            continue
        gx, gy = float(fd.position[0]), float(fd.position[1])
        ax = cos_y * gx - sin_y * gy + x0
        ay = sin_y * gx + cos_y * gy + y0
        records.append((float(fd.stamp), ax, ay))
    return records


# ──────────────────────────────────────────
# 時刻同期 & 誤差計算
# ──────────────────────────────────────────
def sync_and_compute_error(glim_aligned, gps_records, tol=SYNC_TOL):
    """各GLIMフレームに最近傍のGPSをマッチさせ、誤差を計算"""
    gps_stamps = np.array([r[0] for r in gps_records])

    rows = []
    for stamp, ax, ay in glim_aligned:
        idx = int(np.argmin(np.abs(gps_stamps - stamp)))
        if abs(gps_stamps[idx] - stamp) > tol:
            continue
        _, gx, gy, _ = gps_records[idx]
        err = math.hypot(gx - ax, gy - ay)
        rows.append(dict(stamp=stamp,
                         x_gps=gx, y_gps=gy,
                         x_glim_aligned=ax, y_glim_aligned=ay,
                         error=err))
    return rows


# ──────────────────────────────────────────
# プロット
# ──────────────────────────────────────────
def plot_results(gps_records, glim_aligned, sync_rows, out_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    # 軌跡比較
    fig, ax = plt.subplots(figsize=(10, 10))
    gps_xy   = np.array([(r[1], r[2]) for r in gps_records])
    glim_xy  = np.array([(r[1], r[2]) for r in glim_aligned])

    ax.plot(gps_xy[:, 0], gps_xy[:, 1],  'o-', color='red',  ms=3, lw=1,
            label=f'UM982 GPS (n={len(gps_xy)})')
    ax.plot(glim_xy[:, 0], glim_xy[:, 1], '.-', color='blue', ms=3, lw=1,
            label=f'GLIM aligned (n={len(glim_xy)})')
    ax.scatter(gps_xy[0, 0],  gps_xy[0, 1],  s=120, c='red',
               marker='s', edgecolors='k', label='GPS start', zorder=5)
    ax.scatter(glim_xy[0, 0], glim_xy[0, 1], s=120, c='blue',
               marker='^', edgecolors='k', label='GLIM start', zorder=5)
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.set_title('Trajectory: UM982 vs GLIM aligned (combine_glim equivalent)')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend()
    path = os.path.join(out_dir, 'trajectory_compare.png')
    fig.savefig(path, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f"  -> {path}")

    # 時系列誤差
    if sync_rows:
        fig, ax = plt.subplots(figsize=(12, 4))
        t0 = sync_rows[0]['stamp']
        ts   = [r['stamp'] - t0 for r in sync_rows]
        errs = [r['error'] for r in sync_rows]
        ax.plot(ts, errs, 'b.-', ms=4)
        ax.axhline(np.mean(errs), color='r', ls='--',
                   label=f'mean={np.mean(errs):.2f}m')
        ax.set_xlabel('time [s]'); ax.set_ylabel('error [m]')
        ax.set_title(f'GPS vs GLIM-aligned error  '
                     f'(mean={np.mean(errs):.2f}, max={np.max(errs):.2f} m)')
        ax.grid(True, alpha=0.3)
        ax.legend()
        path = os.path.join(out_dir, 'error_over_time.png')
        fig.savefig(path, dpi=120, bbox_inches='tight')
        plt.close(fig)
        print(f"  -> {path}")


def save_csv(sync_rows, out_dir):
    if not sync_rows:
        return
    path = os.path.join(out_dir, 'trajectory_compare.csv')
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(sync_rows[0].keys()))
        w.writeheader()
        w.writerows(sync_rows)
    print(f"  -> {path}")


# ──────────────────────────────────────────
# main
# ──────────────────────────────────────────
def main():
    p = argparse.ArgumentParser()
    p.add_argument('--dump', default='~/ros2_ws/dump/nakaniwa_0522')
    p.add_argument('--bag',  default='~/ros2_ws/bag/0522/nakaniwa_0522_bag/'
                             'rosbag2_2026_05_22-10_12_14_0.db3')
    p.add_argument('--out',  default='~/ros2_ws/dump/nakaniwa_0522/analysis/')
    args = p.parse_args()

    dump_dir = os.path.expanduser(args.dump)
    bag_path = os.path.expanduser(args.bag)
    out_dir  = os.path.expanduser(args.out)
    os.makedirs(out_dir, exist_ok=True)

    print(f"[1] UM982 読み出し: {bag_path}")
    gps_records = read_um982(bag_path)
    if not gps_records:
        print("ERROR: UM982 メッセージなし"); sys.exit(1)
    s, x0, y0, yaw0 = gps_records[0]
    print(f"  GPS={len(gps_records)} サンプル")
    print(f"  初期姿勢: x0={x0:.3f}, y0={y0:.3f}, yaw0={math.degrees(yaw0):.2f}°")

    print(f"\n[2] GLIM dump 読み出し & アライメント: {dump_dir}")
    glim_aligned = read_glim_aligned(dump_dir, x0, y0, yaw0)
    print(f"  GLIM aligned={len(glim_aligned)} サンプル")

    print(f"\n[3] 時刻同期 & 誤差計算 (tol={SYNC_TOL}s)")
    sync_rows = sync_and_compute_error(glim_aligned, gps_records)
    if sync_rows:
        errs = [r['error'] for r in sync_rows]
        print(f"  マッチ {len(sync_rows)} / {len(glim_aligned)}")
        print(f"  誤差: mean={np.mean(errs):.3f}m, "
              f"max={np.max(errs):.3f}m, min={np.min(errs):.3f}m")
    else:
        print("  WARN: マッチ0件")

    print(f"\n[4] 保存: {out_dir}")
    save_csv(sync_rows, out_dir)
    plot_results(gps_records, glim_aligned, sync_rows, out_dir)
    print("\n完了")


if __name__ == '__main__':
    main()
