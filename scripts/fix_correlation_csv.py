#!/usr/bin/env python3
"""
fix_correlation_csv.py
======================
trajectory_compare.csv のアライメント済み error を使って
correlation.csv の gps_error 列を正しい値に書き換える。
"""
import os, sys, pandas as pd, numpy as np

OUT_DIR = os.path.expanduser("~/ros2_ws/dump/nakaniwa_0522/analysis/")

occ_df  = pd.read_csv(os.path.join(OUT_DIR, "occlusion_rate.csv"))
traj_df = pd.read_csv(os.path.join(OUT_DIR, "trajectory_compare.csv"))

# stampでマージ（最近傍）
occ_stamps  = occ_df["stamp"].values
traj_stamps = traj_df["stamp"].values

rows = []
for _, o in occ_df.iterrows():
    idx = int(np.argmin(np.abs(traj_stamps - o["stamp"])))
    if abs(traj_stamps[idx] - o["stamp"]) > 1.0:
        continue
    t = traj_df.iloc[idx]
    rows.append(dict(
        stamp=o["stamp"],
        occlusion_rate=o["occlusion_rate"],
        gps_error=t["error"],
        x_glim=t["x_glim_aligned"], y_glim=t["y_glim_aligned"],
        x_gps=t["x_gps"], y_gps=t["y_gps"],
    ))

out = pd.DataFrame(rows)
out_path = os.path.join(OUT_DIR, "correlation.csv")
out.to_csv(out_path, index=False)
print(f"上書き: {out_path} ({len(out)}行)")
print(f"  gps_error: mean={out['gps_error'].mean():.3f}m, "
      f"max={out['gps_error'].max():.3f}m")
