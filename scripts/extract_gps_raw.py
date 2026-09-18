#!/usr/bin/env python3
"""/gps_raw (+ /gnss/ppp_status) -> CSV

GGA quality と PPP のσを1本のCSVにまとめる。
平面直角座標（既定 EPSG:6677 = 第IX系）と ENU yaw も出力するので、
analysis_pipeline.py の --gps-csv にそのまま渡せる。
"""

import argparse
import bisect
import csv
import math
import re
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

SENTENCE_RE = re.compile(r"(HDT|GGA)\s*:\s*std_msgs\.msg\.String\(data='([^']*)'\)")

QUALITY_LABEL = {
    0: "invalid", 1: "single", 2: "dgps", 3: "pps", 4: "rtk_fix",
    5: "rtk_float", 6: "dead_reckoning", 7: "manual", 8: "simulation",
}

PPP_FIELDS = ["pos_type", "lat_sd", "lon_sd", "alt_sd",
              "diff_age", "sol_age", "num_tracked", "num_used", "ppp_valid"]


def dm_to_deg(value, hemi):
    if not value:
        return None
    dot = value.find(".")
    deg_len = dot - 2 if dot >= 2 else len(value) - 2
    dec = float(value[:deg_len]) + float(value[deg_len:]) / 60.0
    return -dec if hemi in ("S", "W") else dec


def to_float(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def to_int(s):
    try:
        return int(s)
    except (TypeError, ValueError):
        return None


def parse_gga(sentence):
    f = sentence.split("*")[0].split(",")
    if len(f) < 15 or "GGA" not in f[0]:
        return None
    return {
        "utc": f[1],
        "lat_deg": dm_to_deg(f[2], f[3]),
        "lon_deg": dm_to_deg(f[4], f[5]),
        "quality": to_int(f[6]),
        "quality_label": QUALITY_LABEL.get(to_int(f[6]), "unknown"),
        "num_sv": to_int(f[7]),
        "hdop": to_float(f[8]),
        "alt_m": to_float(f[9]),
        "geoid_m": to_float(f[11]),
        "diff_age_s": to_float(f[13]),
        "station_id": f[14],
    }


def parse_hdt(sentence):
    f = sentence.split("*")[0].split(",")
    return to_float(f[1]) if len(f) >= 2 else None


def hdt_to_enu_yaw(hdt_deg, offset_deg=0.0):
    if hdt_deg is None:
        return None
    yaw = math.pi / 2.0 - math.radians(hdt_deg + offset_deg)
    return (yaw + math.pi) % (2 * math.pi) - math.pi


def open_reader(bag_path, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return reader


def read_ppp(bag_path, topic, pkg="orange_msgs"):
    """/gnss/ppp_status -> [(stamp, dict)]。orange_msgs が無ければ None。"""
    try:
        from importlib import import_module
        PppNav = getattr(import_module(f"{pkg}.msg"), "PppNav")
    except ImportError:
        print(f"WARNING: {pkg} not found - skipping {topic}", file=sys.stderr)
        return None

    records = []
    reader = open_reader(bag_path, [topic])
    while reader.has_next():
        _, data, t_ns = reader.read_next()
        m = deserialize_message(data, PppNav)
        valid = bool(m.valid)
        records.append((t_ns / 1e9, {
            "pos_type": m.pos_type,
            "lat_sd": m.lat_sd if valid else None,
            "lon_sd": m.lon_sd if valid else None,
            "alt_sd": m.alt_sd if valid else None,
            "diff_age": getattr(m, "diff_age", None),
            "sol_age": m.sol_age,
            "num_tracked": int(m.num_tracked),
            "num_used": int(m.num_used),
            "ppp_valid": int(valid),
        }))
    records.sort(key=lambda r: r[0])
    return records


def nearest(stamps, records, query):
    i = bisect.bisect_left(stamps, query)
    cands = [j for j in (i - 1, i) if 0 <= j < len(records)]
    if not cands:
        return None, None
    j = min(cands, key=lambda k: abs(stamps[k] - query))
    return records[j][1], stamps[j] - query


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("-t", "--topic", default="/gps_raw")
    ap.add_argument("--ppp-topic", default="/gnss/ppp_status")
    ap.add_argument("--no-ppp", action="store_true")
    ap.add_argument("--ppp-pkg", default="orange_msgs",
                    help="PppNav を提供するパッケージ名（録画時の定義に合わせる）")
    ap.add_argument("--ppp-tol", type=float, default=1.0,
                    help="PPP突合の許容時間差 [s]")
    ap.add_argument("-o", "--out", default="gps_raw.csv")
    ap.add_argument("--epsg", type=int, default=6677)
    ap.add_argument("--hdt-offset", type=float, default=-90.0,
                    help="HDT(基線方位) から機体前方への補正 [deg]。左右2アンテナ配置なら -90")
    args = ap.parse_args()

    from pyproj import Transformer
    tf = Transformer.from_crs("EPSG:6668", f"EPSG:{args.epsg}", always_xy=True)

    ppp = None if args.no_ppp else read_ppp(args.bag, args.ppp_topic, args.ppp_pkg)
    ppp_stamps = [r[0] for r in ppp] if ppp else []

    rows, n_skipped, n_ppp_miss = [], 0, 0
    reader = open_reader(args.bag, [args.topic])
    while reader.has_next():
        _, data, t_ns = reader.read_next()
        payload = deserialize_message(data, String).data
        found = dict(SENTENCE_RE.findall(payload))
        gga = parse_gga(found["GGA"]) if "GGA" in found else None
        if gga is None or gga["lat_deg"] is None:
            n_skipped += 1
            continue

        gga["t_ns"] = t_ns
        gga["stamp"] = t_ns / 1e9
        gga["x"], gga["y"] = tf.transform(gga["lon_deg"], gga["lat_deg"])
        hdt = parse_hdt(found["HDT"]) if "HDT" in found else None
        gga["hdt_deg"] = hdt
        gga["yaw"] = hdt_to_enu_yaw(hdt, args.hdt_offset)

        if ppp:
            rec, dt = nearest(ppp_stamps, ppp, gga["stamp"])
            if rec is not None and abs(dt) <= args.ppp_tol:
                gga.update(rec)
                gga["ppp_dt"] = round(dt, 4)
            else:
                n_ppp_miss += 1
        rows.append(gga)

    if not rows:
        sys.exit("no GGA parsed - check the topic name and payload format")

    cols = ["stamp", "t_ns", "utc", "x", "y", "yaw", "lat_deg", "lon_deg",
            "quality", "quality_label", "num_sv", "hdop", "alt_m",
            "geoid_m", "diff_age_s", "station_id", "hdt_deg"]
    if ppp:
        cols += PPP_FIELDS + ["ppp_dt"]

    with open(args.out, "w", newline="") as fp:
        w = csv.DictWriter(fp, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)

    print(f"wrote {args.out}  rows={len(rows)}  skipped={n_skipped}")
    print(f"duration: {rows[-1]['stamp'] - rows[0]['stamp']:.1f} s")
    print(f"stamp range: {rows[0]['stamp']:.3f} .. {rows[-1]['stamp']:.3f}")
    print(f"origin (EPSG:{args.epsg}): x={rows[0]['x']:.3f} y={rows[0]['y']:.3f}")

    counts = {}
    for r in rows:
        key = f"{r['quality']}:{r['quality_label']}"
        counts[key] = counts.get(key, 0) + 1
    print("GGA quality distribution:")
    for k in sorted(counts):
        print(f"  {k:20s} {counts[k]:5d}  ({100*counts[k]/len(rows):5.1f}%)")

    if ppp:
        print(f"PPP: {len(ppp)} msgs, unmatched={n_ppp_miss}")
        pt = {}
        for r in rows:
            pt[r.get("pos_type", "-")] = pt.get(r.get("pos_type", "-"), 0) + 1
        print("pos_type distribution:")
        for k in sorted(pt):
            print(f"  {str(k):20s} {pt[k]:5d}  ({100*pt[k]/len(rows):5.1f}%)")
        sd = [r["lat_sd"] for r in rows if r.get("lat_sd") is not None]
        if sd:
            sd_s = sorted(sd)
            print(f"lat_sd [m]: min={sd_s[0]:.3f} med={sd_s[len(sd_s)//2]:.3f} "
                  f"max={sd_s[-1]:.3f}  (valid={len(sd)}/{len(rows)})")


if __name__ == "__main__":
    main()
