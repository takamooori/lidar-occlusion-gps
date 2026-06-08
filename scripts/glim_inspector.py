#!/usr/bin/env python3
"""
GLIM Inspector - GLIMダンプデータの確認・可視化ツール

使い方:
    python3 glim_inspector.py
    python3 glim_inspector.py --dump ~/ros2_ws/dump/nakaniwa_0522

[3] でフレームの点群を RViz に流す際、以下のトピックも一緒に publish される:
    /debug_points     : 全点群
    /debug_upper      : フィルタ通過点（遮蔽計算の対象になる点）
    /debug_min_sphere : 近傍除去ゾーン（半透明赤の小球, r=MIN_DIST）
    /debug_max_sphere : 距離上限境界（青ワイヤー上半球, r=MAX_DIST）
    /debug_el_cone    : 仰角境界（黄ワイヤー円錐, el=EL_MIN_DEG）
  RViz 側で各トピックの ON/OFF を切り替えて比較できる。
"""

# ============================================================
# 設定（ここだけ変える）
# ============================================================
DEFAULT_DUMP = "~/ros2_ws/dump/nakaniwa_0522"
MAX_DIST     = 30.0   # 距離上限 [m]
MIN_DIST     = 2.0    # 近傍除去 [m]（人間・自己反射対策）
EL_MIN_DEG   = 15.0   # 仰角カットオフ [度]（GPS mask角に対応）
# ============================================================

import argparse
import os
import re
import sys
import time
import threading

import numpy as np


# ============================================================
# ユーティリティ
# ============================================================

def get_folders(dump_dir):
    return sorted([
        f for f in os.listdir(dump_dir)
        if re.match(r'^\d{6}$', f) and os.path.isdir(os.path.join(dump_dir, f))
    ])


def parse_frame0_pose(data_txt):
    with open(data_txt) as f:
        content = f.read()
    mat44 = (
        r'\s*([\d.e+\-]+)\s+([\d.e+\-]+)\s+([\d.e+\-]+)\s+([\d.e+\-]+)\s*\n'
        r'\s*([\d.e+\-]+)\s+([\d.e+\-]+)\s+([\d.e+\-]+)\s+([\d.e+\-]+)\s*\n'
        r'\s*([\d.e+\-]+)\s+([\d.e+\-]+)\s+([\d.e+\-]+)\s+([\d.e+\-]+)\s*\n'
        r'\s*([\d.e+\-]+)\s+([\d.e+\-]+)\s+([\d.e+\-]+)\s+([\d.e+\-]+)'
    )
    m = re.search(r'frame_0.*?T_world_lidar:\s*\n' + mat44, content, re.DOTALL)
    if not m:
        return None
    return np.array([float(v) for v in m.groups()]).reshape(4, 4)


def load_points(dump_dir, folder):
    return np.fromfile(
        os.path.join(dump_dir, folder, "points_compact.bin"),
        dtype=np.float32
    ).reshape(-1, 3)


def filter_points(pts, min_dist=MIN_DIST, max_dist=MAX_DIST, el_min_deg=EL_MIN_DEG):
    """
    遮蔽計算の有効フィルタを points に適用する。
    occlusion_core.compute_occlusion の in_range と同じロジックを再現:
      Z > 0  かつ  min_dist < dist < max_dist  かつ  elevation > el_min_deg
    """
    dists = np.linalg.norm(pts, axis=1)
    safe  = np.maximum(dists, 1e-6)
    sin_el = np.sin(np.deg2rad(el_min_deg))
    mask = (
        (pts[:, 2] > 0)
        & (pts[:, 2] / safe > sin_el)
        & (dists > min_dist)
        & (dists < max_dist)
    )
    return mask


def make_cloud_msg(pts, frame_id="map"):
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header
    msg = PointCloud2()
    msg.header = Header()
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(pts)
    msg.fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(pts)
    msg.is_dense = True
    msg.data = pts.astype(np.float32).tobytes()
    return msg


# ============================================================
# Marker ヘルパ（フィルタ境界の可視化）
# ============================================================

def _new_marker(marker_type, marker_id, color_rgba, ns="filter",
                frame_id="map", line_width=0.05):
    """Markerメッセージの共通初期化。"""
    from visualization_msgs.msg import Marker
    m = Marker()
    m.header.frame_id = frame_id
    m.ns = ns
    m.id = int(marker_id)
    m.type = marker_type
    m.action = Marker.ADD
    m.scale.x = float(line_width)
    m.pose.position.x = 0.0
    m.pose.position.y = 0.0
    m.pose.position.z = 0.0
    m.pose.orientation.x = 0.0
    m.pose.orientation.y = 0.0
    m.pose.orientation.z = 0.0
    m.pose.orientation.w = 1.0
    r, g, b, a = color_rgba
    m.color.r = float(r); m.color.g = float(g)
    m.color.b = float(b); m.color.a = float(a)
    return m


def make_sphere_marker(radius, color_rgba, marker_id):
    """半透明solid球。MIN_DIST用（近傍除去ゾーンをベタ塗りで示す）。"""
    from visualization_msgs.msg import Marker
    m = _new_marker(Marker.SPHERE, marker_id, color_rgba)
    # SPHERE の scale は直径
    m.scale.x = float(2.0 * radius)
    m.scale.y = float(2.0 * radius)
    m.scale.z = float(2.0 * radius)
    return m


def make_wireframe_hemisphere_marker(radius, color_rgba, marker_id,
                                     n_lat=5, n_lon=12,
                                     seg_ring=36, seg_meridian=18):
    """上半球のワイヤーフレーム。LINE_LIST型。
       n_lat        : 緯度リング本数（φ=0:赤道 〜 φ→π/2:北極方向）
       n_lon        : 経度メリディアン本数
       seg_ring     : 緯度リングの分割数（円の滑らかさ）
       seg_meridian : 経度メリディアンの分割数
    """
    from visualization_msgs.msg import Marker
    from geometry_msgs.msg import Point
    m = _new_marker(Marker.LINE_LIST, marker_id, color_rgba, line_width=0.05)

    pts = []
    # 緯度リング（φ = 0..π/2 を n_lat等分。i=0 が赤道、i=n_lat 近くが極）
    for i in range(n_lat):
        phi = (np.pi / 2.0) * i / n_lat
        z  = radius * np.sin(phi)
        hr = radius * np.cos(phi)
        for j in range(seg_ring):
            t1 = 2.0 * np.pi * j / seg_ring
            t2 = 2.0 * np.pi * (j + 1) / seg_ring
            p1 = Point(); p1.x = float(hr*np.cos(t1)); p1.y = float(hr*np.sin(t1)); p1.z = float(z)
            p2 = Point(); p2.x = float(hr*np.cos(t2)); p2.y = float(hr*np.sin(t2)); p2.z = float(z)
            pts.append(p1); pts.append(p2)

    # 経度メリディアン（θを n_lon等分、φ:0→π/2 で半円を描画）
    for k in range(n_lon):
        theta = 2.0 * np.pi * k / n_lon
        ct = np.cos(theta); st = np.sin(theta)
        for j in range(seg_meridian):
            phi1 = (np.pi / 2.0) * j / seg_meridian
            phi2 = (np.pi / 2.0) * (j + 1) / seg_meridian
            p1 = Point()
            p1.x = float(radius*np.cos(phi1)*ct)
            p1.y = float(radius*np.cos(phi1)*st)
            p1.z = float(radius*np.sin(phi1))
            p2 = Point()
            p2.x = float(radius*np.cos(phi2)*ct)
            p2.y = float(radius*np.cos(phi2)*st)
            p2.z = float(radius*np.sin(phi2))
            pts.append(p1); pts.append(p2)

    m.points = pts
    return m


def make_cone_wireframe_marker(el_deg, r_max, color_rgba, marker_id,
                               n_rings=3, n_meridians=24, seg_ring=36):
    """円錐ワイヤー（仰角el_degの境界面）。LINE_LIST型。
       中心軸=+Z, apex=原点, 開口部=上方（仰角の境界を示す）。
       n_rings     : 半径方向のリング本数（r_max を n_rings 等分）
       n_meridians : 母線の本数
       seg_ring    : リングの分割数（円の滑らかさ）
    """
    from visualization_msgs.msg import Marker
    from geometry_msgs.msg import Point
    m = _new_marker(Marker.LINE_LIST, marker_id, color_rgba, line_width=0.05)

    el_rad = np.deg2rad(el_deg)
    cos_el = float(np.cos(el_rad))
    sin_el = float(np.sin(el_rad))

    pts = []

    # 円錐面上のリング（複数半径）
    for ring_i in range(1, n_rings + 1):
        r  = r_max * ring_i / n_rings
        hr = r * cos_el
        z  = r * sin_el
        for j in range(seg_ring):
            t1 = 2.0 * np.pi * j / seg_ring
            t2 = 2.0 * np.pi * (j + 1) / seg_ring
            p1 = Point(); p1.x = float(hr*np.cos(t1)); p1.y = float(hr*np.sin(t1)); p1.z = float(z)
            p2 = Point(); p2.x = float(hr*np.cos(t2)); p2.y = float(hr*np.sin(t2)); p2.z = float(z)
            pts.append(p1); pts.append(p2)

    # 母線（原点から外周まで）
    for k in range(n_meridians):
        theta = 2.0 * np.pi * k / n_meridians
        p1 = Point(); p1.x = 0.0; p1.y = 0.0; p1.z = 0.0
        p2 = Point()
        p2.x = float(r_max * cos_el * np.cos(theta))
        p2.y = float(r_max * cos_el * np.sin(theta))
        p2.z = float(r_max * sin_el)
        pts.append(p1); pts.append(p2)

    m.points = pts
    return m


# ============================================================
# RViz 設定生成
# ============================================================

def generate_rviz_config(dump_dir, topics):
    type_map  = {
        "PointCloud2": "rviz_default_plugins/PointCloud2",
        "Path":        "rviz_default_plugins/Path",
        "Marker":      "rviz_default_plugins/Marker",
    }
    color_map = {
        "/glim_map":     "r: 1.0\n          g: 1.0\n          b: 1.0",
        "/debug_points": "r: 0.0\n          g: 1.0\n          b: 0.5",
        "/debug_upper":  "r: 1.0\n          g: 0.8\n          b: 0.0",
        "/glim_path":    "r: 1.0\n          g: 0.2\n          b: 0.2",
    }
    displays = ""
    for dtype, topic in topics:
        plugin = type_map.get(dtype, f"rviz_default_plugins/{dtype}")
        if dtype == "Marker":
            # Marker は色をメッセージ側で持つので Color block 不要
            displays += f"""
    - Class: {plugin}
      Enabled: true
      Name: {topic}
      Topic:
        Value: {topic}
"""
        else:
            col   = color_map.get(topic, "r: 1.0\n          g: 1.0\n          b: 1.0")
            extra = "\n        Color Transformer: AxisColor\n        Axis: Z\n        Size (m): 0.05" \
                    if dtype == "PointCloud2" else ""
            displays += f"""
    - Class: {plugin}
      Enabled: true
      Name: {topic}
      Topic:
        Value: {topic}
      Color:
        {col}{extra}
"""
    dump_name = os.path.basename(dump_dir.rstrip('/'))
    rviz_path = os.path.join(dump_dir, f"{dump_name}.rviz")
    with open(rviz_path, 'w') as f:
        f.write(f"""Visualization Manager:
  Class: ""
  Displays:{displays}
  Enabled: true
  Global Options:
    Fixed Frame: map
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
Window Geometry:
  Height: 900
  Width: 1400
""")
    print(f"RViz設定: {rviz_path}")
    print(f"起動:     rviz2 -d {rviz_path}\n")


# ============================================================
# フレーム切り替えpublishループ
# ============================================================

def interactive_publish(dump_dir, folders, start_idx, build_msgs_fn):
    """
    build_msgs_fn(node, idx) -> [(publisher, msg), ...]
    PointCloud2 / Marker 両方を扱う。msg.header.stamp は両者に存在するので共通処理可。
    publishしながら [n]/[p]/数字 でフレームを切り替える
    """
    import rclpy

    rclpy.init()
    node = rclpy.create_node("glim_interactive_pub")

    state = {
        "idx":  start_idx,
        "msgs": build_msgs_fn(node, start_idx),
        "stop": False,
    }

    def publish_worker():
        while not state["stop"] and rclpy.ok():
            now = node.get_clock().now().to_msg()
            for pub, msg in state["msgs"]:
                msg.header.stamp = now
                pub.publish(msg)
            time.sleep(0.5)

    t = threading.Thread(target=publish_worker, daemon=True)
    t.start()

    print("publish開始 — rviz2 を別ターミナルで起動してください")
    while True:
        folder = folders[state["idx"]]
        print(f"\n現在: [{state['idx']:3d}] {folder}   "
              f"[n]次  [p]前  [数字]直接指定  [q]終了")
        cmd = input("> ").strip().lower()

        if cmd == 'q':
            state["stop"] = True
            break
        elif cmd == 'n':
            new_idx = min(state["idx"] + 1, len(folders) - 1)
        elif cmd == 'p':
            new_idx = max(state["idx"] - 1, 0)
        elif cmd.isdigit() and 0 <= int(cmd) < len(folders):
            new_idx = int(cmd)
        else:
            print("無効な入力")
            continue

        state["idx"]  = new_idx
        state["msgs"] = build_msgs_fn(node, new_idx)
        print(f"→ フレーム {folders[new_idx]} に切り替え")

    rclpy.shutdown()


# ============================================================
# 各メニュー機能
# ============================================================

def menu_summary(dump_dir, folders):
    print(f"\n--- データ概要: {os.path.basename(dump_dir)} ---")
    positions, total_pts = [], 0
    for folder in folders:
        T = parse_frame0_pose(os.path.join(dump_dir, folder, "data.txt"))
        if T is not None:
            positions.append(T[:3, 3])
        pts_bin = os.path.join(dump_dir, folder, "points_compact.bin")
        if os.path.exists(pts_bin):
            total_pts += os.path.getsize(pts_bin) // 12
    positions = np.array(positions)
    travel = np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1))
    print(f"総キーフレーム数: {len(folders)}")
    print(f"取得ポーズ数:     {len(positions)}")
    print(f"総移動距離:       {travel:.1f} m")
    print(f"X範囲: {positions[:,0].min():.1f} 〜 {positions[:,0].max():.1f} m")
    print(f"Y範囲: {positions[:,1].min():.1f} 〜 {positions[:,1].max():.1f} m")
    print(f"Z範囲: {positions[:,2].min():.1f} 〜 {positions[:,2].max():.1f} m")
    print(f"総点数（概算）:   {total_pts:,} 点")


def menu_coord_check(dump_dir, folders, frame_idx):
    folder    = folders[frame_idx]
    pts       = load_points(dump_dir, folder)
    T         = parse_frame0_pose(os.path.join(dump_dir, folder, "data.txt"))
    lidar_pos = T[:3, 3]
    centroid  = pts.mean(axis=0)
    diff      = np.linalg.norm(centroid - lidar_pos)
    print(f"\n--- 座標系判定: {folder} ---")
    print(f"点数:            {len(pts)}")
    print(f"点群centroid:    {centroid.round(2)}")
    print(f"LiDAR world位置: {lidar_pos.round(2)}")
    print(f"差（ノルム）:    {diff:.2f} m")
    print("→ LiDARローカル座標（正常）" if diff < 5.0 else "→ world座標の可能性あり（要確認）")


def menu_dist_stats(dump_dir, folders, frame_idx):
    folder = folders[frame_idx]
    pts    = load_points(dump_dir, folder)
    dists  = np.linalg.norm(pts, axis=1)
    upper  = (pts[:,2] > 0).sum()
    print(f"\n--- 距離分布: {folder} ---")
    print(f"全点数: {len(pts)}  max={dists.max():.1f}m  mean={dists.mean():.1f}m\n")
    for d in [10, 20, 30, 40, 50, 60, 80, 100]:
        n   = (dists < d).sum()
        bar = '█' * int(30 * n / len(pts))
        print(f"  dist < {d:3d}m: {n:6d}点 ({100*n/len(pts):5.1f}%) {bar}")
    print(f"\n上半球(Z>0)={upper}点 に対するMAX_DIST別カバー率:")
    for d in [30, 40, 50, 60]:
        n    = ((pts[:,2] > 0) & (dists < d)).sum()
        mark = " ← 現在の設定" if d == MAX_DIST else ""
        print(f"  MAX_DIST={d}m: {100*n/max(upper,1):.1f}%{mark}")

    # 新フィルタ通過点数（MIN_DIST + EL_MIN_DEG を含む実フィルタ）
    safe   = np.maximum(dists, 1e-6)
    sin_el = np.sin(np.deg2rad(EL_MIN_DEG))
    near_removed = int(((pts[:,2] > 0) & (dists <= MIN_DIST)).sum())
    el_removed   = int(((pts[:,2] > 0) & (dists > MIN_DIST) &
                        (pts[:,2]/safe <= sin_el)).sum())
    passed = int(filter_points(pts).sum())
    print(f"\n現在のフィルタ通過状況 "
          f"(MIN={MIN_DIST}m, MAX={MAX_DIST}m, EL_MIN={EL_MIN_DEG}°):")
    print(f"  上半球Z>0 のうち,")
    print(f"   - 近傍除去 (dist≤{MIN_DIST}m):  {near_removed:6d}点")
    print(f"   - 仰角除去 (el≤{EL_MIN_DEG}°):   {el_removed:6d}点")
    print(f"   - 最終通過 (遮蔽計算対象):       {passed:6d}点 "
          f"({100*passed/max(upper,1):.1f}% of upper)")


def menu_frame_rviz(dump_dir, folders, start_idx):
    """[3] 点群 → RViz（全点 + 上半球 + 境界Marker、フレーム切り替えあり）"""
    from sensor_msgs.msg import PointCloud2
    from visualization_msgs.msg import Marker

    print(f"フィルタ: MIN_DIST={MIN_DIST}m, MAX_DIST={MAX_DIST}m, EL_MIN={EL_MIN_DEG}°"
          f"（変更はスクリプト冒頭の設定を編集）")
    generate_rviz_config(dump_dir, [
        ("PointCloud2", "/debug_points"),
        ("PointCloud2", "/debug_upper"),
        ("Marker",      "/debug_min_sphere"),
        ("Marker",      "/debug_max_sphere"),
        ("Marker",      "/debug_el_cone"),
    ])
    print("RViz2 でトピックのチェックを切り替えて表示を選択してください")
    print(f"  /debug_points     : 全点群")
    print(f"  /debug_upper      : フィルタ通過点（遮蔽計算の対象）")
    print(f"  /debug_min_sphere : 近傍除去ゾーン (赤・半透明solid球, r={MIN_DIST}m)")
    print(f"  /debug_max_sphere : 距離上限境界 (青ワイヤー上半球, r={MAX_DIST}m)")
    print(f"  /debug_el_cone    : 仰角境界 (黄ワイヤー円錐, el={EL_MIN_DEG}°)\n")

    pubs = {}

    def build_msgs(node, idx):
        if "all" not in pubs:
            pubs["all"]     = node.create_publisher(PointCloud2, "/debug_points",     10)
            pubs["upper"]   = node.create_publisher(PointCloud2, "/debug_upper",      10)
            pubs["min_sph"] = node.create_publisher(Marker,      "/debug_min_sphere", 10)
            pubs["max_sph"] = node.create_publisher(Marker,      "/debug_max_sphere", 10)
            pubs["el_cone"] = node.create_publisher(Marker,      "/debug_el_cone",    10)

        pts  = load_points(dump_dir, folders[idx])
        mask = filter_points(pts)
        upper = pts[mask]
        print(f"  全{len(pts):,}点 / フィルタ通過{len(upper):,}点 "
              f"({100*len(upper)/max(len(pts),1):.1f}%)")

        # 境界Marker（フレームに依らず固定だが、stamp更新のため毎回作る）
        min_mk  = make_sphere_marker(MIN_DIST,
                                      (1.0, 0.25, 0.25, 0.25), 0)
        max_mk  = make_wireframe_hemisphere_marker(MAX_DIST,
                                                    (0.3, 0.6, 1.0, 0.7), 1)
        cone_mk = make_cone_wireframe_marker(EL_MIN_DEG, MAX_DIST,
                                              (1.0, 0.95, 0.3, 0.8), 2)

        return [
            (pubs["all"],     make_cloud_msg(pts)),
            (pubs["upper"],   make_cloud_msg(upper)),
            (pubs["min_sph"], min_mk),
            (pubs["max_sph"], max_mk),
            (pubs["el_cone"], cone_mk),
        ]

    interactive_publish(dump_dir, folders, start_idx, build_msgs)


def menu_full_map_rviz(dump_dir, folders):
    """[4] 全軌跡 + マップ → RViz"""
    import rclpy
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Header

    stride_kf  = int(input("キーフレーム間引き (デフォルト3): ").strip() or "3")
    stride_pts = int(input("点群間引き        (デフォルト10): ").strip() or "10")

    print("データ読み込み中...")
    all_poses, all_pts_world = [], []

    for folder in folders:
        T = parse_frame0_pose(os.path.join(dump_dir, folder, "data.txt"))
        if T is not None:
            all_poses.append((folder, T))

    for i, (folder, T) in enumerate(all_poses):
        if i % stride_kf != 0:
            continue
        pts_bin = os.path.join(dump_dir, folder, "points_compact.bin")
        if not os.path.exists(pts_bin):
            continue
        pts   = np.fromfile(pts_bin, dtype=np.float32).reshape(-1, 3)[::stride_pts]
        pts_h = np.hstack([pts, np.ones((len(pts), 1))])
        all_pts_world.append((T @ pts_h.T).T[:, :3].astype(np.float32))

    all_pts_world = np.vstack(all_pts_world)
    print(f"表示点数: {len(all_pts_world):,}")

    generate_rviz_config(dump_dir, [
        ("PointCloud2", "/glim_map"),
        ("Path",        "/glim_path"),
    ])

    path_msg = Path()
    path_msg.header = Header()
    path_msg.header.frame_id = "map"
    for _, T in all_poses:
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.pose.position.x = float(T[0, 3])
        ps.pose.position.y = float(T[1, 3])
        ps.pose.position.z = float(T[2, 3])
        ps.pose.orientation.w = 1.0
        path_msg.poses.append(ps)

    cloud_msg = make_cloud_msg(all_pts_world)

    rclpy.init()
    node     = rclpy.create_node("glim_map_pub")
    pub_map  = node.create_publisher(PointCloud2, "/glim_map",  10)
    pub_path = node.create_publisher(Path,        "/glim_path", 10)

    print("publish中... (Ctrl+C で終了)")
    while rclpy.ok():
        now = node.get_clock().now().to_msg()
        cloud_msg.header.stamp = now
        path_msg.header.stamp  = now
        pub_map.publish(cloud_msg)
        pub_path.publish(path_msg)
        time.sleep(1.0)


# ============================================================
# フレーム選択
# ============================================================

def select_frame(folders):
    print(f"\nフレーム選択 (総数: {len(folders)}):")
    print(f"  [a] 最初  ({folders[0]})")
    print(f"  [m] 中間  ({folders[len(folders)//2]})")
    print(f"  [e] 最後  ({folders[-1]})")
    print(f"  数字で直接指定 (0 〜 {len(folders)-1})")
    choice = input("選択 > ").strip().lower()
    if choice == 'a':
        return 0
    elif choice == 'm':
        return len(folders) // 2
    elif choice == 'e':
        return len(folders) - 1
    elif choice.isdigit() and 0 <= int(choice) < len(folders):
        return int(choice)
    else:
        print("無効な入力。最初のフレームを使用")
        return 0


# ============================================================
# メインメニュー
# ============================================================

def main():
    parser = argparse.ArgumentParser(description="GLIM Dump Inspector")
    parser.add_argument('--dump', default=os.path.expanduser(DEFAULT_DUMP))
    args = parser.parse_args()

    dump_dir = os.path.expanduser(args.dump)
    if not os.path.isdir(dump_dir):
        print(f"エラー: {dump_dir} が見つかりません")
        sys.exit(1)

    folders = get_folders(dump_dir)
    if not folders:
        print("キーフレームが見つかりません")
        sys.exit(1)

    while True:
        print(f"""
=== GLIM Inspector ===
DUMP    : {dump_dir}
フレーム: {len(folders)} 個 ({folders[0]} 〜 {folders[-1]})
フィルタ: MIN_DIST={MIN_DIST}m, MAX_DIST={MAX_DIST}m, EL_MIN={EL_MIN_DEG}°
        （変更はスクリプト冒頭）

  [1] データ概要確認
  [2] 座標系判定
  [3] 点群 + 境界Marker → RViz  (/debug_points + /debug_upper + 球/円錐境界)
                                  ← フレーム切り替えあり
  [4] 全軌跡 + マップ → RViz (/glim_path + /glim_map)
  [5] 距離分布 + フィルタ通過状況確認
  [q] 終了
""")
        choice = input("選択 > ").strip().lower()

        if choice == 'q':
            print("終了")
            break
        elif choice == '1':
            menu_summary(dump_dir, folders)
        elif choice == '2':
            menu_coord_check(dump_dir, folders, select_frame(folders))
        elif choice == '3':
            menu_frame_rviz(dump_dir, folders, select_frame(folders))
        elif choice == '4':
            menu_full_map_rviz(dump_dir, folders)
        elif choice == '5':
            menu_dist_stats(dump_dir, folders, select_frame(folders))
        else:
            print("無効な入力です")


if __name__ == '__main__':
    main()
