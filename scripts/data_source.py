#!/usr/bin/env python3
"""
data_source.py - 点群データソース抽象化レイヤ
================================================
遮蔽率計算のための「フレームごとの点群 + 姿勢」を供給する共通インターフェース。

設計意図:
  現行は GLIM dump (points_compact.bin) を使うが、将来 rosbag 生点群 +
  traj_lidar.txt の姿勢に移行する。その際にこのファイルへ RosbagSource を
  足すだけで、上位ツール（遮蔽率計算・可視化・相関分析）は一切変更不要に
  なるよう、データ取得を 1 箇所に閉じ込めている。

提供するインターフェース (FrameData):
  - folder      : フレーム識別子 (str)
  - stamp       : UNIX時刻 [秒] (float)
  - T_world_lidar : world座標系でのLiDAR姿勢 4x4 (np.ndarray)
  - points_lidar  : LiDARローカル座標の点群 (N,3) (np.ndarray)
  - coord_note  : 座標系の由来メモ (str, デバッグ用)

重要な座標系ルール:
  上位ツールは「points_lidar は必ず LiDARローカル座標」という前提で動く。
  各 Source はこのルールを保証する責任を持つ。dump が world 座標で
  入っていれば Source 側で T^-1 を掛けて lidar 座標へ落とす。
"""

import os
import re
import numpy as np
from dataclasses import dataclass, field
from typing import Optional, List


@dataclass
class FrameData:
    """1フレーム分の点群と姿勢。points_lidar は必ず LiDARローカル座標。"""
    folder: str
    stamp: Optional[float]
    T_world_lidar: np.ndarray          # 4x4
    points_lidar: np.ndarray            # (N, 3) LiDARローカル座標
    coord_note: str = ""

    @property
    def position(self) -> np.ndarray:
        """world座標でのLiDAR位置 (x, y, z)"""
        return self.T_world_lidar[:3, 3]


# ----------------------------------------------------------------------
#  共通パーサ
# ----------------------------------------------------------------------
_MAT44 = (
    r'\s*([\d.e+\-]+)\s+([\d.e+\-]+)\s+([\d.e+\-]+)\s+([\d.e+\-]+)\s*\n'
    r'\s*([\d.e+\-]+)\s+([\d.e+\-]+)\s+([\d.e+\-]+)\s+([\d.e+\-]+)\s*\n'
    r'\s*([\d.e+\-]+)\s+([\d.e+\-]+)\s+([\d.e+\-]+)\s+([\d.e+\-]+)\s*\n'
    r'\s*([\d.e+\-]+)\s+([\d.e+\-]+)\s+([\d.e+\-]+)\s+([\d.e+\-]+)'
)


def parse_pose_and_stamp(data_txt_path):
    """
    data.txt から frame_0 の stamp と姿勢行列を取得する。
    T_world_lidar を優先し、無ければ T_odom_lidar を使う。
    Returns: (stamp, T_world_lidar, used_key)
    """
    with open(data_txt_path, 'r') as f:
        content = f.read()

    stamp_match = re.search(r'stamp:\s*([\d.]+)', content)
    stamp = float(stamp_match.group(1)) if stamp_match else None

    # T_world_lidar を優先（ループ閉合後の値）
    m = re.search(r'T_world_lidar:\s*\n' + _MAT44, content)
    used_key = "T_world_lidar"
    if not m:
        m = re.search(r'T_odom_lidar:\s*\n' + _MAT44, content)
        used_key = "T_odom_lidar"
    if not m:
        return stamp, None, None

    T = np.array([float(v) for v in m.groups()]).reshape(4, 4)
    return stamp, T, used_key


def detect_coord_frame(points, T_world_lidar, threshold=5.0):
    """
    点群が lidar座標か world座標かを判定する。

    判定ロジック（頑健版）:
      centroid だけでは点群の広がり（地面が左右非対称等）で誤判定するため、
      「点群が原点をどれだけ内包しているか」で見る。
      - lidar座標: 原点(LiDAR自身)が点群バウンディングボックスの内側〜近傍にある
      - world座標: 原点は遠く離れ、点群はLiDAR world位置の周囲に固まる

    具体的には、原点からの最小距離(近接点までの距離)を見る。
    lidar座標なら直近に点があり小さい。world座標なら原点付近に点はなく大きい。
    あわせて LiDAR world位置への近接も確認して総合判定する。

    Returns: ("lidar" | "world", diagnostic_value)
    """
    if len(points) == 0:
        return "lidar", 0.0

    lidar_pos = T_world_lidar[:3, 3]
    lidar_pos_norm = float(np.linalg.norm(lidar_pos))

    # LiDAR world位置がほぼ原点なら、両座標系は区別できない → lidar扱いで安全
    if lidar_pos_norm < threshold:
        return "lidar", lidar_pos_norm

    # サンプリングして高速化
    n = len(points)
    idx = np.random.default_rng(0).choice(n, size=min(n, 5000), replace=False)
    sample = points[idx]

    # 原点(=lidar座標での自分の位置)への最小距離
    dist_to_origin = float(np.min(np.linalg.norm(sample, axis=1)))
    # LiDAR world位置への最小距離
    dist_to_lidarpos = float(np.min(np.linalg.norm(sample - lidar_pos, axis=1)))

    # 原点付近に点がある(lidar座標) vs world位置付近に点がある(world座標)
    if dist_to_origin <= dist_to_lidarpos:
        return "lidar", dist_to_origin
    return "world", dist_to_lidarpos


# ----------------------------------------------------------------------
#  Source 基底
# ----------------------------------------------------------------------
class DataSource:
    """データソース基底クラス。"""

    def list_frames(self) -> List[str]:
        raise NotImplementedError

    def get_frame(self, folder: str) -> Optional[FrameData]:
        raise NotImplementedError

    def __len__(self):
        return len(self.list_frames())


# ----------------------------------------------------------------------
#  GLIM dump ソース（現行）
# ----------------------------------------------------------------------
class GlimDumpSource(DataSource):
    """
    GLIM dump ディレクトリから読む。
    points_compact.bin が world / lidar どちらでも、最終的に
    points_lidar (LiDARローカル) へ正規化して返す。
    """

    def __init__(self, dump_dir, auto_detect_coord=True):
        self.dump_dir = os.path.expanduser(dump_dir)
        if not os.path.isdir(self.dump_dir):
            raise FileNotFoundError(f"dump_dir が見つかりません: {self.dump_dir}")
        self.auto_detect_coord = auto_detect_coord
        self._folders = None

    def list_frames(self):
        if self._folders is None:
            self._folders = sorted([
                f for f in os.listdir(self.dump_dir)
                if re.match(r'^\d{6}$', f)
                and os.path.isdir(os.path.join(self.dump_dir, f))
            ])
        return self._folders

    def get_frame(self, folder):
        frame_dir = os.path.join(self.dump_dir, folder)
        data_txt = os.path.join(frame_dir, 'data.txt')
        pts_bin = os.path.join(frame_dir, 'points_compact.bin')
        if not (os.path.exists(data_txt) and os.path.exists(pts_bin)):
            return None

        stamp, T, used_key = parse_pose_and_stamp(data_txt)
        if T is None:
            return None

        points_raw = np.fromfile(pts_bin, dtype=np.float32).reshape(-1, 3)

        # 座標系を正規化して lidar 座標へ
        note = f"pose={used_key}"
        if self.auto_detect_coord:
            frame_type, diag = detect_coord_frame(points_raw, T)
            note += f", detected={frame_type}(d={diag:.1f}m)"
            if frame_type == "world":
                T_inv = np.linalg.inv(T)
                pts_h = np.hstack([points_raw, np.ones((len(points_raw), 1))])
                points_lidar = (T_inv @ pts_h.T).T[:, :3].astype(np.float32)
            else:
                points_lidar = points_raw
        else:
            points_lidar = points_raw
            note += ", assumed=lidar"

        return FrameData(
            folder=folder, stamp=stamp, T_world_lidar=T,
            points_lidar=points_lidar, coord_note=note,
        )


# ----------------------------------------------------------------------
#  rosbag ソース（将来用スタブ）
# ----------------------------------------------------------------------
class RosbagSource(DataSource):
    """
    将来: rosbag の /livox/lidar2 生点群 + traj_lidar.txt の姿勢を組み合わせる。
    まだ traj_lidar.txt の正確なフォーマットが未確認なので実装はスタブ。
    実装時にここだけ埋めれば上位ツールは無変更で動く。
    """

    def __init__(self, bag_path, traj_path, lidar_topic="/livox/lidar2",
                 accumulate_window=0.0):
        self.bag_path = os.path.expanduser(bag_path)
        self.traj_path = os.path.expanduser(traj_path)
        self.lidar_topic = lidar_topic
        self.accumulate_window = accumulate_window  # 死角補完用の時間窓[秒]

    def list_frames(self):
        raise NotImplementedError(
            "RosbagSource は未実装です。traj_lidar.txt のフォーマット確認後に実装します。"
        )

    def get_frame(self, folder):
        raise NotImplementedError("RosbagSource は未実装です。")


# ----------------------------------------------------------------------
#  ファクトリ
# ----------------------------------------------------------------------
def make_source(kind="dump", **kwargs):
    if kind == "dump":
        return GlimDumpSource(kwargs["dump_dir"],
                              auto_detect_coord=kwargs.get("auto_detect_coord", True))
    elif kind == "rosbag":
        return RosbagSource(kwargs["bag_path"], kwargs["traj_path"],
                            kwargs.get("lidar_topic", "/livox/lidar2"),
                            kwargs.get("accumulate_window", 0.0))
    raise ValueError(f"未知のデータソース種別: {kind}")
