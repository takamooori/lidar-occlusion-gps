# よく使うコマンド

## 1. rosbag 録画

```bash
ros2 bag record \
    /imu/spresense \
    /livox/imu /livox/imu2 \
    /livox/lidar /livox/lidar2 \
    /odom /odom/UM982 \
    /glim_ros/odom_corrected \
```

## 2. 解析パイプライン実行

```bash
python3 analysis_pipeline.py \
  --dump ~/ros2_ws/maps/dump_nakaniwa_0522 \
  --bag  ~/ros2_ws/bag/0522/nakaniwa_0522_glim_bag/rosbag2_2026_06_02-05_43_34_0.db3 \
  --out  ~/ros2_ws/maps/analysis/
```
```
実行コマンド
ターミナル1：GLIMノードの起動

ros2 run glim_ros glim_rosnode --ros-args -p use_simtime:=true
ターミナル2：Livox LiDAR → PointCloud2 変換ノードの起動

ros2 launch livox_to_pointcloud2 livox_to_pointcloud2.launch.py
```
