# よく使うコマンド

## 1. rosbag 録画

```bash
ros2 bag record \
    /imu/spresense \
    /livox/imu /livox/imu2 \
    /livox/lidar /livox/lidar2 \
    /odom /odom/UM982 \
    /odom/wheel_spimu \
    /glim_ros/odom_corrected \
    /odom/combine_glim
```

## 2. 解析パイプライン実行

```bash
python3 analysis_pipeline.py \
  --dump ~/ros2_ws/maps/dump_nakaniwa_0522 \
  --bag  ~/ros2_ws/bag/0522/nakaniwa_0522_glim_bag/rosbag2_2026_06_02-05_43_34_0.db3 \
  --out  ~/ros2_ws/maps/analysis/
```
