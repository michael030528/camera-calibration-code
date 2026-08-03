# Camera-Livox chessboard calibration (ROS 2 Jazzy)

This package records timestamp-matched Hikvision RTSP images and Livox
`PointCloud2`, detects an 11 x 8 chessboard, fits the board plane, and
estimates `lidar -> camera` extrinsics from several board poses.

## Livox connection

The direct receiver is configured for a MID-360 at `192.168.1.53` and host IP
`192.168.1.41`. It publishes `/livox/lidar`. Launch arguments `lidar_ip`,
`host_ip`, and `cloud_topic` can override these defaults. Do not run Livox
Viewer or another Livox driver at the same time.

## Build and run

```bash
cd '/mnt/c/Users/33850/OneDrive/文档/摄像头，雷达标定/ros2_ws'
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --merge-install --install-base install_merge
source setup_calibration.bash
ros2 launch camera_lidar_chessboard_calibration calibration.launch.py \
  password:='YOUR_CAMERA_PASSWORD'
```

## Frozen synchronized-frame selection

RViz displays `/calibration/selection_cloud`, not the raw live cloud. Every
displayed cloud is matched to the nearest image timestamp in a recent buffer.

1. Choose RViz `Publish Point`.
2. The first corner click freezes that exact image/cloud pair.
3. Select top-left, top-right, bottom-right, and bottom-left on the frozen cloud.
4. After the fourth click the pair is processed and, if valid, saved. Live
   synchronized preview then resumes.

Press `C` in the camera window to cancel a partial selection and resume live
preview. Press `Q` or Escape to quit. Capture at least 8-15 poses with varied
distance, yaw, and pitch.

Board dimensions, ROI, synchronization tolerance, and selection thresholds are
configured in `config/calibration.yaml`.

## Solve extrinsics

```bash
ros2 run camera_lidar_chessboard_calibration calibrate \
  --samples calibration_samples \
  --intrinsics camera_intrinsics_192_168_1_102_new_filtered.yml \
  --output lidar_to_camera.yaml
```

Plane-only calibration requires substantially varied board orientations.
Parallel poses make translation underconstrained.

## Timing note

The RTSP camera and LiDAR are not hardware-triggered. The package selects the
nearest buffered image/cloud timestamps within `sync_slop=0.05 s` and freezes
that pair for all four clicks. This prevents corners from different LiDAR
frames being mixed, but remains approximate software synchronization. True
hardware synchronization requires camera trigger or PTP support.
