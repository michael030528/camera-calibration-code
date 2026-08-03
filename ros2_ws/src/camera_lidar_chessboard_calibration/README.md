# Camera-Livox chessboard calibration (ROS 2 Jazzy)

This package records timestamp-matched Hikvision RTSP images and Livox
`PointCloud2`, detects an 11 x 8 chessboard, fits the board plane, and
estimates `lidar -> camera` extrinsics from several board poses.

The checked-in final MID-360 transform is
`config/lidar_to_camera_mid360_final.yaml` (16 accumulated samples, 14 robust
inliers, 2.46 degree normal RMSE and 1.67 cm plane-distance RMSE).

## Livox connection

The direct receiver is configured for a MID-360 at `192.168.1.152` and host IP
`192.168.1.41`. It publishes `/livox/lidar`. Launch arguments `lidar_ip`,
`host_ip`, and `cloud_topic` can override these defaults. Do not run Livox
Viewer or another Livox driver at the same time.

## Build and run

```bash
cd '/mnt/c/Users/33850/OneDrive/文档/摄像头，雷达标定/ros2_ws'
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --merge-install --install-base install_merge
source setup_calibration.bash
export HIKVISION_PASSWORD='YOUR_CAMERA_PASSWORD'
./start_calibration.bash
```

## Frozen synchronized-frame selection

RViz displays `/calibration/selection_cloud`, not the raw live cloud. It is a
motion-checked accumulation of the five LiDAR frames nearest the synchronized
camera frame. Each source frame is retained so motion can be checked before a
sample is saved.

1. Choose RViz `Chessboard Select`.
2. Press the left mouse button beside the board; this freezes the exact
   synchronized image/cloud pair.
3. Drag around the complete `0.80 x 0.60 m` physical board: include the white
   outer border and the outermost chess squares, but exclude hands, body, and
   background points.
4. On release, every source frame gets a separate plane fit. The sample is
   saved only if at least three frames agree in normal, plane distance, and
   board center. Live synchronized preview then resumes.

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

## Record camera + MID360 LiDAR + IMU

The direct MID360 receiver publishes both `/livox/lidar` and `/livox/imu`.
The synchronizer uses each LiDAR scan midpoint as the reference, selects the
nearest camera frame and IMU sample, and publishes one matched triplet with one
common timestamp:

```text
/aligned/camera/image_raw
/aligned/livox/lidar
/aligned/livox/imu
/aligned/sync_status
```

Build once, then start all three sensors and rosbag recording with one command:

```bash
cd '/mnt/c/Users/33850/OneDrive/文档/摄像头，雷达标定/ros2_ws'
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --merge-install --install-base install_merge
source setup_calibration.bash
ros2 launch camera_lidar_chessboard_calibration record_sensors.launch.py \
  password:='YOUR_CAMERA_PASSWORD' \
  bag:="$HOME/camera_mid360_$(date +%Y%m%d_%H%M%S)" \
  sync_tolerance_ms:=60.0
```

Stop with `Ctrl+C`. Playback preserves the common aligned timestamps:

```bash
ros2 launch camera_lidar_chessboard_calibration play_sensors.launch.py \
  bag:='/path/to/camera_mid360_YYYYMMDD_HHMMSS'
```

`/aligned/sync_status` records the original camera/LiDAR and IMU/LiDAR time
errors for every accepted triplet. Samples outside `sync_tolerance_ms` are
dropped. The original topics retain their acquisition timestamps; only the
`/aligned/*` copies share the LiDAR scan timestamp.

For true clock alignment, configure the MID360 and host to the same PTP grandmaster.
An ordinary RTSP camera has transport and decode jitter and cannot be made
hardware-synchronous by software. Measure that fixed latency and pass
`camera_time_offset_ms` to `camera_rtsp` (or put it in `calibration.yaml`) before
recording dynamic scenes.
