# Camera Calibration Tools

This repository contains Python tools for chessboard-based camera calibration:

- Single-camera intrinsic calibration
- Automatic single-camera chessboard capture and intrinsic calibration
- Paired-camera chessboard capture
- Camera-to-camera extrinsic calibration

The current project uses:

```text
image size: 1920 x 1080
chessboard inner corners: 11 x 8
square size: 0.06 m
```

## Install

```powershell
pip install opencv-python numpy
```

## 1. Single-Camera Intrinsics

Use existing chessboard images:

```powershell
python calibrate_camera_intrinsics_chessboard.py `
  --images .\images\cam0 `
  --board-cols 11 `
  --board-rows 8 `
  --square-size 0.06 `
  --output camera_intrinsics_cam0.yml
```

The script outputs:

- `camera_matrix`
- `dist_coeffs`
- `calibrate_camera_rms`
- `reprojection_error_px`
- per-image reprojection errors

## 2. Automatic Intrinsic Capture

Automatically capture chessboard images from one IP camera and calibrate after enough valid frames:

```powershell
python auto_capture_camera_intrinsics.py `
  --ip 192.168.1.105 `
  --board-cols 11 `
  --board-rows 8 `
  --square-size 0.06 `
  --auto-count 30 `
  --calibration-output camera_intrinsics_192_168_1_105.yml
```

Default login parameters:

```text
username: admin
password: pass with --password or HIKVISION_PASSWORD; never commit it
resolution: 1920 x 1080
```

Keyboard:

```text
s        save current detected chessboard frame manually
q / Esc  finish capture and calibrate if enough images exist
```

## 3. Paired Camera Capture

Capture synchronized chessboard image pairs from two IP cameras:

```powershell
python manual_capture_camera_pair.py `
  --auto `
  --ip1 192.168.1.102 `
  --ip2 192.168.1.105 `
  --board-cols 11 `
  --board-rows 8 `
  --width 1920 `
  --height 1080 `
  --max-pair-delay-ms 80 `
  --auto-count 30 `
  --output .\pairs\cam102_cam105
```

Output layout:

```text
pairs/cam102_cam105/
  cam1/
    0001.jpg
  cam2/
    0001.jpg
  capture_log.csv
```

## 4. Camera-to-Camera Extrinsics

Use paired chessboard images:

```powershell
python calibrate_camera_pair_chessboard.py `
  --cam1 .\pairs\cam102_cam105\cam1 `
  --cam2 .\pairs\cam102_cam105\cam2 `
  --board-cols 11 `
  --board-rows 8 `
  --square-size 0.06 `
  --intrinsics1 .\camera_intrinsics_192_168_1_102.yml `
  --intrinsics2 .\camera_intrinsics_192_168_1_105.yml `
  --output cam102_to_cam105.yml
```

The output transform is:

```text
X_cam2 = R_cam1_to_cam2 * X_cam1 + T_cam1_to_cam2
```

## Quality Reference

Intrinsic calibration:

| Reprojection Error | Quality |
|---:|---|
| `< 0.3 px` | excellent |
| `0.3 - 0.8 px` | good |
| `0.8 - 1.5 px` | usable, can improve |
| `> 1.5 px` | not recommended |

Camera-to-camera extrinsic calibration:

| Stereo Reprojection Error | Quality |
|---:|---|
| `< 0.5 px` | excellent |
| `0.5 - 1.0 px` | good |
| `1.0 - 1.5 px` | usable |
| `1.5 - 2.0 px` | marginal |
| `> 2.0 px` | not recommended |
