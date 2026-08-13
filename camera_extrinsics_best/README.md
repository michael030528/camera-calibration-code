# Best Camera Extrinsics

Best camera-to-camera extrinsic calibration results currently kept for the rig.

Chessboard settings:

```text
inner corners: 11 x 8
square size: 0.06 m
image size: 1920 x 1080
```

| Pair | File | Valid Pairs | Stereo RMS | Stereo Reprojection Error | Quality |
|---|---|---:|---:|---:|---|
| `cam101 -> cam103` | `camera_pair_101_103.yml` | 29 | `0.7017 px` | `0.9020 px` | Good |

For `cam101 -> cam103`:

```text
P_cam103 = R_cam101_to_cam103 * P_cam101 + T_cam101_to_cam103
```

Translation is stored in meters.
