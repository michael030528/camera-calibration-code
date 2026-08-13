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
| `cam103 -> cam104` | `camera_pair_103_104.yml` | 17 | `0.9033 px` | `1.1798 px` | Usable |
| `cam104 -> cam106` | `camera_pair_104_106.yml` | 32 | `0.6673 px` | `0.8739 px` | Good |
| `cam102 -> cam105` | `camera_pair_102_105.yml` | 12 | `0.8207 px` | `1.0505 px` | Usable |

For `cam101 -> cam103`:

```text
P_cam103 = R_cam101_to_cam103 * P_cam101 + T_cam101_to_cam103
```

For `cam104 -> cam106`:

```text
P_cam106 = R_cam104_to_cam106 * P_cam104 + T_cam104_to_cam106
```

For `cam102 -> cam105`:

```text
P_cam105 = R_cam102_to_cam105 * P_cam102 + T_cam102_to_cam105
```

Translation is stored in meters.
