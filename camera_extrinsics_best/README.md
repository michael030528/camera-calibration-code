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
| `cam106 -> cam102` | `camera_pair_106_102.yml` | 30 | `0.7178 px` | `0.9077 px` | Good |
| `cam102 -> cam105` | `camera_pair_102_105.yml` | 17 | `0.6285 px` | `0.8724 px` | Good |
| `cam105 -> cam101` | `camera_pair_105_101.yml` | 18 | `0.6399 px` | `0.9375 px` | Good |

Transform convention:

```text
P_target = R_source_to_target * P_source + T_source_to_target
```

Translation is stored in meters.
