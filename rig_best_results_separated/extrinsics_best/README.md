# Best Camera Extrinsics

Best camera-to-camera extrinsic calibration files for the current 6-camera rig loop.

Transform convention:

```text
P_target = R_source_to_target * P_source + T_source_to_target
```

| Pair | File | Stereo RMS | Cam1 Error | Cam2 Error | Stereo Error | Tx | Ty | Tz | Distance | Quality |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| `cam101 -> cam103` | `camera_pair_101_103.yml` | `0.7017 px` | `0.4801 px` | `0.5220 px` | `0.9020 px` | `0.2464 m` | `-0.0166 m` | `-0.0864 m` | `0.2617 m` | Good |
| `cam103 -> cam104` | `camera_pair_103_104.yml` | `0.9033 px` | `0.6368 px` | `0.3567 px` | `1.1798 px` | `0.2348 m` | `-0.0122 m` | `-0.1079 m` | `0.2586 m` | Usable |
| `cam104 -> cam106` | `camera_pair_104_106.yml` | `0.6673 px` | `0.3563 px` | `0.4143 px` | `0.8739 px` | `0.2506 m` | `0.0002 m` | `-0.0893 m` | `0.2660 m` | Good |
| `cam106 -> cam102` | `camera_pair_106_102.yml` | `0.7178 px` | `0.4297 px` | `0.5067 px` | `0.9077 px` | `0.2144 m` | `-0.0213 m` | `-0.1517 m` | `0.2634 m` | Good |
| `cam102 -> cam105` | `camera_pair_102_105.yml` | `0.6285 px` | `0.3380 px` | `0.4998 px` | `0.8724 px` | `0.1937 m` | `-0.0130 m` | `-0.1698 m` | `0.2579 m` | Good |
| `cam105 -> cam101` | `camera_pair_105_101.yml` | `0.6399 px` | `0.3414 px` | `0.2686 px` | `0.9375 px` | `0.2377 m` | `-0.0102 m` | `-0.1087 m` | `0.2615 m` | Good |

Rig loop:

```text
101 -> 103 -> 104 -> 106 -> 102 -> 105 -> 101
```
