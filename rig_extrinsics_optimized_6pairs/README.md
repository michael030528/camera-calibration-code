# Optimized 6 Camera Extrinsics

This folder contains the six pairwise extrinsics recomputed from the global pose graph optimized camera poses.

Transform convention:

```text
P_target = R_source_to_target * P_source + T_source_to_target
```

## Closure Comparison

| Version | Rotation Closure Error | Translation Closure Error | Translation Residual xyz |
|---|---:|---:|---|
| Raw best extrinsics | `9.648676 deg` | `3.864 cm` | `[1.919, 2.636, 2.074] cm` |
| Optimized extrinsics | `0.000004 deg` | `0.000000 cm` | `[0.000000, 0.000000, -0.000000] cm` |

## Optimized Pair Files

| Pair | File | Raw Stereo RMS | Raw Stereo Error | Rotation Adjustment | Translation Adjustment |
|---|---|---:|---:|---:|---:|
| `cam101 -> cam103` | `optimized_pair_cam101_cam103.yml` | `0.7017 px` | `0.9020 px` | `1.7570 deg` | `0.696 cm` |
| `cam103 -> cam104` | `optimized_pair_cam103_cam104.yml` | `0.5870 px` | `0.7657 px` | `1.2041 deg` | `0.501 cm` |
| `cam104 -> cam106` | `optimized_pair_cam104_cam106.yml` | `0.6673 px` | `0.8739 px` | `1.5020 deg` | `0.653 cm` |
| `cam106 -> cam102` | `optimized_pair_cam106_cam102.yml` | `0.7178 px` | `0.9077 px` | `1.6557 deg` | `0.704 cm` |
| `cam102 -> cam105` | `optimized_pair_cam102_cam105.yml` | `0.6285 px` | `0.8724 px` | `1.6064 deg` | `0.651 cm` |
| `cam105 -> cam101` | `optimized_pair_cam105_cam101.yml` | `0.6399 px` | `0.9375 px` | `1.9286 deg` | `0.751 cm` |

Rig loop:

```text
101 -> 103 -> 104 -> 106 -> 102 -> 105 -> 101
```

Note: these optimized pair files are best for global rig consistency. For single pair reprojection checks, use the raw calibrated pair files.
