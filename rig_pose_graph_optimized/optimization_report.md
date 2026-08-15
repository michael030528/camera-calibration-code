# Rig Pose Graph Optimization Report

Input directory: `rig_best_results_separated\extrinsics_best`
Fixed rig camera: `cam101`

## Input Closure

- Rotation closure error: `9.648676 deg`
- Translation closure error: `0.038640 m` / `3.864 cm`
- Translation residual xyz: `[0.019185, 0.026360, 0.020740] m`

## Optimization

- Initial weighted cost: `0.01655630`
- Final weighted cost: `0.00324143`
- Iterations recorded: `19`

## Edge Residuals After Optimization

| Pair | Input Stereo Error | Optimized Rotation Residual | Optimized Translation Residual |
|---|---:|---:|---:|
| `cam101 -> cam103` | `0.9020 px` | `1.756993 deg` | `0.006955 m` |
| `cam103 -> cam104` | `0.7657 px` | `1.204090 deg` | `0.005013 m` |
| `cam104 -> cam106` | `0.8739 px` | `1.502039 deg` | `0.006529 m` |
| `cam106 -> cam102` | `0.9077 px` | `1.655722 deg` | `0.007044 m` |
| `cam102 -> cam105` | `0.8724 px` | `1.606365 deg` | `0.006506 m` |
| `cam105 -> cam101` | `0.9375 px` | `1.928626 deg` | `0.007514 m` |

## Output Files

- `camera_poses/`: optimized camera-to-rig poses
- `optimized_pairs/`: pairwise extrinsics recomputed from optimized poses
