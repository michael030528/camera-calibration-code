# Best Camera Intrinsics

Best intrinsic calibration results currently available for cameras in the `192.168.1.101-106` range.

Chessboard settings:

```text
inner corners: 11 x 8
square size: 0.06 m
image size: 1920 x 1080
```

| Camera IP | File | Valid Images | Reprojection Error | Quality |
|---|---|---:|---:|---|
| `192.168.1.101` | `camera_intrinsics_192_168_1_101.yml` | 30 | `0.5680 px` | Good |
| `192.168.1.102` | `camera_intrinsics_192_168_1_102.yml` | 19 | `0.5371 px` | Good |
| `192.168.1.103` | `camera_intrinsics_192_168_1_103.yml` | 28 | `0.6065 px` | Good |
| `192.168.1.104` | `camera_intrinsics_192_168_1_104.yml` | 30 | `0.8755 px` | Usable |
| `192.168.1.105` | `camera_intrinsics_192_168_1_105.yml` | 29 | `0.6137 px` | Good |
| `192.168.1.106` | `camera_intrinsics_192_168_1_106.yml` | 30 | `0.9044 px` | Usable |

Recommended quality target:

```text
< 0.8 px  good
0.8-1.5 px usable
> 1.5 px  not recommended
```
