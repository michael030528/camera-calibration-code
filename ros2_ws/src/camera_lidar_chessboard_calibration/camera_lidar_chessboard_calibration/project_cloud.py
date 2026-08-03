import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml


def read_intrinsics(path):
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    k = fs.getNode('camera_matrix').mat()
    d = fs.getNode('dist_coeffs').mat()
    if d is None:
        d = fs.getNode('distortion_coefficients').mat()
    fs.release()
    if k is None:
        raise RuntimeError('Intrinsic file has no camera_matrix')
    return k, np.zeros((1, 5)) if d is None else d


def main():
    parser = argparse.ArgumentParser(description='Project a saved LiDAR frame onto its synchronized camera image')
    parser.add_argument('--sample', required=True, help='Sample basename, JSON path, PNG path, or NPZ path')
    parser.add_argument('--intrinsics', required=True)
    parser.add_argument('--extrinsics', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--point-radius', type=int, default=3)
    args = parser.parse_args()

    base = Path(args.sample)
    if base.suffix.lower() in {'.json', '.png', '.npz'}:
        base = base.with_suffix('')
    image = cv2.imread(str(base.with_suffix('.png')), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f'Cannot read {base.with_suffix(".png")}')
    points = np.load(base.with_suffix('.npz'))['points'][:, :3].astype(np.float64)
    meta = json.loads(base.with_suffix('.json').read_text(encoding='utf-8'))
    k, d = read_intrinsics(args.intrinsics)
    ext = yaml.safe_load(Path(args.extrinsics).read_text(encoding='utf-8'))
    t_lc = np.asarray(ext['T_lidar_to_camera'], dtype=np.float64)

    camera_points = points @ t_lc[:3, :3].T + t_lc[:3, 3]
    valid = np.isfinite(camera_points).all(axis=1) & (camera_points[:, 2] > 0.15)
    camera_points = camera_points[valid]
    uv, _ = cv2.projectPoints(camera_points, np.zeros(3), np.zeros(3), k, d)
    uv = uv.reshape(-1, 2)
    h, w = image.shape[:2]
    inside = ((uv[:, 0] >= 0) & (uv[:, 0] < w) &
              (uv[:, 1] >= 0) & (uv[:, 1] < h))
    uv, depth = uv[inside], camera_points[inside, 2]
    if not len(uv):
        raise RuntimeError('No LiDAR points project inside the image; check the transform direction')

    low, high = np.percentile(depth, [2, 98]) if len(depth) > 2 else (depth.min(), depth.max())
    span = max(float(high - low), 1e-6)
    scaled = np.clip((depth - low) / span, 0, 1)
    colors = cv2.applyColorMap(np.uint8((1.0 - scaled) * 255), cv2.COLORMAP_TURBO)[:, 0, :]
    overlay = image.copy()
    for (u, v), color in zip(uv, colors):
        cv2.circle(overlay, (int(round(u)), int(round(v))), args.point_radius,
                   tuple(int(x) for x in color), -1, cv2.LINE_AA)
    result = cv2.addWeighted(overlay, 0.82, image, 0.18, 0)

    corners = np.asarray(meta.get('image_points', []), dtype=np.float32)
    if len(corners) >= 4:
        hull = cv2.convexHull(corners.astype(np.int32))
        cv2.polylines(result, [hull], True, (70, 255, 70), 3, cv2.LINE_AA)
    label = (f'MID-360 projection | visible points: {len(uv)} | '
             f'depth: {depth.min():.2f}-{depth.max():.2f} m | green: chessboard')
    cv2.rectangle(result, (12, 12), (min(w - 12, 1050), 58), (0, 0, 0), -1)
    cv2.putText(result, label, (25, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                (255, 255, 255), 2, cv2.LINE_AA)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), result):
        raise RuntimeError(f'Cannot write {output}')
    print(f'Saved {output} ({len(uv)} visible points)')


if __name__ == '__main__':
    main()
