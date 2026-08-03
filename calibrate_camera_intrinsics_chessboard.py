#!/usr/bin/env python3
"""
Calibrate camera intrinsics from chessboard images.

Example:
    python calibrate_camera_intrinsics_chessboard.py ^
        --images images/camera ^
        --board-cols 9 ^
        --board-rows 6 ^
        --square-size 25.0 ^
        --output camera_intrinsics.yml

Notes:
    --board-cols and --board-rows are the number of INNER chessboard corners,
    not the number of black/white squares.
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np


IMAGE_EXTENSIONS = ("*.jpg", "*.jpeg", "*.png", "*.bmp", "*.tif", "*.tiff")


def collect_images(folder: str) -> list[Path]:
    paths: list[Path] = []
    for ext in IMAGE_EXTENSIONS:
        paths.extend(Path(folder).glob(ext))
        paths.extend(Path(folder).glob(ext.upper()))
    return sorted(set(paths), key=lambda p: p.name)


def make_object_points(board_cols: int, board_rows: int, square_size: float) -> np.ndarray:
    object_points = np.zeros((board_rows * board_cols, 3), np.float32)
    object_points[:, :2] = np.mgrid[0:board_cols, 0:board_rows].T.reshape(-1, 2)
    object_points *= square_size
    return object_points


def find_chessboard_corners(
    image_path: Path,
    board_size: tuple[int, int],
    save_debug_dir: str | None,
) -> tuple[np.ndarray | None, tuple[int, int]]:
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"Cannot read image: {image_path}")

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image_size = (gray.shape[1], gray.shape[0])

    flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        | cv2.CALIB_CB_NORMALIZE_IMAGE
        | cv2.CALIB_CB_FAST_CHECK
    )
    found, corners = cv2.findChessboardCorners(gray, board_size, flags)
    if not found:
        return None, image_size

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )
    refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    if save_debug_dir:
        debug_dir = Path(save_debug_dir)
        debug_dir.mkdir(parents=True, exist_ok=True)
        drawn = image.copy()
        cv2.drawChessboardCorners(drawn, board_size, refined, True)
        cv2.imwrite(str(debug_dir / image_path.name), drawn)

    return refined, image_size


def calibrate_camera(
    object_points: list[np.ndarray],
    image_points: list[np.ndarray],
    image_size: tuple[int, int],
    use_rational_model: bool,
    fix_principal_point: bool,
    zero_tangent_dist: bool,
) -> tuple[float, np.ndarray, np.ndarray, tuple[np.ndarray, ...], tuple[np.ndarray, ...]]:
    flags = 0
    if use_rational_model:
        flags |= cv2.CALIB_RATIONAL_MODEL
    if fix_principal_point:
        flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
    if zero_tangent_dist:
        flags |= cv2.CALIB_ZERO_TANGENT_DIST

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
        flags=flags,
    )
    return rms, camera_matrix, dist_coeffs, rvecs, tvecs


def reprojection_errors(
    object_points: list[np.ndarray],
    image_points: list[np.ndarray],
    rvecs: Iterable[np.ndarray],
    tvecs: Iterable[np.ndarray],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> tuple[list[float], float]:
    per_image: list[float] = []
    total_squared_error = 0.0
    total_points = 0

    for obj, img, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, dist_coeffs)
        diff = img.reshape(-1, 2) - projected.reshape(-1, 2)
        squared_error = float(np.sum(diff * diff))
        point_count = len(obj)
        per_image.append(math.sqrt(squared_error / point_count))
        total_squared_error += squared_error
        total_points += point_count

    overall = math.sqrt(total_squared_error / total_points) if total_points else float("nan")
    return per_image, overall


def save_result(
    output_path: str,
    image_size: tuple[int, int],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    rms: float,
    reprojection_error: float,
    valid_image_count: int,
) -> None:
    fs = cv2.FileStorage(output_path, cv2.FILE_STORAGE_WRITE)
    if not fs.isOpened():
        raise RuntimeError(f"Cannot open output file for writing: {output_path}")

    fs.write("image_width", image_size[0])
    fs.write("image_height", image_size[1])
    fs.write("camera_matrix", camera_matrix)
    fs.write("dist_coeffs", dist_coeffs)
    fs.write("calibrate_camera_rms", rms)
    fs.write("reprojection_error_px", reprojection_error)
    fs.write("valid_image_count", valid_image_count)
    fs.release()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Calibrate camera intrinsics with chessboard images.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--images", required=True, help="Folder containing chessboard images.")
    parser.add_argument("--board-cols", type=int, required=True, help="Inner corner count along chessboard columns.")
    parser.add_argument("--board-rows", type=int, required=True, help="Inner corner count along chessboard rows.")
    parser.add_argument("--square-size", type=float, required=True, help="Chessboard square size, e.g. millimeters.")
    parser.add_argument("--output", default="camera_intrinsics.yml", help="Output OpenCV YAML file.")
    parser.add_argument("--min-images", type=int, default=8, help="Minimum valid chessboard images required.")
    parser.add_argument("--debug-dir", help="Optional folder for images with detected corners drawn.")
    parser.add_argument("--rational-model", action="store_true", help="Estimate k4, k5, k6 distortion terms.")
    parser.add_argument("--fix-principal-point", action="store_true", help="Fix principal point at the image center.")
    parser.add_argument("--zero-tangent-dist", action="store_true", help="Force tangential distortion p1 and p2 to zero.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    images = collect_images(args.images)
    if not images:
        raise RuntimeError(f"No images found in folder: {args.images}")

    board_size = (args.board_cols, args.board_rows)
    one_object_points = make_object_points(args.board_cols, args.board_rows, args.square_size)

    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    valid_names: list[str] = []
    image_size: tuple[int, int] | None = None

    print(f"Reading {len(images)} image(s)...")
    for image_path in images:
        corners, size = find_chessboard_corners(image_path, board_size, args.debug_dir)

        if image_size is None:
            image_size = size
        elif image_size != size:
            raise RuntimeError(f"All images must have the same size. {image_path.name} is {size}, expected {image_size}")

        if corners is None:
            print(f"[skip] chessboard not found: {image_path.name}")
            continue

        object_points.append(one_object_points.copy())
        image_points.append(corners)
        valid_names.append(image_path.name)
        print(f"[ok] {image_path.name}")

    if image_size is None:
        raise RuntimeError("No readable images found.")
    if len(valid_names) < args.min_images:
        raise RuntimeError(f"Only {len(valid_names)} valid images found; need at least {args.min_images}.")

    print("\nCalibrating camera intrinsics...")
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = calibrate_camera(
        object_points,
        image_points,
        image_size,
        args.rational_model,
        args.fix_principal_point,
        args.zero_tangent_dist,
    )

    per_image_errors, overall_error = reprojection_errors(
        object_points,
        image_points,
        rvecs,
        tvecs,
        camera_matrix,
        dist_coeffs,
    )

    save_result(
        args.output,
        image_size,
        camera_matrix,
        dist_coeffs,
        rms,
        overall_error,
        len(valid_names),
    )

    print("\nCalibration finished.")
    print(f"Valid images:              {len(valid_names)}")
    print(f"calibrateCamera RMS:       {rms:.6f} px")
    print(f"Reprojection error:        {overall_error:.6f} px")
    print("\nCamera matrix:")
    print(camera_matrix)
    print("\nDistortion coefficients:")
    print(dist_coeffs.reshape(-1))
    print(f"\nSaved result: {os.path.abspath(args.output)}")

    print("\nPer-image reprojection error:")
    for name, error in zip(valid_names, per_image_errors):
        print(f"  {name}: {error:.4f}px")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
