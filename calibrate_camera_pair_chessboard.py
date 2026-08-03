#!/usr/bin/env python3
"""
Calibrate extrinsics between two cameras using images of one chessboard.

Example:
    python calibrate_camera_pair_chessboard.py ^
        --cam1 images/cam1 ^
        --cam2 images/cam2 ^
        --board-cols 9 ^
        --board-rows 6 ^
        --square-size 25.0 ^
        --output calibration_result.yml

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


def find_chessboard_corners(image_path: Path, board_size: tuple[int, int]) -> tuple[np.ndarray | None, tuple[int, int]]:
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
    return refined, image_size


def calibrate_single_camera(
    object_points: list[np.ndarray],
    image_points: list[np.ndarray],
    image_size: tuple[int, int],
) -> tuple[float, np.ndarray, np.ndarray, tuple[np.ndarray, ...], tuple[np.ndarray, ...]]:
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )
    return rms, camera_matrix, dist_coeffs, rvecs, tvecs


def load_camera_intrinsics(path: str) -> tuple[np.ndarray, np.ndarray]:
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Cannot open intrinsics file: {path}")

    camera_matrix = fs.getNode("camera_matrix").mat()
    if camera_matrix is None:
        camera_matrix = fs.getNode("camera_matrix_1").mat()

    dist_coeffs = fs.getNode("dist_coeffs").mat()
    if dist_coeffs is None:
        dist_coeffs = fs.getNode("dist_coeffs_1").mat()

    fs.release()

    if camera_matrix is None or dist_coeffs is None:
        raise RuntimeError(
            "Intrinsics YAML must contain camera_matrix/dist_coeffs, "
            "or camera_matrix_1/dist_coeffs_1."
        )
    return camera_matrix, dist_coeffs


def estimate_board_poses(
    object_points: list[np.ndarray],
    image_points: list[np.ndarray],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> tuple[tuple[np.ndarray, ...], tuple[np.ndarray, ...]]:
    rvecs: list[np.ndarray] = []
    tvecs: list[np.ndarray] = []
    for obj, img in zip(object_points, image_points):
        success, rvec, tvec = cv2.solvePnP(
            obj,
            img,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not success:
            raise RuntimeError("solvePnP failed while estimating board pose with fixed intrinsics.")
        rvecs.append(rvec)
        tvecs.append(tvec)
    return tuple(rvecs), tuple(tvecs)


def per_view_reprojection_errors(
    object_points: list[np.ndarray],
    image_points: list[np.ndarray],
    rvecs: Iterable[np.ndarray],
    tvecs: Iterable[np.ndarray],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> tuple[list[float], float]:
    errors: list[float] = []
    total_squared_error = 0.0
    total_points = 0

    for obj, img, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, dist_coeffs)
        diff = img.reshape(-1, 2) - projected.reshape(-1, 2)
        squared_error = float(np.sum(diff * diff))
        point_count = len(obj)
        errors.append(math.sqrt(squared_error / point_count))
        total_squared_error += squared_error
        total_points += point_count

    overall = math.sqrt(total_squared_error / total_points) if total_points else float("nan")
    return errors, overall


def stereo_reprojection_errors(
    object_points: list[np.ndarray],
    image_points1: list[np.ndarray],
    image_points2: list[np.ndarray],
    rvecs1: Iterable[np.ndarray],
    tvecs1: Iterable[np.ndarray],
    camera_matrix1: np.ndarray,
    dist_coeffs1: np.ndarray,
    camera_matrix2: np.ndarray,
    dist_coeffs2: np.ndarray,
    rotation_1_to_2: np.ndarray,
    translation_1_to_2: np.ndarray,
) -> tuple[list[float], float]:
    errors: list[float] = []
    total_squared_error = 0.0
    total_points = 0

    for obj, img1, img2, rvec1, tvec1 in zip(object_points, image_points1, image_points2, rvecs1, tvecs1):
        projected1, _ = cv2.projectPoints(obj, rvec1, tvec1, camera_matrix1, dist_coeffs1)

        rotation_board_to_1, _ = cv2.Rodrigues(rvec1)
        rotation_board_to_2 = rotation_1_to_2 @ rotation_board_to_1
        translation_board_to_2 = rotation_1_to_2 @ tvec1.reshape(3, 1) + translation_1_to_2.reshape(3, 1)
        rvec2, _ = cv2.Rodrigues(rotation_board_to_2)
        projected2, _ = cv2.projectPoints(
            obj,
            rvec2,
            translation_board_to_2,
            camera_matrix2,
            dist_coeffs2,
        )

        diff1 = img1.reshape(-1, 2) - projected1.reshape(-1, 2)
        diff2 = img2.reshape(-1, 2) - projected2.reshape(-1, 2)
        squared_error = float(np.sum(diff1 * diff1) + np.sum(diff2 * diff2))
        point_count = len(obj) * 2
        errors.append(math.sqrt(squared_error / point_count))
        total_squared_error += squared_error
        total_points += point_count

    overall = math.sqrt(total_squared_error / total_points) if total_points else float("nan")
    return errors, overall


def save_result(
    output_path: str,
    camera_matrix1: np.ndarray,
    dist_coeffs1: np.ndarray,
    camera_matrix2: np.ndarray,
    dist_coeffs2: np.ndarray,
    rotation_1_to_2: np.ndarray,
    translation_1_to_2: np.ndarray,
    essential: np.ndarray,
    fundamental: np.ndarray,
    image_size1: tuple[int, int],
    image_size2: tuple[int, int],
    stereo_rms: float,
    reprojection_error_cam1: float,
    reprojection_error_cam2: float,
    reprojection_error_stereo: float,
) -> None:
    fs = cv2.FileStorage(output_path, cv2.FILE_STORAGE_WRITE)
    if not fs.isOpened():
        raise RuntimeError(f"Cannot open output file for writing: {output_path}")

    fs.write("image_width_1", image_size1[0])
    fs.write("image_height_1", image_size1[1])
    fs.write("image_width_2", image_size2[0])
    fs.write("image_height_2", image_size2[1])
    fs.write("camera_matrix_1", camera_matrix1)
    fs.write("dist_coeffs_1", dist_coeffs1)
    fs.write("camera_matrix_2", camera_matrix2)
    fs.write("dist_coeffs_2", dist_coeffs2)
    fs.write("R_cam1_to_cam2", rotation_1_to_2)
    fs.write("T_cam1_to_cam2", translation_1_to_2)
    fs.write("E", essential)
    fs.write("F", fundamental)
    fs.write("stereo_calibrate_rms", stereo_rms)
    fs.write("reprojection_error_cam1_px", reprojection_error_cam1)
    fs.write("reprojection_error_cam2_px", reprojection_error_cam2)
    fs.write("reprojection_error_stereo_px", reprojection_error_stereo)
    fs.release()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Calibrate camera-to-camera extrinsics with chessboard images.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--cam1", required=True, help="Folder containing camera 1 chessboard images.")
    parser.add_argument("--cam2", required=True, help="Folder containing camera 2 chessboard images.")
    parser.add_argument("--board-cols", type=int, required=True, help="Inner corner count along chessboard columns.")
    parser.add_argument("--board-rows", type=int, required=True, help="Inner corner count along chessboard rows.")
    parser.add_argument("--square-size", type=float, required=True, help="Chessboard square size, e.g. millimeters.")
    parser.add_argument("--output", default="camera_pair_calibration.yml", help="Output OpenCV YAML file.")
    parser.add_argument("--min-pairs", type=int, default=8, help="Minimum valid image pairs required.")
    parser.add_argument("--intrinsics1", help="Optional OpenCV YAML containing camera 1 intrinsics.")
    parser.add_argument("--intrinsics2", help="Optional OpenCV YAML containing camera 2 intrinsics.")
    parser.add_argument(
        "--no-fix-intrinsics",
        action="store_true",
        help="Let stereoCalibrate optimize intrinsics after initial single-camera calibration.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    images1 = collect_images(args.cam1)
    images2 = collect_images(args.cam2)
    if not images1:
        raise RuntimeError(f"No images found in cam1 folder: {args.cam1}")
    if not images2:
        raise RuntimeError(f"No images found in cam2 folder: {args.cam2}")
    if len(images1) != len(images2):
        raise RuntimeError(
            f"Image count mismatch: cam1={len(images1)}, cam2={len(images2)}. "
            "Use matching image names/counts, or remove unmatched images."
        )

    board_size = (args.board_cols, args.board_rows)
    one_object_points = make_object_points(args.board_cols, args.board_rows, args.square_size)

    object_points: list[np.ndarray] = []
    image_points1: list[np.ndarray] = []
    image_points2: list[np.ndarray] = []
    valid_pair_names: list[tuple[str, str]] = []
    image_size1: tuple[int, int] | None = None
    image_size2: tuple[int, int] | None = None

    print(f"Reading {len(images1)} image pairs...")
    for path1, path2 in zip(images1, images2):
        corners1, size1 = find_chessboard_corners(path1, board_size)
        corners2, size2 = find_chessboard_corners(path2, board_size)

        if image_size1 is None:
            image_size1 = size1
        elif image_size1 != size1:
            raise RuntimeError(f"All cam1 images must have the same size. {path1.name} is {size1}, expected {image_size1}")

        if image_size2 is None:
            image_size2 = size2
        elif image_size2 != size2:
            raise RuntimeError(f"All cam2 images must have the same size. {path2.name} is {size2}, expected {image_size2}")

        if corners1 is None or corners2 is None:
            print(f"[skip] chessboard not found in pair: {path1.name}, {path2.name}")
            continue

        object_points.append(one_object_points.copy())
        image_points1.append(corners1)
        image_points2.append(corners2)
        valid_pair_names.append((path1.name, path2.name))
        print(f"[ok] {path1.name} <-> {path2.name}")

    if image_size1 is None or image_size2 is None:
        raise RuntimeError("No readable image pairs found.")
    if len(object_points) < args.min_pairs:
        raise RuntimeError(f"Only {len(object_points)} valid pairs found; need at least {args.min_pairs}.")

    if args.intrinsics1:
        print("\nLoading camera 1 intrinsics...")
        camera_matrix1, dist_coeffs1 = load_camera_intrinsics(args.intrinsics1)
        rvecs1, tvecs1 = estimate_board_poses(object_points, image_points1, camera_matrix1, dist_coeffs1)
        rms1 = float("nan")
    else:
        print("\nCalibrating camera 1 intrinsics...")
        rms1, camera_matrix1, dist_coeffs1, rvecs1, tvecs1 = calibrate_single_camera(
            object_points,
            image_points1,
            image_size1,
        )

    if args.intrinsics2:
        print("Loading camera 2 intrinsics...")
        camera_matrix2, dist_coeffs2 = load_camera_intrinsics(args.intrinsics2)
        rvecs2, tvecs2 = estimate_board_poses(object_points, image_points2, camera_matrix2, dist_coeffs2)
        rms2 = float("nan")
    else:
        print("Calibrating camera 2 intrinsics...")
        rms2, camera_matrix2, dist_coeffs2, rvecs2, tvecs2 = calibrate_single_camera(
            object_points,
            image_points2,
            image_size2,
        )

    stereo_flags = cv2.CALIB_FIX_INTRINSIC
    if args.no_fix_intrinsics:
        stereo_flags = 0
        if image_size1 != image_size2:
            print(
                "[warn] --no-fix-intrinsics was set but camera resolutions differ. "
                "OpenCV stereoCalibrate accepts one image size, so cam1 size will be used."
            )

    print("Calibrating camera-to-camera extrinsics...")
    stereo_rms, camera_matrix1, dist_coeffs1, camera_matrix2, dist_coeffs2, rotation, translation, essential, fundamental = (
        cv2.stereoCalibrate(
            object_points,
            image_points1,
            image_points2,
            camera_matrix1,
            dist_coeffs1,
            camera_matrix2,
            dist_coeffs2,
            image_size1,
            flags=stereo_flags,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6),
        )
    )

    errors1, overall_error1 = per_view_reprojection_errors(
        object_points,
        image_points1,
        rvecs1,
        tvecs1,
        camera_matrix1,
        dist_coeffs1,
    )
    errors2, overall_error2 = per_view_reprojection_errors(
        object_points,
        image_points2,
        rvecs2,
        tvecs2,
        camera_matrix2,
        dist_coeffs2,
    )
    stereo_errors, stereo_error = stereo_reprojection_errors(
        object_points,
        image_points1,
        image_points2,
        rvecs1,
        tvecs1,
        camera_matrix1,
        dist_coeffs1,
        camera_matrix2,
        dist_coeffs2,
        rotation,
        translation,
    )

    save_result(
        args.output,
        camera_matrix1,
        dist_coeffs1,
        camera_matrix2,
        dist_coeffs2,
        rotation,
        translation,
        essential,
        fundamental,
        image_size1,
        image_size2,
        stereo_rms,
        overall_error1,
        overall_error2,
        stereo_error,
    )

    print("\nCalibration finished.")
    print(f"Valid image pairs: {len(object_points)}")
    if not math.isnan(rms1):
        print(f"Camera 1 calibrateCamera RMS: {rms1:.6f} px")
    else:
        print("Camera 1 calibrateCamera RMS: loaded intrinsics")
    if not math.isnan(rms2):
        print(f"Camera 2 calibrateCamera RMS: {rms2:.6f} px")
    else:
        print("Camera 2 calibrateCamera RMS: loaded intrinsics")
    print(f"Stereo calibrate RMS:         {stereo_rms:.6f} px")
    print(f"Camera 1 reprojection error:  {overall_error1:.6f} px")
    print(f"Camera 2 reprojection error:  {overall_error2:.6f} px")
    print(f"Stereo reprojection error:    {stereo_error:.6f} px")
    print("\nR_cam1_to_cam2:")
    print(rotation)
    print("\nT_cam1_to_cam2:")
    print(translation.reshape(3))
    print(f"\nSaved result: {os.path.abspath(args.output)}")

    print("\nPer-pair stereo reprojection error:")
    for (name1, name2), err1, err2, err_stereo in zip(valid_pair_names, errors1, errors2, stereo_errors):
        print(f"  {name1} <-> {name2}: cam1={err1:.4f}px, cam2={err2:.4f}px, stereo={err_stereo:.4f}px")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
