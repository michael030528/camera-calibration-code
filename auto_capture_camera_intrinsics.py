#!/usr/bin/env python3
"""
Automatically capture chessboard images from one IP camera and calibrate intrinsics.

Default target camera:
    192.168.1.105

Keyboard:
    s          save current frame manually if the chessboard is detected
    q or Esc   quit and calibrate with captured frames if enough images exist

Example:
    python auto_capture_camera_intrinsics.py ^
        --ip 192.168.1.105 ^
        --board-cols 11 ^
        --board-rows 8 ^
        --square-size 0.06 ^
        --auto-count 30
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import threading
import time
from pathlib import Path
from urllib.parse import quote

import cv2
import numpy as np


def build_rtsp_url(ip: str, username: str, password: str, port: int, path: str) -> str:
    encoded_user = quote(username, safe="")
    encoded_password = quote(password, safe="")
    normalized_path = path if path.startswith("/") else f"/{path}"
    return f"rtsp://{encoded_user}:{encoded_password}@{ip}:{port}{normalized_path}"


def open_camera(url: str, width: int, height: int) -> cv2.VideoCapture:
    capture = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    if not capture.isOpened():
        raise RuntimeError(f"Cannot open camera: {url}")
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if width > 0:
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height > 0:
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return capture


class LatestFrameReader:
    def __init__(self, capture: cv2.VideoCapture) -> None:
        self.capture = capture
        self.lock = threading.Lock()
        self.frame = None
        self.timestamp = 0.0
        self.frame_index = 0
        self.running = False
        self.thread: threading.Thread | None = None

    def start(self) -> None:
        self.running = True
        self.thread = threading.Thread(target=self._loop, name="camera-reader", daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=2.0)

    def _loop(self) -> None:
        while self.running:
            ok, frame = self.capture.read()
            timestamp = time.time()
            if not ok or frame is None:
                time.sleep(0.01)
                continue
            with self.lock:
                self.frame = frame
                self.timestamp = timestamp
                self.frame_index += 1

    def get_latest(self):
        with self.lock:
            if self.frame is None:
                return None, 0.0, 0
            return self.frame.copy(), self.timestamp, self.frame_index


def make_object_points(board_cols: int, board_rows: int, square_size: float) -> np.ndarray:
    object_points = np.zeros((board_rows * board_cols, 3), np.float32)
    object_points[:, :2] = np.mgrid[0:board_cols, 0:board_rows].T.reshape(-1, 2)
    object_points *= square_size
    return object_points


def detect_chessboard(frame, board_size: tuple[int, int]):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        | cv2.CALIB_CB_NORMALIZE_IMAGE
        | cv2.CALIB_CB_FAST_CHECK
    )
    found, corners = cv2.findChessboardCorners(gray, board_size, flags)
    if not found:
        return False, None, frame

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )
    refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    drawn = frame.copy()
    cv2.drawChessboardCorners(drawn, board_size, refined, True)
    return True, refined, drawn


def resize_for_preview(frame, max_width: int):
    height, width = frame.shape[:2]
    if width <= max_width:
        return frame
    scale = max_width / width
    return cv2.resize(frame, (max_width, int(height * scale)), interpolation=cv2.INTER_AREA)


def draw_status(frame, text: str):
    output = frame.copy()
    for index, line in enumerate(text.splitlines()):
        cv2.putText(
            output,
            line,
            (16, 34 + 34 * index),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.85,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
    return output


def imwrite_unicode(path: Path, image) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, encoded = cv2.imencode(path.suffix or ".jpg", image)
    if not ok:
        raise RuntimeError(f"Failed to encode image: {path}")
    path.write_bytes(encoded.tobytes())


def next_index(folder: Path, digits: int) -> int:
    numbers: list[int] = []
    for path in folder.glob("*.jpg"):
        try:
            numbers.append(int(path.stem))
        except ValueError:
            continue
    return max(numbers) + 1 if numbers else 1


def reprojection_errors(
    object_points: list[np.ndarray],
    image_points: list[np.ndarray],
    rvecs,
    tvecs,
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
        per_image.append(math.sqrt(squared_error / len(obj)))
        total_squared_error += squared_error
        total_points += len(obj)
    overall = math.sqrt(total_squared_error / total_points) if total_points else float("nan")
    return per_image, overall


def save_calibration(
    output_path: Path,
    image_size: tuple[int, int],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    rms: float,
    reprojection_error: float,
    valid_image_count: int,
) -> None:
    fs = cv2.FileStorage(str(output_path), cv2.FILE_STORAGE_WRITE)
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


def calibrate_from_points(
    object_points: list[np.ndarray],
    image_points: list[np.ndarray],
    image_size: tuple[int, int],
    output_path: Path,
    image_names: list[str],
) -> None:
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )
    per_image, overall = reprojection_errors(
        object_points,
        image_points,
        rvecs,
        tvecs,
        camera_matrix,
        dist_coeffs,
    )
    save_calibration(output_path, image_size, camera_matrix, dist_coeffs, rms, overall, len(image_points))

    print("\nCalibration finished.")
    print(f"Valid images:        {len(image_points)}")
    print(f"calibrateCamera RMS: {rms:.6f} px")
    print(f"Reprojection error:  {overall:.6f} px")
    print("\nCamera matrix:")
    print(camera_matrix)
    print("\nDistortion coefficients:")
    print(dist_coeffs.reshape(-1))
    print(f"\nSaved result: {os.path.abspath(output_path)}")
    print("\nPer-image reprojection error:")
    for name, error in zip(image_names, per_image):
        print(f"  {name}: {error:.4f}px")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Automatically capture chessboard images from one IP camera and calibrate intrinsics.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--ip", default="192.168.1.105", help="Camera IP address.")
    parser.add_argument("--username", default="admin", help="Camera username.")
    parser.add_argument("--password", default="111qqq!!!", help="Camera password.")
    parser.add_argument("--port", type=int, default=554, help="RTSP port.")
    parser.add_argument("--rtsp-path", default="/Streaming/Channels/101", help="RTSP path used when --url is not provided.")
    parser.add_argument("--url", help="Full RTSP URL. Overrides --ip/username/password/path.")
    parser.add_argument("--width", type=int, default=1920, help="Requested camera frame width. Use 0 to keep stream default.")
    parser.add_argument("--height", type=int, default=1080, help="Requested camera frame height. Use 0 to keep stream default.")
    parser.add_argument("--board-cols", type=int, default=11, help="Inner corner count along chessboard columns.")
    parser.add_argument("--board-rows", type=int, default=8, help="Inner corner count along chessboard rows.")
    parser.add_argument("--square-size", type=float, default=0.06, help="Chessboard square size in meters.")
    parser.add_argument("--output-dir", help="Folder for captured images. Default is auto_capture_intrinsics/<ip>.")
    parser.add_argument("--calibration-output", help="Output OpenCV YAML file. Default is camera_intrinsics_<ip>.yml.")
    parser.add_argument("--auto-count", type=int, default=30, help="Stop and calibrate after this many captures. Use 0 for unlimited.")
    parser.add_argument("--min-images", type=int, default=8, help="Minimum captured images required before calibration.")
    parser.add_argument("--auto-interval", type=float, default=1.0, help="Minimum seconds between automatic captures.")
    parser.add_argument("--preview-width", type=int, default=900, help="Max preview window width.")
    parser.add_argument("--warmup-seconds", type=float, default=1.0, help="Seconds to wait after starting camera reader.")
    parser.add_argument("--digits", type=int, default=4, help="Filename number width.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    safe_ip = args.ip.replace(".", "_")
    output_dir = Path(args.output_dir or f"auto_capture_intrinsics/{safe_ip}")
    calibration_output = Path(args.calibration_output or f"camera_intrinsics_{safe_ip}.yml")
    log_path = output_dir / "capture_log.csv"

    url = args.url or build_rtsp_url(args.ip, args.username, args.password, args.port, args.rtsp_path)
    cap = open_camera(url, args.width, args.height)
    reader = LatestFrameReader(cap)
    board_size = (args.board_cols, args.board_rows)
    one_object_points = make_object_points(args.board_cols, args.board_rows, args.square_size)

    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    image_names: list[str] = []
    image_size: tuple[int, int] | None = None
    last_capture_time = 0.0
    index = next_index(output_dir, args.digits)

    output_dir.mkdir(parents=True, exist_ok=True)
    log_exists = log_path.exists()
    log_file = log_path.open("a", newline="", encoding="utf-8")
    log_writer = csv.writer(log_file)
    if not log_exists:
        log_writer.writerow(["filename", "save_time", "frame_time", "frame_index", "mode"])

    print("Opening camera...")
    print(f"camera: {args.ip if not args.url else args.url}")
    print(f"Saving images to: {os.path.abspath(output_dir)}")
    print(f"Calibration output: {os.path.abspath(calibration_output)}")
    print("Auto saves when the full chessboard is detected.")
    print("Press s to save manually, q or Esc to finish.")

    try:
        reader.start()
        time.sleep(args.warmup_seconds)

        while True:
            frame, frame_time, frame_index = reader.get_latest()
            if frame is None:
                key = cv2.waitKey(10) & 0xFF
                if key in (ord("q"), 27):
                    break
                continue

            if image_size is None:
                image_size = (frame.shape[1], frame.shape[0])

            found, corners, drawn = detect_chessboard(frame, board_size)
            saved_count = len(image_points)
            status = (
                f"{args.ip} | {frame.shape[1]}x{frame.shape[0]}\n"
                f"board: {'Y' if found else 'N'} | saved: {saved_count}/{args.auto_count if args.auto_count else 'inf'}"
            )
            cv2.imshow("auto capture intrinsics", draw_status(resize_for_preview(drawn, args.preview_width), status))

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break

            now = time.time()
            manual_save = key == ord("s")
            auto_save = found and (now - last_capture_time >= args.auto_interval)
            if found and (manual_save or auto_save):
                filename = f"{index:0{args.digits}d}.jpg"
                image_path = output_dir / filename
                imwrite_unicode(image_path, frame)
                object_points.append(one_object_points.copy())
                image_points.append(corners)
                image_names.append(filename)
                log_writer.writerow([
                    filename,
                    time.strftime("%Y-%m-%d %H:%M:%S"),
                    f"{frame_time:.6f}",
                    frame_index,
                    "manual" if manual_save else "auto",
                ])
                log_file.flush()
                print(f"saved {filename}")
                index += 1
                last_capture_time = now

                if args.auto_count > 0 and len(image_points) >= args.auto_count:
                    print(f"Auto capture target reached: {len(image_points)}")
                    break
    finally:
        log_file.close()
        reader.stop()
        cap.release()
        cv2.destroyAllWindows()

    if image_size is None:
        raise RuntimeError("No frames were read from the camera.")
    if len(image_points) < args.min_images:
        raise RuntimeError(f"Only {len(image_points)} images captured; need at least {args.min_images}.")

    calibrate_from_points(object_points, image_points, image_size, calibration_output, image_names)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
