#!/usr/bin/env python3
"""
Manually or automatically capture paired chessboard images from two IP cameras.

Default cameras:
    192.168.1.102
    192.168.1.105

Keyboard:
    c or Space  capture one paired image
    q or Esc    quit

Example:
    python manual_capture_camera_pair.py --auto ^
        --board-cols 11 ^
        --board-rows 8 ^
        --output pairs/cam102_cam105 ^
        --username admin

If the default RTSP path does not work for your camera, pass full URLs:
    python manual_capture_camera_pair.py ^
        --url1 "rtsp://admin:URL_ENCODED_PASSWORD@192.168.1.102:554/Streaming/Channels/101" ^
        --url2 "rtsp://admin:URL_ENCODED_PASSWORD@192.168.1.105:554/Streaming/Channels/101"
"""

from __future__ import annotations

import argparse
import csv
import os
import threading
import time
from pathlib import Path
from urllib.parse import quote

import cv2


def build_rtsp_url(ip: str, username: str, password: str, port: int, path: str) -> str:
    encoded_user = quote(username, safe="")
    encoded_password = quote(password, safe="")
    normalized_path = path if path.startswith("/") else f"/{path}"
    return f"rtsp://{encoded_user}:{encoded_password}@{ip}:{port}{normalized_path}"


def open_camera(name: str, url: str, width: int, height: int) -> cv2.VideoCapture:
    capture = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    if not capture.isOpened():
        raise RuntimeError(f"Cannot open {name}: {url}")
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if width > 0:
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height > 0:
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return capture


class LatestFrameReader:
    def __init__(self, name: str, capture: cv2.VideoCapture) -> None:
        self.name = name
        self.capture = capture
        self.lock = threading.Lock()
        self.frame = None
        self.timestamp = 0.0
        self.frame_index = 0
        self.running = False
        self.thread: threading.Thread | None = None
        self.last_error: str | None = None

    def start(self) -> None:
        self.running = True
        self.thread = threading.Thread(target=self._loop, name=f"{self.name}-reader", daemon=True)
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
                self.last_error = f"Cannot read frame from {self.name}."
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


def resize_for_preview(frame, max_width: int):
    height, width = frame.shape[:2]
    if width <= max_width:
        return frame
    scale = max_width / width
    new_size = (max_width, int(height * scale))
    return cv2.resize(frame, new_size, interpolation=cv2.INTER_AREA)


def draw_label(frame, label: str, status: str = ""):
    output = frame.copy()
    cv2.putText(
        output,
        label,
        (16, 34),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    if status:
        cv2.putText(
            output,
            status,
            (16, 68),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
    return output


def detect_chessboard(frame, board_size: tuple[int, int], draw: bool):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        | cv2.CALIB_CB_NORMALIZE_IMAGE
        | cv2.CALIB_CB_FAST_CHECK
    )
    found, corners = cv2.findChessboardCorners(gray, board_size, flags)
    if not found:
        return False, frame

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )
    refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    output = frame
    if draw:
        output = frame.copy()
        cv2.drawChessboardCorners(output, board_size, refined, found)
    return True, output


def next_index(folder1: Path, folder2: Path, digits: int) -> int:
    existing_numbers: list[int] = []
    for folder in (folder1, folder2):
        for path in folder.glob("*.jpg"):
            try:
                existing_numbers.append(int(path.stem))
            except ValueError:
                continue
    if not existing_numbers:
        return 1
    return max(existing_numbers) + 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Manually capture paired images from two IP cameras for stereo calibration.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--ip1", default="192.168.1.102", help="First camera IP address.")
    parser.add_argument("--ip2", default="192.168.1.105", help="Second camera IP address.")
    parser.add_argument("--username", default="admin", help="Camera username.")
    parser.add_argument(
        "--password",
        default=os.environ.get("HIKVISION_PASSWORD", ""),
        help="Camera password (defaults to HIKVISION_PASSWORD).",
    )
    parser.add_argument("--port", type=int, default=554, help="RTSP port.")
    parser.add_argument(
        "--rtsp-path",
        default="/Streaming/Channels/101",
        help="RTSP path used when --url1/--url2 are not provided.",
    )
    parser.add_argument("--url1", help="Full RTSP URL for camera 1. Overrides --ip1/username/password/path.")
    parser.add_argument("--url2", help="Full RTSP URL for camera 2. Overrides --ip2/username/password/path.")
    parser.add_argument("--output", default="pairs/cam102_cam105", help="Output folder for paired images.")
    parser.add_argument("--cam1-name", default="cam1", help="Subfolder name for camera 1 images.")
    parser.add_argument("--cam2-name", default="cam2", help="Subfolder name for camera 2 images.")
    parser.add_argument("--digits", type=int, default=4, help="Filename number width.")
    parser.add_argument("--preview-width", type=int, default=800, help="Max preview width for each camera.")
    parser.add_argument("--width", type=int, default=1920, help="Requested camera frame width. Use 0 to keep stream default.")
    parser.add_argument("--height", type=int, default=1080, help="Requested camera frame height. Use 0 to keep stream default.")
    parser.add_argument("--warmup-seconds", type=float, default=1.0, help="Seconds to wait after starting readers.")
    parser.add_argument("--max-pair-delay-ms", type=float, default=80.0, help="Warn when latest frame timestamps differ more than this.")
    parser.add_argument("--board-cols", type=int, default=11, help="Inner corner count along chessboard columns for auto capture.")
    parser.add_argument("--board-rows", type=int, default=8, help="Inner corner count along chessboard rows for auto capture.")
    parser.add_argument("--auto", action="store_true", help="Automatically save when both cameras detect the full chessboard.")
    parser.add_argument("--auto-count", type=int, default=30, help="Stop after this many automatic captures. Use 0 for unlimited.")
    parser.add_argument("--auto-interval", type=float, default=1.0, help="Minimum seconds between automatic captures.")
    return parser.parse_args()


def save_pair(
    cam1_dir: Path,
    cam2_dir: Path,
    filename: str,
    frame1,
    frame2,
    timestamp1: float,
    timestamp2: float,
    pair_delay_ms: float,
    frame_index1: int,
    frame_index2: int,
    log_writer,
    log_file,
    max_pair_delay_ms: float,
    mode: str,
) -> None:
    path1 = cam1_dir / filename
    path2 = cam2_dir / filename
    ok1 = cv2.imwrite(str(path1), frame1)
    ok2 = cv2.imwrite(str(path2), frame2)
    if not ok1 or not ok2:
        raise RuntimeError(f"Failed to save pair {filename}.")

    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    log_writer.writerow([
        filename,
        timestamp,
        f"{timestamp1:.6f}",
        f"{timestamp2:.6f}",
        f"{pair_delay_ms:.3f}",
        frame_index1,
        frame_index2,
        mode,
    ])
    log_file.flush()
    warning = " WARNING: delay too large" if pair_delay_ms > max_pair_delay_ms else ""
    print(f"[{timestamp}] saved pair {filename}, delay={pair_delay_ms:.1f}ms{warning}")


def main() -> int:
    args = parse_args()

    url1 = args.url1 or build_rtsp_url(args.ip1, args.username, args.password, args.port, args.rtsp_path)
    url2 = args.url2 or build_rtsp_url(args.ip2, args.username, args.password, args.port, args.rtsp_path)

    output_root = Path(args.output)
    cam1_dir = output_root / args.cam1_name
    cam2_dir = output_root / args.cam2_name
    cam1_dir.mkdir(parents=True, exist_ok=True)
    cam2_dir.mkdir(parents=True, exist_ok=True)

    print("Opening cameras...")
    print(f"camera 1: {args.ip1 if not args.url1 else args.url1}")
    print(f"camera 2: {args.ip2 if not args.url2 else args.url2}")
    cap1 = open_camera("camera 1", url1, args.width, args.height)
    cap2 = open_camera("camera 2", url2, args.width, args.height)
    reader1 = LatestFrameReader("camera 1", cap1)
    reader2 = LatestFrameReader("camera 2", cap2)
    log_path = output_root / "capture_log.csv"

    try:
        reader1.start()
        reader2.start()
        time.sleep(args.warmup_seconds)

        index = next_index(cam1_dir, cam2_dir, args.digits)
        print("\nPreview started.")
        print("Press c or Space to capture one pair. Press q or Esc to quit.")
        if args.auto:
            print(
                f"Auto capture enabled: board={args.board_cols}x{args.board_rows}, "
                f"target={args.auto_count if args.auto_count else 'unlimited'}, interval={args.auto_interval:.2f}s"
            )
        print(f"Saving to: {os.path.abspath(output_root)}")
        print(f"Pair delay warning threshold: {args.max_pair_delay_ms:.1f} ms")

        log_exists = log_path.exists()
        log_file = log_path.open("a", newline="", encoding="utf-8")
        log_writer = csv.writer(log_file)
        if not log_exists:
            log_writer.writerow([
                "filename",
                "save_time",
                "cam1_frame_time",
                "cam2_frame_time",
                "pair_delay_ms",
                "cam1_frame_index",
                "cam2_frame_index",
                "mode",
            ])

        board_size = (args.board_cols, args.board_rows)
        last_auto_capture_time = 0.0
        auto_saved_count = 0

        while True:
            frame1, timestamp1, frame_index1 = reader1.get_latest()
            frame2, timestamp2, frame_index2 = reader2.get_latest()
            if frame1 is None or frame2 is None:
                key = cv2.waitKey(10) & 0xFF
                if key in (ord("q"), 27):
                    break
                continue

            pair_delay_ms = abs(timestamp1 - timestamp2) * 1000.0
            found1 = False
            found2 = False
            display_frame1 = frame1
            display_frame2 = frame2
            if args.auto:
                found1, display_frame1 = detect_chessboard(frame1, board_size, True)
                found2, display_frame2 = detect_chessboard(frame2, board_size, True)

            status = f"delay: {pair_delay_ms:.1f} ms | board: {'Y' if found1 else 'N'}/{'Y' if found2 else 'N'}"

            preview1 = draw_label(resize_for_preview(display_frame1, args.preview_width), f"{args.ip1} / {args.cam1_name}", status)
            preview2 = draw_label(resize_for_preview(display_frame2, args.preview_width), f"{args.ip2} / {args.cam2_name}", status)

            cv2.imshow("camera 1", preview1)
            cv2.imshow("camera 2", preview2)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            should_manual_save = key in (ord("c"), ord(" "), 13)
            should_auto_save = False
            now = time.time()
            if args.auto and found1 and found2 and pair_delay_ms <= args.max_pair_delay_ms:
                should_auto_save = now - last_auto_capture_time >= args.auto_interval

            if should_manual_save or should_auto_save:
                filename = f"{index:0{args.digits}d}.jpg"
                save_pair(
                    cam1_dir,
                    cam2_dir,
                    filename,
                    frame1,
                    frame2,
                    timestamp1,
                    timestamp2,
                    pair_delay_ms,
                    frame_index1,
                    frame_index2,
                    log_writer,
                    log_file,
                    args.max_pair_delay_ms,
                    "auto" if should_auto_save else "manual",
                )
                if should_auto_save:
                    last_auto_capture_time = now
                    auto_saved_count += 1
                index += 1
                if args.auto and args.auto_count > 0 and auto_saved_count >= args.auto_count:
                    print(f"Auto capture target reached: {auto_saved_count}")
                    break
    finally:
        try:
            log_file.close()
        except UnboundLocalError:
            pass
        reader1.stop()
        reader2.stop()
        cap1.release()
        cap2.release()
        cv2.destroyAllWindows()

    print("Capture finished.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
