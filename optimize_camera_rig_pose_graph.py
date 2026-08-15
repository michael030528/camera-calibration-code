#!/usr/bin/env python3
"""
Optimize a 6-camera rig pose graph from pairwise camera extrinsics.

The input pair files use this convention:

    P_cam2 = R_cam1_to_cam2 * P_cam1 + T_cam1_to_cam2

This script fixes one camera as the rig frame, optimizes all camera-to-rig
poses, then writes optimized rig poses and pairwise extrinsics.

Example:
    python optimize_camera_rig_pose_graph.py ^
        --input-dir rig_best_results_separated/extrinsics_best ^
        --output-dir rig_pose_graph_optimized ^
        --fixed-camera cam101
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import cv2
import numpy as np


DEFAULT_EDGES = [
    ("cam101", "cam103", "camera_pair_101_103.yml"),
    ("cam103", "cam104", "camera_pair_103_104.yml"),
    ("cam104", "cam106", "camera_pair_104_106.yml"),
    ("cam106", "cam102", "camera_pair_106_102.yml"),
    ("cam102", "cam105", "camera_pair_102_105.yml"),
    ("cam105", "cam101", "camera_pair_105_101.yml"),
]


def skew(v: np.ndarray) -> np.ndarray:
    x, y, z = v.reshape(3)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=np.float64)


def so3_exp(w: np.ndarray) -> np.ndarray:
    return cv2.Rodrigues(w.reshape(3, 1))[0]


def so3_log(R: np.ndarray) -> np.ndarray:
    return cv2.Rodrigues(R)[0].reshape(3)


def se3_exp(xi: np.ndarray) -> np.ndarray:
    """SE(3) exponential. xi = [omega_x, omega_y, omega_z, v_x, v_y, v_z]."""
    xi = xi.reshape(6)
    w = xi[:3]
    v = xi[3:]
    theta = float(np.linalg.norm(w))
    W = skew(w)
    R = so3_exp(w)
    if theta < 1e-10:
        V = np.eye(3) + 0.5 * W + (1.0 / 6.0) * (W @ W)
    else:
        theta2 = theta * theta
        V = (
            np.eye(3)
            + ((1.0 - math.cos(theta)) / theta2) * W
            + ((theta - math.sin(theta)) / (theta2 * theta)) * (W @ W)
        )
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = V @ v
    return T


def se3_log(T: np.ndarray) -> np.ndarray:
    """SE(3) logarithm. Returns [omega_x, omega_y, omega_z, v_x, v_y, v_z]."""
    R = T[:3, :3]
    t = T[:3, 3]
    w = so3_log(R)
    theta = float(np.linalg.norm(w))
    W = skew(w)
    if theta < 1e-10:
        V_inv = np.eye(3) - 0.5 * W + (1.0 / 12.0) * (W @ W)
    else:
        half_theta = 0.5 * theta
        cot_half = 1.0 / math.tan(half_theta)
        V_inv = (
            np.eye(3)
            - 0.5 * W
            + (1.0 - theta * cot_half / 2.0) / (theta * theta) * (W @ W)
        )
    v = V_inv @ t
    return np.hstack([w, v])


def invert(T: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    R = T[:3, :3]
    t = T[:3, 3]
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def read_pair(path: Path) -> tuple[np.ndarray, float, float]:
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    R = fs.getNode("R_cam1_to_cam2").mat()
    t = fs.getNode("T_cam1_to_cam2").mat().reshape(3)
    rms = float(fs.getNode("stereo_calibrate_rms").real())
    stereo_err = float(fs.getNode("reprojection_error_stereo_px").real())
    fs.release()
    if R is None or t is None:
        raise RuntimeError(f"Cannot read extrinsic from {path}")
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T, rms, stereo_err


def write_transform_yml(path: Path, key_prefix: str, T: np.ndarray) -> None:
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    fs.write(f"R_{key_prefix}", T[:3, :3])
    fs.write(f"T_{key_prefix}", T[:3, 3].reshape(3, 1))
    fs.release()


def rotation_angle_deg(R: np.ndarray) -> float:
    c = max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0))
    return math.degrees(math.acos(c))


def initialize_poses(edges: list[dict], fixed_camera: str) -> dict[str, np.ndarray]:
    """Initialize C_cam_to_rig poses from the edge chain."""
    poses: dict[str, np.ndarray] = {fixed_camera: np.eye(4)}
    changed = True
    while changed:
        changed = False
        for edge in edges:
            src, dst, Z = edge["src"], edge["dst"], edge["T"]
            if src in poses and dst not in poses:
                poses[dst] = poses[src] @ invert(Z)
                changed = True
            elif dst in poses and src not in poses:
                poses[src] = poses[dst] @ Z
                changed = True
    all_cameras = {e["src"] for e in edges} | {e["dst"] for e in edges}
    missing = sorted(all_cameras - set(poses))
    if missing:
        raise RuntimeError(f"Cannot initialize poses for: {missing}")
    return poses


def edge_residual(edge: dict, poses: dict[str, np.ndarray]) -> np.ndarray:
    """Residual for measurement Z src->dst with camera-to-rig poses C."""
    C_src = poses[edge["src"]]
    C_dst = poses[edge["dst"]]
    Z_pred = invert(C_dst) @ C_src
    err_T = invert(edge["T"]) @ Z_pred
    r = se3_log(err_T)
    return edge["weight"] * r


def all_residuals(edges: list[dict], poses: dict[str, np.ndarray]) -> np.ndarray:
    return np.concatenate([edge_residual(edge, poses) for edge in edges])


def optimize(
    edges: list[dict],
    poses: dict[str, np.ndarray],
    fixed_camera: str,
    iterations: int,
    damping: float,
) -> tuple[dict[str, np.ndarray], list[float]]:
    cameras = sorted(poses)
    variable_cameras = [c for c in cameras if c != fixed_camera]
    history: list[float] = []

    for _ in range(iterations):
        r0 = all_residuals(edges, poses)
        cost0 = 0.5 * float(r0 @ r0)
        history.append(cost0)

        n_vars = 6 * len(variable_cameras)
        J = np.zeros((r0.size, n_vars), dtype=np.float64)
        eps = 1e-6

        for cam_idx, cam in enumerate(variable_cameras):
            base_col = 6 * cam_idx
            for k in range(6):
                delta = np.zeros(6)
                delta[k] = eps
                perturbed = dict(poses)
                perturbed[cam] = se3_exp(delta) @ poses[cam]
                rk = all_residuals(edges, perturbed)
                J[:, base_col + k] = (rk - r0) / eps

        A = J.T @ J + damping * np.eye(n_vars)
        b = -J.T @ r0
        try:
            dx = np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            dx = np.linalg.lstsq(A, b, rcond=None)[0]

        max_step = float(np.max(np.abs(dx))) if dx.size else 0.0
        candidate = dict(poses)
        for cam_idx, cam in enumerate(variable_cameras):
            delta = dx[6 * cam_idx : 6 * cam_idx + 6]
            candidate[cam] = se3_exp(delta) @ poses[cam]

        cost1 = 0.5 * float(all_residuals(edges, candidate) @ all_residuals(edges, candidate))
        if cost1 < cost0:
            poses = candidate
            if max_step < 1e-9:
                break
        else:
            damping *= 10.0

    history.append(0.5 * float(all_residuals(edges, poses) @ all_residuals(edges, poses)))
    return poses, history


def closure_error(edges: list[dict]) -> tuple[float, float, np.ndarray, np.ndarray]:
    T = np.eye(4)
    for edge in edges:
        T = edge["T"] @ T
    return rotation_angle_deg(T[:3, :3]), float(np.linalg.norm(T[:3, 3])), T[:3, 3], T


def predicted_edge(src: str, dst: str, poses: dict[str, np.ndarray]) -> np.ndarray:
    return invert(poses[dst]) @ poses[src]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Optimize global 6-camera rig poses from pairwise extrinsics.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--input-dir", default="rig_best_results_separated/extrinsics_best")
    parser.add_argument("--output-dir", default="rig_pose_graph_optimized")
    parser.add_argument("--fixed-camera", default="cam101")
    parser.add_argument("--iterations", type=int, default=30)
    parser.add_argument("--damping", type=float, default=1e-4)
    parser.add_argument(
        "--unweighted",
        action="store_true",
        help="Use equal edge weights instead of weighting by stereo reprojection error.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir)
    poses_dir = output_dir / "camera_poses"
    pairs_dir = output_dir / "optimized_pairs"
    poses_dir.mkdir(parents=True, exist_ok=True)
    pairs_dir.mkdir(parents=True, exist_ok=True)

    edges: list[dict] = []
    for src, dst, filename in DEFAULT_EDGES:
        T, rms, stereo_err = read_pair(input_dir / filename)
        weight = 1.0 if args.unweighted else 1.0 / max(stereo_err, 1e-6)
        edges.append(
            {
                "src": src,
                "dst": dst,
                "filename": filename,
                "T": T,
                "rms": rms,
                "stereo_err": stereo_err,
                "weight": weight,
            }
        )

    initial_rot, initial_trans, initial_vec, initial_T = closure_error(edges)
    poses0 = initialize_poses(edges, args.fixed_camera)
    poses, history = optimize(edges, poses0, args.fixed_camera, args.iterations, args.damping)

    report: list[str] = []
    report.append("# Rig Pose Graph Optimization Report")
    report.append("")
    report.append(f"Input directory: `{input_dir}`")
    report.append(f"Fixed rig camera: `{args.fixed_camera}`")
    report.append("")
    report.append("## Input Closure")
    report.append("")
    report.append(f"- Rotation closure error: `{initial_rot:.6f} deg`")
    report.append(f"- Translation closure error: `{initial_trans:.6f} m` / `{initial_trans * 100:.3f} cm`")
    report.append(
        f"- Translation residual xyz: `[{initial_vec[0]:.6f}, {initial_vec[1]:.6f}, {initial_vec[2]:.6f}] m`"
    )
    report.append("")
    report.append("## Optimization")
    report.append("")
    report.append(f"- Initial weighted cost: `{history[0]:.8f}`")
    report.append(f"- Final weighted cost: `{history[-1]:.8f}`")
    report.append(f"- Iterations recorded: `{len(history) - 1}`")
    report.append("")
    report.append("## Edge Residuals After Optimization")
    report.append("")
    report.append("| Pair | Input Stereo Error | Optimized Rotation Residual | Optimized Translation Residual |")
    report.append("|---|---:|---:|---:|")

    optimized_edges: list[dict] = []
    for edge in edges:
        T_opt = predicted_edge(edge["src"], edge["dst"], poses)
        delta = invert(edge["T"]) @ T_opt
        rot_res = rotation_angle_deg(delta[:3, :3])
        trans_res = float(np.linalg.norm(delta[:3, 3]))
        report.append(
            f"| `{edge['src']} -> {edge['dst']}` | `{edge['stereo_err']:.4f} px` | "
            f"`{rot_res:.6f} deg` | `{trans_res:.6f} m` |"
        )
        optimized_edges.append({**edge, "T_opt": T_opt, "rot_res": rot_res, "trans_res": trans_res})

    report.append("")
    report.append("## Output Files")
    report.append("")
    report.append("- `camera_poses/`: optimized camera-to-rig poses")
    report.append("- `optimized_pairs/`: pairwise extrinsics recomputed from optimized poses")

    for cam, T in sorted(poses.items()):
        write_transform_yml(poses_dir / f"{cam}_to_rig.yml", f"{cam}_to_rig", T)

    for edge in optimized_edges:
        filename = f"optimized_pair_{edge['src']}_{edge['dst']}.yml"
        write_transform_yml(pairs_dir / filename, f"{edge['src']}_to_{edge['dst']}", edge["T_opt"])

    (output_dir / "optimization_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")

    print("Optimization finished.")
    print(f"Input closure rotation:      {initial_rot:.6f} deg")
    print(f"Input closure translation:   {initial_trans:.6f} m ({initial_trans * 100:.3f} cm)")
    print(f"Initial weighted cost:       {history[0]:.8f}")
    print(f"Final weighted cost:         {history[-1]:.8f}")
    print(f"Saved output:                {output_dir.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
