#!/usr/bin/env python3
"""Visualize the sentry gimbal/camera/muzzle coordinate chain.

This script does not change runtime code. It loads the same YAML extrinsics
used by src/sentry/main.cpp and draws the frames in a manually selected world
pose.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import yaml


def as_matrix(values: list[float], rows: int, cols: int) -> np.ndarray:
    if len(values) != rows * cols:
        raise ValueError(f"expected {rows * cols} values, got {len(values)}")
    return np.array(values, dtype=float).reshape((rows, cols))


def as_vector(values: list[float], size: int) -> np.ndarray:
    if len(values) != size:
        raise ValueError(f"expected {size} values, got {len(values)}")
    return np.array(values, dtype=float)


def rotation_matrix_ypr(yaw: float, pitch: float, roll: float) -> np.ndarray:
    """Match tools::rotation_matrix: yaw about z, pitch about y, roll about x."""
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cr = math.cos(roll)
    sr = math.sin(roll)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def rotation_matrix_quat_wxyz(quat: list[float]) -> np.ndarray:
    """Return rotation matrix from quaternion in w, x, y, z order."""
    w, x, y, z = [float(v) for v in quat]
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-12:
        raise ValueError("zero quaternion")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def set_axes_equal(ax) -> None:
    limits = np.array([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()])
    center = limits.mean(axis=1)
    radius = 0.5 * max(limits[:, 1] - limits[:, 0])
    ax.set_xlim3d([center[0] - radius, center[0] + radius])
    ax.set_ylim3d([center[1] - radius, center[1] + radius])
    ax.set_zlim3d([center[2] - radius, center[2] + radius])


def draw_frame(ax, origin: np.ndarray, rotation: np.ndarray, label: str, scale: float = 0.08) -> None:
    colors = ["tab:red", "tab:green", "tab:blue"]
    names = ["x", "y", "z"]
    for i, (color, name) in enumerate(zip(colors, names)):
        axis = rotation[:, i] * scale
        ax.quiver(
            origin[0],
            origin[1],
            origin[2],
            axis[0],
            axis[1],
            axis[2],
            color=color,
            linewidth=2,
            arrow_length_ratio=0.18,
        )
        end = origin + axis * 1.15
        ax.text(end[0], end[1], end[2], f"{label} {name}", color=color, fontsize=8)
    ax.scatter([origin[0]], [origin[1]], [origin[2]], s=18, color="black")
    ax.text(origin[0], origin[1], origin[2], f" {label}", color="black", fontsize=9)


def print_matrix(name: str, value: np.ndarray) -> None:
    print(name)
    for row in value:
        print("  " + " ".join(f"{x: .4f}" for x in row))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-c", "--config", default="configs/demo.yaml")
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--pitch-deg", type=float, default=0.0)
    parser.add_argument("--roll-deg", type=float, default=0.0)
    parser.add_argument(
        "--quat",
        type=float,
        nargs=4,
        metavar=("W", "X", "Y", "Z"),
        help="IMU quaternion in w x y z order. If set, use the same formula as Solver::set_R_gimbal2world.",
    )
    parser.add_argument("--target", type=float, nargs=3, metavar=("X", "Y", "Z"))
    parser.add_argument("-o", "--output", default="/tmp/sentry_frames.png")
    args = parser.parse_args()

    config_path = Path(args.config)
    with config_path.open("r", encoding="utf-8") as file:
        cfg = yaml.safe_load(file)

    R_gimbal2imubody = as_matrix(cfg["R_gimbal2imubody"], 3, 3)
    R_camera2gimbal = as_matrix(cfg["R_camera2gimbal"], 3, 3)
    t_camera2gimbal = as_vector(cfg["t_camera2gimbal"], 3)
    t_muzzle2gimbal = as_vector(cfg.get("t_muzzle2gimbal", [0.0, 0.0, 0.0]), 3)
    imu_horizontal_mount = bool(cfg.get("imu_horizontal_mount", False))

    if args.quat:
        R_imubody2world = rotation_matrix_quat_wxyz(args.quat)
        if imu_horizontal_mount:
            # Runtime horizontal-mount chain: q is imu body -> world, config is gimbal -> imu body.
            R_gimbal2world = R_imubody2world @ R_gimbal2imubody
        else:
            R_gimbal2world = R_gimbal2imubody.T @ R_imubody2world @ R_gimbal2imubody
        pose_source = f"imu quat wxyz: {args.quat}"
    else:
        ypr = np.deg2rad([args.yaw_deg, args.pitch_deg, args.roll_deg])
        R_gimbal2world = rotation_matrix_ypr(ypr[0], ypr[1], ypr[2])
        pose_source = (
            f"manual gimbal ypr deg: "
            f"[{args.yaw_deg:.2f}, {args.pitch_deg:.2f}, {args.roll_deg:.2f}]"
        )
    R_camera2world = R_gimbal2world @ R_camera2gimbal
    R_imu2gimbal = R_gimbal2imubody.T
    R_imu2world = R_gimbal2world @ R_imu2gimbal

    gimbal_origin_world = np.zeros(3)
    camera_origin_world = R_gimbal2world @ t_camera2gimbal
    muzzle_origin_world = R_gimbal2world @ t_muzzle2gimbal
    barrel_assumed_world = R_gimbal2world @ np.array([1.0, 0.0, 0.0])

    print(f"config: {config_path}")
    print(pose_source)
    print(f"t_camera2gimbal: {t_camera2gimbal}")
    print(f"t_muzzle2gimbal: {t_muzzle2gimbal}")
    print(f"camera_origin_world: {camera_origin_world}")
    print(f"muzzle_origin_world: {muzzle_origin_world}")
    print(f"assumed barrel +X in world: {barrel_assumed_world}")
    print_matrix("R_gimbal2world", R_gimbal2world)
    print_matrix("R_camera2gimbal", R_camera2gimbal)
    print_matrix("R_imu2gimbal (drawn at gimbal origin; IMU position is not configured)", R_imu2gimbal)

    target_world = np.array(args.target, dtype=float) if args.target else None
    if target_world is not None:
        relative = target_world - muzzle_origin_world
        yaw = math.atan2(relative[1], relative[0])
        pitch = math.atan2(relative[2], math.hypot(relative[0], relative[1]))
        print(f"target_world: {target_world}")
        print(f"target - muzzle: {relative}")
        print(f"geometric yaw/pitch deg: [{math.degrees(yaw):.2f}, {math.degrees(pitch):.2f}]")

    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title("Sentry frame visualization")
    ax.set_xlabel("world X")
    ax.set_ylabel("world Y")
    ax.set_zlabel("world Z")

    draw_frame(ax, gimbal_origin_world, R_gimbal2world, "gimbal", 0.12)
    draw_frame(ax, camera_origin_world, R_camera2world, "camera", 0.07)
    draw_frame(ax, gimbal_origin_world, R_imu2world, "imu", 0.07)

    ax.scatter(
        [muzzle_origin_world[0]],
        [muzzle_origin_world[1]],
        [muzzle_origin_world[2]],
        color="magenta",
        s=35,
        label="muzzle origin",
    )
    ax.text(
        muzzle_origin_world[0],
        muzzle_origin_world[1],
        muzzle_origin_world[2],
        " muzzle",
        color="magenta",
    )

    barrel_end = muzzle_origin_world + barrel_assumed_world * 0.18
    ax.plot(
        [muzzle_origin_world[0], barrel_end[0]],
        [muzzle_origin_world[1], barrel_end[1]],
        [muzzle_origin_world[2], barrel_end[2]],
        color="magenta",
        linewidth=3,
        label="assumed barrel +X",
    )

    ax.plot(
        [gimbal_origin_world[0], camera_origin_world[0]],
        [gimbal_origin_world[1], camera_origin_world[1]],
        [gimbal_origin_world[2], camera_origin_world[2]],
        color="gray",
        linestyle="--",
        linewidth=1,
        label="gimbal to camera t",
    )

    if np.linalg.norm(t_muzzle2gimbal) > 1e-9:
        ax.plot(
            [gimbal_origin_world[0], muzzle_origin_world[0]],
            [gimbal_origin_world[1], muzzle_origin_world[1]],
            [gimbal_origin_world[2], muzzle_origin_world[2]],
            color="purple",
            linestyle="--",
            linewidth=1,
            label="gimbal to muzzle t",
        )

    if target_world is not None:
        ax.scatter([target_world[0]], [target_world[1]], [target_world[2]], color="orange", s=35)
        ax.text(target_world[0], target_world[1], target_world[2], " target", color="orange")
        ax.plot(
            [muzzle_origin_world[0], target_world[0]],
            [muzzle_origin_world[1], target_world[1]],
            [muzzle_origin_world[2], target_world[2]],
            color="orange",
            linewidth=2,
            label="target - muzzle",
        )

    all_points = [gimbal_origin_world, camera_origin_world, muzzle_origin_world, barrel_end]
    if target_world is not None:
        all_points.append(target_world)
    points = np.array(all_points)
    margin = 0.08
    ax.set_xlim(points[:, 0].min() - margin, points[:, 0].max() + margin)
    ax.set_ylim(points[:, 1].min() - margin, points[:, 1].max() + margin)
    ax.set_zlim(points[:, 2].min() - margin, points[:, 2].max() + margin)
    set_axes_equal(ax)
    ax.legend(loc="upper left", fontsize=8)
    ax.view_init(elev=25, azim=-55)

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    print(f"wrote: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
