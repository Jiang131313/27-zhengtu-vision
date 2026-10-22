#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从 Kalibr 联合标定结果换算出 demo.yaml 需要的外参。

用法:
    python3 calibration/extract_extrinsics.py <camchain-imucam.yaml>

输入: Kalibr 输出的 *-camchain-imucam.yaml（含 T_cam_imu、timeshift_cam_imu）
输出: 直接可粘进 configs/demo.yaml 的三行
      - imu_time_offset_s
      - R_camera2gimbal
      - t_camera2gimbal
并做三轴方向自检。

前置条件（脚本不标定、不猜，必须你手填对）:
    R_gimbal2imubody 在 demo.yaml 里要填字面 "云台->IMU" 方向（行优先）。
    代码横装分支 solver.cpp:77  R_gimbal2world = R_imubody2world * R_gimbal2imubody_
    不取逆，所以变量名和填法一致，都是 gimbal->imu。

换算公式（链路 camera -> imu -> gimbal）:
    R_camera2gimbal = (R_gimbal2imubody)^T * (R_cam_imu)^T
    t_camera2gimbal = (R_gimbal2imubody)^T * ( -R_cam_imu^T * t_cam_imu )
其中 R_cam_imu / t_cam_imu 取自 Kalibr 的 T_cam_imu，方向是 imu->cam
（p_cam = R_cam_imu * p_imu + t_cam_imu）。

约定:
    云台系: x前 / y左 / z上
    相机系: z前 / x右 / y下  (OpenCV/PnP 标准)
    本机安装(IMU壳子标注): 云台前=IMU+Y, 云台右=IMU+X, 云台上=IMU+Z
      => R_gimbal2imubody = [0,-1,0, 1,0,0, 0,0,1] (行优先)
"""

import sys
import numpy as np
import yaml

# ---- 本机 IMU 安装关系（IMU 没动安装就别改这里）----
# 行优先，含义: 云台->IMU 旋转。列 = 云台第j个基在 IMU 系下的坐标。
# 云台x(前)->IMU(0,1,0); 云台y(左)->IMU(-1,0,0); 云台z(上)->IMU(0,0,1)
R_GIMBAL2IMUBODY_ROWMAJOR = [0, -1, 0, 1, 0, 0, 0, 0, 1]


def load_camchain(path):
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    cam0 = data["cam0"]
    T = np.array(cam0["T_cam_imu"], dtype=float)  # 4x4
    R_cam_imu = T[:3, :3]          # imu->cam
    t_cam_imu = T[:3, 3]           # IMU原点在相机系
    timeshift = float(cam0["timeshift_cam_imu"])
    return R_cam_imu, t_cam_imu, timeshift


def compute(R_cam_imu, t_cam_imu, R_g2i_rowmajor):
    R_g2i = np.array(R_g2i_rowmajor, dtype=float).reshape(3, 3)
    R_i2g = R_g2i.T
    R_c2g = R_i2g @ R_cam_imu.T
    t_c2g = R_i2g @ (-R_cam_imu.T @ t_cam_imu)
    return R_c2g, t_c2g


def check_axes(R_c2g):
    """验相机三轴映到云台系的方向是否正确。"""
    checks = [
        ("相机前 z (0,0,1) -> 云台 +x(前)", np.array([0, 0, 1]), np.array([1, 0, 0])),
        ("相机右 x (1,0,0) -> 云台 -y(右)", np.array([1, 0, 0]), np.array([0, -1, 0])),
        ("相机下 y (0,1,0) -> 云台 -z(下)", np.array([0, 1, 0]), np.array([0, 0, -1])),
    ]
    print("\n=== 三轴方向自检 ===")
    all_ok = True
    for name, cam_axis, expect in checks:
        got = R_c2g @ cam_axis
        ok = np.allclose(got, expect, atol=0.05)
        all_ok = all_ok and ok
        flag = "OK " if ok else "FAIL"
        print(f"  [{flag}] {name}: 得到 {np.round(got, 3)} 期望 {expect}")
    if not all_ok:
        print("\n  !! 有轴方向不对。多半是 R_gimbal2imubody 填反了，或 IMU 安装变了。")
    return all_ok


def fmt_rowmajor(M, ndigits=8):
    return "[" + ", ".join(f"{v:.{ndigits}f}" for v in M.flatten("C")) + "]"


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)

    path = sys.argv[1]
    R_cam_imu, t_cam_imu, timeshift = load_camchain(path)
    R_c2g, t_c2g = compute(R_cam_imu, t_cam_imu, R_GIMBAL2IMUBODY_ROWMAJOR)

    print(f"读取: {path}")
    print(f"  R_cam_imu (imu->cam) 模长行列式: {np.linalg.det(R_cam_imu):.6f} (应≈1)")
    print(f"  t_cam_imu 模长: {np.linalg.norm(t_cam_imu):.4f} m", end="")
    if np.linalg.norm(t_cam_imu) < 0.02:
        print("  ⚠ <2cm，平移激励不足，t_camera2gimbal 不可信，建议重录带平移运动的数据")
    else:
        print("  (OK)")

    check_axes(R_c2g)

    print("\n=== 复制到 configs/demo.yaml 的三行 ===")
    print(f"imu_time_offset_s: {timeshift}")
    print(f"R_camera2gimbal: {fmt_rowmajor(R_c2g)}")
    print(f"t_camera2gimbal: {fmt_rowmajor(t_c2g, ndigits=6)}")

    print("\n（R_gimbal2imubody 不变，仍为:",
          fmt_rowmajor(np.array(R_GIMBAL2IMUBODY_ROWMAJOR).reshape(3, 3), 0) + "）")


if __name__ == "__main__":
    main()
