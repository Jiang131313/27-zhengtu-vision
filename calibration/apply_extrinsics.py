#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
一条龙：读 Kalibr camchain -> 换算外参 -> 直接写进 configs/demo.yaml。

用法:
    python3 calibration/apply_extrinsics.py <camchain-imucam.yaml> [demo.yaml路径]
    python3 calibration/apply_extrinsics.py calibration/output/dataset_xxx-camchain-imucam.yaml
    python3 calibration/apply_extrinsics.py calibration/output/dataset_xxx-camchain-imucam.yaml configs/demo.yaml

行为:
    - 覆盖 demo.yaml 里的三行: imu_time_offset_s / R_camera2gimbal / t_camera2gimbal
    - 写之前先做三轴方向自检 + 平移模长检查；自检不过会中止，不写文件
    - 先备份原文件到 <demo.yaml>.bak，再原地覆盖
    - R_gimbal2imubody 不动（代表 IMU 物理安装，靠手填，不靠标定）

注意:
    本脚本只替换「键: 值」整行，要求这三个键在 demo.yaml 里各只出现一次（行首匹配）。
    换算公式与 extract_extrinsics.py 一致，见该文件头部注释。
"""

import os
import re
import sys
import numpy as np
import yaml

R_GIMBAL2IMUBODY_ROWMAJOR = [0, -1, 0, 1, 0, 0, 0, 0, 1]  # 本机 IMU 安装，行优先，gimbal->imu


def load_camchain(path):
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    cam0 = data["cam0"]
    T = np.array(cam0["T_cam_imu"], dtype=float)
    return T[:3, :3], T[:3, 3], float(cam0["timeshift_cam_imu"])


def compute(R_cam_imu, t_cam_imu):
    R_g2i = np.array(R_GIMBAL2IMUBODY_ROWMAJOR, dtype=float).reshape(3, 3)
    R_i2g = R_g2i.T
    R_c2g = R_i2g @ R_cam_imu.T
    t_c2g = R_i2g @ (-R_cam_imu.T @ t_cam_imu)
    return R_c2g, t_c2g


def check_axes(R_c2g):
    checks = [
        ("相机前 z", np.array([0, 0, 1]), np.array([1, 0, 0])),
        ("相机右 x", np.array([1, 0, 0]), np.array([0, -1, 0])),
        ("相机下 y", np.array([0, 1, 0]), np.array([0, 0, -1])),
    ]
    ok_all = True
    for name, cam_axis, expect in checks:
        got = R_c2g @ cam_axis
        ok = np.allclose(got, expect, atol=0.05)
        ok_all = ok_all and ok
        print(f"  [{'OK ' if ok else 'FAIL'}] {name} -> {np.round(got, 3)} 期望 {expect}")
    return ok_all


def fmt_rowmajor(M, ndigits=8):
    return "[" + ", ".join(f"{v:.{ndigits}f}" for v in M.flatten("C")) + "]"


def replace_key_line(text, key, new_value):
    """把 demo.yaml 中 `key: ...` 整行替换为 `key: new_value`。要求该 key 行首唯一。"""
    pattern = re.compile(rf"^{re.escape(key)}:\s*.*$", re.MULTILINE)
    matches = pattern.findall(text)
    if len(matches) != 1:
        raise RuntimeError(f"demo.yaml 里 '{key}:' 出现 {len(matches)} 次，应为 1 次，中止")
    return pattern.sub(f"{key}: {new_value}", text, count=1)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    camchain = sys.argv[1]
    demo = sys.argv[2] if len(sys.argv) >= 3 else "configs/demo.yaml"
    if not os.path.isabs(demo):
        demo = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", demo)
        demo = os.path.normpath(demo)

    if not os.path.isfile(camchain):
        print(f"camchain 不存在: {camchain}", file=sys.stderr)
        sys.exit(1)
    if not os.path.isfile(demo):
        print(f"demo.yaml 不存在: {demo}", file=sys.stderr)
        sys.exit(1)

    R_cam_imu, t_cam_imu, timeshift = load_camchain(camchain)
    R_c2g, t_c2g = compute(R_cam_imu, t_cam_imu)

    print(f"读取: {camchain}")
    print(f"  t_cam_imu 模长: {np.linalg.norm(t_cam_imu):.4f} m", end="")
    if np.linalg.norm(t_cam_imu) < 0.02:
        print("  ⚠ <2cm 平移激励不足，t_camera2gimbal 不可信，已中止写入")
        sys.exit(1)
    print(" (OK)")

    print("=== 三轴方向自检 ===")
    if not check_axes(R_c2g):
        print("\n自检不过，多半 R_gimbal2imubody 填反或 IMU 安装变了，已中止写入")
        sys.exit(1)

    new_values = {
        "imu_time_offset_s": f"{timeshift}",
        "R_camera2gimbal": fmt_rowmajor(R_c2g),
        "t_camera2gimbal": fmt_rowmajor(t_c2g, ndigits=6),
    }

    with open(demo, "r", encoding="utf-8") as f:
        text = f.read()

    for key, val in new_values.items():
        text = replace_key_line(text, key, val)

    backup = demo + ".bak"
    os.replace(demo, backup)
    with open(demo, "w", encoding="utf-8") as f:
        f.write(text)

    print(f"\n已写入 {demo}（备份在 {backup}）:")
    for key, val in new_values.items():
        print(f"  {key}: {val}")
    print("\n（R_gimbal2imubody 不变；内参请人工核对是否需要更新。）")


if __name__ == "__main__":
    main()
