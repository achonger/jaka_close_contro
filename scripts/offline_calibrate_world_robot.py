#!/usr/bin/env python3
"""
离线求解 world -> robot base 外参（按机器人独立输出）
输入：record_world_robot_dataset.py 生成的 CSV
输出：world_robot_extrinsic_offline_<robot>.yaml
"""
import argparse
import csv
import os
from typing import List

import yaml
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", help="录制的 CSV 数据集")
    parser.add_argument("--robot-id", type=int, default=None)
    parser.add_argument("--robot-name", default=None)
    parser.add_argument("--tool-offset", nargs=3, type=float, default=[0.0, 0.0, 0.077], help="tool 到 cube_center 的平移 (m)")
    parser.add_argument("--output-dir", default=None)
    return parser.parse_args()


def quat_to_matrix(qw, qx, qy, qz):
    q = np.array([qw, qx, qy, qz], dtype=float)
    q = q / np.linalg.norm(q)
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ])


def tf_from_list(vals: List[float]):
    tx, ty, tz, qx, qy, qz, qw = vals
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(qw, qx, qy, qz)
    T[:3, 3] = [tx, ty, tz]
    return T


def invert_tf(T: np.ndarray):
    R = T[:3, :3]
    t = T[:3, 3]
    Tinv = np.eye(4)
    Tinv[:3, :3] = R.T
    Tinv[:3, 3] = -R.T @ t
    return Tinv


def average_quaternion(quats: List[np.ndarray]):
    # Markley method
    A = np.zeros((4, 4))
    for q in quats:
        qn = q / np.linalg.norm(q)
        A += np.outer(qn, qn)
    eigvals, eigvecs = np.linalg.eigh(A)
    q_opt = eigvecs[:, np.argmax(eigvals)]
    if q_opt[0] < 0:
        q_opt = -q_opt
    return q_opt / np.linalg.norm(q_opt)


def main():
    args = parse_args()
    dataset_path = args.dataset
    out_dir = args.output_dir or os.path.dirname(os.path.abspath(dataset_path))
    os.makedirs(out_dir, exist_ok=True)

    samples = []
    with open(dataset_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            rid = int(row.get("robot_id", -1)) if row.get("robot_id") else None
            rname = row.get("robot_name", "")
            if args.robot_id is not None and rid != args.robot_id:
                continue
            if args.robot_name is not None and rname != args.robot_name:
                continue
            wc_vals = [float(row[k]) for k in [
                "world_cube_tx", "world_cube_ty", "world_cube_tz",
                "world_cube_qx", "world_cube_qy", "world_cube_qz", "world_cube_qw"
            ]]
            bt_vals = [float(row[k]) for k in [
                "base_tool_tx", "base_tool_ty", "base_tool_tz",
                "base_tool_qx", "base_tool_qy", "base_tool_qz", "base_tool_qw"
            ]]
            samples.append((wc_vals, bt_vals, rname or f"jaka{rid}"))

    if not samples:
        raise RuntimeError("No samples found for selected robot")

    tool_offset = np.eye(4)
    tool_offset[:3, 3] = np.array(args.tool_offset)

    T_world_base_list = []
    for wc_vals, bt_vals, _ in samples:
        T_world_cube = tf_from_list(wc_vals)
        T_base_tool = tf_from_list(bt_vals)
        T_base_cube = T_base_tool @ tool_offset
        T_world_base = T_world_cube @ invert_tf(T_base_cube)
        T_world_base_list.append(T_world_base)

    translations = [T[:3, 3] for T in T_world_base_list]
    quats = []
    for T in T_world_base_list:
        R = T[:3, :3]
        qw = np.sqrt(max(0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
        qx = (R[2, 1] - R[1, 2]) / (4 * qw)
        qy = (R[0, 2] - R[2, 0]) / (4 * qw)
        qz = (R[1, 0] - R[0, 1]) / (4 * qw)
        quats.append(np.array([qw, qx, qy, qz]))

    t_mean = np.mean(np.stack(translations, axis=0), axis=0)
    q_mean = average_quaternion(quats)

    robot_name = args.robot_name or samples[0][2]
    out_path = os.path.join(out_dir, f"world_robot_extrinsic_offline_{robot_name}.yaml")

    data = {
        "robot": robot_name,
        "translation": {
            "x": float(t_mean[0]),
            "y": float(t_mean[1]),
            "z": float(t_mean[2]),
        },
        "quaternion": {
            "w": float(q_mean[0]),
            "x": float(q_mean[1]),
            "y": float(q_mean[2]),
            "z": float(q_mean[3]),
        },
        "samples": len(samples),
    }

    with open(out_path, "w") as f:
        yaml.safe_dump(data, f)

    print(f"Saved offline calibration to {out_path} with {len(samples)} samples")


if __name__ == "__main__":
    main()
