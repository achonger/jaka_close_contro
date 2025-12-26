#!/usr/bin/env python3
"""
离线 world_base 标定脚本（基于采集的数据集做平均求解）

用法示例：
  python3 scripts/world_robot_calib_offline.py --mode world_base --robot_name jaka1 \\
      --source_csv config/world_robot_calib_dataset_xxx_jaka1.csv \\
      --output_yaml config/world_robot_extrinsic_offline_jaka1.yaml

实现假设：
- 数据集中已经包含 world_cube_* 与 base_tool_* 列（由自动录制节点生成）
- 工装棱方体坐标系与 tool_frame 重合，使用 world_cube * inv(base_tool) 直接估计 world->base
- 对所有样本做平均（平移算术均值 + 四元数 Markley 平均）
"""

import argparse
import csv
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import yaml

def parse_args():
    parser = argparse.ArgumentParser(description="Offline world-base calibration (averaged estimator)")
    parser.add_argument("--mode", default="world_base", choices=["world_base"], help="标定模式，当前仅支持 world_base")
    parser.add_argument("--robot_name", required=True, help="jaka1/jaka2/jaka3/jaka4")
    parser.add_argument("--source_csv", required=True, help="录制生成的数据集 CSV 路径")
    parser.add_argument(
        "--output_yaml",
        default=None,
        help="输出 world->base YAML 路径（默认 config/world_robot_extrinsic_offline_<robot>.yaml）",
    )
    parser.add_argument(
        "--output_dir",
        default=None,
        help="可选，覆盖输出目录，文件名仍为 world_robot_extrinsic_offline_<robot>.yaml",
    )
    parser.add_argument("--tool_translation_init_m", nargs=3, type=float, default=[0.0, 0.0, 0.064],
                        help="工具外参平移初值（m）")
    parser.add_argument("--tool_quat_init_xyzw", nargs=4, type=float, default=[0.0, 0.0, 0.0, 1.0],
                        help="工具外参四元数初值（x y z w）")
    parser.add_argument("--opt_tool_extrinsic", type=int, choices=[0, 1], default=0,
                        help="是否联合优化工具外参")
    parser.add_argument("--weight_mode", default="robust",
                        choices=["robust", "auto", "inliers", "faces", "none"],
                        help="残差加权模式（自定义逻辑占位）")
    return parser.parse_args()


def load_dataset(path: Path):
    with path.open("r") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    return rows


def quat_to_rot(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    R = np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ]
    )
    return R


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    trace = np.trace(R)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    else:
        idx = int(np.argmax(np.diag(R)))
        if idx == 0:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif idx == 1:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    q = np.array([x, y, z, w], dtype=float)
    return q / np.linalg.norm(q)


def invert_transform(t: np.ndarray, R: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    R_inv = R.T
    t_inv = -R_inv.dot(t)
    return t_inv, R_inv


def compose_transform(t1: np.ndarray, R1: np.ndarray, t2: np.ndarray, R2: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    R = R1.dot(R2)
    t = t1 + R1.dot(t2)
    return t, R


def average_quaternions(quats: List[np.ndarray]) -> np.ndarray:
    if not quats:
        raise ValueError("No quaternions to average")
    M = np.zeros((4, 4))
    for q in quats:
        qn = q / np.linalg.norm(q)
        M += np.outer(qn, qn)
    eigenvalues, eigenvectors = np.linalg.eigh(M)
    q_avg = eigenvectors[:, np.argmax(eigenvalues)]
    return q_avg / np.linalg.norm(q_avg)


def average_transforms(transforms: List[Tuple[np.ndarray, np.ndarray]]) -> Tuple[np.ndarray, np.ndarray]:
    translations = [t for t, _ in transforms]
    rotations = [R for _, R in transforms]
    t_mean = np.mean(translations, axis=0)
    quat_list = [rot_to_quat(R) for R in rotations]
    q_mean = average_quaternions(quat_list)
    R_mean = quat_to_rot(q_mean)
    return t_mean, R_mean


def parse_transform(row: Dict[str, str], prefix: str) -> Tuple[np.ndarray, np.ndarray]:
    keys = [f"{prefix}_tx", f"{prefix}_ty", f"{prefix}_tz", f"{prefix}_qx", f"{prefix}_qy", f"{prefix}_qz", f"{prefix}_qw"]
    if any(k not in row or row[k] == "" for k in keys):
        raise ValueError(f"Missing columns for prefix {prefix}")
    t = np.array([float(row[keys[0]]), float(row[keys[1]]), float(row[keys[2]])], dtype=float)
    q = np.array([float(row[keys[3]]), float(row[keys[4]]), float(row[keys[5]]), float(row[keys[6]])], dtype=float)
    return t, q


def main():
    args = parse_args()
    dataset_path = Path(args.source_csv)
    if not dataset_path.exists():
        raise FileNotFoundError(f"Dataset not found: {dataset_path}")

    rows = load_dataset(dataset_path)
    if args.output_yaml is None:
        base_dir = Path(args.output_dir) if args.output_dir else Path(__file__).resolve().parent.parent / "config"
        args.output_yaml = str(base_dir / f"world_robot_extrinsic_offline_{args.robot_name}.yaml")

    transforms_world_base: List[Tuple[np.ndarray, np.ndarray]] = []
    robot_id = None
    base_frame = "Link_0"
    world_frame = "world"
    for row in rows:
        try:
            t_wc, q_wc = parse_transform(row, "world_cube")
            t_bt, q_bt = parse_transform(row, "base_tool")
        except ValueError:
            continue

        T_wc_R = quat_to_rot(q_wc)
        T_bt_R = quat_to_rot(q_bt)
        t_bt_inv, R_bt_inv = invert_transform(t_bt, T_bt_R)
        t_wb, R_wb = compose_transform(t_wc, T_wc_R, t_bt_inv, R_bt_inv)
        transforms_world_base.append((t_wb, R_wb))

        if robot_id is None and "robot_id" in row:
            try:
                robot_id = int(float(row["robot_id"]))
            except Exception:
                robot_id = None
        if "base_frame" in row and row["base_frame"]:
            base_frame = row["base_frame"]
        if "world_frame" in row and row["world_frame"]:
            world_frame = row["world_frame"]

    if not transforms_world_base:
        raise ValueError("No valid samples with world_cube/base_tool columns were found in dataset")
    if robot_id is None:
        robot_id = 0

    t_mean, R_mean = average_transforms(transforms_world_base)
    q_mean = rot_to_quat(R_mean)

    output = {
        "world_robot_extrinsic_offline": {
            "robot_name": args.robot_name,
            "robot_id": robot_id,
            "world_frame": world_frame,
            "base_frame": base_frame,
            "method": "avg(world_cube * inv(base_tool)) 假设 cube_frame 与 tool_frame 重合",
            "dataset": str(dataset_path),
            "samples_used": len(transforms_world_base),
            "translation": [float(v) for v in t_mean.tolist()],
            "quat_xyzw": [float(v) for v in q_mean.tolist()],
        }
    }

    out_path = Path(args.output_yaml)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w") as f:
        yaml.safe_dump(output, f, sort_keys=False, allow_unicode=True)

    print(f"[offline] mode={args.mode}, robot={args.robot_name}")
    print(f"[offline] loaded {len(rows)} samples from {dataset_path}")
    print(f"[offline] used {len(transforms_world_base)} samples with world_cube/base_tool")
    print(f"[offline] saved estimate to {out_path}")


if __name__ == "__main__":
    main()
