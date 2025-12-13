#!/usr/bin/env python3
"""
联合标定脚本：
  - 读取自动标定采样生成的 joint_calib_samples.csv
  - 同时求解 camera(world)->base 与 flange(Link_6)->cube_center 的 6D 外参
  - 输出 world_robot_extrinsic.yaml，并在其中附加 tool_extrinsic（flange->cube_center）

保持与现有自动标定流程兼容：CSV 格式、frame 命名不变。
"""

import argparse
import math
import os
import sys
from typing import List, Tuple

import numpy as np
import yaml


def quaternion_to_matrix(q: np.ndarray) -> np.ndarray:
    """Quaternion (x,y,z,w) -> 4x4 homogeneous rotation matrix."""
    q = q.astype(float)
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    R = np.array(
        [
            [1 - 2 * (y ** 2 + z ** 2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x ** 2 + z ** 2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x ** 2 + y ** 2)],
        ]
    )
    T = np.eye(4)
    T[:3, :3] = R
    return T


def matrix_to_quaternion(T: np.ndarray) -> np.ndarray:
    """4x4 homogeneous matrix -> quaternion (x,y,z,w)."""
    R = T[:3, :3]
    qw = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    if qw < 1e-8:
        # 数值退化时回退对角线最大分量的公式
        diag = np.diag(R)
        idx = int(np.argmax(diag))
        q = np.zeros(4)
        q[idx] = math.sqrt(max(0.0, 1 + 2 * diag[idx] - R.trace())) / 2.0
        j = (idx + 1) % 3
        k = (idx + 2) % 3
        if q[idx] > 1e-8:
            q[j] = (R[j, idx] + R[idx, j]) / (4 * q[idx])
            q[k] = (R[k, idx] + R[idx, k]) / (4 * q[idx])
        return np.array([q[0], q[1], q[2], qw])

    qx = (R[2, 1] - R[1, 2]) / (4 * qw)
    qy = (R[0, 2] - R[2, 0]) / (4 * qw)
    qz = (R[1, 0] - R[0, 1]) / (4 * qw)
    return np.array([qx, qy, qz, qw])


def make_transform(t: np.ndarray, q: np.ndarray) -> np.ndarray:
    T = quaternion_to_matrix(q)
    T[:3, 3] = t
    return T


def transform_inverse(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def markley_average(quats: List[np.ndarray], weights: List[float]) -> np.ndarray:
    if len(quats) == 1:
        q = quats[0]
        return q / np.linalg.norm(q)

    M = np.zeros((4, 4))
    for q, w in zip(quats, weights):
        qn = q / np.linalg.norm(q)
        M += w * np.outer(qn, qn)

    eigen_values, eigen_vectors = np.linalg.eigh(M)
    q_max = eigen_vectors[:, np.argmax(eigen_values)]
    # eigenvectors 列顺序为 [x,y,z,w]
    return q_max / np.linalg.norm(q_max)


def average_transform(transforms: List[np.ndarray], weights: List[float]) -> np.ndarray:
    if not transforms:
        return np.eye(4)
    weights = [w if w > 0 else 1.0 for w in weights]
    t_stack = np.stack([T[:3, 3] for T in transforms], axis=0)
    weight_sum = float(np.sum(weights))
    t_mean = np.sum(t_stack * np.array(weights)[:, None], axis=0) / max(weight_sum, 1e-9)

    quats = [matrix_to_quaternion(T) for T in transforms]
    # 半球对齐：使用第一帧作参考
    ref = quats[0]
    aligned = []
    for q in quats:
        if np.dot(q, ref) < 0:
            aligned.append(-q)
        else:
            aligned.append(q)
    q_mean = markley_average(aligned, weights)

    T_mean = make_transform(t_mean, q_mean)
    return T_mean


def pose_angle_deg(q1: np.ndarray, q2: np.ndarray) -> float:
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)
    dot = abs(float(np.dot(q1, q2)))
    dot = max(min(dot, 1.0), -1.0)
    return math.degrees(2 * math.acos(dot))


def load_samples(csv_path: str) -> List[dict]:
    samples = []
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"找不到采样 CSV：{csv_path}")

    with open(csv_path, "r") as f:
        header = f.readline().strip().split(',')
        expected = [
            "stamp",
            "base_tx",
            "base_ty",
            "base_tz",
            "base_qx",
            "base_qy",
            "base_qz",
            "base_qw",
            "cube_tx",
            "cube_ty",
            "cube_tz",
            "cube_qx",
            "cube_qy",
            "cube_qz",
            "cube_qw",
            "n_obs",
            "inliers",
            "dropped_faces",
        ]
        if header[: len(expected)] != expected:
            raise ValueError(f"CSV 表头不匹配，期望 {expected}，实际 {header}")

        for line in f:
            if not line.strip():
                continue
            parts = line.strip().split(',')
            if len(parts) < len(expected):
                continue
            vals = [float(x) for x in parts[: len(expected)]]
            sample = {
                "stamp": vals[0],
                "base_t": np.array(vals[1:4]),
                "base_q": np.array(vals[4:8]),
                "cube_t": np.array(vals[8:11]),
                "cube_q": np.array(vals[11:15]),
                "n_obs": vals[15],
                "inliers": vals[16],
                "dropped_faces": vals[17],
            }
            samples.append(sample)

    if not samples:
        raise ValueError("CSV 中没有有效样本")
    return samples


def iterative_solve(samples: List[dict], init_cb: np.ndarray, init_ft: np.ndarray, max_iter: int = 10) -> Tuple[np.ndarray, np.ndarray]:
    T_cb = init_cb.copy()
    T_ft = init_ft.copy()

    for _ in range(max_iter):
        T_cb_prev = T_cb.copy()
        T_ft_prev = T_ft.copy()

        # 求解 camera->base
        cb_candidates = []
        weights = []
        T_ft_inv = transform_inverse(T_ft)
        for s in samples:
            T_ct = make_transform(s["cube_t"], s["cube_q"])
            T_bf = make_transform(s["base_t"], s["base_q"])
            candidate = T_ct @ T_ft_inv @ transform_inverse(T_bf)
            cb_candidates.append(candidate)
            weights.append(max(s.get("inliers", 1.0), 1.0))
        T_cb = average_transform(cb_candidates, weights)

        # 求解 flange->cube
        ft_candidates = []
        weights_ft = []
        T_cb_inv = transform_inverse(T_cb)
        for s in samples:
            T_ct = make_transform(s["cube_t"], s["cube_q"])
            T_bf = make_transform(s["base_t"], s["base_q"])
            candidate = transform_inverse(T_bf) @ T_cb_inv @ T_ct
            ft_candidates.append(candidate)
            weights_ft.append(max(s.get("inliers", 1.0), 1.0))
        T_ft = average_transform(ft_candidates, weights_ft)

        # 收敛性检测
        trans_delta = np.linalg.norm(T_ft[:3, 3] - T_ft_prev[:3, 3]) + np.linalg.norm(T_cb[:3, 3] - T_cb_prev[:3, 3])
        rot_delta = pose_angle_deg(matrix_to_quaternion(T_ft), matrix_to_quaternion(T_ft_prev)) + pose_angle_deg(
            matrix_to_quaternion(T_cb), matrix_to_quaternion(T_cb_prev)
        )
        if trans_delta < 1e-6 and rot_delta < 1e-4:
            break

    return T_cb, T_ft


def compute_residuals(samples: List[dict], T_cb: np.ndarray, T_ft: np.ndarray) -> Tuple[dict, List[dict]]:
    pos_err = []
    rot_err = []
    per_sample = []
    for s in samples:
        T_ct = make_transform(s["cube_t"], s["cube_q"])
        T_bf = make_transform(s["base_t"], s["base_q"])
        T_pred = T_cb @ T_bf @ T_ft
        dp = np.linalg.norm(T_pred[:3, 3] - T_ct[:3, 3])
        dq = pose_angle_deg(matrix_to_quaternion(T_pred), matrix_to_quaternion(T_ct))
        pos_err.append(dp)
        rot_err.append(dq)
        per_sample.append({"dp": dp, "dth": dq})

    def stats(arr):
        arr = np.asarray(arr)
        return {
            "mean": float(np.mean(arr)),
            "rmse": float(math.sqrt(np.mean(arr ** 2))),
            "max": float(np.max(arr)),
        }

    return {"pos": stats(pos_err), "rot": stats(rot_err)}, per_sample


def save_yaml(output_path: str, world_frame: str, base_frame: str, T_cb: np.ndarray, flange_frame: str, cube_frame: str, T_ft: np.ndarray):
    data = {
        "world_frame": world_frame,
        "robot_base_frame": base_frame,
        "translation": {
            "x": float(T_cb[0, 3]),
            "y": float(T_cb[1, 3]),
            "z": float(T_cb[2, 3]),
        },
        "rotation": {
            "x": float(matrix_to_quaternion(T_cb)[0]),
            "y": float(matrix_to_quaternion(T_cb)[1]),
            "z": float(matrix_to_quaternion(T_cb)[2]),
            "w": float(matrix_to_quaternion(T_cb)[3]),
        },
        "tool_extrinsic": {
            "parent_frame": flange_frame,
            "child_frame": cube_frame,
            "translation": {
                "x": float(T_ft[0, 3]),
                "y": float(T_ft[1, 3]),
                "z": float(T_ft[2, 3]),
            },
            "rotation": {
                "x": float(matrix_to_quaternion(T_ft)[0]),
                "y": float(matrix_to_quaternion(T_ft)[1]),
                "z": float(matrix_to_quaternion(T_ft)[2]),
                "w": float(matrix_to_quaternion(T_ft)[3]),
            },
        },
    }

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=False)


def run_joint_calibration(args):
    csv_path = os.path.abspath(args.csv)
    samples = load_samples(csv_path)
    print(f"读取样本 {len(samples)} 条：{csv_path}")

    # 初值
    T_cb_init = np.eye(4)
    if args.init_identity is False and os.path.isfile(args.output):
        try:
            with open(args.output, "r") as f:
                existing = yaml.safe_load(f)
            t = existing.get("translation", {})
            q = existing.get("rotation", {})
            if all(k in t for k in ("x", "y", "z")) and all(k in q for k in ("x", "y", "z", "w")):
                T_cb_init = make_transform(np.array([t["x"], t["y"], t["z"]]), np.array([q["x"], q["y"], q["z"], q["w"]]))
                print("使用已有 world->base 结果作为初值")
        except Exception:
            T_cb_init = np.eye(4)

    T_ft_init = np.eye(4)
    T_ft_init[:3, 3] = np.array([0.0, 0.0, 0.077])

    T_cb, T_ft = iterative_solve(samples, T_cb_init, T_ft_init, max_iter=args.max_iter)
    stats, per_sample = compute_residuals(samples, T_cb, T_ft)

    print("\n=== 联合标定结果 ===")
    print(f"camera/world -> base (frame={args.world_frame}) 平移: {T_cb[:3, 3]}")
    print(f"camera/world -> base 四元数(xyzw): {matrix_to_quaternion(T_cb)}")
    print(f"flange -> cube_center 平移: {T_ft[:3, 3]}")
    print(f"flange -> cube_center 四元数(xyzw): {matrix_to_quaternion(T_ft)}")
    print("残差：")
    print(f"  位置 RMSE={stats['pos']['rmse']:.4f} m, mean={stats['pos']['mean']:.4f} m, max={stats['pos']['max']:.4f} m")
    print(f"  姿态 RMSE={stats['rot']['rmse']:.3f} deg, mean={stats['rot']['mean']:.3f} deg, max={stats['rot']['max']:.3f} deg")

    save_yaml(args.output, args.world_frame or args.camera_frame, args.robot_base_frame, T_cb, args.flange_frame, args.cube_frame, T_ft)
    print(f"已写入外参 YAML：{args.output}")


def parse_args():
    parser = argparse.ArgumentParser(description="联合标定：求解 world/camera->base 与 flange->cube_center 外参")
    default_csv = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config", "joint_calib_samples.csv"))
    default_out = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config", "world_robot_extrinsic.yaml"))
    parser.add_argument("--csv", default=default_csv, help="联合标定采样 CSV 路径")
    parser.add_argument("--output", default=default_out, help="输出 YAML 路径（含 tool_extrinsic）")
    parser.add_argument("--world-frame", dest="world_frame", default=None, help="世界/相机坐标系名，默认为 camera_frame")
    parser.add_argument("--camera-frame", dest="camera_frame", default="zed2i_left_camera_optical_frame", help="相机坐标系名")
    parser.add_argument("--robot-base-frame", dest="robot_base_frame", default="Link_0", help="机器人基座坐标系名")
    parser.add_argument("--flange-frame", dest="flange_frame", default="Link_6", help="法兰坐标系名")
    parser.add_argument("--cube-frame", dest="cube_frame", default="cube_center", help="立方体中心坐标系名")
    parser.add_argument("--max-iter", dest="max_iter", type=int, default=10, help="迭代次数上限")
    parser.add_argument("--init-identity", action="store_true", help="忽略已有 YAML，使用单位初值")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    if args.world_frame is None:
        args.world_frame = args.camera_frame
    run_joint_calibration(args)