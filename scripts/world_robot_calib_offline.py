#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线 world_base 标定脚本（从录制数据集估计 X = T_world_base）

推荐模型（与你们的录制/融合流程一致）：
    A(i) = T_world_cube(i)
    B(i) = T_base_tool(i)          (base -> Link_6 / tool_frame)
    Y    = T_tool_cube             (Link_6 -> cube_center，工具外参，通常约 [0,0,0.064])

    A(i) ≈ X * B(i) * Y
=>  X_i = A(i) * inv( B(i) * Y )

本脚本做法：
- 对每条样本求闭式解 X_i
- 对所有 X_i 求平均：
  - 平移：算术均值
  - 旋转：四元数 Markley 平均
- 计算残差统计：比较 A_hat = X_mean * B * Y 与 A 的差
- （可选）开放工具外参 Y 的联合优化：在 Y_init 附近迭代更新 Y，并做“邻域约束”（clamp）

使用方式（两种任选其一）：
1) 直接改文件开头的 SOURCE_CSV / OUTPUT_YAML 等，然后运行：
   python3 scripts/world_robot_calib_offline.py

2) 命令行覆盖（可选）：
   python3 scripts/world_robot_calib_offline.py --robot_name jaka1 \
       --source_csv config/world_robot_calib_dataset_xxx_jaka1.csv \
       --output_yaml config/world_robot_extrinsic_offline_jaka1.yaml

输入 CSV 需要包含列（每行一个样本）：
- world_cube_tx ty tz qx qy qz qw
- base_tool_tx  ty tz qx qy qz qw
可选列（若存在会写入输出/用于权重）：
- robot_id, world_frame, base_frame, tool_frame, cube_frame, stamp, inliers, num_faces, pos_err_rms, ang_err_rms_deg
"""

import argparse
import csv
import math
from pathlib import Path
from typing import Dict, List, Tuple, Optional

import numpy as np
import yaml

# ==========================================================
# 0) 你只需要改这里（不想用命令行的话）
# ==========================================================
ROBOT_NAME = "jaka1"

# 输入数据集（自动录制节点输出的 CSV）
SOURCE_CSV = "config/world_robot_calib_dataset_jaka1_1767344301.csv"

# 输出 YAML（world -> base）
OUTPUT_YAML = "config/world_robot_extrinsic_offline_jaka1.yaml"

# 工具外参：Y = T_tool_cube（Link_6 -> cube_center）
# 你手动输入的“名义值/初值”
TOOL_TRANSLATION_M = np.array([0.0, 0.0, 0.064], dtype=float)
TOOL_QUAT_XYZW = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

# 是否开放工具外参联合优化（0 关 / 1 开）
OPT_TOOL_EXTRINSIC = 1

# 工具外参“邻域约束”：联合优化后的 Y 必须在初值附近
# - 平移：||Δt|| <= PRIOR_MAX_DELTA_TRANS_M
# - 旋转：Δθ <= PRIOR_MAX_DELTA_ROT_DEG
PRIOR_MAX_DELTA_TRANS_M = 0.01   # 例如 10mm
PRIOR_MAX_DELTA_ROT_DEG = 10.0   # 例如 10deg

# 联合优化迭代次数（一般 5~20 足够）
OPT_ITERS = 10

# 联合优化步长混合（0~1）：越小越稳（推荐 0.3~0.7）
OPT_ALPHA = 0.5

# 是否对样本做简单加权（可选）
# - none: 全部权重=1
# - robust: 用 (inliers*num_faces)/(pos_err_rms+1e-3)/(ang_err_rms_deg+1) 作为权重（若列存在）
WEIGHT_MODE = "robust"  # "none" 或 "robust"

# ==========================================================
# 1) 基础工具：四元数/旋转/SE3
# ==========================================================

_EPS = 1e-12

def quat_normalize_xyzw(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float).reshape(4)
    n = float(np.linalg.norm(q))
    if n < _EPS:
        raise ValueError("Quaternion norm is zero")
    return q / n

def quat_canonicalize_xyzw(q: np.ndarray) -> np.ndarray:
    q = quat_normalize_xyzw(q)
    # 统一符号：qw >= 0
    if q[3] < 0.0:
        q = -q
    return q

def quat_to_rot(q_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = quat_canonicalize_xyzw(q_xyzw)
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    R = np.array(
        [
            [1.0 - 2.0*(yy + zz), 2.0*(xy - wz),       2.0*(xz + wy)],
            [2.0*(xy + wz),       1.0 - 2.0*(xx + zz), 2.0*(yz - wx)],
            [2.0*(xz - wy),       2.0*(yz + wx),       1.0 - 2.0*(xx + yy)],
        ],
        dtype=float
    )
    return R

def rot_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
    R = np.asarray(R, dtype=float).reshape(3, 3)
    tr = float(np.trace(R))
    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            w = (R[2, 1] - R[1, 2]) / S
            x = 0.25 * S
            y = (R[0, 1] + R[1, 0]) / S
            z = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            w = (R[0, 2] - R[2, 0]) / S
            x = (R[0, 1] + R[1, 0]) / S
            y = 0.25 * S
            z = (R[1, 2] + R[2, 1]) / S
        else:
            S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            w = (R[1, 0] - R[0, 1]) / S
            x = (R[0, 2] + R[2, 0]) / S
            y = (R[1, 2] + R[2, 1]) / S
            z = 0.25 * S
    return quat_canonicalize_xyzw(np.array([x, y, z, w], dtype=float))

def T_from_tq(t: np.ndarray, q_xyzw: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=float)
    T[:3, :3] = quat_to_rot(q_xyzw)
    T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T

def inv_T(T: np.ndarray) -> np.ndarray:
    T = np.asarray(T, dtype=float).reshape(4, 4)
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4, dtype=float)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti

def angle_deg_from_R(R_err: np.ndarray) -> float:
    # angle = acos((trace(R)-1)/2)
    tr = float(np.trace(R_err))
    c = (tr - 1.0) * 0.5
    c = max(-1.0, min(1.0, c))
    return float(math.degrees(math.acos(c)))

def rot_to_axis_angle(R: np.ndarray) -> Tuple[np.ndarray, float]:
    """返回 (axis(3,), angle_rad)，angle∈[0,pi]"""
    R = np.asarray(R, dtype=float).reshape(3, 3)
    tr = float(np.trace(R))
    c = (tr - 1.0) * 0.5
    c = max(-1.0, min(1.0, c))
    angle = float(math.acos(c))
    if angle < 1e-12:
        return np.array([1.0, 0.0, 0.0], dtype=float), 0.0
    s = 2.0 * math.sin(angle)
    axis = np.array(
        [(R[2, 1] - R[1, 2]) / s,
         (R[0, 2] - R[2, 0]) / s,
         (R[1, 0] - R[0, 1]) / s],
        dtype=float
    )
    n = float(np.linalg.norm(axis))
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0], dtype=float), angle
    return axis / n, angle

def axis_angle_to_rot(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float).reshape(3)
    n = float(np.linalg.norm(axis))
    if n < 1e-12 or abs(angle_rad) < 1e-12:
        return np.eye(3, dtype=float)
    a = axis / n
    x, y, z = float(a[0]), float(a[1]), float(a[2])
    K = np.array([[0.0, -z,   y],
                  [z,   0.0, -x],
                  [-y,  x,   0.0]], dtype=float)
    I = np.eye(3, dtype=float)
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    R = I + s * K + (1.0 - c) * (K @ K)
    return R

def clamp_transform_near_prior(T: np.ndarray, T0: np.ndarray,
                              max_trans_m: float, max_rot_deg: float) -> np.ndarray:
    """
    将 T 投影到 T0 附近的邻域：
    - ΔT = inv(T0) * T
    - clamp ||Δt|| <= max_trans_m
    - clamp Δθ <= max_rot_deg
    """
    dT = inv_T(T0) @ T
    dt = dT[:3, 3].copy()
    dR = dT[:3, :3].copy()

    # clamp translation
    dt_norm = float(np.linalg.norm(dt))
    if np.isfinite(dt_norm) and dt_norm > max_trans_m > 0:
        dt = dt * (max_trans_m / dt_norm)

    # clamp rotation
    axis, ang = rot_to_axis_angle(dR)
    max_ang = float(math.radians(max_rot_deg))
    if np.isfinite(ang) and ang > max_ang > 0:
        dR = axis_angle_to_rot(axis, max_ang)

    dT_clamped = np.eye(4, dtype=float)
    dT_clamped[:3, :3] = dR
    dT_clamped[:3, 3] = dt
    return T0 @ dT_clamped

# ==========================================================
# 2) 解析 CSV
# ==========================================================

def _getf(row: Dict[str, str], key: str, default: float = float("nan")) -> float:
    v = row.get(key, "")
    if v is None or v == "":
        return default
    try:
        return float(v)
    except ValueError:
        return default

def parse_transform(row: Dict[str, str], prefix: str) -> np.ndarray:
    keys = [f"{prefix}_tx", f"{prefix}_ty", f"{prefix}_tz",
            f"{prefix}_qx", f"{prefix}_qy", f"{prefix}_qz", f"{prefix}_qw"]
    if any(k not in row or row[k] == "" for k in keys):
        raise ValueError(f"Missing columns for prefix {prefix}")
    t = np.array([float(row[keys[0]]), float(row[keys[1]]), float(row[keys[2]])], dtype=float)
    q = np.array([float(row[keys[3]]), float(row[keys[4]]), float(row[keys[5]]), float(row[keys[6]])], dtype=float)
    return T_from_tq(t, q)

def sample_weight(row: Dict[str, str], mode: str) -> float:
    if mode == "none":
        return 1.0
    if mode != "robust":
        return 1.0

    inliers = _getf(row, "inliers", 1.0)
    num_faces = _getf(row, "num_faces", 1.0)
    pos_err_rms = _getf(row, "pos_err_rms", float("nan"))
    ang_err_rms_deg = _getf(row, "ang_err_rms_deg", float("nan"))

    w = float(max(1e-6, inliers) * max(1.0, num_faces))
    if np.isfinite(pos_err_rms) and pos_err_rms >= 0:
        w = w / (pos_err_rms + 1e-3)
    if np.isfinite(ang_err_rms_deg) and ang_err_rms_deg >= 0:
        w = w / (ang_err_rms_deg + 1.0)
    return float(max(1e-6, w))

# ==========================================================
# 3) 平均（平移均值 + Markley 四元数平均）
# ==========================================================

def average_quaternions_markley(quats: List[np.ndarray], weights: Optional[np.ndarray] = None) -> np.ndarray:
    if not quats:
        raise ValueError("No quaternions to average")
    if weights is None:
        weights = np.ones(len(quats), dtype=float)
    w = np.asarray(weights, dtype=float).reshape(-1)
    w = np.maximum(w, 0.0)
    if float(np.sum(w)) < _EPS:
        w[:] = 1.0
    w = w / float(np.sum(w))

    M = np.zeros((4, 4), dtype=float)
    for qi, wi in zip(quats, w):
        qn = quat_canonicalize_xyzw(qi)
        M += wi * np.outer(qn, qn)

    eigvals, eigvecs = np.linalg.eigh(M)
    q_avg = eigvecs[:, int(np.argmax(eigvals))]
    return quat_canonicalize_xyzw(q_avg)

def average_transforms(T_list: List[np.ndarray], weights: np.ndarray) -> np.ndarray:
    w = np.asarray(weights, dtype=float).reshape(-1)
    w = np.maximum(w, 0.0)
    if float(np.sum(w)) < _EPS:
        w[:] = 1.0
    w = w / float(np.sum(w))

    t_stack = np.stack([T[:3, 3] for T in T_list], axis=0)
    t_mean = np.sum(t_stack * w[:, None], axis=0)

    quat_list = [rot_to_quat_xyzw(T[:3, :3]) for T in T_list]
    q_mean = average_quaternions_markley(quat_list, weights=w)
    R_mean = quat_to_rot(q_mean)

    Tm = np.eye(4, dtype=float)
    Tm[:3, :3] = R_mean
    Tm[:3, 3] = t_mean
    return Tm

# ==========================================================
# 4) 统计
# ==========================================================

def stats_1d(x: np.ndarray) -> Dict[str, float]:
    x = np.asarray(x, dtype=float).reshape(-1)
    return dict(
        mean=float(np.mean(x)),
        std=float(np.std(x)),
        min=float(np.min(x)),
        max=float(np.max(x)),
        rmse=float(np.sqrt(np.mean(x * x))),
    )

def compute_residual_stats(X: np.ndarray, rows: List[Dict[str, str]], Y: np.ndarray) -> Dict[str, Dict[str, float]]:
    pos_err = []
    ang_err = []
    for row in rows:
        try:
            A = parse_transform(row, "world_cube")
            B = parse_transform(row, "base_tool")
        except ValueError:
            continue
        A_hat = X @ (B @ Y)
        pos_err.append(float(np.linalg.norm(A_hat[:3, 3] - A[:3, 3])))
        ang_err.append(angle_deg_from_R(A_hat[:3, :3].T @ A[:3, :3]))

    if len(pos_err) == 0:
        raise ValueError("No valid samples for residual stats")
    return dict(
        pos_err_m=stats_1d(np.array(pos_err, dtype=float)),
        ang_err_deg=stats_1d(np.array(ang_err, dtype=float)),
        num_used=int(len(pos_err)),
    )

# ==========================================================
# 5) 参数与主流程
# ==========================================================

def parse_args():
    p = argparse.ArgumentParser(description="Offline world-base calibration (closed-form per-sample + averaging)")
    p.add_argument("--robot_name", default=ROBOT_NAME, help="jaka1/jaka2/jaka3/jaka4（可选，默认用文件开头常量）")
    p.add_argument("--source_csv", default=SOURCE_CSV, help="输入数据集 CSV（可选，默认用文件开头常量）")
    p.add_argument("--output_yaml", default=OUTPUT_YAML, help="输出 YAML（可选，默认用文件开头常量）")
    p.add_argument("--weight_mode", default=WEIGHT_MODE, choices=["none", "robust"], help="样本权重模式")
    p.add_argument("--opt_tool_extrinsic", type=int, default=OPT_TOOL_EXTRINSIC, choices=[0, 1], help="是否联合优化工具外参")
    return p.parse_args()

def main():
    args = parse_args()

    dataset_path = Path(args.source_csv)
    if not dataset_path.is_absolute():
        dataset_path = (Path.cwd() / dataset_path).resolve()
    if not dataset_path.exists():
        raise FileNotFoundError(f"Dataset not found: {dataset_path}")

    out_path = Path(args.output_yaml)
    if not out_path.is_absolute():
        out_path = (Path.cwd() / out_path).resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # 读取 CSV
    with dataset_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        raise ValueError(f"Empty dataset: {dataset_path}")

    # 预解析所有样本的 A, B, w，避免重复 parse
    A_list: List[np.ndarray] = []
    B_list: List[np.ndarray] = []
    w_list: List[float] = []

    world_frame = "world"
    base_frame = "Link_0"
    tool_frame = "Link_6"
    cube_frame = "cube_center"
    robot_id = None

    for row in rows:
        try:
            A = parse_transform(row, "world_cube")
            B = parse_transform(row, "base_tool")
        except ValueError:
            continue

        A_list.append(A)
        B_list.append(B)
        w_list.append(sample_weight(row, args.weight_mode))

        if "world_frame" in row and row["world_frame"]:
            world_frame = row["world_frame"]
        if "base_frame" in row and row["base_frame"]:
            base_frame = row["base_frame"]
        if "tool_frame" in row and row["tool_frame"]:
            tool_frame = row["tool_frame"]
        if "cube_frame" in row and row["cube_frame"]:
            cube_frame = row["cube_frame"]

        if robot_id is None and "robot_id" in row and row["robot_id"] != "":
            try:
                robot_id = int(float(row["robot_id"]))
            except Exception:
                robot_id = None

    if len(A_list) == 0:
        raise ValueError("No valid samples with world_cube/base_tool columns were found in dataset")

    weights = np.array(w_list, dtype=float)

    # 初值工具外参 Y0
    Y0 = T_from_tq(TOOL_TRANSLATION_M, TOOL_QUAT_XYZW)
    Y = Y0.copy()

    # =============== 核心：闭式 X +（可选）联合优化 Y ===============
    # 交替更新：
    #   X <- avg_i ( A_i * inv(B_i * Y) )
    #   Y <- avg_i ( inv(B_i) * inv(X) * A_i )   然后 clamp 到 Y0 邻域，并做混合
    X_mean = np.eye(4, dtype=float)

    iters = int(OPT_ITERS) if args.opt_tool_extrinsic == 1 else 1
    for k in range(iters):
        # 1) 更新 X
        X_list: List[np.ndarray] = []
        for A, B in zip(A_list, B_list):
            Xi = A @ inv_T(B @ Y)
            X_list.append(Xi)
        X_new = average_transforms(X_list, weights)

        if args.opt_tool_extrinsic == 0:
            X_mean = X_new
            break

        # 2) 更新 Y（由当前 X_new 推回每个样本的 Y_i）
        Y_list: List[np.ndarray] = []
        Xinv = inv_T(X_new)
        for A, B in zip(A_list, B_list):
            Yi = inv_T(B) @ (Xinv @ A)
            Y_list.append(Yi)
        Y_data = average_transforms(Y_list, weights)

        # 3) 邻域约束：投影到 Y0 附近
        Y_proj = clamp_transform_near_prior(
            T=Y_data,
            T0=Y0,
            max_trans_m=float(PRIOR_MAX_DELTA_TRANS_M),
            max_rot_deg=float(PRIOR_MAX_DELTA_ROT_DEG),
        )

        # 4) 混合更新（避免震荡）
        alpha = float(OPT_ALPHA)
        alpha = max(0.0, min(1.0, alpha))
        # 用“在李群上插值”更严谨，但这里用小步近似：对 ΔT 做插值
        dT = inv_T(Y) @ Y_proj
        dt = dT[:3, 3]
        dR = dT[:3, :3]
        axis, ang = rot_to_axis_angle(dR)
        dR_blend = axis_angle_to_rot(axis, ang * alpha)
        dt_blend = dt * alpha
        dT_blend = np.eye(4, dtype=float)
        dT_blend[:3, :3] = dR_blend
        dT_blend[:3, 3] = dt_blend
        Y = Y @ dT_blend

        X_mean = X_new

    # 残差统计（A_hat = X_mean * B * Y vs A）
    # 注意：这里用“原始 rows”再 parse 一遍也可以；为了保持一致，这里用 rows（可跳过无效行）
    res_stats = compute_residual_stats(X_mean, rows, Y)

    # 计算工具外参偏移量（相对初值）
    dY = inv_T(Y0) @ Y
    dY_trans_norm = float(np.linalg.norm(dY[:3, 3]))
    dY_rot_deg = angle_deg_from_R(dY[:3, :3])

    # 输出
    q_xyzw = rot_to_quat_xyzw(X_mean[:3, :3])
    t_xyz = X_mean[:3, 3]

    tool_q_init = quat_canonicalize_xyzw(TOOL_QUAT_XYZW)
    tool_q_final = rot_to_quat_xyzw(Y[:3, :3])
    tool_t_final = Y[:3, 3].astype(float)

    payload = {
        "mode": "world_base",
        "extrinsic": {
            "parent": world_frame,
            "child": base_frame,
            "robot_name": args.robot_name,
            "robot_id": int(robot_id) if robot_id is not None else None,
            "method": "Xi = world_cube * inv(base_tool * tool_to_cube), then avg(t) + Markley avg(q)",
            "source_csv": str(dataset_path),
            "samples_total": int(len(rows)),
            "samples_used": int(res_stats["num_used"]),
            "translation": [float(t_xyz[0]), float(t_xyz[1]), float(t_xyz[2])],
            "rotation_xyzw": [float(q_xyzw[0]), float(q_xyzw[1]), float(q_xyzw[2]), float(q_xyzw[3])],
        },
        "tool_extrinsic_tool_to_cube": {
            "tool_frame": tool_frame,
            "cube_frame": cube_frame,
            "opt_enabled": int(args.opt_tool_extrinsic),
            "translation_init_m": [float(TOOL_TRANSLATION_M[0]), float(TOOL_TRANSLATION_M[1]), float(TOOL_TRANSLATION_M[2])],
            "rotation_init_xyzw": [float(tool_q_init[0]), float(tool_q_init[1]), float(tool_q_init[2]), float(tool_q_init[3])],
            "translation_final_m": [float(tool_t_final[0]), float(tool_t_final[1]), float(tool_t_final[2])],
            "rotation_final_xyzw": [float(tool_q_final[0]), float(tool_q_final[1]), float(tool_q_final[2]), float(tool_q_final[3])],
            "prior_max_delta_trans_m": float(PRIOR_MAX_DELTA_TRANS_M),
            "prior_max_delta_rot_deg": float(PRIOR_MAX_DELTA_ROT_DEG),
            "delta_trans_norm_m": float(dY_trans_norm),
            "delta_rot_norm_deg": float(dY_rot_deg),
            "opt_iters": int(OPT_ITERS if args.opt_tool_extrinsic == 1 else 0),
            "opt_alpha": float(OPT_ALPHA),
        },
        "residual_stats": {
            "pos_err_m": res_stats["pos_err_m"],
            "ang_err_deg": res_stats["ang_err_deg"],
        },
    }

    # 残差统计写入 YAML 顶部注释（不影响 YAML body）
    pe = res_stats["pos_err_m"]
    ae = res_stats["ang_err_deg"]
    header_lines = [
        "# Offline calibration result",
        "# CALIB_MODE: world_base",
        "# TARGET: X = T_world_base",
        "# MEAS: A = T_world_cube",
        "# model: A ≈ X * (B_base_tool) * (Y_tool_to_cube)",
        f"# source_csv: {dataset_path}",
        f"# num_samples_total: {len(rows)}",
        f"# num_samples_used: {res_stats['num_used']}",
        f"# weight_mode: {args.weight_mode}",
        f"# opt_tool_extrinsic: {int(args.opt_tool_extrinsic)}",
        "# tool_extrinsic (Link_6 -> cube_center):",
        f"#   translation_init[m]: {[float(TOOL_TRANSLATION_M[0]), float(TOOL_TRANSLATION_M[1]), float(TOOL_TRANSLATION_M[2])]}",
        f"#   rotation_init(qx,qy,qz,qw): {[float(tool_q_init[0]), float(tool_q_init[1]), float(tool_q_init[2]), float(tool_q_init[3])]}",
        f"#   translation_final[m]: {[float(tool_t_final[0]), float(tool_t_final[1]), float(tool_t_final[2])]}",
        f"#   rotation_final(qx,qy,qz,qw): {[float(tool_q_final[0]), float(tool_q_final[1]), float(tool_q_final[2]), float(tool_q_final[3])]}",
        f"#   prior_max_delta_trans[m]: {float(PRIOR_MAX_DELTA_TRANS_M)}",
        f"#   prior_max_delta_rot[deg]: {float(PRIOR_MAX_DELTA_ROT_DEG)}",
        f"#   delta_trans_norm[m]: {float(dY_trans_norm)}",
        f"#   delta_rot_norm[deg]: {float(dY_rot_deg)}",
        "# residual_stats (prediction X*B*Y vs measurement A):",
        f"#   pos_err[m]: mean={pe['mean']:.6e}, std={pe['std']:.6e}, min={pe['min']:.6e}, max={pe['max']:.6e}, rmse={pe['rmse']:.6e}",
        f"#   ang_err[deg]: mean={ae['mean']:.6e}, std={ae['std']:.6e}, min={ae['min']:.6e}, max={ae['max']:.6e}, rmse={ae['rmse']:.6e}",
        "#",
    ]

    yaml_body = yaml.safe_dump(payload, sort_keys=False, allow_unicode=True)

    with out_path.open("w", encoding="utf-8") as f:
        f.write("\n".join(header_lines))
        f.write(yaml_body)

    print(f"[offline] robot={args.robot_name}, mode=world_base")
    print(f"[offline] dataset: {dataset_path}  (total={len(rows)}, used={res_stats['num_used']})")
    print(f"[offline] opt_tool_extrinsic={int(args.opt_tool_extrinsic)}  "
          f"prior_max_trans={PRIOR_MAX_DELTA_TRANS_M} m, prior_max_rot={PRIOR_MAX_DELTA_ROT_DEG} deg")
    print(f"[offline] tool_delta: trans_norm={dY_trans_norm:.6e} m, rot_norm={dY_rot_deg:.6e} deg")
    print(f"[offline] residual pos_err[m]: mean={pe['mean']:.6e}, std={pe['std']:.6e}, rmse={pe['rmse']:.6e}, "
          f"min={pe['min']:.6e}, max={pe['max']:.6e}")
    print(f"[offline] residual ang_err[deg]: mean={ae['mean']:.6e}, std={ae['std']:.6e}, rmse={ae['rmse']:.6e}, "
          f"min={ae['min']:.6e}, max={ae['max']:.6e}")
    print(f"[offline] saved: {out_path}")

if __name__ == "__main__":
    main()
