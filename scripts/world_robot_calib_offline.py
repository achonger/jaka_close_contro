#!/usr/bin/env python3
"""
离线 world_base 标定脚本（占位版）

用法示例：
  python3 scripts/world_robot_calib_offline.py --mode world_base --robot_name jaka1 \\
      --source_csv config/world_robot_calib_dataset_xxx_jaka1.csv \\
      --output_yaml config/world_robot_extrinsic_jaka1.yaml

当前脚本仅加载数据并打印 TODO。需自行实现优化求解：
  A_i = T_world_cube(i)
  B_i = T_base_tool(i)
  Y   = T_tool_cube (可选求)
  A_i ≈ X * B_i * Y, 其中 X = T_world_base
若数据集中没有 world_cube_*，可用 world_cam_* 与 cam_cube_* 组合得到 A_i。
"""

import argparse
import csv
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description="Offline world-base calibration (placeholder)")
    parser.add_argument("--mode", default="world_base", choices=["world_base"], help="标定模式，当前仅支持 world_base")
    parser.add_argument("--robot_name", required=True, help="jaka1/jaka2/jaka3/jaka4")
    parser.add_argument("--source_csv", required=True, help="录制生成的数据集 CSV 路径")
    parser.add_argument("--output_yaml", required=True, help="输出 world->base YAML 路径")
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


def main():
    args = parse_args()
    dataset_path = Path(args.source_csv)
    if not dataset_path.exists():
        raise FileNotFoundError(f"Dataset not found: {dataset_path}")

    rows = load_dataset(dataset_path)
    print(f"[offline] mode={args.mode}, robot={args.robot_name}")
    print(f"[offline] loaded {len(rows)} samples from {dataset_path}")
    print(f"[offline] TODO: 实现优化，输出 YAML: {args.output_yaml}")
    print(f"[offline] 初始工具外参平移={args.tool_translation_init_m}, quat={args.tool_quat_init_xyzw}, opt_tool={args.opt_tool_extrinsic}")


if __name__ == "__main__":
    main()
