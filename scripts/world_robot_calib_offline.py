#!/usr/bin/env python3
"""
Placeholder offline solver for world-robot calibration datasets.

Usage:
  rosrun jaka_close_contro world_robot_calib_offline.py \
      --input path/to/world_robot_calib_dataset.csv \
      --output-world-robot config/world_robot_extrinsic.yaml \
      --output-tool-offset config/tool_offset_current.yaml

This script currently only loads the dataset and prints a TODO message.
Implement Tsai/Park-Martin style hand-eye/world-robot solvers here.
"""

import argparse
import csv
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description="Offline world-robot calibration placeholder")
    parser.add_argument("--input", required=True, help="Input dataset CSV produced by world_robot_calib_recorder_node")
    parser.add_argument("--output-world-robot", default="world_robot_extrinsic.yaml",
                        help="Path to write world->robot_base extrinsic")
    parser.add_argument("--output-tool-offset", default="tool_offset_current.yaml",
                        help="Path to write Link_6->cube_center extrinsic")
    return parser.parse_args()


def load_dataset(path: Path):
    with path.open("r") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    return rows


def main():
    args = parse_args()
    dataset_path = Path(args.input)
    if not dataset_path.exists():
        raise FileNotFoundError(f"Dataset not found: {dataset_path}")

    rows = load_dataset(dataset_path)
    print(f"Loaded {len(rows)} samples from {dataset_path}")
    print("TODO: implement offline optimization to produce:")
    print(f"  World->robot YAML: {args.output_world_robot}")
    print(f"  Tool offset YAML:  {args.output_tool_offset}")


if __name__ == "__main__":
    main()
