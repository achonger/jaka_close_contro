#!/usr/bin/env python3
"""
记录世界-机器人标定数据集（按机器人、按姿态 CSV）
- 支持 robot_id/robot_name 选择
- 支持自定义姿态 CSV
- 自动写入 world_cam/world_cube/base_tool 等字段
"""
import argparse
import csv
import os
import time
from typing import Dict, Any

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
from jaka_sdk_driver.srv import JointMove, JointMoveRequest
import yaml


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-id", type=int, default=1)
    parser.add_argument("--robot-name", type=str, default=None)
    parser.add_argument("--pose-csv", required=True, help="关节姿态 CSV，逗号分隔 6 关节")
    parser.add_argument("--out-dir", default="data", help="数据集输出目录")
    parser.add_argument("--dataset-prefix", default=None, help="输出文件名前缀，默认包含机器人名")
    parser.add_argument("--speed-scale", type=float, default=0.15, help="速度比例 (<=0.15)")
    parser.add_argument("--robots-config", default="config/robots.yaml", help="机器人注册表 YAML")
    parser.add_argument("--world-frame", default="world")
    parser.add_argument("--camera-frame", default="zed2i_left_camera_optical_frame")
    parser.add_argument("--cube-frame-prefix", default="cube_center_")
    return parser.parse_args(rospy.myargv()[1:])


def load_robot_info(path: str, rid: int, override_name: str = None) -> Dict[str, Any]:
    default = {
        "id": rid,
        "name": override_name or f"jaka{rid}",
        "ns": f"/jaka{rid}",
        "base_frame": "Link_0",
        "tool_frame": "Link_6",
    }
    if not os.path.exists(path):
        rospy.logwarn("[Recorder] robots config %s missing, using defaults", path)
        return default
    try:
        data = yaml.safe_load(open(path))
        for r in data.get("robots", []):
            if int(r.get("id", -1)) == rid:
                return {
                    "id": rid,
                    "name": override_name or r.get("name", f"jaka{rid}"),
                    "ns": r.get("ns", f"/jaka{rid}"),
                    "base_frame": r.get("base_frame", "Link_0"),
                    "tool_frame": r.get("tool_frame", "Link_6"),
                }
    except Exception as exc:  # noqa: BLE001
        rospy.logwarn("[Recorder] Failed to load %s: %s", path, exc)
    return default


def read_poses(path: str):
    poses = []
    with open(path) as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            try:
                values = [float(v) for v in row]
                if len(values) < 6:
                    rospy.logwarn("[Recorder] Skip row (need 6 joints): %s", row)
                    continue
                poses.append(values[:6])
            except ValueError:
                rospy.logwarn("[Recorder] Skip invalid row: %s", row)
    return poses


def transform_to_list(tf_msg: TransformStamped):
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    return [t.x, t.y, t.z, q.x, q.y, q.z, q.w]


def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)


def main():
    args = parse_args()
    rospy.init_node("world_robot_dataset_recorder", anonymous=True)

    robot = load_robot_info(args.robots_config, args.robot_id, args.robot_name)
    robot_name = robot["name"]
    cube_frame = f"{args.cube_frame_prefix}{robot_name}"

    dataset_prefix = args.dataset_prefix or f"world_robot_calib_dataset_{robot_name}"
    ensure_dir(args.out_dir)
    dataset_path = os.path.join(
        args.out_dir,
        f"{dataset_prefix}_{int(time.time())}.csv",
    )

    poses = read_poses(args.pose_csv)
    if not poses:
        rospy.logerr("[Recorder] No poses loaded from %s", args.pose_csv)
        return

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    joint_move_service = f"{robot['ns']}/jaka_driver/joint_move"
    move_client = rospy.ServiceProxy(joint_move_service, JointMove)

    speed_scale = min(args.speed_scale, 0.15)

    header = [
        "stamp", "robot_id", "robot_name", "pose_idx",
        "world_frame", "camera_frame", "cube_frame", "base_frame", "tool_frame",
        "world_cam_tx", "world_cam_ty", "world_cam_tz", "world_cam_qx", "world_cam_qy", "world_cam_qz", "world_cam_qw",
        "world_cube_tx", "world_cube_ty", "world_cube_tz", "world_cube_qx", "world_cube_qy", "world_cube_qz", "world_cube_qw",
        "base_tool_tx", "base_tool_ty", "base_tool_tz", "base_tool_qx", "base_tool_qy", "base_tool_qz", "base_tool_qw",
        "note",
    ]

    with open(dataset_path, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(header)

        for idx, pose in enumerate(poses):
            rospy.loginfo("[Recorder] Moving to pose %d/%d", idx + 1, len(poses))
            try:
                move_client.wait_for_service(1.0)
            except rospy.ROSException:
                rospy.logwarn("[Recorder] JointMove service %s unavailable, skip pose", joint_move_service)
                continue

            req = JointMoveRequest()
            req.pose = pose
            req.mvvelo = speed_scale
            req.mvacc = speed_scale
            req.mvradius = 0.0
            try:
                resp = move_client(req)
                if resp.ret != 0:
                    rospy.logwarn("[Recorder] Move failed: %s", getattr(resp, "message", ""))
                    continue
            except rospy.ServiceException as exc:  # noqa: BLE001
                rospy.logwarn("[Recorder] Move service call failed: %s", exc)
                continue

            rospy.sleep(2.0)

            try:
                T_world_cam = tf_buffer.lookupTransform(args.world_frame, args.camera_frame, rospy.Time(0), rospy.Duration(1.0))
                T_world_cube = tf_buffer.lookupTransform(args.world_frame, cube_frame, rospy.Time(0), rospy.Duration(1.0))
                T_base_tool = tf_buffer.lookupTransform(robot["base_frame"], robot["tool_frame"], rospy.Time(0), rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as exc:
                rospy.logwarn("[Recorder] Missing TF (world/cube/base_tool), skip sample: %s", exc)
                continue

            stamp_now = rospy.Time.now().to_sec()
            row = [
                f"{stamp_now:.9f}",
                robot["id"],
                robot_name,
                idx,
                args.world_frame,
                args.camera_frame,
                cube_frame,
                robot["base_frame"],
                robot["tool_frame"],
                *transform_to_list(T_world_cam),
                *transform_to_list(T_world_cube),
                *transform_to_list(T_base_tool),
                "",
            ]
            writer.writerow(row)
            rospy.loginfo("[Recorder] Sample %d recorded", idx + 1)

    rospy.loginfo("[Recorder] Dataset saved to %s", dataset_path)


if __name__ == "__main__":
    main()
