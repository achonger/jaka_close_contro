#!/usr/bin/env python3
import argparse
import sys

import rospy
from jaka_close_contro.srv import GetCubePoseWorld


def parse_args():
    p = argparse.ArgumentParser(description="Query cube_center pose in world frame")
    p.add_argument("--robot", required=True, help="robot name, e.g., jaka1/jaka2")
    return p.parse_args()


def main():
    args = parse_args()
    rospy.init_node("get_cube_pose_world_cli", anonymous=True)
    srv_name = f"/{args.robot}/pose_servo_world/get_cube_pose_world"
    rospy.wait_for_service(srv_name)
    cli = rospy.ServiceProxy(srv_name, GetCubePoseWorld)
    resp = cli()
    if not resp.ok:
        print(f"[cli] {srv_name} -> ok={resp.ok}, message={resp.message}, age={resp.age_sec:.3f}s", file=sys.stderr)
    else:
        print(f"[cli] {srv_name} -> ok={resp.ok}, message={resp.message}, age={resp.age_sec:.3f}s")
    p = resp.pose.pose
    stamp = resp.pose.header.stamp.to_sec()
    print(
        f"world cube pose: x={p.position.x:.4f} y={p.position.y:.4f} z={p.position.z:.4f} "
        f"qx={p.orientation.x:.4f} qy={p.orientation.y:.4f} qz={p.orientation.z:.4f} qw={p.orientation.w:.4f} "
        f"stamp={stamp:.3f} age={resp.age_sec:.3f}s"
    )


if __name__ == "__main__":
    main()
