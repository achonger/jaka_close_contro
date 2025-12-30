#!/usr/bin/env python3
import argparse
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from jaka_close_contro.srv import SetPoseTarget, SetPoseTargetRequest


def parse_args():
    p = argparse.ArgumentParser(description="Send world target pose to pose_servo_world")
    p.add_argument("--robot", required=True, help="robot name, e.g., jaka1/jaka2")
    p.add_argument("--x", type=float, help="position x (m)")
    p.add_argument("--y", type=float, help="position y (m)")
    p.add_argument("--z", type=float, help="position z (m)")
    p.add_argument("--roll_deg", type=float, help="roll deg")
    p.add_argument("--pitch_deg", type=float, help="pitch deg")
    p.add_argument("--yaw_deg", type=float, help="yaw deg")
    p.add_argument("--qx", type=float, help="quat x")
    p.add_argument("--qy", type=float, help="quat y")
    p.add_argument("--qz", type=float, help="quat z")
    p.add_argument("--qw", type=float, help="quat w")
    p.add_argument("--stop", action="store_true", help="call stop service")
    p.add_argument("--clear", action="store_true", help="call clear_target service")
    return p.parse_args()


def main():
    args = parse_args()
    rospy.init_node("send_world_target_cli", anonymous=True)

    ns = f"/{args.robot}/pose_servo_world"
    stop_srv = rospy.ServiceProxy(ns + "/stop", Empty)
    clear_srv = rospy.ServiceProxy(ns + "/clear_target", Empty)
    set_target_srv = rospy.ServiceProxy(ns + "/set_target", SetPoseTarget)

    if args.stop:
        stop_srv()
        print(f"[cli] stop sent to {ns}")
        return
    if args.clear:
        clear_srv()
        print(f"[cli] clear_target sent to {ns}")
        return

    if None in (args.x, args.y, args.z):
        print("position x/y/z are required unless stop/clear", file=sys.stderr)
        sys.exit(1)

    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.pose.position.x = args.x
    pose.pose.position.y = args.y
    pose.pose.position.z = args.z

    use_quat = None not in (args.qx, args.qy, args.qz, args.qw)
    use_rpy = None not in (args.roll_deg, args.pitch_deg, args.yaw_deg)
    if use_quat:
        pose.pose.orientation.x = args.qx
        pose.pose.orientation.y = args.qy
        pose.pose.orientation.z = args.qz
        pose.pose.orientation.w = args.qw
    elif use_rpy:
        import tf.transformations as tft

        q = tft.quaternion_from_euler(
            args.roll_deg * 3.1415926535 / 180.0,
            args.pitch_deg * 3.1415926535 / 180.0,
            args.yaw_deg * 3.1415926535 / 180.0,
        )
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
    else:
        print("orientation required: either qx/qy/qz/qw or roll_deg/pitch_deg/yaw_deg", file=sys.stderr)
        sys.exit(1)

    req = SetPoseTargetRequest(target=pose)
    resp = set_target_srv(req)
    if resp.ok:
        print(f"[cli] target accepted by {ns}: {resp.message}")
    else:
        print(f"[cli] target rejected by {ns}: {resp.message}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
