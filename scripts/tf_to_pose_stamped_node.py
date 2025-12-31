#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node("tf_to_pose_stamped_node")
    parent_frame = rospy.get_param("~parent_frame", "world")
    child_frame = rospy.get_param("~child_frame", "cube_center")
    output_topic = rospy.get_param("~output_topic", "/vision/cube_center_world")
    rate_hz = rospy.get_param("~rate_hz", 30.0)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    pub = rospy.Publisher(output_topic, PoseStamped, queue_size=1)
    rate = rospy.Rate(rate_hz)

    rospy.loginfo(
        "[tf_to_pose_stamped] parent_frame=%s child_frame=%s output_topic=%s rate=%.2f",
        parent_frame,
        child_frame,
        output_topic,
        rate_hz,
    )

    while not rospy.is_shutdown():
        try:
            tf = tf_buffer.lookup_transform(
                parent_frame, child_frame, rospy.Time(0), rospy.Duration(0.5)
            )
            msg = PoseStamped()
            msg.header.frame_id = parent_frame
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = tf.transform.translation.x
            msg.pose.position.y = tf.transform.translation.y
            msg.pose.position.z = tf.transform.translation.z
            msg.pose.orientation = tf.transform.rotation
            pub.publish(msg)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            rospy.logwarn_throttle(
                5.0,
                "[tf_to_pose_stamped] TF %s->%s unavailable; waiting...",
                parent_frame,
                child_frame,
            )
        rate.sleep()


if __name__ == "__main__":
    main()
