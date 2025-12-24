// world_tag_node.cpp
// 集成到 jaka_close_contro 包
// fiducial_msgs::FiducialTransform 表示 camera->fiducial，这里需要取逆得到 world->camera

#include <ros/ros.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <string>

class WorldTagNode {
public:
  WorldTagNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), has_prev_(false) {
    if (pnh_.hasParam("world_id")) {
      pnh_.param("world_id", world_id_, 500);
    } else {
      pnh_.param("world_fiducial_id", world_id_, 500);
    }
    pnh_.param<std::string>("fiducial_topic", fiducial_topic_, "/world_fiducials");
    pnh_.param<std::string>("world_frame", world_frame_, "world");
    pnh_.param<std::string>("camera_frame", camera_frame_, "zed2i_left_camera_optical_frame");
    pnh_.param("alpha_pos", alpha_pos_, 0.3);
    pnh_.param("alpha_rot", alpha_rot_, 0.3);

    ROS_INFO_STREAM("[WorldTag] world_fiducial_id = " << world_id_);
    ROS_INFO_STREAM("[WorldTag] fiducial_topic    = " << fiducial_topic_);
    ROS_INFO_STREAM("[WorldTag] world_frame       = " << world_frame_);
    ROS_INFO_STREAM("[WorldTag] camera_frame      = " << camera_frame_);

    sub_ = nh_.subscribe(fiducial_topic_, 1, &WorldTagNode::fiducialCb, this);
  }

private:
  void fiducialCb(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg) {
    bool found = false;
    geometry_msgs::Transform transform;

    for (const auto& ft : msg->transforms) {
      if (ft.fiducial_id == world_id_) {
        transform = ft.transform;
        found = true;
        break;
      }
    }

    if (!found) {
      ROS_WARN_THROTTLE(2.0, "[WorldTag] World fiducial ID=%d not found.", world_id_);
      return;
    }

    tf2::Vector3 origin(
      transform.translation.x,
      transform.translation.y,
      transform.translation.z
    );
    tf2::Quaternion rotation(
      transform.rotation.x,
      transform.rotation.y,
      transform.rotation.z,
      transform.rotation.w
    );
    rotation.normalize();

    // fiducial_msgs::FiducialTransform 表示 camera->fiducial，这里需要取逆得到 world->camera
    tf2::Transform T_cam_world;
    T_cam_world.setOrigin(origin);
    T_cam_world.setRotation(rotation);
    tf2::Transform T_world_cam = T_cam_world.inverse();

    if (!has_prev_) {
      prev_T_world_cam_ = T_world_cam;
      has_prev_ = true;
    } else {
      tf2::Vector3 p_new = T_world_cam.getOrigin();
      tf2::Vector3 p_old = prev_T_world_cam_.getOrigin();
      tf2::Vector3 p_f = alpha_pos_ * p_new + (1.0 - alpha_pos_) * p_old;

      tf2::Quaternion q_new = T_world_cam.getRotation();
      tf2::Quaternion q_old = prev_T_world_cam_.getRotation();
      q_new.normalize(); 
      q_old.normalize();
      tf2::Quaternion q_f = q_old.slerp(q_new, alpha_rot_);
      q_f.normalize();

      prev_T_world_cam_.setOrigin(p_f);
      prev_T_world_cam_.setRotation(q_f);
    }

    geometry_msgs::TransformStamped ts;
    ts.header.stamp = msg->header.stamp;
    ts.header.frame_id = world_frame_;
    ts.child_frame_id = camera_frame_;

    const auto& p = prev_T_world_cam_.getOrigin();
    const auto& q = prev_T_world_cam_.getRotation();
    ts.transform.translation.x = p.x();
    ts.transform.translation.y = p.y();
    ts.transform.translation.z = p.z();
    ts.transform.rotation.x = q.x();
    ts.transform.rotation.y = q.y();
    ts.transform.rotation.z = q.z();
    ts.transform.rotation.w = q.w();

    tf_broadcaster_.sendTransform(ts);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  int world_id_;
  std::string fiducial_topic_, world_frame_, camera_frame_;
  double alpha_pos_, alpha_rot_;
  bool has_prev_;
  tf2::Transform prev_T_world_cam_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "world_tag_node");
  ros::NodeHandle nh, pnh("~");
  
  try {
    WorldTagNode node(nh, pnh);
    ROS_INFO("[WorldTag] Started successfully");
    ros::spin();
    return 0;
  } catch (const std::exception& e) {
    ROS_FATAL("[WorldTag] Initialization failed: %s", e.what());
    return 1;
  }
}
