#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <stdexcept>

class CubeWorldClosedLoopNode {
public:
  CubeWorldClosedLoopNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : tf_listener_(tf_buffer_) {
    pnh.param<std::string>("world_frame", world_frame_, std::string("world"));
    pnh.param<std::string>("cube_frame", cube_frame_, std::string("cube_center"));
    pnh.param<std::string>("camera_frame", camera_frame_, std::string("zed2i_left_camera_optical_frame"));
    pnh.param<std::string>("robot_base_frame", robot_base_frame_, std::string("Link_0"));
    pnh.param<std::string>("tool_frame", tool_frame_, std::string("Link_6"));
    pnh.param<std::string>("cube_topic", cube_topic_, std::string("/cube_center_fused"));

    const std::string default_tool_offset_yaml = ros::package::getPath("jaka_close_contro") + "/config/tool_offset_current.yaml";
    pnh.param<std::string>("tool_offset_yaml", tool_offset_yaml_, default_tool_offset_yaml);
    loadToolOffset();

    double target_x = target_pos_world_.x();
    double target_y = target_pos_world_.y();
    double target_z = target_pos_world_.z();
    double target_qx = target_quat_world_.x();
    double target_qy = target_quat_world_.y();
    double target_qz = target_quat_world_.z();
    double target_qw = target_quat_world_.w();

    pnh.param<double>("target_world_x", target_x, 0.0);
    pnh.param<double>("target_world_y", target_y, 0.0);
    pnh.param<double>("target_world_z", target_z, 0.0);
    pnh.param<double>("target_world_qx", target_qx, 0.0);
    pnh.param<double>("target_world_qy", target_qy, 0.0);
    pnh.param<double>("target_world_qz", target_qz, 0.0);
    pnh.param<double>("target_world_qw", target_qw, 1.0);

    target_pos_world_.setValue(target_x, target_y, target_z);
    target_quat_world_.setValue(target_qx, target_qy, target_qz, target_qw);
    target_quat_world_.normalize();

    cube_sub_ = nh.subscribe(cube_topic_, 1, &CubeWorldClosedLoopNode::cubeCallback, this);

    ROS_INFO("[CubeWorld] 节点已启动");
    ROS_INFO("[CubeWorld] world_frame=%s, cube_frame=%s, camera_frame=%s", world_frame_.c_str(), cube_frame_.c_str(),
             camera_frame_.c_str());
    ROS_INFO("[CubeWorld] robot_base_frame=%s, tool_frame=%s, tool_offset_yaml=%s", robot_base_frame_.c_str(),
             tool_frame_.c_str(), tool_offset_yaml_.c_str());
    ROS_INFO_STREAM("[CubeWorld] 目标世界位姿: pos = (" << target_pos_world_.x() << ", " << target_pos_world_.y() << ", "
                    << target_pos_world_.z() << ")"
                    << ", quat = (" << target_quat_world_.x() << ", " << target_quat_world_.y() << ", "
                    << target_quat_world_.z() << ", " << target_quat_world_.w() << ")");
  }

private:
  ros::Subscriber cube_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string world_frame_;
  std::string cube_frame_;
  std::string camera_frame_;
  std::string robot_base_frame_;
  std::string tool_frame_;
  std::string cube_topic_;

  std::string tool_offset_yaml_;
  tf2::Transform tool_offset_;
  tf2::Vector3 target_pos_world_{0.0, 0.0, 0.0};
  tf2::Quaternion target_quat_world_{0.0, 0.0, 0.0, 1.0};

  geometry_msgs::PoseStamped latest_cube_cam_;
  bool has_cube_pose_ = false;

  tf2::Transform poseToTf(const geometry_msgs::Pose& pose) const {
    tf2::Transform tf;
    tf2::fromMsg(pose, tf);
    return tf;
  }

  geometry_msgs::Pose tfToPose(const tf2::Transform& tf) const {
    geometry_msgs::Pose pose;
    pose.position.x = tf.getOrigin().x();
    pose.position.y = tf.getOrigin().y();
    pose.position.z = tf.getOrigin().z();
    pose.orientation = tf2::toMsg(tf.getRotation());
    return pose;
  }

  void loadToolOffset() {
    try {
      YAML::Node root = YAML::LoadFile(tool_offset_yaml_);
      if (!root["tool_offset"] || !root["tool_offset"]["translation"] || !root["tool_offset"]["rotation"]) {
        throw std::runtime_error("tool_offset 缺少 translation/rotation");
      }
      auto trans = root["tool_offset"]["translation"];
      auto rot = root["tool_offset"]["rotation"];
      if (trans.size() != 3 || rot.size() != 4) {
        throw std::runtime_error("tool_offset translation/rotation 尺寸不正确");
      }
      tf2::Quaternion q(rot[0].as<double>(), rot[1].as<double>(), rot[2].as<double>(), rot[3].as<double>());
      q.normalize();
      tool_offset_.setOrigin(tf2::Vector3(trans[0].as<double>(), trans[1].as<double>(), trans[2].as<double>()));
      tool_offset_.setRotation(q);
      ROS_INFO("[CubeWorld] 已加载 tool_offset: trans[%.3f %.3f %.3f], quat[%.3f %.3f %.3f %.3f]", trans[0].as<double>(),
               trans[1].as<double>(), trans[2].as<double>(), q.x(), q.y(), q.z(), q.w());
    } catch (const std::exception& e) {
      tool_offset_.setOrigin(tf2::Vector3(0.0, 0.0, 0.077));
      tool_offset_.setRotation(tf2::Quaternion::getIdentity());
      ROS_WARN("[CubeWorld] 读取 tool_offset_yaml 失败 (%s)，使用默认 [0,0,0.077]", e.what());
    }
  }

  bool lookupTransform(const std::string& target, const std::string& source, const ros::Time& stamp,
                       tf2::Transform& out) {
    geometry_msgs::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_.lookupTransform(target, source, stamp, ros::Duration(0.1));
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1.0, "[CubeWorld] TF 查询失败 %s -> %s: %s", target.c_str(), source.c_str(), ex.what());
      return false;
    }
    tf2::fromMsg(tf_msg.transform, out);
    return true;
  }

  void cubeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    latest_cube_cam_ = *msg;
    has_cube_pose_ = true;

    geometry_msgs::PoseStamped cube_world;
    try {
      tf_buffer_.transform(*msg, cube_world, world_frame_, ros::Duration(0.1));
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1.0, "[CubeWorld] 将 cube 转换到 world 失败: %s", ex.what());
      return;
    }

    tf2::Vector3 p_world(cube_world.pose.position.x, cube_world.pose.position.y, cube_world.pose.position.z);
    tf2::Quaternion q_world;
    tf2::fromMsg(cube_world.pose.orientation, q_world);
    q_world.normalize();

    ROS_INFO_STREAM_THROTTLE(0.3, "[CubeWorld] cube in world:\n"
                                      << "  pos  = (" << p_world.x() << ", " << p_world.y() << ", " << p_world.z() << ")\n"
                                      << "  quat = (" << q_world.x() << ", " << q_world.y() << ", " << q_world.z() << ", "
                                      << q_world.w() << ")");

    // 计算世界坐标误差
    tf2::Vector3 pos_error = target_pos_world_ - p_world;
    tf2::Quaternion q_err = q_world.inverse() * target_quat_world_;
    q_err.normalize();
    ROS_DEBUG_STREAM_THROTTLE(0.5, "[CubeWorld] 世界系误差: pos = (" << pos_error.x() << ", " << pos_error.y() << ", "
                                      << pos_error.z() << "), quat = (" << q_err.x() << ", " << q_err.y() << ", "
                                      << q_err.z() << ", " << q_err.w() << ")");

    // 生成末端目标姿态（暂不下发控制，仅做示意）
    tf2::Transform T_w_b;
    if (!lookupTransform(world_frame_, robot_base_frame_, cube_world.header.stamp, T_w_b)) {
      return;
    }

    tf2::Transform T_w_cube_target;
    T_w_cube_target.setOrigin(target_pos_world_);
    T_w_cube_target.setRotation(target_quat_world_);

    tf2::Transform T_b_tool_target = T_w_b.inverse() * T_w_cube_target * tool_offset_.inverse();
    geometry_msgs::Pose target_pose_base = tfToPose(T_b_tool_target);

    ROS_DEBUG_STREAM_THROTTLE(0.5, "[CubeWorld] 期望末端 (base系): pos=(" << target_pose_base.position.x << ", "
                                       << target_pose_base.position.y << ", " << target_pose_base.position.z << ") quat=("
                                       << target_pose_base.orientation.x << ", " << target_pose_base.orientation.y << ", "
                                       << target_pose_base.orientation.z << ", " << target_pose_base.orientation.w << ")");

    // TODO: 调用具体的机械臂控制接口下发 target_pose_base
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "cube_world_closedloop_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    CubeWorldClosedLoopNode node(nh, pnh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("[CubeWorld] 节点启动失败: %s", e.what());
    return 1;
  }

  return 0;
}
