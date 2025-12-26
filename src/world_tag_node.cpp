// world_tag_node.cpp
// 集成到 jaka_close_contro 包
// 修正后的 TF 方向 (无需求逆)

#include <ros/ros.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <string>
#include <set>
#include <xmlrpcpp/XmlRpcValue.h>

class WorldTagNode {
public:
  WorldTagNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), has_prev_(false) {
    pnh_.param("world_fiducial_id", world_board_id_, 500);
    pnh_.param("world_board_id", world_board_id_, world_board_id_);
    pnh_.param<std::string>("fiducial_topic", fiducial_topic_, "/world_fiducials");
    pnh_.param<std::string>("world_frame", world_frame_, "world");
    pnh_.param<std::string>("camera_frame", camera_frame_, "zed2i_left_camera_optical_frame");
    pnh_.param("alpha_pos", alpha_pos_, 0.3);
    pnh_.param("alpha_rot", alpha_rot_, 0.3);
    loadWorldIds();

    ROS_INFO_STREAM("[WorldTag] world_board_id = " << world_board_id_);
    ROS_INFO_STREAM("[WorldTag] world_ids size = " << world_ids_.size());
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
      if (ft.fiducial_id == world_board_id_) {
        transform = ft.transform;
        found = true;
        break;
      }
      if (world_ids_.count(ft.fiducial_id) > 0) {
        transform = ft.transform;
        found = true;
        break;
      }
    }

    if (!found) {
      ROS_WARN_THROTTLE(2.0, "[WorldTag] World fiducial ID=%d not found.", world_board_id_);
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

    tf2::Transform T_world_cam;
    T_world_cam.setOrigin(origin);
    T_world_cam.setRotation(rotation);

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

  int world_board_id_;
  std::string fiducial_topic_, world_frame_, camera_frame_;
  double alpha_pos_, alpha_rot_;
  bool has_prev_;
  tf2::Transform prev_T_world_cam_;
  std::set<int> world_ids_;

  void loadWorldIds() {
    XmlRpc::XmlRpcValue ids_param;
    if (pnh_.hasParam("world_ids") && pnh_.getParam("world_ids", ids_param)) {
      if (ids_param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < ids_param.size(); ++i) {
          if (ids_param[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
            world_ids_.insert(static_cast<int>(ids_param[i]));
          } else if (ids_param[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
            try {
              world_ids_.insert(std::stoi(static_cast<std::string>(ids_param[i])));
            } catch (...) {
              ROS_WARN("[WorldTag] Failed to parse world_ids entry");
            }
          }
        }
      }
    }
    if (world_ids_.empty()) {
      // 默认使用 2x2 world board 四个角的 ID
      world_ids_ = {500, 501, 502, 503};
    }
  }
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
