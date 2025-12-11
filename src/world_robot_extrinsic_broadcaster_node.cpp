#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

class WorldRobotExtrinsicBroadcaster {
public:
  WorldRobotExtrinsicBroadcaster(ros::NodeHandle& pnh) : use_static_(true) {
    std::string default_yaml = ros::package::getPath("jaka_close_contro") + "/config/world_robot_extrinsic.yaml";
    pnh.param<std::string>("extrinsic_yaml", extrinsic_yaml_, default_yaml);
    pnh.param<std::string>("world_frame", world_frame_override_, std::string(""));
    pnh.param<std::string>("robot_base_frame", robot_base_frame_override_, std::string(""));
    pnh.param<bool>("use_static", use_static_, true);

    if (!loadExtrinsic()) {
      throw std::runtime_error("无法加载世界-机器人外参 YAML: " + extrinsic_yaml_);
    }

    if (use_static_) {
      static_broadcaster_.sendTransform(tf_);
      ROS_INFO("[Extrinsic] 已发布静态 TF %s -> %s", tf_.header.frame_id.c_str(), tf_.child_frame_id.c_str());
    } else {
      timer_ = pnh.createTimer(ros::Duration(0.1), &WorldRobotExtrinsicBroadcaster::publishTimer, this);
      ROS_INFO("[Extrinsic] 正以 10Hz 发布 TF %s -> %s", tf_.header.frame_id.c_str(), tf_.child_frame_id.c_str());
    }
  }

private:
  std::string extrinsic_yaml_;
  std::string world_frame_override_;
  std::string robot_base_frame_override_;
  bool use_static_;

  geometry_msgs::TransformStamped tf_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  tf2_ros::TransformBroadcaster dynamic_broadcaster_;
  ros::Timer timer_;

  bool loadExtrinsic() {
    ROS_INFO("[Extrinsic] 读取外参文件: %s", extrinsic_yaml_.c_str());
    YAML::Node config;
    try {
      config = YAML::LoadFile(extrinsic_yaml_);
    } catch (const std::exception& e) {
      ROS_ERROR("[Extrinsic] 加载 YAML 失败: %s", e.what());
      return false;
    }

    const std::string world_frame = !world_frame_override_.empty() ? world_frame_override_ : config["world_frame"].as<std::string>("world");
    const std::string base_frame = !robot_base_frame_override_.empty() ? robot_base_frame_override_ : config["robot_base_frame"].as<std::string>("Link_0");

    if (!config["translation"] || !config["rotation"]) {
      ROS_ERROR("[Extrinsic] YAML 中缺少 translation / rotation 字段");
      return false;
    }

    tf_.header.stamp = ros::Time::now();
    tf_.header.frame_id = world_frame;
    tf_.child_frame_id = base_frame;

    tf_.transform.translation.x = config["translation"].as<YAML::Node>()["x"].as<double>();
    tf_.transform.translation.y = config["translation"].as<YAML::Node>()["y"].as<double>();
    tf_.transform.translation.z = config["translation"].as<YAML::Node>()["z"].as<double>();

    tf_.transform.rotation.x = config["rotation"].as<YAML::Node>()["x"].as<double>();
    tf_.transform.rotation.y = config["rotation"].as<YAML::Node>()["y"].as<double>();
    tf_.transform.rotation.z = config["rotation"].as<YAML::Node>()["z"].as<double>();
    tf_.transform.rotation.w = config["rotation"].as<YAML::Node>()["w"].as<double>();

    tf2::Quaternion q(tf_.transform.rotation.x, tf_.transform.rotation.y, tf_.transform.rotation.z, tf_.transform.rotation.w);
    q.normalize();
    tf_.transform.rotation = tf2::toMsg(q);

    ROS_INFO_STREAM("[Extrinsic] world_frame=" << tf_.header.frame_id
                    << ", robot_base_frame=" << tf_.child_frame_id
                    << "\n  translation = (" << tf_.transform.translation.x << ", " << tf_.transform.translation.y << ", " << tf_.transform.translation.z << ")"
                    << "\n  rotation    = (" << tf_.transform.rotation.x << ", " << tf_.transform.rotation.y << ", " << tf_.transform.rotation.z << ", " << tf_.transform.rotation.w << ")");
    return true;
  }

  void publishTimer(const ros::TimerEvent&) {
    tf_.header.stamp = ros::Time::now();
    dynamic_broadcaster_.sendTransform(tf_);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "world_robot_extrinsic_broadcaster_node");
  ros::NodeHandle nh("~");
  try {
    WorldRobotExtrinsicBroadcaster broadcaster(nh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("[Extrinsic] 启动失败: %s", e.what());
    return 1;
  }
  return 0;
}
