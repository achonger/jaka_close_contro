// fiducial_relay_node.cpp
// 集成到 jaka_close_contro 包
// 将 /fiducial_transforms 分流为 /world_fiducials 和 /tool_fiducials
// 修复：正确处理字符串和整数类型的参数

#include <ros/ros.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <vector>
#include <algorithm>
#include <string>
#include <cstdlib>
#include <sstream>
#include <xmlrpcpp/XmlRpcValue.h>

class FiducialRelay {
public:
  FiducialRelay(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    // 读取 world_id (支持字符串和整数)
    if (pnh_.hasParam("world_id")) {
      std::string world_id_str;
      if (pnh_.getParam("world_id", world_id_str)) {
        // 尝试转换为整数
        try {
          world_id_ = std::stoi(world_id_str);
          ROS_INFO("[Relay] world_id (string->int): %d", world_id_);
        } catch (...) {
          ROS_ERROR("[Relay] Failed to convert world_id '%s' to integer", world_id_str.c_str());
          world_id_ = 0;
        }
      } else {
        // 尝试直接读取整数
        pnh_.param<int>("world_id", world_id_, 0);
        ROS_INFO("[Relay] world_id (int): %d", world_id_);
      }
    } else {
      world_id_ = 0;
      ROS_INFO("[Relay] world_id not set, using default: %d", world_id_);
    }

    // 读取 tool_ids (支持字符串向量和整数向量)
    tool_ids_.clear();
    XmlRpc::XmlRpcValue tool_ids_param;
    if (pnh_.hasParam("tool_ids") && pnh_.getParam("tool_ids", tool_ids_param)) {
      if (tool_ids_param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < tool_ids_param.size(); ++i) {
          if (tool_ids_param[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
            // 字符串类型，尝试转换为整数
            std::string id_str = static_cast<std::string>(tool_ids_param[i]);
            try {
              int id = std::stoi(id_str);
              tool_ids_.push_back(id);
              ROS_INFO("[Relay] Added tool_id (string->int): %d", id);
            } catch (...) {
              ROS_WARN("[Relay] Failed to convert tool_id '%s' to integer", id_str.c_str());
            }
          } else if (tool_ids_param[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
            // 整数类型
            int id = static_cast<int>(tool_ids_param[i]);
            tool_ids_.push_back(id);
            ROS_INFO("[Relay] Added tool_id (int): %d", id);
          }
        }
      } else if (tool_ids_param.getType() == XmlRpc::XmlRpcValue::TypeString) {
        // 单个字符串，可能是逗号分隔
        std::string ids_str = static_cast<std::string>(tool_ids_param);
        std::istringstream iss(ids_str);
        std::string id_str;
        while (std::getline(iss, id_str, ',')) {
          try {
            // 去除空格
            id_str.erase(0, id_str.find_first_not_of(" "));
            id_str.erase(id_str.find_last_not_of(" ") + 1);
            int id = std::stoi(id_str);
            tool_ids_.push_back(id);
            ROS_INFO("[Relay] Added tool_id (comma-separated): %d", id);
          } catch (...) {
            ROS_WARN("[Relay] Failed to convert tool_id '%s' to integer", id_str.c_str());
          }
        }
      }
    }

    // 如果没有设置或转换失败，使用默认值
    if (tool_ids_.empty()) {
      tool_ids_ = {10, 11, 12, 13};  // 默认工具ID
      ROS_INFO("[Relay] Using default tool_ids: [10, 11, 12, 13]");
    }

    // 读取话题名称
    pnh_.param<std::string>("input_topic", input_topic_, "/fiducial_transforms");
    pnh_.param<std::string>("output_world_topic", world_topic_, std::string(""));
    pnh_.param<std::string>("output_tool_topic", tool_topic_, std::string(""));
    if (world_topic_.empty()) {
      pnh_.param<std::string>("world_topic", world_topic_, "/world_fiducials");
    }
    if (tool_topic_.empty()) {
      pnh_.param<std::string>("tool_topic", tool_topic_, "/tool_fiducials");
    }

    std::ostringstream tool_ids_ss;
    tool_ids_ss << "[";
    for (size_t i = 0; i < tool_ids_.size(); ++i) {
      tool_ids_ss << tool_ids_[i];
      if (i + 1 < tool_ids_.size()) {
        tool_ids_ss << ", ";
      }
    }
    tool_ids_ss << "]";

    ROS_INFO("[Relay] input_topic=%s", input_topic_.c_str());
    ROS_INFO("[Relay] output_world_topic=%s", world_topic_.c_str());
    ROS_INFO("[Relay] output_tool_topic=%s", tool_topic_.c_str());
    ROS_INFO("[Relay] world_id=%d", world_id_);
    ROS_INFO("[Relay] tool_ids=%s", tool_ids_ss.str().c_str());

    sub_ = nh_.subscribe(input_topic_, 1, &FiducialRelay::callback, this);
    world_pub_ = nh_.advertise<fiducial_msgs::FiducialTransformArray>(world_topic_, 1);
    tool_pub_ = nh_.advertise<fiducial_msgs::FiducialTransformArray>(tool_topic_, 1);
  }

private:
  void callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg) {
    fiducial_msgs::FiducialTransformArray world_msg, tool_msg;
    world_msg.header = msg->header;
    tool_msg.header = msg->header;

    for (const auto& ft : msg->transforms) {
      if (ft.fiducial_id == world_id_) {
        world_msg.transforms.push_back(ft);
        ROS_DEBUG("[Relay] Routed world fiducial ID=%d", ft.fiducial_id);
      } else if (std::find(tool_ids_.begin(), tool_ids_.end(), ft.fiducial_id) != tool_ids_.end()) {
        tool_msg.transforms.push_back(ft);
        ROS_DEBUG("[Relay] Routed tool fiducial ID=%d", ft.fiducial_id);
      } else {
        ROS_DEBUG("[Relay] Ignored fiducial ID=%d (not world or tool)", ft.fiducial_id);
      }
    }

    world_pub_.publish(world_msg);
    tool_pub_.publish(tool_msg);

    if (world_msg.transforms.empty()) {
      ROS_WARN_THROTTLE(1.0, "[Relay] World fiducial ID=%d not found in input", world_id_);
    }
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  ros::Publisher world_pub_, tool_pub_;

  int world_id_;
  std::vector<int> tool_ids_;
  std::string input_topic_, world_topic_, tool_topic_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "fiducial_relay_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    FiducialRelay relay(nh, pnh);
    ROS_INFO("[Relay] Node started successfully");
    ros::spin();
    return 0;
  } catch (const std::exception& e) {
    ROS_FATAL("[Relay] Exception: %s", e.what());
    return 1;
  }
}
