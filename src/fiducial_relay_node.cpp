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
  FiducialRelay(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : pnh_(pnh) {
    loadWorldIds();
    loadRobotIds();
    loadToolIds();

    // 读取话题名称
    pnh_.param<std::string>("input_topic", input_topic_, "/fiducial_transforms");
    pnh_.param<std::string>("world_topic", world_topic_, "/world_fiducials");
    pnh_.param<std::string>("tool_topic", tool_topic_, "/tool_fiducials");

    ROS_INFO("[Relay] input: %s", input_topic_.c_str());
    ROS_INFO("[Relay] world output: %s", world_topic_.c_str());
    ROS_INFO("[Relay] tool output: %s", tool_topic_.c_str());

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
      if (isWorldId(ft.fiducial_id)) {
        world_msg.transforms.push_back(ft);
        ROS_DEBUG("[Relay] Routed world fiducial ID=%d", ft.fiducial_id);
        continue;
      }

      // 默认策略：如果没有 tool_ids 白名单，则转发所有非 world_id
      if (tool_ids_.empty()) {
        tool_msg.transforms.push_back(ft);
        continue;
      }

      if (std::find(tool_ids_.begin(), tool_ids_.end(), ft.fiducial_id) != tool_ids_.end()) {
        tool_msg.transforms.push_back(ft);
        ROS_DEBUG("[Relay] Routed tool fiducial ID=%d", ft.fiducial_id);
      } else {
        ROS_DEBUG("[Relay] Ignored fiducial ID=%d (not world or tool)", ft.fiducial_id);
      }
    }

    world_pub_.publish(world_msg);
    tool_pub_.publish(tool_msg);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  ros::Publisher world_pub_, tool_pub_;

  std::vector<int> world_ids_;
  std::vector<int> tool_ids_;
  std::vector<int> robot_ids_;
  int robot_stride_{100};
  int face_id_base_{10};
  int face_count_{4};
  std::string input_topic_, world_topic_, tool_topic_;

  void loadWorldIds() {
    world_ids_.clear();
    if (pnh_.hasParam("world_ids")) {
      XmlRpc::XmlRpcValue wids;
      pnh_.getParam("world_ids", wids);
      parseIdList(wids, world_ids_);
    }

    if (world_ids_.empty()) {
      // 兼容旧参数 world_id
      if (pnh_.hasParam("world_id")) {
        std::string wid_str;
        if (pnh_.getParam("world_id", wid_str)) {
          try {
            world_ids_.push_back(std::stoi(wid_str));
          } catch (...) {
            ROS_WARN("[Relay] world_id string conversion failed, using default 0");
          }
        } else {
          int wid_int = 0;
          pnh_.param<int>("world_id", wid_int, 0);
          world_ids_.push_back(wid_int);
        }
      } else {
        world_ids_ = {500, 501, 502, 503};
      }
    }

    ROS_INFO("[Relay] world_ids:");
    for (int wid : world_ids_) {
      ROS_INFO("  %d", wid);
    }
  }

  void loadRobotIds() {
    XmlRpc::XmlRpcValue rid_param;
    if (pnh_.hasParam("robot_ids") && pnh_.getParam("robot_ids", rid_param)) {
      parseIdList(rid_param, robot_ids_);
    }
    if (robot_ids_.empty()) {
      robot_ids_ = {1};
    }
    pnh_.param("robot_stride", robot_stride_, 100);
    pnh_.param("face_id_base", face_id_base_, 10);
    pnh_.param("face_count", face_count_, 4);
  }

  void loadToolIds() {
    tool_ids_.clear();
    XmlRpc::XmlRpcValue tool_ids_param;
    if (pnh_.hasParam("tool_ids") && pnh_.getParam("tool_ids", tool_ids_param)) {
      parseIdList(tool_ids_param, tool_ids_);
    } else {
      bool generate_tool_ids = false;
      pnh_.param("generate_tool_ids", generate_tool_ids, false);
      if (!generate_tool_ids) {
        ROS_INFO("[Relay] tool_ids empty -> pass through all non-world fiducials");
        return;
      }
      // 自动根据 robot_ids 生成 face_id 白名单：face_id_base + [0..face_count-1] + robot_stride*(rid-1)
      for (int rid : robot_ids_) {
        for (int face_idx = 0; face_idx < face_count_; ++face_idx) {
          tool_ids_.push_back(face_id_base_ + face_idx + (rid - 1) * robot_stride_);
        }
      }
    }

    ROS_INFO("[Relay] tool_ids (empty means pass-through):");
    for (int tid : tool_ids_) {
      ROS_INFO("  %d", tid);
    }
  }

  bool isWorldId(int id) const {
    return std::find(world_ids_.begin(), world_ids_.end(), id) != world_ids_.end();
  }

  void parseIdList(const XmlRpc::XmlRpcValue& param, std::vector<int>& out) {
    if (param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int i = 0; i < param.size(); ++i) {
        if (param[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
          out.push_back(static_cast<int>(param[i]));
        } else if (param[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
          try {
            out.push_back(std::stoi(static_cast<std::string>(param[i])));
          } catch (...) {
            ROS_WARN("[Relay] Failed to parse id string");
          }
        }
      }
    } else if (param.getType() == XmlRpc::XmlRpcValue::TypeString) {
      std::istringstream ss(static_cast<std::string>(param));
      std::string token;
      while (std::getline(ss, token, ',')) {
        try {
          token.erase(0, token.find_first_not_of(" "));
          token.erase(token.find_last_not_of(" ") + 1);
          out.push_back(std::stoi(token));
        } catch (...) {
          ROS_WARN("[Relay] Failed to parse id from comma list");
        }
      }
    }
  }
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
