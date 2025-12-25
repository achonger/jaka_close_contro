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
    loadWorldIds();

    // 读取 tool_ids（可选，若为空则默认将非 world_id 的全部转发到 tool_topic）
    loadToolIds();

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

    auto vecToStr = [](const std::vector<int>& v) {
      std::ostringstream ss;
      ss << "[";
      for (size_t i = 0; i < v.size(); ++i) {
        ss << v[i];
        if (i + 1 < v.size()) ss << ", ";
      }
      ss << "]";
      return ss.str();
    };

    ROS_INFO("[Relay] input_topic=%s", input_topic_.c_str());
    ROS_INFO("[Relay] output_world_topic=%s", world_topic_.c_str());
    ROS_INFO("[Relay] output_tool_topic=%s", tool_topic_.c_str());
    ROS_INFO("[Relay] world_ids=%s", vecToStr(world_ids_).c_str());
    ROS_INFO("[Relay] tool_ids=%s (empty=转发非world全部)", vecToStr(tool_ids_).c_str());

    sub_ = nh_.subscribe(input_topic_, 1, &FiducialRelay::callback, this);
    world_pub_ = nh_.advertise<fiducial_msgs::FiducialTransformArray>(world_topic_, 1);
    tool_pub_ = nh_.advertise<fiducial_msgs::FiducialTransformArray>(tool_topic_, 1);
  }

private:
  void callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg) {
    fiducial_msgs::FiducialTransformArray world_msg, tool_msg;
    world_msg.header = msg->header;
    tool_msg.header = msg->header;

    std::vector<int> detected_ids;
    for (const auto& ft : msg->transforms) {
      detected_ids.push_back(ft.fiducial_id);
      if (isWorldId(ft.fiducial_id)) {
        world_msg.transforms.push_back(ft);
        ROS_DEBUG("[Relay] Routed world fiducial ID=%d", ft.fiducial_id);
      } else if (tool_ids_.empty() || std::find(tool_ids_.begin(), tool_ids_.end(), ft.fiducial_id) != tool_ids_.end()) {
        tool_msg.transforms.push_back(ft);
        ROS_DEBUG("[Relay] Routed tool fiducial ID=%d", ft.fiducial_id);
      } else {
        ROS_DEBUG("[Relay] Ignored fiducial ID=%d (not world or tool)", ft.fiducial_id);
      }
    }

    world_pub_.publish(world_msg);
    tool_pub_.publish(tool_msg);

    if (world_msg.transforms.empty()) {
      std::ostringstream oss;
      for (size_t i = 0; i < detected_ids.size(); ++i) {
        oss << detected_ids[i];
        if (i + 1 < detected_ids.size()) oss << ",";
      }
      ROS_WARN_THROTTLE(1.0, "[Relay] World fiducial IDs not found in input. Detected IDs: [%s]",
                        oss.str().c_str());
    }
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  ros::Publisher world_pub_, tool_pub_;

  std::vector<int> world_ids_;
  std::vector<int> tool_ids_;
  std::string input_topic_, world_topic_, tool_topic_;

  bool isWorldId(int id) const {
    return std::find(world_ids_.begin(), world_ids_.end(), id) != world_ids_.end();
  }

  void loadWorldIds() {
    world_ids_.clear();
    XmlRpc::XmlRpcValue param;
    if (pnh_.getParam("world_ids", param) && param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int i = 0; i < param.size(); ++i) {
        if (param[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
          world_ids_.push_back(static_cast<int>(param[i]));
        } else if (param[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
          try {
            world_ids_.push_back(std::stoi(static_cast<std::string>(param[i])));
          } catch (...) {
            ROS_WARN("[Relay] world_ids 元素无法解析，已跳过");
          }
        }
      }
    }

    if (world_ids_.empty()) {
      // 兼容单 world_id 参数
      int world_id_single = 0;
      if (pnh_.hasParam("world_id")) {
        std::string world_id_str;
        if (pnh_.getParam("world_id", world_id_str)) {
          try {
            world_id_single = std::stoi(world_id_str);
          } catch (...) {
            pnh_.param<int>("world_id", world_id_single, 0);
          }
        } else {
          pnh_.param<int>("world_id", world_id_single, 0);
        }
      }
      world_ids_.push_back(world_id_single);
    }

    if (world_ids_.empty()) {
      world_ids_ = {500};
    }
  }

  void loadToolIds() {
    tool_ids_.clear();
    XmlRpc::XmlRpcValue tool_ids_param;
    if (pnh_.hasParam("tool_ids") && pnh_.getParam("tool_ids", tool_ids_param)) {
      if (tool_ids_param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < tool_ids_param.size(); ++i) {
          if (tool_ids_param[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
            std::string id_str = static_cast<std::string>(tool_ids_param[i]);
            try {
              int id = std::stoi(id_str);
              tool_ids_.push_back(id);
            } catch (...) {
              ROS_WARN("[Relay] Failed to convert tool_id '%s' to integer", id_str.c_str());
            }
          } else if (tool_ids_param[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
            int id = static_cast<int>(tool_ids_param[i]);
            tool_ids_.push_back(id);
          }
        }
      } else if (tool_ids_param.getType() == XmlRpc::XmlRpcValue::TypeString) {
        std::string ids_str = static_cast<std::string>(tool_ids_param);
        std::istringstream iss(ids_str);
        std::string id_str;
        while (std::getline(iss, id_str, ',')) {
          try {
            id_str.erase(0, id_str.find_first_not_of(" "));
            id_str.erase(id_str.find_last_not_of(" ") + 1);
            int id = std::stoi(id_str);
            tool_ids_.push_back(id);
          } catch (...) {
            ROS_WARN("[Relay] Failed to convert tool_id '%s' to integer", id_str.c_str());
          }
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
