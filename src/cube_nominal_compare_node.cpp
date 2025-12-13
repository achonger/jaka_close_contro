#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

#include <fstream>
#include <map>
#include <string>
#include <vector>
#include <cmath>

namespace
{
geometry_msgs::Pose transformToPoseMsg(const tf2::Transform &tf)
{
  geometry_msgs::Pose out;
  const tf2::Vector3 &t = tf.getOrigin();
  out.position.x = t.x();
  out.position.y = t.y();
  out.position.z = t.z();
  tf2::Quaternion q = tf.getRotation();
  q.normalize();
  out.orientation = tf2::toMsg(q);
  return out;
}

struct StatResult
{
  double mean{0.0};
  double rms{0.0};
  double stddev{0.0};
  double max{0.0};
};

StatResult computeStats(const std::vector<double> &vals)
{
  StatResult out;
  if (vals.empty())
  {
    return out;
  }
  const double N = static_cast<double>(vals.size());
  double sum = 0.0;
  double sum_sq = 0.0;
  out.max = 0.0;
  for (double v : vals)
  {
    sum += v;
    sum_sq += v * v;
    if (v > out.max)
    {
      out.max = v;
    }
  }
  out.mean = sum / N;
  out.rms = std::sqrt(sum_sq / N);
  if (vals.size() > 1)
  {
    double var = 0.0;
    for (double v : vals)
    {
      const double d = v - out.mean;
      var += d * d;
    }
    var /= (N - 1.0);
    out.stddev = std::sqrt(var);
  }
  return out;
}
} // namespace

class CubeNominalCompareNode
{
public:
  CubeNominalCompareNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
      : nh_(nh), pnh_(pnh), tf_listener_(tf_buffer_)
  {
    pnh_.param<std::string>("robot_base_frame", robot_base_frame_, std::string("Link_0"));
    pnh_.param<std::string>("robot_tool_frame", robot_tool_frame_, std::string("Link_6"));
    pnh_.param<std::string>("camera_frame", camera_frame_, std::string("zed2i_left_camera_optical_frame"));
    pnh_.param<std::string>("cube_frame", cube_frame_, std::string("cube_center"));
    pnh_.param<std::string>("cube_topic", cube_topic_, std::string("/cube_center_fused"));
    pnh_.param<std::string>("output_csv", output_csv_, std::string(""));
    pnh_.param<bool>("publish_nominal_pose", publish_nominal_pose_, true);
    pnh_.param<bool>("publish_vision_pose", publish_vision_pose_, true);

    const std::string default_tool_offset_yaml = ros::package::getPath("jaka_close_contro") + "/config/tool_offset_current.yaml";
    pnh_.param<std::string>("tool_offset_yaml", tool_offset_yaml_path_, default_tool_offset_yaml);

    loadToolToCube();

    cube_sub_ = nh_.subscribe(cube_topic_, 1, &CubeNominalCompareNode::cubeCallback, this);
    if (publish_nominal_pose_)
    {
      nominal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/cube_center_nominal", 1);
    }
    if (publish_vision_pose_)
    {
      vision_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/cube_center_base_vision", 1);
    }

    if (!output_csv_.empty())
    {
      csv_.open(output_csv_, std::ios::out | std::ios::trunc);
      if (csv_.is_open())
      {
        csv_ << "stamp,pos_err_m,ang_err_deg,vision_x,vision_y,vision_z,nominal_x,nominal_y,nominal_z\n";
        ROS_INFO("[NominalCompare] CSV 输出: %s", output_csv_.c_str());
      }
      else
      {
        ROS_WARN("[NominalCompare] 无法打开 CSV: %s", output_csv_.c_str());
      }
    }

    ROS_INFO("[NominalCompare] base=%s tool=%s camera=%s cube=%s", robot_base_frame_.c_str(), robot_tool_frame_.c_str(),
             camera_frame_.c_str(), cube_frame_.c_str());
  }

private:
  void loadToolToCube()
  {
    try
    {
      YAML::Node root = YAML::LoadFile(tool_offset_yaml_path_);
      if (!root["tool_offset"] || !root["tool_offset"]["translation"] || !root["tool_offset"]["rotation"])
      {
        throw std::runtime_error("tool_offset 缺少 translation/rotation");
      }

      auto trans = root["tool_offset"]["translation"];
      auto rot = root["tool_offset"]["rotation"];
      if (trans.size() != 3 || rot.size() != 4)
      {
        throw std::runtime_error("tool_offset translation/rotation 尺寸不正确");
      }

      tf2::Quaternion q(rot[0].as<double>(), rot[1].as<double>(), rot[2].as<double>(), rot[3].as<double>());
      q.normalize();

      T_tool_cube_.setOrigin(tf2::Vector3(trans[0].as<double>(), trans[1].as<double>(), trans[2].as<double>()));
      T_tool_cube_.setRotation(q);

      ROS_INFO("[NominalCompare] 从 YAML 读取 T_tool_cube: trans[%.3f %.3f %.3f], quat[%.3f %.3f %.3f %.3f]",
               trans[0].as<double>(), trans[1].as<double>(), trans[2].as<double>(), q.x(), q.y(), q.z(), q.w());
    }
    catch (const std::exception &e)
    {
      tf2::Quaternion q_default(0, 0, 0, 1);
      T_tool_cube_.setOrigin(tf2::Vector3(0.0, 0.0, 0.077));
      T_tool_cube_.setRotation(q_default);
      ROS_WARN("[NominalCompare] 读取 tool_offset_yaml 失败 (%s)，使用默认 [0,0,0.077]", e.what());
    }
  }

  void cubeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    tf2::Transform T_cam_cube;
    tf2::fromMsg(msg->pose, T_cam_cube);
    T_cam_cube.getRotation().normalize();

    tf2::Transform T_base_tool;
    if (!lookupTransform(robot_base_frame_, robot_tool_frame_, msg->header.stamp, T_base_tool))
    {
      return;
    }

    tf2::Transform T_base_cube_nominal = T_base_tool * T_tool_cube_;

    tf2::Transform T_base_cam;
    std::string cam_frame = msg->header.frame_id.empty() ? camera_frame_ : msg->header.frame_id;
    if (!lookupTransform(robot_base_frame_, cam_frame, msg->header.stamp, T_base_cam))
    {
      // 尝试用视觉 tool 反推 base->cam
      tf2::Transform T_cam_tool;
      if (lookupTransform(cam_frame, robot_tool_frame_, msg->header.stamp, T_cam_tool))
      {
        T_base_cam = T_base_tool * T_cam_tool.inverse();
        ROS_WARN_THROTTLE(2.0, "[NominalCompare] 使用视觉 tool TF 反推 base->cam，仅供诊断");
      }
      else
      {
        ROS_WARN_THROTTLE(1.0, "[NominalCompare] 无法获取 base->cam TF，跳过本帧");
        return;
      }
    }

    tf2::Transform T_base_cube_vision = T_base_cam * T_cam_cube;

    const tf2::Vector3 dp = T_base_cube_nominal.getOrigin() - T_base_cube_vision.getOrigin();
    const double pos_err = dp.length();

    tf2::Quaternion q_nom = T_base_cube_nominal.getRotation();
    q_nom.normalize();
    tf2::Quaternion q_vis = T_base_cube_vision.getRotation();
    q_vis.normalize();
    tf2::Quaternion q_err = q_nom.inverse() * q_vis;
    q_err.normalize();
    const double ang_err = q_err.getAngle() * 180.0 / M_PI;

    pos_errs_.push_back(pos_err);
    ang_errs_.push_back(ang_err);

    const StatResult pos_stat = computeStats(pos_errs_);
    const StatResult ang_stat = computeStats(ang_errs_);
    ROS_INFO_STREAM("[NominalCompare] pos_err=" << pos_err << " m, ang_err=" << ang_err << " deg"
                                                << " | RMS(pos/ang)=" << pos_stat.rms << " m / " << ang_stat.rms << " deg"
                                                << " | max(pos/ang)=" << pos_stat.max << " m / " << ang_stat.max << " deg");

    if (publish_nominal_pose_)
    {
      geometry_msgs::PoseStamped nominal_msg;
      nominal_msg.header.frame_id = robot_base_frame_;
      nominal_msg.header.stamp = msg->header.stamp;
      nominal_msg.pose = transformToPoseMsg(T_base_cube_nominal);
      nominal_pub_.publish(nominal_msg);
    }
    if (publish_vision_pose_)
    {
      geometry_msgs::PoseStamped vis_msg;
      vis_msg.header.frame_id = robot_base_frame_;
      vis_msg.header.stamp = msg->header.stamp;
      vis_msg.pose = transformToPoseMsg(T_base_cube_vision);
      vision_pub_.publish(vis_msg);
    }

    if (csv_.is_open())
    {
      csv_ << msg->header.stamp.toSec() << "," << pos_err << "," << ang_err << ","
           << T_base_cube_vision.getOrigin().x() << "," << T_base_cube_vision.getOrigin().y() << ","
           << T_base_cube_vision.getOrigin().z() << "," << T_base_cube_nominal.getOrigin().x() << ","
           << T_base_cube_nominal.getOrigin().y() << "," << T_base_cube_nominal.getOrigin().z() << "\n";
    }
  }

  bool lookupTransform(const std::string &target, const std::string &source, const ros::Time &stamp, tf2::Transform &out)
  {
    try
    {
      geometry_msgs::TransformStamped tf_msg = tf_buffer_.lookupTransform(target, source, stamp, ros::Duration(0.2));
      tf2::fromMsg(tf_msg.transform, out);
      out.getRotation().normalize();
      return true;
    }
    catch (const tf2::TransformException &ex)
    {
      ROS_WARN_THROTTLE(1.0, "[NominalCompare] TF %s -> %s 失败: %s", target.c_str(), source.c_str(), ex.what());
      return false;
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber cube_sub_;
  ros::Publisher nominal_pub_;
  ros::Publisher vision_pub_;

  std::string robot_base_frame_;
  std::string robot_tool_frame_;
  std::string camera_frame_;
  std::string cube_frame_;
  std::string cube_topic_;
  std::string output_csv_;
  std::string tool_offset_yaml_path_;
  bool publish_nominal_pose_{true};
  bool publish_vision_pose_{true};

  tf2::Transform T_tool_cube_;

  std::vector<double> pos_errs_;
  std::vector<double> ang_errs_;

  std::ofstream csv_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cube_nominal_compare_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try
  {
    CubeNominalCompareNode node(nh, pnh);
    ros::spin();
  }
  catch (const std::exception &e)
  {
    ROS_FATAL("[NominalCompare] 节点启动失败: %s", e.what());
    return 1;
  }
  return 0;
}

