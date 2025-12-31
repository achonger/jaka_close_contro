#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <jaka_msgs/Move.h>
#include <jaka_close_contro/SetPoseTarget.h>
#include <jaka_close_contro/GetCubePoseWorld.h>

#include <string>
#include <boost/optional.hpp>
#include <sstream>

namespace
{
struct CalibData
{
  Eigen::Isometry3d T_world_base{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d T_tool_to_cube{Eigen::Isometry3d::Identity()};
};

Eigen::Isometry3d poseMsgToIso(const geometry_msgs::Pose &p)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  q.normalize();
  T.linear() = q.toRotationMatrix();
  return T;
}

geometry_msgs::Pose isoToPoseMsg(const Eigen::Isometry3d &T)
{
  geometry_msgs::Pose p;
  p.position.x = T.translation().x();
  p.position.y = T.translation().y();
  p.position.z = T.translation().z();
  Eigen::Quaterniond q(T.rotation());
  q.normalize();
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

Eigen::Vector3d logSO3(const Eigen::Matrix3d &R)
{
  double cos_theta = (R.trace() - 1.0) * 0.5;
  cos_theta = std::min(1.0, std::max(-1.0, cos_theta));
  double theta = std::acos(cos_theta);
  if (theta < 1e-9)
  {
    return Eigen::Vector3d::Zero();
  }
  Eigen::Vector3d omega;
  omega << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
  omega *= 0.5 * theta / std::sin(theta);
  return omega;
}

Eigen::Isometry3d expSE3(const Eigen::Matrix<double, 6, 1> &xi)
{
  Eigen::Vector3d v = xi.head<3>();
  Eigen::Vector3d w = xi.tail<3>();
  double theta = w.norm();
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d V = Eigen::Matrix3d::Identity();
  if (theta > 1e-9)
  {
    Eigen::Matrix3d wx;
    wx << 0, -w.z(), w.y(),
        w.z(), 0, -w.x(),
        -w.y(), w.x(), 0;
    R = Eigen::AngleAxisd(theta, w.normalized()).toRotationMatrix();
    V = Eigen::Matrix3d::Identity() + (1 - std::cos(theta)) / (theta * theta) * wx +
        (theta - std::sin(theta)) / (theta * theta * theta) * (wx * wx);
  }
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = R;
  T.translation() = V * v;
  return T;
}

Eigen::Matrix<double, 6, 1> computeError(const Eigen::Isometry3d &T_est, const Eigen::Isometry3d &T_des)
{
  Eigen::Isometry3d T_err = T_est.inverse() * T_des;
  Eigen::Vector3d ep = T_err.translation();
  Eigen::Vector3d etheta = logSO3(T_err.rotation());
  Eigen::Matrix<double, 6, 1> xi;
  xi.head<3>() = ep;
  xi.tail<3>() = etheta;
  return xi;
}

boost::optional<CalibData> loadCalib(const std::string &path)
{
  try
  {
    ROS_INFO("[Calib] loading calib yaml: %s", path.c_str());
    YAML::Node root = YAML::LoadFile(path);
    auto rootKeysStr = [&root]() {
      std::stringstream ss;
      bool first = true;
      for (const auto &kv : root)
      {
        if (!first)
        {
          ss << ", ";
        }
        first = false;
        ss << kv.first.as<std::string>();
      }
      return ss.str();
    };
    CalibData out;

    YAML::Node node_wb;
    std::string branch_used;
    std::vector<std::string> quat_keys;
    if (root["world_robot_extrinsic_offline"])
    {
      node_wb = root["world_robot_extrinsic_offline"];
      branch_used = "world_robot_extrinsic_offline";
      quat_keys = {"quat_xyzw", "rotation_xyzw"};
    }
    else if (root["extrinsic"])
    {
      node_wb = root["extrinsic"];
      branch_used = "extrinsic";
      quat_keys = {"rotation_xyzw", "quat_xyzw"};
    }
    else
    {
      ROS_ERROR("calib yaml missing world_robot_extrinsic_offline/extrinsic; root keys: %s", rootKeysStr().c_str());
      return boost::none;
    }
    ROS_INFO("[Calib] using %s branch for T_world_base", branch_used.c_str());

    if (!node_wb["translation"])
    {
      ROS_ERROR("calib yaml missing %s.translation; root keys: %s", branch_used.c_str(), rootKeysStr().c_str());
      return boost::none;
    }
    std::vector<double> t = node_wb["translation"].as<std::vector<double>>();
    if (t.size() != 3)
    {
      ROS_ERROR("calib yaml %s.translation length=%zu expected=3", branch_used.c_str(), t.size());
      ROS_ERROR("root keys: %s", rootKeysStr().c_str());
      return boost::none;
    }
    std::vector<double> q;
    std::string q_key_used;
    for (const auto &k : quat_keys)
    {
      if (node_wb[k])
      {
        q = node_wb[k].as<std::vector<double>>();
        q_key_used = k;
        break;
      }
    }
    if (q_key_used.empty())
    {
      ROS_ERROR("calib yaml missing quaternion (%s)", branch_used.c_str());
      ROS_ERROR("root keys: %s", rootKeysStr().c_str());
      return boost::none;
    }
    if (q.size() != 4)
    {
      ROS_ERROR("calib yaml %s.%s length=%zu expected=4", branch_used.c_str(), q_key_used.c_str(), q.size());
      ROS_ERROR("root keys: %s", rootKeysStr().c_str());
      return boost::none;
    }
    Eigen::Isometry3d T_wb = Eigen::Isometry3d::Identity();
    T_wb.translation() = Eigen::Vector3d(t[0], t[1], t[2]);
    Eigen::Quaterniond q_wb(q[3], q[0], q[1], q[2]);
    q_wb.normalize();
    T_wb.linear() = q_wb.toRotationMatrix();
    out.T_world_base = T_wb;

    YAML::Node node_tc;
    std::string tool_source;
    if (root["tool_extrinsic_tool_to_cube"])
    {
      node_tc = root["tool_extrinsic_tool_to_cube"];
      tool_source = "root";
    }
    else if (node_wb["tool_extrinsic_tool_to_cube"])
    {
      node_tc = node_wb["tool_extrinsic_tool_to_cube"];
      tool_source = branch_used;
    }

    if (node_tc)
    {
      ROS_INFO("[Calib] tool_extrinsic_tool_to_cube found at %s level", tool_source.c_str());
      std::vector<double> t_tc;
      std::vector<double> q_tc;
      std::string t_key_used;
      std::string q_key_used_tc;
      if (node_tc["translation_final_m"])
      {
        t_tc = node_tc["translation_final_m"].as<std::vector<double>>();
        t_key_used = "translation_final_m";
      }
      else if (node_tc["translation_init_m"])
      {
        t_tc = node_tc["translation_init_m"].as<std::vector<double>>();
        t_key_used = "translation_init_m";
      }
      if (node_tc["rotation_final_xyzw"])
      {
        q_tc = node_tc["rotation_final_xyzw"].as<std::vector<double>>();
        q_key_used_tc = "rotation_final_xyzw";
      }
      else if (node_tc["rotation_init_xyzw"])
      {
        q_tc = node_tc["rotation_init_xyzw"].as<std::vector<double>>();
        q_key_used_tc = "rotation_init_xyzw";
      }
      if (t_tc.size() == 3 && q_tc.size() == 4)
      {
        Eigen::Isometry3d T_tc = Eigen::Isometry3d::Identity();
        T_tc.translation() = Eigen::Vector3d(t_tc[0], t_tc[1], t_tc[2]);
        Eigen::Quaterniond qtc(q_tc[3], q_tc[0], q_tc[1], q_tc[2]);
        qtc.normalize();
        T_tc.linear() = qtc.toRotationMatrix();
        out.T_tool_to_cube = T_tc;
      }
      else
      {
        ROS_WARN("tool_extrinsic_tool_to_cube malformed (t_key=%s len=%zu expected=3, q_key=%s len=%zu expected=4), using identity",
                 t_key_used.c_str(), t_tc.size(), q_key_used_tc.c_str(), q_tc.size());
      }
    }
    else
    {
      ROS_WARN("tool_extrinsic_tool_to_cube not found at root or %s level, using identity", branch_used.c_str());
    }
    return out;
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("Failed to load calib yaml %s: %s", path.c_str(), e.what());
    return boost::none;
  }
}
} // namespace

class PoseServoWorld
{
public:
  explicit PoseServoWorld(ros::NodeHandle &nh, ros::NodeHandle &pnh)
      : nh_(nh), pnh_(pnh)
  {
    pnh_.param<std::string>("robot_name", robot_name_, std::string("jaka1"));
    pnh_.param<int>("robot_id", robot_id_, 1);
    pnh_.param<bool>("enable", enable_, true);
    pnh_.param<bool>("connect_robot", connect_robot_, true);
    pnh_.param<double>("control_rate_hz", control_rate_hz_, 20.0);
    pnh_.param<double>("kp_pos", kp_pos_, 0.6);
    pnh_.param<double>("kp_ang", kp_ang_, 0.8);
    pnh_.param<double>("v_max", v_max_, 0.02);
    pnh_.param<double>("w_max_deg", w_max_deg_, 10.0);
    pnh_.param<double>("eps_pos_m", eps_pos_m_, 0.0015);
    pnh_.param<double>("eps_ang_deg", eps_ang_deg_, 0.5);
    pnh_.param<double>("vision_timeout_s", vision_timeout_s_, 0.2);
    pnh_.param<double>("servo_timeout_s", servo_timeout_s_, 10.0);
    pnh_.param<double>("query_timeout_s", query_timeout_s_, 1.0);
    pnh_.param<std::string>("linear_move_service", linear_move_service_, std::string("jaka_driver/linear_move"));
    pnh_.param<std::string>("world_frame", world_frame_, std::string("world"));
    pnh_.param<bool>("strict_cube_frame", strict_cube_frame_, true);
    pnh_.param<std::string>("cube_pose_topic", cube_pose_topic_,
                            std::string("/vision/" + robot_name_ + "/cube_center_world"));
    pnh_.param<std::string>("target_topic", target_topic_, std::string("target_tool_world"));
    std::string calib_yaml;
    pnh_.param<std::string>("calib_yaml", calib_yaml,
                            std::string("$(find jaka_close_contro)/config/world_robot_extrinsic_offline_" + robot_name_ + ".yaml"));

    w_max_rad_ = w_max_deg_ * M_PI / 180.0;
    eps_ang_rad_ = eps_ang_deg_ * M_PI / 180.0;

    // Resolve calib path (expand package keyword if present)
    if (calib_yaml.find("$(find") != std::string::npos)
    {
      std::string pkg_path = ros::package::getPath("jaka_close_contro");
      std::string token = "$(find jaka_close_contro)";
      size_t pos = calib_yaml.find(token);
      if (pos != std::string::npos)
      {
        calib_yaml.replace(pos, token.size(), pkg_path);
      }
    }
    boost::optional<CalibData> calib = loadCalib(calib_yaml);
    if (!calib)
    {
      throw std::runtime_error("calibration load failed");
    }
    T_world_base_ = calib->T_world_base;
    T_tool_to_cube_ = calib->T_tool_to_cube;

    ROS_INFO("[PoseServo] robot=%s id=%d enable=%s connect_robot=%s", robot_name_.c_str(), robot_id_,
             enable_ ? "true" : "false", connect_robot_ ? "true" : "false");
    ROS_INFO("[PoseServo] linear_move_service=%s", linear_move_service_.c_str());
    ROS_INFO("[PoseServo] T_world_base t=[%.4f %.4f %.4f]", T_world_base_.translation().x(),
             T_world_base_.translation().y(), T_world_base_.translation().z());
    Eigen::Quaterniond qwb(T_world_base_.rotation());
    ROS_INFO("[PoseServo] T_world_base q=[%.4f %.4f %.4f %.4f]", qwb.x(), qwb.y(), qwb.z(), qwb.w());
    ROS_INFO("[PoseServo] T_tool_to_cube t=[%.4f %.4f %.4f]", T_tool_to_cube_.translation().x(),
             T_tool_to_cube_.translation().y(), T_tool_to_cube_.translation().z());
    Eigen::Quaterniond qtc(T_tool_to_cube_.rotation());
    ROS_INFO("[PoseServo] T_tool_to_cube q=[%.4f %.4f %.4f %.4f]", qtc.x(), qtc.y(), qtc.z(), qtc.w());
    ROS_INFO("[PoseServo] cube_pose_topic=%s target_topic=%s", cube_pose_topic_.c_str(), target_topic_.c_str());

    cube_sub_ = nh_.subscribe(cube_pose_topic_, 1, &PoseServoWorld::cubeCb, this);
    target_sub_ = pnh_.subscribe(target_topic_, 1, &PoseServoWorld::targetCb, this);
    status_pub_ = pnh_.advertise<std_msgs::String>("status", 1, true);
    err_pub_ = pnh_.advertise<geometry_msgs::Twist>("err", 1, true);

    set_target_srv_ = pnh_.advertiseService("set_target", &PoseServoWorld::setTargetSrv, this);
    stop_srv_ = pnh_.advertiseService("stop", &PoseServoWorld::stopSrv, this);
    clear_target_srv_ = pnh_.advertiseService("clear_target", &PoseServoWorld::clearTargetSrv, this);
    get_cube_pose_srv_ = pnh_.advertiseService("get_cube_pose_world", &PoseServoWorld::getCubePoseSrv, this);

    linear_move_client_ = nh_.serviceClient<jaka_msgs::Move>(linear_move_service_);

    control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_hz_), &PoseServoWorld::controlTimer, this);
  }

private:
  enum class State
  {
    IDLE,
    RUN,
    HOLD,
    DISCONNECTED
  };

  void cubeCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    if (msg->header.frame_id != world_frame_)
    {
      if (strict_cube_frame_)
      {
        ROS_WARN_THROTTLE(1.0, "[PoseServo] cube pose frame_id=%s (expected %s); dropping message",
                          msg->header.frame_id.c_str(), world_frame_.c_str());
        return;
      }
      else
      {
        ROS_WARN_THROTTLE(1.0, "[PoseServo] cube pose frame_id=%s (expected %s)", msg->header.frame_id.c_str(),
                          world_frame_.c_str());
      }
    }
    last_cube_pose_ = *msg;
    have_cube_pose_ = true;
    last_cube_rx_wall_ = ros::WallTime::now();
    if (last_cube_pose_.header.stamp.isZero())
    {
      ros::Time now = ros::Time::now();
      if (!now.isZero())
      {
        last_cube_pose_.header.stamp = now;
      }
    }
  }

  void targetCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    if (msg->header.frame_id != world_frame_)
    {
      ROS_WARN_THROTTLE(1.0, "[PoseServo] target frame_id=%s (expected %s)", msg->header.frame_id.c_str(),
                        world_frame_.c_str());
      return;
    }
    target_tool_world_ = *msg;
    target_active_ = true;
    target_start_time_ = ros::Time::now();
    state_ = State::RUN;
  }

  bool setTargetSrv(jaka_close_contro::SetPoseTarget::Request &req,
                    jaka_close_contro::SetPoseTarget::Response &res)
  {
    if (req.target.header.frame_id != world_frame_ && !req.target.header.frame_id.empty())
    {
      res.ok = false;
      res.message = "frame_id must match world_frame";
      return true;
    }
    target_tool_world_ = req.target;
    if (target_tool_world_.header.stamp == ros::Time(0))
    {
      target_tool_world_.header.stamp = ros::Time::now();
    }
    target_active_ = true;
    target_start_time_ = ros::Time::now();
    state_ = State::RUN;
    res.ok = true;
    res.message = "target accepted";
    return true;
  }

  bool stopSrv(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
  {
    state_ = State::HOLD;
    target_active_ = false;
    return true;
  }

  bool clearTargetSrv(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
  {
    target_active_ = false;
    state_ = State::IDLE;
    return true;
  }

  bool getCubePoseSrv(jaka_close_contro::GetCubePoseWorld::Request &,
                      jaka_close_contro::GetCubePoseWorld::Response &res)
  {
    double age_sec = 0.0;
    if (!computeCubeAgeSec(age_sec))
    {
      res.ok = false;
      res.message = "no cube pose received";
      res.age_sec = -1.0;
      return true;
    }
    res.pose = last_cube_pose_;
    res.age_sec = age_sec;
    if (res.age_sec > query_timeout_s_)
    {
      res.ok = false;
      res.message = "cube pose stale";
    }
    else
    {
      res.ok = true;
      res.message = "ok";
    }
    return true;
  }

  void publishStatus(const Eigen::Vector3d &ep, const Eigen::Vector3d &etheta, double age)
  {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "state=" << stateToString(state_) << " ep=" << ep.norm() << " etheta=" << etheta.norm()
       << " cube_age=" << age;
    msg.data = ss.str();
    status_pub_.publish(msg);

    geometry_msgs::Twist err;
    err.linear.x = ep.x();
    err.linear.y = ep.y();
    err.linear.z = ep.z();
    err.angular.x = etheta.x();
    err.angular.y = etheta.y();
    err.angular.z = etheta.z();
    err_pub_.publish(err);
  }

  std::string stateToString(State s) const
  {
    switch (s)
    {
    case State::IDLE:
      return "IDLE";
    case State::RUN:
      return "RUN";
    case State::HOLD:
      return "HOLD";
    case State::DISCONNECTED:
      return "DISCONNECTED";
    }
    return "UNKNOWN";
  }

  bool computeCubeAgeSec(double &age_out) const
  {
    if (!have_cube_pose_)
    {
      age_out = 1e9;
      return false;
    }
    ros::Time now = ros::Time::now();
    if (!now.isZero() && !last_cube_pose_.header.stamp.isZero())
    {
      age_out = (now - last_cube_pose_.header.stamp).toSec();
    }
    else
    {
      age_out = (ros::WallTime::now() - last_cube_rx_wall_).toSec();
    }
    return true;
  }

  bool visionTimedOut(double &age_out) const
  {
    computeCubeAgeSec(age_out);
    return age_out > vision_timeout_s_;
  }

  bool sendCommand(const Eigen::Isometry3d &T_base_tool)
  {
    if (!connect_robot_)
    {
      return true;
    }
    if (!linear_move_client_.exists())
    {
      ROS_WARN_THROTTLE(1.0, "[PoseServo] robot driver not available; entering DISCONNECTED");
      state_ = State::DISCONNECTED;
      return false;
    }
    if (!linear_move_client_.exists())
    {
      linear_move_client_.waitForExistence(ros::Duration(0.1));
    }
    Eigen::Quaterniond q(T_base_tool.rotation());
    q.normalize();
    tf2::Quaternion q_tf(q.x(), q.y(), q.z(), q.w());
    tf2::Matrix3x3 R(q_tf);
    double roll, pitch, yaw;
    R.getRPY(roll, pitch, yaw);

    jaka_msgs::Move srv;
    srv.request.pose = {static_cast<float>(T_base_tool.translation().x() * 1000.0),
                        static_cast<float>(T_base_tool.translation().y() * 1000.0),
                        static_cast<float>(T_base_tool.translation().z() * 1000.0),
                        static_cast<float>(roll),
                        static_cast<float>(pitch),
                        static_cast<float>(yaw)};
    srv.request.mvvelo = static_cast<float>(v_max_ * 1000.0);  // mm/s
    srv.request.mvacc = static_cast<float>(v_max_ * 1000.0 * 2); // rough acc
    srv.request.mvtime = 0.0;
    srv.request.mvradii = 0.0;
    srv.request.coord_mode = 0;
    srv.request.index = 0;

    if (!linear_move_client_.call(srv))
    {
      ROS_WARN_THROTTLE(1.0, "[PoseServo] linear_move call failed");
      state_ = State::DISCONNECTED;
      return false;
    }
    if (srv.response.ret != 0)
    {
      ROS_WARN_THROTTLE(1.0, "[PoseServo] linear_move ret=%d message=%s", srv.response.ret,
                        srv.response.message.c_str());
      state_ = State::DISCONNECTED;
      return false;
    }
    state_ = State::RUN;
    return true;
  }

  void controlTimer(const ros::TimerEvent &)
  {
    double cube_age = 0.0;
    if (visionTimedOut(cube_age))
    {
      if (state_ == State::RUN)
      {
        state_ = State::HOLD;
        ROS_WARN_THROTTLE(1.0, "[PoseServo] vision timeout, holding commands");
      }
      publishStatus(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), cube_age);
      return;
    }
    if (!target_active_)
    {
      publishStatus(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), cube_age);
      return;
    }
    if (state_ == State::DISCONNECTED || !enable_)
    {
      publishStatus(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), cube_age);
      return;
    }

    if ((ros::Time::now() - target_start_time_).toSec() > servo_timeout_s_)
    {
      ROS_WARN_THROTTLE(1.0, "[PoseServo] servo timeout, holding");
      state_ = State::HOLD;
      publishStatus(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), cube_age);
      return;
    }

    Eigen::Isometry3d T_world_cube_est = poseMsgToIso(last_cube_pose_.pose);
    Eigen::Isometry3d T_world_tool_des = poseMsgToIso(target_tool_world_.pose);

    Eigen::Isometry3d T_world_cube_des = T_world_tool_des * T_tool_to_cube_;
    Eigen::Matrix<double, 6, 1> xi = computeError(T_world_cube_est, T_world_cube_des);
    Eigen::Vector3d ep = xi.head<3>();
    Eigen::Vector3d etheta = xi.tail<3>();

    if (ep.norm() < eps_pos_m_ && etheta.norm() < eps_ang_rad_)
    {
      state_ = State::IDLE;
      target_active_ = false;
      publishStatus(ep, etheta, cube_age);
      return;
    }

    Eigen::Vector3d v_cmd = kp_pos_ * ep;
    Eigen::Vector3d w_cmd = kp_ang_ * etheta;
    if (v_cmd.norm() > v_max_)
    {
      v_cmd = v_cmd.normalized() * v_max_;
    }
    if (w_cmd.norm() > w_max_rad_)
    {
      w_cmd = w_cmd.normalized() * w_max_rad_;
    }

    Eigen::Matrix<double, 6, 1> xi_cmd;
    xi_cmd.head<3>() = v_cmd;
    xi_cmd.tail<3>() = w_cmd;
    double dt = 1.0 / control_rate_hz_;
    Eigen::Isometry3d Delta = expSE3(xi_cmd * dt);
    Eigen::Isometry3d T_world_cube_next = T_world_cube_est * Delta;
    Eigen::Isometry3d T_cube_to_tool = T_tool_to_cube_.inverse();
    Eigen::Isometry3d T_world_tool_next = T_world_cube_next * T_cube_to_tool;
    Eigen::Isometry3d T_base_tool_next = T_world_base_.inverse() * T_world_tool_next;

    sendCommand(T_base_tool_next);
    last_command_time_ = ros::Time::now();
    publishStatus(ep, etheta, cube_age);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::string robot_name_;
  int robot_id_{1};
  bool enable_{true};
  bool connect_robot_{true};
  double control_rate_hz_{20.0};
  double kp_pos_{0.6};
  double kp_ang_{0.8};
  double v_max_{0.02};
  double w_max_deg_{10.0};
  double w_max_rad_{10.0 * M_PI / 180.0};
  double eps_pos_m_{0.0015};
  double eps_ang_deg_{0.5};
  double eps_ang_rad_{0.5 * M_PI / 180.0};
  double vision_timeout_s_{0.2};
  double servo_timeout_s_{10.0};
  double query_timeout_s_{1.0};
  std::string linear_move_service_{"jaka_driver/linear_move"};
  std::string cube_pose_topic_;
  std::string target_topic_;
  std::string world_frame_{"world"};
  bool strict_cube_frame_{true};

  ros::Subscriber cube_sub_;
  ros::Subscriber target_sub_;
  ros::Publisher status_pub_;
  ros::Publisher err_pub_;
  ros::ServiceServer set_target_srv_;
  ros::ServiceServer stop_srv_;
  ros::ServiceServer clear_target_srv_;
  ros::ServiceServer get_cube_pose_srv_;
  ros::ServiceClient linear_move_client_;
  ros::Timer control_timer_;

  geometry_msgs::PoseStamped last_cube_pose_;
  geometry_msgs::PoseStamped target_tool_world_;
  bool target_active_{false};
  ros::Time target_start_time_;
  ros::Time last_command_time_;
  bool have_cube_pose_{false};
  ros::WallTime last_cube_rx_wall_;

  Eigen::Isometry3d T_world_base_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d T_tool_to_cube_{Eigen::Isometry3d::Identity()};

  State state_{State::IDLE};
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_servo_world_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  try
  {
    PoseServoWorld node(nh, pnh);
    ros::spin();
  }
  catch (const std::exception &e)
  {
    ROS_FATAL("Failed to start pose_servo_world: %s", e.what());
    return 1;
  }
  return 0;
}
