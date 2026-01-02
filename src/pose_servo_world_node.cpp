#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sensor_msgs/JointState.h>
#include <limits>
#include <jaka_msgs/Move.h>
#include <jaka_close_contro/SetPoseTarget.h>
#include <jaka_close_contro/GetCubePoseWorld.h>

#include <string>
#include <boost/optional.hpp>
#include <sstream>
#include <mutex>

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
    pnh_.param<std::string>("joint_state_topic", joint_state_topic_, std::string("joint_states"));
    pnh_.param<double>("motion_joint_threshold_rad", motion_joint_threshold_rad_, 0.002);
    pnh_.param<double>("motion_stable_duration_sec", motion_stable_duration_sec_, 0.5);
    pnh_.param<double>("motion_done_timeout_sec", motion_done_timeout_sec_, 8.0);
    pnh_.param<double>("settle_after_motion_sec", settle_after_motion_sec_, 0.1);
    pnh_.param<double>("send_rate_hz", send_rate_hz_, 5.0);
    pnh_.param<double>("resume_factor", resume_factor_, 2.0);
    pnh_.param<double>("dt_cmd_min", dt_cmd_min_, 0.05);
    pnh_.param<double>("dt_cmd_max", dt_cmd_max_, 1.0);
    pnh_.param<std::string>("dt_cmd_mode", dt_cmd_mode_, std::string("elapsed"));
    pnh_.param<bool>("use_step_limits", use_step_limits_, true);
    pnh_.param<double>("step_pos_max_m", step_pos_max_m_, 0.01);
    pnh_.param<double>("step_ang_max_deg", step_ang_max_deg_, 3.0);
    pnh_.param<double>("step_pos_min_m", step_pos_min_m_, 0.0005);
    pnh_.param<double>("step_ang_min_deg", step_ang_min_deg_, 0.2);
    pnh_.param<double>("progress_timeout_s", progress_timeout_s_, servo_timeout_s_);
    pnh_.param<double>("progress_eps_pos_m", progress_eps_pos_m_, 0.002);
    pnh_.param<double>("progress_eps_ang_deg", progress_eps_ang_deg_, 0.5);
    progress_eps_ang_rad_ = progress_eps_ang_deg_ * M_PI / 180.0;
    pnh_.param<double>("progress_improve_ratio", progress_improve_ratio_, 0.98);
    pnh_.param<std::string>("cube_pose_topic", cube_pose_topic_,
                            std::string("/vision/" + robot_name_ + "/cube_center_world"));
    pnh_.param<std::string>("target_topic", target_topic_, std::string("target_tool_world"));
    std::string calib_yaml;
    pnh_.param<std::string>("calib_yaml", calib_yaml,
                            std::string("$(find jaka_close_contro)/config/world_robot_extrinsic_offline_" + robot_name_ + ".yaml"));

    pnh_.param<bool>("use_two_stage", use_two_stage_, true);
    pnh_.param<double>("coarse_v_max", coarse_v_max_, 0.08);
    pnh_.param<double>("coarse_w_max_deg", coarse_w_max_deg_, 30.0);
    coarse_w_max_rad_ = coarse_w_max_deg_ * M_PI / 180.0;
    pnh_.param<double>("coarse_pos_gate_m", coarse_pos_gate_m_, 0.03);
    pnh_.param<double>("coarse_ang_gate_deg", coarse_ang_gate_deg_, 15.0);
    coarse_ang_gate_rad_ = coarse_ang_gate_deg_ * M_PI / 180.0;
    pnh_.param<double>("coarse_approach_dist_m", coarse_approach_dist_m_, 0.08);
    pnh_.param<std::string>("coarse_approach_axis", coarse_approach_axis_, std::string("tool_-z"));
    pnh_.param<bool>("coarse_keep_current_orientation", coarse_keep_current_orientation_, true);

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
    joint_sub_ = nh_.subscribe(joint_state_topic_, 1, &PoseServoWorld::jointCb, this);
    status_pub_ = pnh_.advertise<std_msgs::String>("status", 1, true);
    err_pub_ = pnh_.advertise<geometry_msgs::Twist>("err", 1, true);
    reached_pub_ = pnh_.advertise<std_msgs::Bool>("reached", 1, true);
    result_pub_ = pnh_.advertise<std_msgs::String>("result", 1, true);
    last_cmd_pub_ = pnh_.advertise<geometry_msgs::PoseStamped>("last_cmd_base_tool", 1, true);

    set_target_srv_ = pnh_.advertiseService("set_target", &PoseServoWorld::setTargetSrv, this);
    stop_srv_ = pnh_.advertiseService("stop", &PoseServoWorld::stopSrv, this);
    clear_target_srv_ = pnh_.advertiseService("clear_target", &PoseServoWorld::clearTargetSrv, this);
    get_cube_pose_srv_ = pnh_.advertiseService("get_cube_pose_world", &PoseServoWorld::getCubePoseSrv, this);

    linear_move_client_ = nh_.serviceClient<jaka_msgs::Move>(linear_move_service_);

    update_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_hz_), &PoseServoWorld::updateTimer, this);
    send_timer_ = nh_.createTimer(ros::Duration(1.0 / send_rate_hz_), &PoseServoWorld::sendTimer, this);
  }

private:
  enum class State
  {
    IDLE,
    SETTLING,
    MOVING,
    RUN,
    HOLD,
    DISCONNECTED
  };

  enum class Phase
  {
    COARSE_APPROACH,
    COARSE_FEEDFORWARD,
    FINE
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

  void jointCb(const sensor_msgs::JointState::ConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    if (msg->position.size() < 6)
    {
      ROS_WARN_THROTTLE(5.0, "[PoseServo] joint_state position size < 6");
      return;
    }
    if (!has_prev_joint_)
    {
      prev_joint_pos_ = msg->position;
      has_prev_joint_ = true;
      last_motion_time_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    }
    double max_diff = 0.0;
    for (size_t i = 0; i < 6; ++i)
    {
      max_diff = std::max(max_diff, std::abs(msg->position[i] - prev_joint_pos_[i]));
    }
    if (max_diff > motion_joint_threshold_rad_)
    {
      last_motion_time_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
      motion_stable_start_ = ros::Time(0);
    }
    prev_joint_pos_ = msg->position;
    last_joint_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    have_joint_state_ = true;
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
    goal_reached_ = false;
    best_ep_norm_ = std::numeric_limits<double>::infinity();
    best_etheta_norm_ = std::numeric_limits<double>::infinity();
    last_progress_time_ = ros::Time::now();
    state_ = State::RUN;
    phase_ = use_two_stage_ ? Phase::COARSE_APPROACH : Phase::FINE;
    coarse_ff_sent_ = false;
    coarse_done_reported_ = false;
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
    goal_reached_ = false;
    best_ep_norm_ = std::numeric_limits<double>::infinity();
    best_etheta_norm_ = std::numeric_limits<double>::infinity();
    last_progress_time_ = ros::Time::now();
    state_ = State::RUN;
    phase_ = use_two_stage_ ? Phase::COARSE_APPROACH : Phase::FINE;
    coarse_ff_sent_ = false;
    coarse_done_reported_ = false;
    res.ok = true;
    res.message = "target accepted";
    return true;
  }

  bool stopSrv(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
  {
    state_ = State::HOLD;
    target_active_ = false;
    publishReached(false, "STOPPED");
    return true;
  }

  bool clearTargetSrv(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
  {
    target_active_ = false;
    state_ = State::IDLE;
    goal_reached_ = false;
    publishReached(false, "STOPPED");
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
    last_ep_ = ep;
    last_etheta_ = etheta;
    std_msgs::String msg;
    std::stringstream ss;
    double now = ros::Time::now().toSec();
    double last_step_age = last_step_time_.isZero() ? -1.0 : now - last_step_time_.toSec();
    double last_progress_age = last_progress_time_.isZero() ? -1.0 : now - last_progress_time_.toSec();
    ss << "state=" << stateToString(state_) << " ep=" << ep.norm() << " etheta=" << etheta.norm()
       << " cube_age=" << age << " dt_cmd_used=" << last_dt_cmd_used_
       << " cmd_seq=" << command_seq_ << " last_step_age=" << last_step_age
       << " last_progress_age=" << last_progress_age << " last_move_ret=" << last_move_ret_
       << " phase=" << phaseToString(phase_);
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
    case State::SETTLING:
      return "SETTLING";
    case State::MOVING:
      return "MOVING";
    case State::RUN:
      return "RUN";
    case State::HOLD:
      return "HOLD";
    case State::DISCONNECTED:
      return "DISCONNECTED";
    }
    return "UNKNOWN";
  }

  std::string phaseToString(Phase p) const
  {
    switch (p)
    {
    case Phase::COARSE_APPROACH:
      return "COARSE_APPROACH";
    case Phase::COARSE_FEEDFORWARD:
      return "COARSE_FEEDFORWARD";
    case Phase::FINE:
      return "FINE";
    }
    return "UNKNOWN";
  }

  enum class SendResult
  {
    OK,
    NO_SERVICE,
    CALL_FAIL,
    RET_FAIL
  };

  void publishReached(bool reached, const std::string &result)
  {
    std_msgs::Bool b;
    b.data = reached;
    reached_pub_.publish(b);
    std_msgs::String r;
    r.data = result;
    result_pub_.publish(r);
    last_result_ = result;
  }

  void publishLastCmd(const Eigen::Isometry3d &T_base_tool, uint64_t seq)
  {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base";
    msg.pose = isoToPoseMsg(T_base_tool);
    last_cmd_pub_.publish(msg);
    last_cmd_seq_ = seq;
  }

  bool motionStable(const ros::Time &now)
  {
    if (!have_joint_state_)
      return false;
    if (last_motion_time_.isZero())
      return false;
    double since_move = (now - last_motion_time_).toSec();
    if (since_move < motion_stable_duration_sec_)
    {
      motion_stable_start_ = ros::Time(0);
      return false;
    }
    if (motion_stable_start_.isZero())
    {
      motion_stable_start_ = now;
      return false;
    }
    return (now - motion_stable_start_).toSec() >= settle_after_motion_sec_;
  }

  bool motionTimedOut(const ros::Time &now) const
  {
    if (last_command_sent_time_.isZero())
      return false;
    return (now - last_command_sent_time_).toSec() > motion_done_timeout_sec_;
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

  Eigen::Vector3d approachOffset(const Eigen::Matrix3d &R_des) const
  {
    Eigen::Vector3d offset = Eigen::Vector3d::Zero();
    if (coarse_approach_axis_ == "tool_-z")
    {
      offset = R_des * (Eigen::Vector3d(0, 0, -1.0) * coarse_approach_dist_m_);
    }
    else if (coarse_approach_axis_ == "tool_+z")
    {
      offset = R_des * (Eigen::Vector3d(0, 0, 1.0) * coarse_approach_dist_m_);
    }
    else if (coarse_approach_axis_ == "world_+z")
    {
      offset = Eigen::Vector3d(0, 0, coarse_approach_dist_m_);
    }
    else if (coarse_approach_axis_ == "world_-z")
    {
      offset = Eigen::Vector3d(0, 0, -coarse_approach_dist_m_);
    }
    else
    {
      ROS_WARN_THROTTLE(1.0, "[PoseServo] unknown coarse_approach_axis=%s, defaulting to tool_-z", coarse_approach_axis_.c_str());
      offset = R_des * (Eigen::Vector3d(0, 0, -1.0) * coarse_approach_dist_m_);
    }
    return offset;
  }

  Eigen::Matrix3d coarseOrientation(const Eigen::Isometry3d &T_world_tool_des) const
  {
    if (coarse_keep_current_orientation_ && have_cube_pose_)
    {
      Eigen::Isometry3d T_world_tool_est = latest_T_world_cube_est_ * T_tool_to_cube_.inverse();
      return T_world_tool_est.rotation();
    }
    return T_world_tool_des.rotation();
  }

  SendResult sendCommand(const Eigen::Isometry3d &T_base_tool, double v_override = -1.0, double acc_override = -1.0)
  {
    if (!connect_robot_)
    {
      return SendResult::OK;
    }
    if (!linear_move_client_.exists())
    {
      if (!linear_move_client_.waitForExistence(ros::Duration(0.2)))
      {
        return SendResult::NO_SERVICE;
      }
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
    double mv_velo = v_override > 0.0 ? v_override : v_max_;
    double mv_acc = acc_override > 0.0 ? acc_override : mv_velo * 2.0;
    srv.request.mvvelo = static_cast<float>(mv_velo * 1000.0);  // mm/s
    srv.request.mvacc = static_cast<float>(mv_acc * 1000.0); // rough acc
    srv.request.mvtime = 0.0;
    srv.request.mvradii = 0.0;
    srv.request.coord_mode = 0;
    srv.request.index = 0;

    if (!linear_move_client_.call(srv))
    {
      ROS_WARN_THROTTLE(1.0, "[PoseServo] linear_move call failed");
      return SendResult::CALL_FAIL;
    }
    if (srv.response.ret != 0)
    {
      ROS_WARN_THROTTLE(1.0, "[PoseServo] linear_move ret=%d message=%s", srv.response.ret,
                        srv.response.message.c_str());
      return SendResult::RET_FAIL;
    }
    return SendResult::OK;
  }

  void updateTimer(const ros::TimerEvent &)
  {
    double cube_age = 0.0;
    if (visionTimedOut(cube_age))
    {
      if (state_ == State::RUN || state_ == State::MOVING)
      {
        state_ = State::HOLD;
        ROS_WARN_THROTTLE(1.0, "[PoseServo] vision timeout, holding commands");
        publishReached(false, "VISION_TIMEOUT");
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

    latest_T_world_cube_est_ = poseMsgToIso(last_cube_pose_.pose);
    latest_T_world_tool_des_ = poseMsgToIso(target_tool_world_.pose);

    Eigen::Isometry3d T_world_cube_des = latest_T_world_tool_des_ * T_tool_to_cube_;
    Eigen::Matrix<double, 6, 1> xi = computeError(latest_T_world_cube_est_, T_world_cube_des);
    latest_ep_ = xi.head<3>();
    latest_etheta_ = xi.tail<3>();

    double ep_norm = latest_ep_.norm();
    double etheta_norm = latest_etheta_.norm();

    if (ep_norm < eps_pos_m_ && etheta_norm < eps_ang_rad_)
    {
      state_ = State::IDLE;
      goal_reached_ = true;
      publishReached(true, "REACHED");
      publishStatus(latest_ep_, latest_etheta_, cube_age);
      return;
    }

    // progress tracking
    bool improved = false;
    if (ep_norm < best_ep_norm_ * progress_improve_ratio_ || best_ep_norm_ - ep_norm > progress_eps_pos_m_)
    {
      best_ep_norm_ = ep_norm;
      improved = true;
    }
    if (etheta_norm < best_etheta_norm_ * progress_improve_ratio_ || best_etheta_norm_ - etheta_norm > progress_eps_ang_rad_)
    {
      best_etheta_norm_ = etheta_norm;
      improved = true;
    }
    if (improved || last_progress_time_.isZero())
    {
      last_progress_time_ = ros::Time::now();
    }
    else
    {
      if (!last_progress_time_.isZero() &&
          (ros::Time::now() - last_progress_time_).toSec() > progress_timeout_s_)
      {
        state_ = State::HOLD;
        publishReached(false, "SERVO_TIMEOUT_NO_PROGRESS");
        publishStatus(latest_ep_, latest_etheta_, cube_age);
        return;
      }
    }

    if (state_ == State::IDLE && goal_reached_)
    {
      if (ep_norm > resume_factor_ * eps_pos_m_ || etheta_norm > resume_factor_ * eps_ang_rad_)
      {
        state_ = State::RUN;
      }
      publishStatus(latest_ep_, latest_etheta_, cube_age);
      return;
    }

    publishStatus(latest_ep_, latest_etheta_, cube_age);
  }

  void sendTimer(const ros::TimerEvent &)
  {
    if (!target_active_ || !enable_ || state_ == State::HOLD || state_ == State::DISCONNECTED)
    {
      return;
    }
    if (!have_cube_pose_)
    {
      return;
    }

    ros::Time now = ros::Time::now();
    if ((now - target_start_time_).toSec() > servo_timeout_s_)
    {
      ROS_WARN_THROTTLE(1.0, "[PoseServo] servo timeout, holding");
      state_ = State::HOLD;
      publishReached(false, "SERVO_TIMEOUT");
      return;
    }

    if (state_ == State::MOVING)
    {
      if (motionTimedOut(now))
      {
        ROS_WARN("[PoseServo] motion timeout, holding");
        state_ = State::HOLD;
        publishReached(false, "MOTION_DONE_TIMEOUT");
        return;
      }
      if (!motionStable(now))
      {
        ROS_DEBUG_THROTTLE(2.0, "[PoseServo] waiting for motion to settle");
        return;
      }
      state_ = State::SETTLING;
      settle_until_ = now + ros::Duration(settle_after_motion_sec_);
      ROS_INFO("[PoseServo] motion settled, entering SETTLING");
      return;
    }
    if (state_ == State::SETTLING)
    {
      if (now < settle_until_)
      {
        return;
      }
      state_ = State::RUN;
    }

    if (state_ == State::IDLE && goal_reached_)
    {
      double ep_norm = latest_ep_.norm();
      double etheta_norm = latest_etheta_.norm();
      if (ep_norm > resume_factor_ * eps_pos_m_ || etheta_norm > resume_factor_ * eps_ang_rad_)
      {
        state_ = State::RUN;
      }
      else
      {
        return;
      }
    }

    if (state_ != State::RUN)
    {
      return;
    }

    Eigen::Isometry3d T_world_cube_est = latest_T_world_cube_est_;
    Eigen::Isometry3d T_world_tool_des = latest_T_world_tool_des_;

    Eigen::Isometry3d T_world_cube_des = T_world_tool_des * T_tool_to_cube_;
    Eigen::Matrix<double, 6, 1> xi = computeError(T_world_cube_est, T_world_cube_des);
    Eigen::Vector3d ep = xi.head<3>();
    Eigen::Vector3d etheta = xi.tail<3>();

    auto shouldEnterFine = [&]() -> bool {
      if (!have_cube_pose_)
      {
        return true;
      }
      if (ep.norm() < coarse_pos_gate_m_ && etheta.norm() < coarse_ang_gate_rad_)
      {
        return true;
      }
      if (ep.norm() < coarse_pos_gate_m_)
      {
        return true;
      }
      return false;
    };

    if (phase_ == Phase::COARSE_FEEDFORWARD && coarse_ff_sent_ && state_ == State::RUN)
    {
      if (shouldEnterFine())
      {
        phase_ = Phase::FINE;
        if (!coarse_done_reported_)
        {
          publishReached(false, "COARSE_DONE");
          coarse_done_reported_ = true;
        }
      }
      else
      {
        coarse_ff_sent_ = false;
      }
    }

    auto dispatchCommand = [&](const Eigen::Isometry3d &T_base_tool_next, double v_forced, const std::string &tag) -> bool {
      if (!linear_move_client_.exists())
      {
        ROS_WARN_THROTTLE(1.0, "[PoseServo] robot driver not available; entering DISCONNECTED");
        state_ = State::DISCONNECTED;
        return false;
      }
      uint64_t cmd_seq = command_seq_++;
      ROS_INFO("[PoseServo] cmd seq=%lu tag=%s ep=%.4f etheta=%.4f", cmd_seq, tag.c_str(), ep.norm(), etheta.norm());
      SendResult res = sendCommand(T_base_tool_next, v_forced, v_forced > 0.0 ? v_forced * 2.0 : -1.0);
      last_command_sent_time_ = now;
      last_command_pose_ = T_base_tool_next;
      if (res == SendResult::NO_SERVICE || res == SendResult::CALL_FAIL)
      {
        state_ = State::DISCONNECTED;
        last_move_ret_ = "NO_SERVICE";
        publishReached(false, "NO_SERVICE");
        return false;
      }
      if (res == SendResult::RET_FAIL)
      {
        state_ = State::HOLD;
        last_move_ret_ = "RET_FAIL";
        publishReached(false, "MOVE_RET_FAIL");
        return false;
      }
      state_ = State::MOVING;
      goal_reached_ = false;
      last_step_time_ = now;
      last_progress_time_ = now;
      last_move_ret_ = "OK";
      publishLastCmd(T_base_tool_next, cmd_seq);
      return true;
    };

    if (phase_ == Phase::COARSE_APPROACH)
    {
      Eigen::Isometry3d T_world_tool_approach = T_world_tool_des;
      T_world_tool_approach.translation() += approachOffset(T_world_tool_des.rotation());
      T_world_tool_approach.linear() = coarseOrientation(T_world_tool_des);
      Eigen::Isometry3d T_base_tool_next = T_world_base_.inverse() * T_world_tool_approach;
      if (dispatchCommand(T_base_tool_next, coarse_v_max_, "COARSE_APPROACH"))
      {
        phase_ = Phase::COARSE_FEEDFORWARD;
      }
      return;
    }

    if (phase_ == Phase::COARSE_FEEDFORWARD && !coarse_ff_sent_)
    {
      Eigen::Isometry3d T_world_tool_ff = T_world_tool_des;
      T_world_tool_ff.linear() = coarseOrientation(T_world_tool_des);
      Eigen::Isometry3d T_base_tool_next = T_world_base_.inverse() * T_world_tool_ff;
      if (dispatchCommand(T_base_tool_next, coarse_v_max_, "COARSE_FEEDFORWARD"))
      {
        coarse_ff_sent_ = true;
      }
      return;
    }

    if (ep.norm() < eps_pos_m_ && etheta.norm() < eps_ang_rad_)
    {
      state_ = State::IDLE;
      goal_reached_ = true;
      publishReached(true, "REACHED");
      return;
    }

    Eigen::Matrix<double, 6, 1> xi_cmd;
    if (use_step_limits_)
    {
      Eigen::Vector3d dp = kp_pos_ * ep;
      Eigen::Vector3d dth = kp_ang_ * etheta;
      double pos_max = step_pos_max_m_;
      double pos_min = step_pos_min_m_;
      double ang_max = step_ang_max_deg_ * M_PI / 180.0;
      double ang_min = step_ang_min_deg_ * M_PI / 180.0;
      double dp_norm = dp.norm();
      if (dp_norm > pos_max)
        dp = dp / dp_norm * pos_max;
      else if (dp_norm < pos_min)
      {
        if (dp_norm < 1e-12)
        {
          dp.setZero();
        }
        else
        {
          dp = dp / dp_norm * pos_min;
        }
      }
      double dth_norm = dth.norm();
      if (dth_norm > ang_max)
        dth = dth / dth_norm * ang_max;
      else if (dth_norm < ang_min)
      {
        if (dth_norm < 1e-12)
        {
          dth.setZero();
        }
        else
        {
          dth = dth / dth_norm * ang_min;
        }
      }
      xi_cmd.head<3>() = dp;
      xi_cmd.tail<3>() = dth;
    }
    else
    {
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
      double dt_cmd_raw = last_step_time_.isZero() ? (1.0 / send_rate_hz_) : (now - last_step_time_).toSec();
      if (dt_cmd_mode_ == "fixed")
      {
        dt_cmd_raw = 1.0 / send_rate_hz_;
      }
      double dt_cmd = std::max(dt_cmd_min_, std::min(dt_cmd_raw, dt_cmd_max_));
      xi_cmd.head<3>() = v_cmd;
      xi_cmd.tail<3>() = w_cmd;
      xi_cmd *= dt_cmd;
      last_dt_cmd_used_ = dt_cmd;
    }
    if (use_step_limits_)
    {
      last_dt_cmd_used_ = -1.0;
    }
    if (!xi_cmd.allFinite())
    {
      ROS_WARN("[PoseServo] non-finite xi_cmd detected, holding");
      state_ = State::HOLD;
      publishReached(false, "NON_FINITE_CMD");
      return;
    }
    Eigen::Isometry3d Delta = expSE3(xi_cmd);
    Eigen::Isometry3d T_world_cube_next = T_world_cube_est * Delta;
    Eigen::Isometry3d T_cube_to_tool = T_tool_to_cube_.inverse();
    Eigen::Isometry3d T_world_tool_next = T_world_cube_next * T_cube_to_tool;
    Eigen::Isometry3d T_base_tool_next = T_world_base_.inverse() * T_world_tool_next;

    dispatchCommand(T_base_tool_next, -1.0, "FINE");
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
  std::string joint_state_topic_{"joint_states"};

  ros::Subscriber cube_sub_;
  ros::Subscriber target_sub_;
  ros::Subscriber joint_sub_;
  ros::Publisher status_pub_;
  ros::Publisher err_pub_;
  ros::Publisher reached_pub_;
  ros::Publisher result_pub_;
  ros::Publisher last_cmd_pub_;
  ros::ServiceServer set_target_srv_;
  ros::ServiceServer stop_srv_;
  ros::ServiceServer clear_target_srv_;
  ros::ServiceServer get_cube_pose_srv_;
  ros::ServiceClient linear_move_client_;
  ros::Timer update_timer_;
  ros::Timer send_timer_;

  geometry_msgs::PoseStamped last_cube_pose_;
  geometry_msgs::PoseStamped target_tool_world_;
  bool target_active_{false};
  ros::Time target_start_time_;
  ros::Time last_command_sent_time_;
  bool have_cube_pose_{false};
  ros::WallTime last_cube_rx_wall_;
  bool goal_reached_{false};
  uint64_t command_seq_{0};

  Eigen::Isometry3d T_world_base_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d T_tool_to_cube_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d last_command_pose_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d latest_T_world_cube_est_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d latest_T_world_tool_des_{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d latest_ep_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d latest_etheta_{Eigen::Vector3d::Zero()};

  State state_{State::IDLE};
  Phase phase_{Phase::FINE};

  double motion_joint_threshold_rad_{0.002};
  double motion_stable_duration_sec_{0.5};
  double motion_done_timeout_sec_{5.0};
  double settle_after_motion_sec_{0.1};
  double send_rate_hz_{5.0};
  double resume_factor_{2.0};
  double dt_cmd_min_{0.05};
  double dt_cmd_max_{1.0};
  std::string dt_cmd_mode_{"elapsed"};
  bool use_step_limits_{true};
  double step_pos_max_m_{0.01};
  double step_ang_max_deg_{3.0};
  double step_pos_min_m_{0.0005};
  double step_ang_min_deg_{0.2};
  double progress_timeout_s_{10.0};
  double progress_eps_pos_m_{0.002};
  double progress_eps_ang_deg_{0.5};
  double progress_eps_ang_rad_{0.5 * M_PI / 180.0};
  double progress_improve_ratio_{0.98};
  bool use_two_stage_{true};
  double coarse_v_max_{0.08};
  double coarse_w_max_deg_{30.0};
  double coarse_w_max_rad_{30.0 * M_PI / 180.0};
  double coarse_pos_gate_m_{0.03};
  double coarse_ang_gate_deg_{15.0};
  double coarse_ang_gate_rad_{15.0 * M_PI / 180.0};
  double coarse_approach_dist_m_{0.08};
  std::string coarse_approach_axis_{"tool_-z"};
  bool coarse_keep_current_orientation_{true};
  bool coarse_done_reported_{false};
  bool coarse_ff_sent_{false};

  std::mutex joint_mutex_;
  std::vector<double> prev_joint_pos_;
  bool has_prev_joint_{false};
  bool have_joint_state_{false};
  ros::Time last_motion_time_;
  ros::Time last_joint_stamp_;
  ros::Time motion_stable_start_;
  ros::Time last_step_time_;
  ros::Time settle_until_;
  double last_dt_cmd_used_{0.0};
  uint64_t last_cmd_seq_{0};
  std::string last_move_ret_{"NONE"};
  double best_ep_norm_{std::numeric_limits<double>::infinity()};
  double best_etheta_norm_{std::numeric_limits<double>::infinity()};
  ros::Time last_progress_time_;
  std::string last_result_{"INIT"};

  Eigen::Vector3d last_ep_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_etheta_{Eigen::Vector3d::Zero()};
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
