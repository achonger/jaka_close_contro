#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <jaka_msgs/Move.h>
#include <jaka_close_contro/CubeFusionStats.h>

#include <Eigen/Dense>
#include <boost/filesystem.hpp>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <mutex>
#include <deque>
#include <algorithm>
#include <numeric>

namespace
{
struct CalibPoseRow
{
  std::string name;
  double x_mm = 0.0;
  double y_mm = 0.0;
  double z_mm = 0.0;
  double rx_deg = 0.0;
  double ry_deg = 0.0;
  double rz_deg = 0.0;
};

struct PoseSample
{
  ros::Time stamp;
  Eigen::Vector3d t_cam_cube{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d R_cam_cube{Eigen::Matrix3d::Identity()};
  tf2::Transform T_base_tool;
  bool has_base_tool = false;
  tf2::Transform T_world_cam;
  bool has_world_cam = false;
  int n_obs = 0;
  int inliers = 0;
  float pos_err_rms = 0.0f;
  float ang_err_rms_deg = 0.0f;
};

Eigen::Matrix3d expSO3(const Eigen::Vector3d &phi)
{
  const double theta = phi.norm();
  if (theta < 1e-12)
  {
    return Eigen::Matrix3d::Identity();
  }
  const Eigen::Vector3d a = phi / theta;
  Eigen::Matrix3d A;
  A << 0.0, -a.z(), a.y(), a.z(), 0.0, -a.x(), -a.y(), a.x(), 0.0;
  return Eigen::Matrix3d::Identity() + std::sin(theta) * A + (1.0 - std::cos(theta)) * (A * A);
}

Eigen::Vector3d logSO3(const Eigen::Matrix3d &R)
{
  double cos_theta = (R.trace() - 1.0) * 0.5;
  cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
  const double theta = std::acos(cos_theta);
  if (theta < 1e-12)
  {
    Eigen::Vector3d v;
    v << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
    return 0.5 * v;
  }

  const double sin_theta = std::sin(theta);
  Eigen::Vector3d v;
  v << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
  return (theta / (2.0 * sin_theta)) * v;
}

double rotationAngularDistanceRad(const Eigen::Matrix3d &Ra, const Eigen::Matrix3d &Rb)
{
  return logSO3(Ra.transpose() * Rb).norm();
}

Eigen::Matrix3d averageRotation(const std::vector<Eigen::Matrix3d> &rotations, const std::vector<double> &weights)
{
  if (rotations.empty())
  {
    return Eigen::Matrix3d::Identity();
  }

  Eigen::Matrix3d R_mean = rotations.front();
  for (int iter = 0; iter < 10; ++iter)
  {
    Eigen::Vector3d phi_sum = Eigen::Vector3d::Zero();
    double w_sum = 0.0;
    for (std::size_t i = 0; i < rotations.size(); ++i)
    {
      const double w = weights.empty() ? 1.0 : weights[i];
      phi_sum += w * logSO3(R_mean.transpose() * rotations[i]);
      w_sum += w;
    }

    if (w_sum < 1e-9)
    {
      break;
    }

    const Eigen::Vector3d phi_avg = phi_sum / w_sum;
    if (phi_avg.norm() < 1e-6)
    {
      break;
    }
    R_mean = R_mean * expSO3(phi_avg);
  }

  return R_mean;
}

tf2::Transform toTf2(const Eigen::Vector3d &t, const Eigen::Matrix3d &R)
{
  tf2::Transform T;
  T.setOrigin(tf2::Vector3(t.x(), t.y(), t.z()));
  Eigen::Quaterniond q(R);
  q.normalize();
  T.setRotation(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()));
  return T;
}
} // namespace

class WorldRobotCalibRecorderNode
{
public:
  WorldRobotCalibRecorderNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
      : nh_(nh), pnh_(pnh), tf_listener_(tf_buffer_)
  {
    pnh_.param<std::string>("arm_name", arm_name_, std::string("jaka1"));
    pnh_.param<std::string>("calib_pose_csv", calib_pose_csv_,
                            ros::package::getPath("jaka_close_contro") + "/config/jaka1_world_robot_calibration_pose.csv");
    pnh_.param<std::string>("linear_move_service", linear_move_service_, std::string("jaka_driver/linear_move"));
    pnh_.param<bool>("use_driver", use_driver_, true);
    pnh_.param<bool>("do_motion", do_motion_, true);
    pnh_.param<double>("speed_scale", speed_scale_, 0.15);
    pnh_.param<double>("linear_speed_mm_s", linear_speed_mm_s_, 80.0);
    pnh_.param<double>("linear_acc_mm_s2", linear_acc_mm_s2_, 200.0);
    pnh_.param<double>("motion_done_timeout_sec", motion_done_timeout_sec_, 120.0);
    pnh_.param<double>("motion_stable_duration_sec", motion_stable_duration_sec_, 0.5);
    pnh_.param<double>("motion_joint_threshold_rad", motion_joint_threshold_rad_, 0.002);
    pnh_.param<std::string>("joint_state_topic", joint_state_topic_, std::string("/joint_states"));
    pnh_.param<double>("sample_duration_sec", sample_duration_sec_, 2.0);
    pnh_.param<double>("wait_before_sample_sec", wait_before_sample_sec_, 5.0);
    pnh_.param<double>("wait_after_sample_sec", wait_after_sample_sec_, 2.0);

    pnh_.param<std::string>("fused_topic", fused_topic_, std::string("cube_center_fused"));
    pnh_.param<std::string>("stats_topic", stats_topic_, std::string("cube_fusion_stats"));
    pnh_.param<std::string>("camera_frame", camera_frame_, std::string("zed2i_left_camera_optical_frame"));
    pnh_.param<std::string>("cube_frame", cube_frame_, std::string("cube_center"));
    pnh_.param<std::string>("world_frame", world_frame_, std::string("world"));
    pnh_.param<std::string>("base_frame", base_frame_, std::string("Link_0"));
    pnh_.param<std::string>("tool_frame", tool_frame_, std::string("Link_6"));
    pnh_.param<double>("tf_lookup_timeout_sec", tf_lookup_timeout_sec_, 0.5);
    pnh_.param<double>("world_tf_lookup_timeout_sec", world_tf_lookup_timeout_sec_, 0.1);
    pnh_.param<double>("tf_latest_timeout_sec", tf_latest_timeout_sec_, 0.5);
    pnh_.param<double>("sync_guard_sec", sync_guard_sec_, 0.03);
    pnh_.param<double>("cube_queue_keep_sec", cube_queue_keep_sec_, 5.0);
    int cube_queue_max_len_param = static_cast<int>(cube_queue_max_len_);
    pnh_.param<int>("cube_queue_max_len", cube_queue_max_len_param, 500);
    if (cube_queue_max_len_param < 0)
    {
      cube_queue_max_len_param = 0;
    }
    cube_queue_max_len_ = static_cast<std::size_t>(cube_queue_max_len_param);

    pnh_.param<std::string>("output_dataset_csv", output_dataset_csv_, std::string(""));
    pnh_.param<std::string>("output_dir", output_dir_, ros::package::getPath("jaka_close_contro") + "/config");
    pnh_.param<std::string>("output_prefix", output_prefix_, std::string("world_robot_calib_dataset"));
    if (output_dataset_csv_.empty())
    {
      const ros::Time now = ros::Time::now();
      const std::string stamp = std::to_string(static_cast<long long>(now.toSec()));
      boost::filesystem::create_directories(output_dir_);
      output_dataset_csv_ = output_dir_ + "/" + output_prefix_ + "_" + stamp + "_" + arm_name_ + ".csv";
    }

    linear_move_client_ = nh_.serviceClient<jaka_msgs::Move>(linear_move_service_);
    joint_state_sub_ = nh_.subscribe(joint_state_topic_, 50, &WorldRobotCalibRecorderNode::jointStateCallback, this);
    fused_sub_ = nh_.subscribe(fused_topic_, 10, &WorldRobotCalibRecorderNode::cubeCallback, this);
    stats_sub_ = nh_.subscribe(stats_topic_, 10, &WorldRobotCalibRecorderNode::statsCallback, this);

    if (!loadCalibPoses())
    {
      ROS_ERROR("[CalibRecorder] 无法加载标定姿态 CSV，节点退出");
      ros::shutdown();
      return;
    }

    initDatasetFile();

    ROS_INFO("[CalibRecorder] 臂标识=%s, 标定姿态数: %zu", arm_name_.c_str(), poses_.size());
    ROS_INFO("[CalibRecorder] 速度缩放=%.2f, 线速度=%.1f mm/s, 线加速度=%.1f mm/s^2", speed_scale_, linear_speed_mm_s_, linear_acc_mm_s2_);
    ROS_INFO("[CalibRecorder] 采样窗口：前等待 %.2f s，窗口 %.2f s，后等待 %.2f s", wait_before_sample_sec_, sample_duration_sec_,
             wait_after_sample_sec_);
    ROS_INFO("[CalibRecorder] 数据集输出: %s", output_dataset_csv_.c_str());
    ROS_INFO("[CalibRecorder] use_driver=%s, do_motion=%s", use_driver_ ? "true" : "false", do_motion_ ? "true" : "false");
    ROS_INFO("[CalibRecorder] fused_topic=%s, stats_topic=%s", fused_topic_.c_str(), stats_topic_.c_str());
    ROS_INFO("[CalibRecorder] TF lookup timeout: %.3f s, latest timeout: %.3f s, guard: %.3f s", tf_lookup_timeout_sec_,
             tf_latest_timeout_sec_, sync_guard_sec_);
    ROS_INFO("[CalibRecorder] cube_queue_keep=%.1f s, max_len=%zu", cube_queue_keep_sec_, cube_queue_max_len_);
    ROS_INFO("[CalibRecorder] world_frame=%s, world_tf_lookup_timeout=%.3f s", world_frame_.c_str(),
             world_tf_lookup_timeout_sec_);

    waitForService();
    executeTrajectory();

    ROS_INFO("[CalibRecorder] 丢弃样本：缺融合=%zu，无时间戳=%zu，无同步融合=%zu，TF失败=%zu，世界TF失败=%zu",
             dropped_missing_cube_, dropped_missing_stamp_, dropped_no_sync_cube_, dropped_tf_fail_,
             dropped_world_tf_fail_);
    ROS_INFO("[CalibRecorder] 已采集 %zu 行数据，保存到 %s。请运行离线脚本进行求解。", sample_id_, output_dataset_csv_.c_str());
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string arm_name_;
  std::string calib_pose_csv_;
  std::string linear_move_service_;
  bool use_driver_ = true;
  bool do_motion_ = true;
  double speed_scale_ = 0.15;
  double linear_speed_mm_s_ = 80.0;
  double linear_acc_mm_s2_ = 200.0;
  double motion_done_timeout_sec_ = 120.0;
  double motion_stable_duration_sec_ = 0.5;
  double motion_joint_threshold_rad_ = 0.002;
  double wait_before_sample_sec_ = 5.0;
  double wait_after_sample_sec_ = 2.0;
  double sample_duration_sec_ = 2.0;
  std::string joint_state_topic_ = "joint_states";

  std::string fused_topic_;
  std::string stats_topic_;
  std::string camera_frame_;
  std::string cube_frame_;
  std::string world_frame_;
  std::string base_frame_;
  std::string tool_frame_;
  double tf_lookup_timeout_sec_ = 0.5;
  double world_tf_lookup_timeout_sec_ = 0.1;
  double tf_latest_timeout_sec_ = 0.5;
  double sync_guard_sec_ = 0.03;
  double cube_queue_keep_sec_ = 5.0;
  std::size_t cube_queue_max_len_ = 500;

  std::string output_dataset_csv_;
  std::string output_dir_;
  std::string output_prefix_;
  std::size_t sample_id_ = 0;

  std::size_t target_index_ = 0;
  std::vector<CalibPoseRow> poses_;
  ros::ServiceClient linear_move_client_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber fused_sub_;
  ros::Subscriber stats_sub_;

  std::size_t dropped_missing_cube_ = 0;
  std::size_t dropped_missing_stamp_ = 0;
  std::size_t dropped_tf_fail_ = 0;
  std::size_t dropped_no_sync_cube_ = 0;
  std::size_t dropped_world_tf_fail_ = 0;

  std::vector<double> prev_joint_pos_;
  ros::Time last_joint_stamp_;
  ros::Time last_motion_time_;
  bool has_prev_joint_ = false;
  std::mutex joint_mutex_;

  std::deque<geometry_msgs::PoseStamped> cube_queue_;
  ros::Time last_used_cube_stamp_;
  jaka_close_contro::CubeFusionStats latest_stats_;
  bool has_stats_ = false;
  std::mutex data_mutex_;

  void initDatasetFile()
  {
    std::ofstream fout(output_dataset_csv_, std::ios::out | std::ios::trunc);
    fout.setf(std::ios::fixed);
    fout.precision(6);
    fout << "sample_id,pose_idx,stamp,cam_frame,cube_frame,cam_cube_tx,cam_cube_ty,cam_cube_tz,cam_cube_qx,cam_cube_qy,cam_cube_qz,cam_cube_qw,";
    fout << "base_frame,tool_frame,base_tool_tx,base_tool_ty,base_tool_tz,base_tool_qx,base_tool_qy,base_tool_qz,base_tool_qw,";
    fout << "world_frame,camera_frame,world_cam_tx,world_cam_ty,world_cam_tz,world_cam_qx,world_cam_qy,world_cam_qz,world_cam_qw,";
    fout << "world_cube_tx,world_cube_ty,world_cube_tz,world_cube_qx,world_cube_qy,world_cube_qz,world_cube_qw,";
    fout << "num_faces,inliers,pos_err_rms,ang_err_rms_deg,note\n";
    fout.close();
  }

  void waitForService()
  {
    if (!use_driver_) {
      ROS_WARN("[CalibRecorder] use_driver=false，跳过服务检查");
      return;
    }
    while (ros::ok())
    {
      if (ros::service::waitForService(linear_move_service_, ros::Duration(1.0)))
      {
        ROS_INFO("[CalibRecorder] linear_move 服务就绪: %s", linear_move_service_.c_str());
        return;
      }
      ROS_WARN("[CalibRecorder] 等待 %s ...", linear_move_service_.c_str());
    }
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
  {
    if (msg->position.size() < 6)
    {
      ROS_WARN_THROTTLE(5.0, "[CalibRecorder] joint_state 位置元素不足6个，忽略该帧");
      return;
    }

    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    std::lock_guard<std::mutex> lock(joint_mutex_);

    if (has_prev_joint_)
    {
      double max_diff = 0.0;
      const size_t n = std::min(prev_joint_pos_.size(), msg->position.size());
      for (size_t i = 0; i < n; ++i)
      {
        const double diff = std::abs(msg->position[i] - prev_joint_pos_[i]);
        max_diff = std::max(max_diff, diff);
      }

      if (max_diff > motion_joint_threshold_rad_)
      {
        last_motion_time_ = stamp;
      }
    }
    else
    {
      last_motion_time_ = stamp;
      has_prev_joint_ = true;
    }

    prev_joint_pos_ = msg->position;
    last_joint_stamp_ = stamp;
  }

  void cubeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    if (msg->header.stamp.isZero())
    {
      ++dropped_missing_stamp_;
      ROS_WARN_THROTTLE(1.0, "[CalibRecorder] 收到无时间戳的融合位姿，已丢弃");
      return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    const ros::Time now = ros::Time::now();
    cube_queue_.push_back(*msg);

    while (cube_queue_.size() > cube_queue_max_len_)
    {
      cube_queue_.pop_front();
    }

    while (!cube_queue_.empty() && (now - cube_queue_.front().header.stamp).toSec() > cube_queue_keep_sec_)
    {
      cube_queue_.pop_front();
    }
  }

  void statsCallback(const jaka_close_contro::CubeFusionStats::ConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_stats_ = *msg;
    has_stats_ = true;
  }

  bool loadCalibPoses()
  {
    std::ifstream fin(calib_pose_csv_);
    if (!fin.is_open())
    {
      ROS_ERROR("[CalibRecorder] 无法打开 CSV: %s", calib_pose_csv_.c_str());
      return false;
    }

    std::string line;
    bool header_skipped = false;
    while (std::getline(fin, line))
    {
      if (line.empty() || line[0] == '#')
      {
        continue;
      }
      if (!header_skipped)
      {
        header_skipped = true;
        continue;
      }

      std::stringstream ss(line);
      std::string item;
      std::vector<std::string> cols;
      while (std::getline(ss, item, ','))
      {
        cols.push_back(item);
      }

      if (cols.size() != 7)
      {
        ROS_WARN("[CalibRecorder] CSV 行格式错误（期望7列），已跳过: %s", line.c_str());
        continue;
      }

      CalibPoseRow row;
      row.name = cols[0];
      try
      {
        row.x_mm = std::stod(cols[1]);
        row.y_mm = std::stod(cols[2]);
        row.z_mm = std::stod(cols[3]);
        row.rx_deg = std::stod(cols[4]);
        row.ry_deg = std::stod(cols[5]);
        row.rz_deg = std::stod(cols[6]);
      }
      catch (const std::exception &)
      {
        ROS_WARN("[CalibRecorder] CSV 数值解析失败，跳过该行: %s", line.c_str());
        continue;
      }

      poses_.push_back(row);
    }

    return !poses_.empty();
  }

  bool sendLinearTarget(const CalibPoseRow &target)
  {
    if (!use_driver_ || !do_motion_) {
      ROS_INFO("[CalibRecorder] use_driver/do_motion=false，跳过运动发送");
      return true;
    }
    const double DEG2RAD = M_PI / 180.0;
    jaka_msgs::Move srv;
    srv.request.pose = {static_cast<float>(target.x_mm), static_cast<float>(target.y_mm), static_cast<float>(target.z_mm),
                        static_cast<float>(target.rx_deg * DEG2RAD), static_cast<float>(target.ry_deg * DEG2RAD),
                        static_cast<float>(target.rz_deg * DEG2RAD)};
    srv.request.mvvelo = linear_speed_mm_s_ * speed_scale_;
    srv.request.mvacc = linear_acc_mm_s2_ * speed_scale_;
    srv.request.mvtime = 0.0;
    srv.request.mvradii = 0.0;
    srv.request.coord_mode = 0;
    srv.request.index = static_cast<int32_t>(target_index_);

    if (!linear_move_client_.call(srv))
    {
      ROS_ERROR("[CalibRecorder] linear_move 通信失败，姿态 %s 未发送", target.name.c_str());
      return false;
    }
    if (srv.response.ret != 0)
    {
      ROS_ERROR("[CalibRecorder] linear_move 执行失败，ret=%d, message=%s", srv.response.ret, srv.response.message.c_str());
      return false;
    }

    ROS_INFO("[CalibRecorder] linear_move ret=%d, message=%s", srv.response.ret, srv.response.message.c_str());
    return true;
  }

  bool waitRobotMotionDone(double timeout_sec)
  {
    const ros::Time start = ros::Time::now();
    ros::Rate rate(20.0);
    ros::Time stable_start;

    while (ros::ok())
    {
      ros::spinOnce();

      if ((ros::Time::now() - start).toSec() > timeout_sec)
      {
        ROS_WARN("[CalibRecorder] 等待机器人停止运动超时（%.1f s）", timeout_sec);
        return false;
      }

      ros::Time last_motion;
      ros::Time last_stamp;
      {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        last_motion = last_motion_time_;
        last_stamp = last_joint_stamp_;
      }

      if (last_stamp.isZero())
      {
        ROS_WARN_THROTTLE(2.0, "[CalibRecorder] 尚未收到 joint_state，继续等待...");
        rate.sleep();
        continue;
      }

      const double since_motion = (ros::Time::now() - last_motion).toSec();
      if (since_motion >= motion_stable_duration_sec_)
      {
        if (stable_start.isZero())
        {
          stable_start = ros::Time::now();
        }
        else if ((ros::Time::now() - stable_start).toSec() >= motion_stable_duration_sec_)
        {
          ROS_INFO("[CalibRecorder] 检测到机器人已停止运动");
          return true;
        }
      }
      else
      {
        stable_start = ros::Time(0);
      }

      rate.sleep();
    }

    return false;
  }

  bool lookupLatestTfStamp(ros::Time &stamp)
  {
    try
    {
      const geometry_msgs::TransformStamped tf_msg = tf_buffer_.lookupTransform(
          base_frame_, tool_frame_, ros::Time(0), ros::Duration(tf_latest_timeout_sec_));
      stamp = tf_msg.header.stamp;
      if (stamp.isZero())
      {
        stamp = ros::Time::now();
        ROS_WARN_THROTTLE(1.0, "[CalibRecorder] 最新 TF 时间戳为 0，使用当前时间 %.3f 代替", stamp.toSec());
      }
      return true;
    }
    catch (const tf2::TransformException &ex)
    {
      ROS_WARN_THROTTLE(2.0, "[CalibRecorder] 查询最新 TF 时间失败：%s", ex.what());
      ++dropped_tf_fail_;
      return false;
    }
  }

  bool selectSyncedCube(const ros::Time &cutoff, geometry_msgs::PoseStamped &selected,
                        jaka_close_contro::CubeFusionStats &stats_copy, bool &stats_available)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (cube_queue_.empty())
    {
      ++dropped_missing_cube_;
      return false;
    }

    bool found = false;
    for (auto it = cube_queue_.rbegin(); it != cube_queue_.rend(); ++it)
    {
      if (it->header.stamp <= cutoff && it->header.stamp > last_used_cube_stamp_)
      {
        selected = *it;
        found = true;
        break;
      }
    }

    if (!found)
    {
      ++dropped_no_sync_cube_;
      return false;
    }

    last_used_cube_stamp_ = selected.header.stamp;
    stats_copy = latest_stats_;
    stats_available = has_stats_;
    return true;
  }

  bool fetchSyncedSample(PoseSample &sample, tf2::Transform &T_bt)
  {
    ros::Time tf_latest_stamp;
    if (!lookupLatestTfStamp(tf_latest_stamp))
    {
      return false;
    }

    const ros::Time cutoff = tf_latest_stamp - ros::Duration(sync_guard_sec_);

    geometry_msgs::PoseStamped chosen;
    jaka_close_contro::CubeFusionStats stats_copy;
    bool stats_available = false;
    if (!selectSyncedCube(cutoff, chosen, stats_copy, stats_available))
    {
      return false;
    }

    sample.stamp = chosen.header.stamp;

    tf2::Transform T_cc;
    tf2::fromMsg(chosen.pose, T_cc);
    tf2::Vector3 p = T_cc.getOrigin();
    tf2::Quaternion q = T_cc.getRotation();
    q.normalize();
    Eigen::Quaterniond q_eig(q.w(), q.x(), q.y(), q.z());

    sample.t_cam_cube = Eigen::Vector3d(p.x(), p.y(), p.z());
    sample.R_cam_cube = q_eig.toRotationMatrix();

    sample.n_obs = stats_available ? stats_copy.n_obs : 0;
    sample.inliers = stats_available ? stats_copy.inliers : 0;
    if (stats_available)
    {
      auto rms_vec = [](const std::vector<float> &vals) -> float {
        if (vals.empty())
        {
          return 0.0f;
        }
        double accum = 0.0;
        for (const auto v : vals)
        {
          accum += static_cast<double>(v) * static_cast<double>(v);
        }
        return static_cast<float>(std::sqrt(accum / static_cast<double>(vals.size())));
      };

      sample.pos_err_rms = rms_vec(stats_copy.pos_err_m);
      sample.ang_err_rms_deg = rms_vec(stats_copy.ang_err_deg);
    }
    else
    {
      sample.pos_err_rms = 0.0f;
      sample.ang_err_rms_deg = 0.0f;
    }

    if (!lookupBaseTool(T_bt, sample.stamp))
    {
      return false;
    }

    if (!lookupWorldCam(sample.T_world_cam, sample.stamp))
    {
      return false;
    }
    sample.has_world_cam = true;

    return true;
  }

  bool lookupBaseTool(tf2::Transform &T, const ros::Time &stamp)
  {
    try
    {
      if (!tf_buffer_.canTransform(base_frame_, tool_frame_, stamp, ros::Duration(tf_lookup_timeout_sec_)))
      {
        ++dropped_tf_fail_;
        ROS_WARN_THROTTLE(2.0, "[CalibRecorder] TF %s->%s @ %.3f 不可用", base_frame_.c_str(), tool_frame_.c_str(), stamp.toSec());
        return false;
      }
      geometry_msgs::TransformStamped tf_msg = tf_buffer_.lookupTransform(
          base_frame_, tool_frame_, stamp, ros::Duration(tf_lookup_timeout_sec_));
      tf2::fromMsg(tf_msg.transform, T);
      return true;
    }
    catch (const tf2::TransformException &ex)
    {
      ++dropped_tf_fail_;
      ROS_WARN_THROTTLE(2.0, "[CalibRecorder] 无法获取 %s->%s TF@%.3f：%s", base_frame_.c_str(), tool_frame_.c_str(),
                        stamp.toSec(), ex.what());
      return false;
    }
  }

  bool lookupWorldCam(tf2::Transform &T, const ros::Time &stamp)
  {
    try
    {
      if (!tf_buffer_.canTransform(world_frame_, camera_frame_, stamp, ros::Duration(world_tf_lookup_timeout_sec_)))
      {
        ++dropped_world_tf_fail_;
        ROS_WARN_THROTTLE(2.0, "[CalibRecorder] TF %s->%s @ %.3f 不可用", world_frame_.c_str(),
                          camera_frame_.c_str(), stamp.toSec());
        return false;
      }
      geometry_msgs::TransformStamped tf_msg = tf_buffer_.lookupTransform(
          world_frame_, camera_frame_, stamp, ros::Duration(world_tf_lookup_timeout_sec_));
      tf2::fromMsg(tf_msg.transform, T);
      return true;
    }
    catch (const tf2::TransformException &ex)
    {
      ++dropped_world_tf_fail_;
      ROS_WARN_THROTTLE(2.0, "[CalibRecorder] 无法获取 %s->%s TF@%.3f：%s", world_frame_.c_str(),
                        camera_frame_.c_str(), stamp.toSec(), ex.what());
      return false;
    }
  }

  void computeMeanPose(const std::vector<PoseSample> &samples, Eigen::Vector3d &t_mean, Eigen::Matrix3d &R_mean,
                       double &pos_rms, double &ang_rms)
  {
    if (samples.empty())
    {
      t_mean.setZero();
      R_mean.setIdentity();
      pos_rms = 0.0;
      ang_rms = 0.0;
      return;
    }

    std::vector<Eigen::Vector3d> translations;
    std::vector<Eigen::Matrix3d> rotations;
    translations.reserve(samples.size());
    rotations.reserve(samples.size());
    for (const auto &s : samples)
    {
      translations.push_back(s.t_cam_cube);
      rotations.push_back(s.R_cam_cube);
    }

    t_mean = Eigen::Vector3d::Zero();
    for (const auto &t : translations)
    {
      t_mean += t;
    }
    t_mean /= static_cast<double>(translations.size());

    std::vector<double> weights(rotations.size(), 1.0);
    R_mean = averageRotation(rotations, weights);

    double pos_sq = 0.0;
    double ang_sq = 0.0;
    for (std::size_t i = 0; i < samples.size(); ++i)
    {
      pos_sq += (translations[i] - t_mean).squaredNorm();
      ang_sq += std::pow(rotationAngularDistanceRad(R_mean, rotations[i]), 2);
    }

    pos_rms = std::sqrt(pos_sq / translations.size());
    ang_rms = std::sqrt(ang_sq / rotations.size());
  }

  void computeMeanBaseTool(const std::vector<PoseSample> &samples, Eigen::Vector3d &t_mean, Eigen::Matrix3d &R_mean,
                           std::size_t &count)
  {
    std::vector<Eigen::Vector3d> translations;
    std::vector<Eigen::Matrix3d> rotations;
    for (const auto &s : samples)
    {
      if (!s.has_base_tool)
      {
        continue;
      }
      tf2::Vector3 t = s.T_base_tool.getOrigin();
      tf2::Quaternion q = s.T_base_tool.getRotation();
      q.normalize();
      translations.emplace_back(t.x(), t.y(), t.z());
      rotations.emplace_back(Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()).toRotationMatrix());
    }

    count = translations.size();
    if (translations.empty())
    {
      t_mean.setZero();
      R_mean.setIdentity();
      return;
    }

    t_mean = Eigen::Vector3d::Zero();
    for (const auto &t : translations)
    {
      t_mean += t;
    }
    t_mean /= static_cast<double>(translations.size());

    std::vector<double> weights(rotations.size(), 1.0);
    R_mean = averageRotation(rotations, weights);
  }

  void computeMeanWorldCam(const std::vector<PoseSample> &samples, Eigen::Vector3d &t_mean, Eigen::Matrix3d &R_mean,
                           std::size_t &count)
  {
    std::vector<Eigen::Vector3d> translations;
    std::vector<Eigen::Matrix3d> rotations;
    for (const auto &s : samples)
    {
      if (!s.has_world_cam)
      {
        continue;
      }
      tf2::Vector3 t = s.T_world_cam.getOrigin();
      tf2::Quaternion q = s.T_world_cam.getRotation();
      q.normalize();
      translations.emplace_back(t.x(), t.y(), t.z());
      rotations.emplace_back(Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()).toRotationMatrix());
    }

    count = translations.size();
    if (translations.empty())
    {
      t_mean.setZero();
      R_mean.setIdentity();
      return;
    }

    t_mean = Eigen::Vector3d::Zero();
    for (const auto &t : translations)
    {
      t_mean += t;
    }
    t_mean /= static_cast<double>(translations.size());

    std::vector<double> weights(rotations.size(), 1.0);
    R_mean = averageRotation(rotations, weights);
  }

  bool collectSampleWindow(std::size_t pose_index, const std::string &pose_name)
  {
    const ros::Time sample_start = ros::Time::now();
    ros::Rate rate(50.0);
    std::vector<PoseSample> samples;

    while (ros::ok() && (ros::Time::now() - sample_start).toSec() <= sample_duration_sec_)
    {
      ros::spinOnce();

      PoseSample s;
      tf2::Transform T_bt;
      if (fetchSyncedSample(s, T_bt))
      {
        s.T_base_tool = T_bt;
        s.has_base_tool = true;
        samples.push_back(s);
      }

      rate.sleep();
    }

    if (samples.empty())
    {
      ROS_WARN("[CalibRecorder] 采样窗口内没有有效融合数据，姿态 #%zu (%s)", pose_index + 1, pose_name.c_str());
      return false;
    }

    Eigen::Vector3d t_mean_cam;
    Eigen::Matrix3d R_mean_cam;
    double pos_rms = 0.0;
    double ang_rms = 0.0;
    computeMeanPose(samples, t_mean_cam, R_mean_cam, pos_rms, ang_rms);

    Eigen::Vector3d t_mean_bt;
    Eigen::Matrix3d R_mean_bt;
    std::size_t bt_count = 0;
    computeMeanBaseTool(samples, t_mean_bt, R_mean_bt, bt_count);

    Eigen::Vector3d t_mean_wc;
    Eigen::Matrix3d R_mean_wc;
    std::size_t wc_count = 0;
    computeMeanWorldCam(samples, t_mean_wc, R_mean_wc, wc_count);

    const double stamp_mean = std::accumulate(samples.begin(), samples.end(), 0.0,
                                              [](double acc, const PoseSample &s)
                                              { return acc + s.stamp.toSec(); }) /
                              static_cast<double>(samples.size());

    const int sum_obs = std::accumulate(samples.begin(), samples.end(), 0, [](int v, const PoseSample &s) { return v + s.n_obs; });
    const int sum_inliers = std::accumulate(samples.begin(), samples.end(), 0, [](int v, const PoseSample &s) { return v + s.inliers; });
    const double avg_obs = static_cast<double>(sum_obs) / static_cast<double>(samples.size());
    const double avg_inliers = static_cast<double>(sum_inliers) / static_cast<double>(samples.size());

    Eigen::Quaterniond q_cam(R_mean_cam);
    q_cam.normalize();
    Eigen::Quaterniond q_bt(R_mean_bt);
    q_bt.normalize();
    Eigen::Quaterniond q_wc(R_mean_wc);
    q_wc.normalize();

    Eigen::Vector3d t_world_cube = t_mean_wc + R_mean_wc * t_mean_cam;
    Eigen::Matrix3d R_world_cube = R_mean_wc * R_mean_cam;
    Eigen::Quaterniond q_world_cube(R_world_cube);
    q_world_cube.normalize();

    std::ofstream fout(output_dataset_csv_, std::ios::out | std::ios::app);
    fout.setf(std::ios::fixed);
    fout.precision(6);
    fout << sample_id_++ << "," << pose_index << "," << stamp_mean << "," << camera_frame_ << "," << cube_frame_ << ","
         << t_mean_cam.x() << "," << t_mean_cam.y() << "," << t_mean_cam.z() << ","
         << q_cam.x() << "," << q_cam.y() << "," << q_cam.z() << "," << q_cam.w() << ","
         << base_frame_ << "," << tool_frame_ << ","
         << t_mean_bt.x() << "," << t_mean_bt.y() << "," << t_mean_bt.z() << ","
         << q_bt.x() << "," << q_bt.y() << "," << q_bt.z() << "," << q_bt.w() << ","
         << world_frame_ << "," << camera_frame_ << ","
         << t_mean_wc.x() << "," << t_mean_wc.y() << "," << t_mean_wc.z() << ","
         << q_wc.x() << "," << q_wc.y() << "," << q_wc.z() << "," << q_wc.w() << ","
         << t_world_cube.x() << "," << t_world_cube.y() << "," << t_world_cube.z() << ","
         << q_world_cube.x() << "," << q_world_cube.y() << "," << q_world_cube.z() << "," << q_world_cube.w() << ","
         << avg_obs << "," << avg_inliers << "," << pos_rms << "," << (ang_rms * 180.0 / M_PI) << ","
         << pose_name << "\n";
    fout.close();

    ROS_INFO("[CalibRecorder] 记录姿态 #%zu (%s)：%zu 样本，pos_rms=%.4f m, ang_rms=%.2f deg, base_tool样本=%zu, world_cam样本=%zu",
             pose_index + 1, pose_name.c_str(), samples.size(), pos_rms, ang_rms * 180.0 / M_PI, bt_count, wc_count);

    return true;
  }

  void executeTrajectory()
  {
    const double effective_speed = linear_speed_mm_s_ * speed_scale_;
    const double effective_acc = linear_acc_mm_s2_ * speed_scale_;
    ROS_INFO("[CalibRecorder] 实际发送速度=%.1f mm/s, 加速度=%.1f mm/s^2", effective_speed, effective_acc);

    for (size_t i = 0; i < poses_.size() && ros::ok(); ++i)
    {
      const auto &p = poses_[i];
      target_index_ = i;

      ROS_INFO_STREAM("[CalibRecorder] 发送 linear_move 到标定位姿 #" << (i + 1) << " (" << p.name
                                                                     << "): pos(mm)=(" << p.x_mm << ", " << p.y_mm << ", " << p.z_mm
                                                                     << "), rpy(deg)=(" << p.rx_deg << ", " << p.ry_deg << ", "
                                                                     << p.rz_deg << ")");

      if (do_motion_) {
        if (!sendLinearTarget(p))
        {
          ROS_ERROR("[CalibRecorder] 姿态 %s 发送失败，停止后续执行", p.name.c_str());
          break;
        }

        ROS_INFO("[CalibRecorder] 等待机器人停止运动...");
        waitRobotMotionDone(motion_done_timeout_sec_);

        ROS_INFO_STREAM("[CalibRecorder] 机器人静止，采样前等待 " << wait_before_sample_sec_ << " s");
        ros::Duration(wait_before_sample_sec_).sleep();
      } else {
        ROS_WARN("[CalibRecorder] do_motion=false，跳过自动运动，请手动摆位后等待采样窗口");
        ros::Duration(wait_before_sample_sec_).sleep();
      }

      ROS_INFO_STREAM("[CalibRecorder] 开始采样窗口（姿态 #" << (i + 1) << ", 持续 " << sample_duration_sec_ << " s）");
      collectSampleWindow(i, p.name);

      ROS_INFO_STREAM("[CalibRecorder] 采样结束，采样后等待 " << wait_after_sample_sec_ << " s");
      ros::Duration(wait_after_sample_sec_).sleep();
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "world_robot_calib_recorder_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  WorldRobotCalibRecorderNode node(nh, pnh);
  ros::spin();
  return 0;
}
