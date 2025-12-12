// cube_multi_face_fusion_node.cpp
#include <ros/ros.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <string>
#include <sstream>
#include <vector>

struct Observation
{
  int face_id;
  Eigen::Vector3d t;
  Eigen::Quaterniond q;  // w, x, y, z
  double base_weight;
};

class CubeMultiFaceFusion
{
public:
  CubeMultiFaceFusion(ros::NodeHandle &nh, ros::NodeHandle &pnh)
      : nh_(nh), pnh_(pnh),
        camera_frame_default_("zed2i_left_camera_optical_frame"),
        cube_frame_("cube_center"),
        tool_frame_("Link_6"),
        has_prev_(false),
        has_prev_q_(false),
        has_tool_offset_(false),
        publish_tool_tf_(false),
        tf_buffer_(),
        tf_listener_(tf_buffer_),
        tf_broadcaster_()
  {
    pnh_.param<std::string>("faces_yaml", faces_yaml_path_, "");
    pnh_.param<std::string>("fiducial_topic", fiducial_topic_, "/tool_fiducials");
    pnh_.param<std::string>("camera_frame_default", camera_frame_default_, "zed2i_left_camera_optical_frame");
    pnh_.param<std::string>("cube_frame", cube_frame_, "cube_center");
    pnh_.param("publish_tool_tf", publish_tool_tf_, false);

    // 鲁棒融合参数
    pnh_.param("sigma_ang_deg", sigma_ang_deg_, 6.0);
    pnh_.param("sigma_pos_m", sigma_pos_m_, 0.02);
    pnh_.param("gate_ang_single_deg", gate_ang_single_deg_, 35.0);
    pnh_.param("gate_ang_multi_deg", gate_ang_multi_deg_, 25.0);
    pnh_.param("gate_pos_single_m", gate_pos_single_m_, 0.08);
    pnh_.param("gate_pos_multi_m", gate_pos_multi_m_, 0.08);
    pnh_.param("min_inliers", min_inliers_, 1);

    // 时间一致性（速度上限）
    pnh_.param("max_lin_speed_mps", max_lin_speed_mps_, 0.3);
    pnh_.param("max_ang_speed_dps", max_ang_speed_dps_, 120.0);
    pnh_.param("timeout_sec", timeout_sec_, 0.5);

    ROS_INFO_STREAM("[Fusion] faces_yaml: " << faces_yaml_path_);
    ROS_INFO_STREAM("[Fusion] fiducial_topic: " << fiducial_topic_);
    ROS_INFO_STREAM("[Fusion] cube_frame: " << cube_frame_);
    ROS_INFO("[Fusion] publish_tool_tf: %s", publish_tool_tf_ ? "true" : "false");

    if (faces_yaml_path_.empty() || !loadFacesYaml(faces_yaml_path_))
    {
      ROS_ERROR("[Fusion] Failed to load faces YAML. Exiting.");
      ros::shutdown();
      return;
    }

    sub_ = nh_.subscribe<fiducial_msgs::FiducialTransformArray>(
        fiducial_topic_, 1, &CubeMultiFaceFusion::callback, this);
    pub_ = nh_.advertise<geometry_msgs::PoseStamped>("cube_center_fused", 1);
  }

private:
  // --------------- YAML 读取 ---------------
  bool loadFacesYaml(const std::string &path)
  {
    try
    {
      ROS_INFO("[Fusion] Loading YAML from: %s", path.c_str());
      YAML::Node root = YAML::LoadFile(path);

      if (!root["faces"])
      {
        ROS_ERROR("[Fusion] YAML missing 'faces' section");
        return false;
      }

      for (const auto &f : root["faces"])
      {
        int id = f["id"].as<int>();
        auto tr = f["translation"];
        auto rpy = f["rpy_deg"];
        double tx = tr[0].as<double>();
        double ty = tr[1].as<double>();
        double tz = tr[2].as<double>();
        double rr = rpy[0].as<double>() * M_PI / 180.0;
        double pp = rpy[1].as<double>() * M_PI / 180.0;
        double yy = rpy[2].as<double>() * M_PI / 180.0;
        tf2::Matrix3x3 R;
        R.setRPY(rr, pp, yy);
        tf2::Vector3 t(tx, ty, tz);
        tf2::Transform T_cube_face(R, t);
        face2cube_[id] = T_cube_face.inverse();

        ROS_INFO("[Fusion] Loaded face ID=%d: trans=[%.3f,%.3f,%.3f], rpy_deg=[%.1f,%.1f,%.1f]",
                 id, tx, ty, tz,
                 rpy[0].as<double>(), rpy[1].as<double>(), rpy[2].as<double>());
      }

      if (root["tool_offset"])
      {
        auto to = root["tool_offset"];
        auto trans = to["translation"];
        double tx = trans[0].as<double>();
        double ty = trans[1].as<double>();
        double tz = trans[2].as<double>();

        tool_offset_.setOrigin(tf2::Vector3(tx, ty, tz));
        tool_offset_.setRotation(tf2::Quaternion(0, 0, 0, 1));
        has_tool_offset_ = true;

        ROS_INFO("[Fusion] Tool offset loaded: trans=[%.3f,%.3f,%.3f]", tx, ty, tz);
      }
      else
      {
        ROS_WARN("[Fusion] No 'tool_offset' in YAML. Using default [0,0,0.077]");
        tool_offset_.setOrigin(tf2::Vector3(0, 0, 0.077));
        tool_offset_.setRotation(tf2::Quaternion(0, 0, 0, 1));
        has_tool_offset_ = true;
      }

      return true;
    }
    catch (const std::exception &e)
    {
      ROS_ERROR("[Fusion] YAML load error: %s", e.what());
      return false;
    }
  }

  // --------------- 数学工具 ---------------
  static double clamp(double v, double lo, double hi)
  {
    return std::max(lo, std::min(v, hi));
  }

  static double huberWeight(double r)
  {
    double ar = std::abs(r);
    if (ar <= 1.0)
      return 1.0;
    return 1.0 / ar;
  }

  Eigen::Quaterniond weightedAverageQuaternion(const std::vector<Observation> &obs,
                                               const std::vector<double> &weights) const
  {
    Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
    for (size_t i = 0; i < obs.size(); ++i)
    {
      if (weights[i] <= 0.0)
        continue;
      Eigen::Vector4d qv;
      qv << obs[i].q.w(), obs[i].q.x(), obs[i].q.y(), obs[i].q.z();
      M += weights[i] * (qv * qv.transpose());
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(M);
    Eigen::Vector4d ev = es.eigenvectors().col(3);
    Eigen::Quaterniond q_mean(ev(0), ev(1), ev(2), ev(3));
    q_mean.normalize();
    return q_mean;
  }

  double quaternionAngularDistanceRad(const Eigen::Quaterniond &qa, const Eigen::Quaterniond &qb) const
  {
    double dot = qa.dot(qb);
    dot = clamp(dot, -1.0, 1.0);
    double angle = 2.0 * std::acos(std::abs(dot));
    return angle;
  }

  double computeWeight(const fiducial_msgs::FiducialTransform &tfm) const
  {
    double err = (tfm.object_error > 0.0) ? tfm.object_error : tfm.image_error;
    if (err <= 0.0)
      err = 1.0;
    return 1.0 / err;
  }

  tf2::Transform toTf2(const Eigen::Vector3d &t, const Eigen::Quaterniond &q) const
  {
    tf2::Transform T;
    T.setOrigin(tf2::Vector3(t.x(), t.y(), t.z()));
    tf2::Quaternion tfq(q.x(), q.y(), q.z(), q.w());
    tfq.normalize();
    T.setRotation(tfq);
    return T;
  }

  // --------------- 鲁棒融合 ---------------
  struct FuseResult
  {
    tf2::Transform T;
    size_t inliers{0};
    std::vector<int> dropped_faces;
    std::vector<std::string> drop_logs;
    bool valid{false};
  };

  FuseResult robustFuseSE3(std::vector<Observation> &obs)
  {
    FuseResult result;
    if (obs.empty())
      return result;

    // 四元数同半球：参考上一帧或第一个观测
    Eigen::Quaterniond q_ref;
    if (has_prev_)
    {
      tf2::Quaternion q = prev_.getRotation();
      q.normalize();
      q_ref = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
    }
    else
    {
      q_ref = obs.front().q;
    }

    for (auto &o : obs)
    {
      if (q_ref.dot(o.q) < 0.0)
      {
        o.q.coeffs() *= -1.0;
      }
    }

    std::vector<double> weights(obs.size(), 1.0);
    for (size_t i = 0; i < obs.size(); ++i)
    {
      weights[i] = obs[i].base_weight;
    }

    const double sigma_ang_rad = sigma_ang_deg_ * M_PI / 180.0;
    const double gate_ang_single = gate_ang_single_deg_ * M_PI / 180.0;
    const double gate_ang_multi = gate_ang_multi_deg_ * M_PI / 180.0;

    for (int iter = 0; iter < 2; ++iter)
    {
      double sum_w = 0.0;
      Eigen::Vector3d t_mean = Eigen::Vector3d::Zero();
      for (size_t i = 0; i < obs.size(); ++i)
      {
        if (weights[i] <= 0.0)
          continue;
        sum_w += weights[i];
        t_mean += weights[i] * obs[i].t;
      }

      if (sum_w <= 1e-9)
      {
        ROS_WARN("[Fusion] All observations dropped (zero weight)");
        return result;
      }

      t_mean /= sum_w;
      Eigen::Quaterniond q_mean = weightedAverageQuaternion(obs, weights);

      std::vector<double> new_w(obs.size(), 0.0);
      result.dropped_faces.clear();
      result.drop_logs.clear();

      for (size_t i = 0; i < obs.size(); ++i)
      {
        double ang_rad = quaternionAngularDistanceRad(q_mean, obs[i].q);
        double pos_err = (obs[i].t - t_mean).norm();

        bool is_multi = obs.size() >= 2;
        double gate_ang = is_multi ? gate_ang_multi : gate_ang_single;
        double gate_pos = is_multi ? gate_pos_multi_m_ : gate_pos_single_m_;

        if (ang_rad > gate_ang || pos_err > gate_pos)
        {
          result.dropped_faces.push_back(obs[i].face_id);
          std::ostringstream oss;
          oss << "face=" << obs[i].face_id << ", pos_err=" << pos_err
              << " m, ang_err=" << (ang_rad * 180.0 / M_PI) << " deg (gate)";
          result.drop_logs.push_back(oss.str());
          continue;
        }

        double w_ang = huberWeight(ang_rad / sigma_ang_rad);
        double w_pos = huberWeight(pos_err / sigma_pos_m_);
        new_w[i] = obs[i].base_weight * w_ang * w_pos;
      }

      weights.swap(new_w);
    }

    double sum_w = 0.0;
    Eigen::Vector3d t_mean = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < obs.size(); ++i)
    {
      if (weights[i] <= 0.0)
        continue;
      sum_w += weights[i];
      t_mean += weights[i] * obs[i].t;
    }

    if (sum_w <= 1e-9)
    {
      ROS_WARN("[Fusion] All observations rejected after robust weighting");
      return result;
    }

    t_mean /= sum_w;
    Eigen::Quaterniond q_mean = weightedAverageQuaternion(obs, weights);

    // 统计有效观测
    size_t inliers = 0;
    for (double w : weights)
    {
      if (w > 0.0)
        ++inliers;
    }

    result.T = toTf2(t_mean, q_mean);
    result.inliers = inliers;
    result.valid = (inliers >= static_cast<size_t>(min_inliers_));
    return result;
  }

  // --------------- 时间滤波 ---------------
  tf2::Transform temporalFilter(const tf2::Transform &T_new, const ros::Time &stamp)
  {
    if (!has_prev_)
    {
      prev_ = T_new;
      last_stamp_ = stamp;
      has_prev_ = true;
      return T_new;
    }

    double dt = (stamp - last_stamp_).toSec();
    if (dt <= 0.0 || dt > timeout_sec_)
    {
      prev_ = T_new;
      last_stamp_ = stamp;
      return T_new;
    }

    double max_step_pos = max_lin_speed_mps_ * dt;
    double max_step_ang = (max_ang_speed_dps_ * M_PI / 180.0) * dt;

    tf2::Vector3 p_prev = prev_.getOrigin();
    tf2::Vector3 p_new = T_new.getOrigin();
    tf2::Vector3 dp = p_new - p_prev;
    double dist = dp.length();
    if (dist > max_step_pos && dist > 1e-6)
    {
      dp *= (max_step_pos / dist);
      p_new = p_prev + dp;
    }

    tf2::Quaternion q_prev = prev_.getRotation();
    q_prev.normalize();
    tf2::Quaternion q_new = T_new.getRotation();
    q_new.normalize();
    if (q_prev.x() * q_new.x() + q_prev.y() * q_new.y() + q_prev.z() * q_new.z() + q_prev.w() * q_new.w() < 0.0)
    {
      q_new = tf2::Quaternion(-q_new.x(), -q_new.y(), -q_new.z(), -q_new.w());
    }

    tf2::Quaternion dq = q_prev.inverse() * q_new;
    dq.normalize();
    double ang = dq.getAngle();
    double alpha = 1.0;
    if (ang > max_step_ang && ang > 1e-6)
    {
      alpha = max_step_ang / ang;
    }
    tf2::Quaternion q_f = q_prev.slerp(q_new, alpha);
    q_f.normalize();

    tf2::Transform T_f;
    T_f.setOrigin(p_prev * (1.0 - alpha) + p_new * alpha);
    T_f.setRotation(q_f);

    prev_ = T_f;
    last_stamp_ = stamp;
    return T_f;
  }

  // --------------- 主回调 ---------------
  void callback(const fiducial_msgs::FiducialTransformArray::ConstPtr &msg)
  {
    std::vector<Observation> obs;
    obs.reserve(msg->transforms.size());

    for (const auto &tfm : msg->transforms)
    {
      auto it = face2cube_.find(tfm.fiducial_id);
      if (it == face2cube_.end())
        continue;

      tf2::Transform T_cf;
      tf2::fromMsg(tfm.transform, T_cf);
      tf2::Transform T_cc = T_cf * it->second;

      tf2::Quaternion q2 = T_cc.getRotation();
      q2.normalize();
      Observation o;
      o.face_id = tfm.fiducial_id;
      o.t = Eigen::Vector3d(T_cc.getOrigin().x(), T_cc.getOrigin().y(), T_cc.getOrigin().z());
      o.q = Eigen::Quaterniond(q2.w(), q2.x(), q2.y(), q2.z());
      o.base_weight = computeWeight(tfm);
      obs.push_back(o);
    }

    if (obs.empty())
    {
      ROS_DEBUG_THROTTLE(1.0, "[Fusion] No valid faces detected");
      return;
    }

    // 单面保护：与上一帧差异过大则保持上一帧
    if (obs.size() == 1 && has_prev_)
    {
      tf2::Transform T_single = toTf2(obs[0].t, obs[0].q);
      tf2::Vector3 dp = T_single.getOrigin() - prev_.getOrigin();
      double pos_diff = dp.length();
      tf2::Quaternion dq = prev_.getRotation().inverse() * T_single.getRotation();
      dq.normalize();
      double ang = dq.getAngle();
      if (ang > gate_ang_single_deg_ * M_PI / 180.0 || pos_diff > gate_pos_single_m_)
      {
        ROS_WARN("[Fusion] HOLD_LAST_GOOD (single face %d): pos jump %.3f m, ang jump %.1f deg",
                 obs[0].face_id, pos_diff, ang * 180.0 / M_PI);
        publish(prev_, msg->header.stamp, msg->header.frame_id.empty() ? camera_frame_default_ : msg->header.frame_id);
        return;
      }
    }

    FuseResult fused = robustFuseSE3(obs);
    if (!fused.valid)
    {
      ROS_WARN("[Fusion] HOLD_LAST_GOOD: no valid inliers (n_obs=%zu, dropped=%zu)",
               obs.size(), fused.dropped_faces.size());
      if (has_prev_)
      {
        publish(prev_, msg->header.stamp, msg->header.frame_id.empty() ? camera_frame_default_ : msg->header.frame_id);
      }
      return;
    }

    std::string cam_frame = msg->header.frame_id.empty() ? camera_frame_default_ : msg->header.frame_id;
    ros::Time stamp = msg->header.stamp;

    tf2::Transform T_out = temporalFilter(fused.T, stamp);

    tf2::Quaternion q_curr = T_out.getRotation();
    q_curr.normalize();
    if (has_prev_q_)
    {
      double dot = prev_cube_q_.x() * q_curr.x() + prev_cube_q_.y() * q_curr.y() +
                   prev_cube_q_.z() * q_curr.z() + prev_cube_q_.w() * q_curr.w();
      if (dot < 0.0)
      {
        q_curr = tf2::Quaternion(-q_curr.x(), -q_curr.y(), -q_curr.z(), -q_curr.w());
      }
    }
    T_out.setRotation(q_curr);
    prev_cube_q_ = q_curr;
    has_prev_q_ = true;

    ROS_INFO("[Fusion] obs=%zu, inliers=%zu, dropped=%zu", obs.size(), fused.inliers, fused.dropped_faces.size());
    for (const auto &log : fused.drop_logs)
    {
      ROS_WARN("[Fusion] drop %s", log.c_str());
    }

    publish(T_out, stamp, cam_frame);
  }

  // --------------- 发布 ---------------
  void publish(const tf2::Transform &T_cube, const ros::Time &stamp, const std::string &cam_frame)
  {
    tf2::Transform Tn = T_cube;
    tf2::Quaternion q = Tn.getRotation();
    q.normalize();
    Tn.setRotation(q);

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = cam_frame;
    tf_msg.child_frame_id = cube_frame_;
    tf_msg.transform = tf2::toMsg(Tn);
    tf_broadcaster_.sendTransform(tf_msg);

    geometry_msgs::PoseStamped pmsg;
    pmsg.header = tf_msg.header;
    pmsg.pose.position.x = tf_msg.transform.translation.x;
    pmsg.pose.position.y = tf_msg.transform.translation.y;
    pmsg.pose.position.z = tf_msg.transform.translation.z;
    pmsg.pose.orientation = tf_msg.transform.rotation;
    pub_.publish(pmsg);

    if (publish_tool_tf_ && has_tool_offset_)
    {
      tf2::Transform T_tool = T_cube * tool_offset_;
      geometry_msgs::TransformStamped tf_tool;
      tf_tool.header.stamp = stamp;
      tf_tool.header.frame_id = cam_frame;
      tf_tool.child_frame_id = tool_frame_;
      tf_tool.transform = tf2::toMsg(T_tool);
      tf_broadcaster_.sendTransform(tf_tool);
    }
  }

  // --------------- 成员 ---------------
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::map<int, tf2::Transform> face2cube_;
  tf2::Transform tool_offset_;
  bool has_tool_offset_;

  std::string faces_yaml_path_;
  std::string fiducial_topic_;
  std::string camera_frame_default_;
  std::string cube_frame_;
  std::string tool_frame_;
  bool publish_tool_tf_;

  double sigma_ang_deg_;
  double sigma_pos_m_;
  double gate_ang_single_deg_;
  double gate_ang_multi_deg_;
  double gate_pos_single_m_;
  double gate_pos_multi_m_;
  int min_inliers_;

  double max_lin_speed_mps_;
  double max_ang_speed_dps_;
  double timeout_sec_;

  bool has_prev_;
  tf2::Transform prev_;
  ros::Time last_stamp_;

  tf2::Quaternion prev_cube_q_;
  bool has_prev_q_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cube_multi_face_fusion_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try
  {
    CubeMultiFaceFusion fusion(nh, pnh);
    ros::spin();
    return 0;
  }
  catch (const std::exception &e)
  {
    ROS_FATAL("[Fusion] Fatal error: %s", e.what());
    return 1;
  }
}
