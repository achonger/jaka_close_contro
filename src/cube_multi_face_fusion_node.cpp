// cube_multi_face_fusion_node.cpp
#include <ros/ros.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <jaka_close_contro/cube_geometry_utils.h>

#include <jaka_close_contro/CubeFusionStats.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <string>
#include <sstream>
#include <vector>
#include <limits>

struct Observation
{
  int face_id;
  Eigen::Vector3d t;
  Eigen::Matrix3d R;
  double base_weight;
};

class CubeMultiFaceFusion
{
public:
  CubeMultiFaceFusion(ros::NodeHandle &nh, ros::NodeHandle &pnh)
      : nh_(nh), pnh_(pnh),
        camera_frame_default_("zed2i_left_camera_optical_frame"),
        cube_frame_("cube_center"),
        has_iekf_state_(false)
  {
    const std::string default_faces_yaml = ros::package::getPath("jaka_close_contro") + "/config/cube_faces_current.yaml";
    pnh_.param<std::string>("faces_yaml", faces_yaml_path_, default_faces_yaml);
    pnh_.param<std::string>("fiducial_topic", fiducial_topic_, "tool_fiducials");
    pnh_.param<std::string>("camera_frame_default", camera_frame_default_, "zed2i_left_camera_optical_frame");
    pnh_.param<std::string>("cube_frame", cube_frame_, "cube_center");
    pnh_.param("robot_id", robot_id_, 1);
    pnh_.param("robot_stride", robot_stride_, 100);
    pnh_.param("face_id_base", face_id_base_, 10);

    // 鲁棒融合参数
    pnh_.param("sigma_ang_deg", sigma_ang_deg_, 6.0);
    pnh_.param("sigma_pos_m", sigma_pos_m_, 0.02);
    pnh_.param("gate_ang_single_deg", gate_ang_single_deg_, 35.0);
    pnh_.param("gate_ang_multi_deg", gate_ang_multi_deg_, 25.0);
    pnh_.param("gate_pos_single_m", gate_pos_single_m_, 0.08);
    pnh_.param("gate_pos_multi_m", gate_pos_multi_m_, 0.08);
    pnh_.param("min_inliers", min_inliers_, 1);

    // IEKF 噪声参数
    pnh_.param("proc_noise_pos_m", proc_noise_pos_m_, 0.02);
    pnh_.param("proc_noise_ang_deg", proc_noise_ang_deg_, 5.0);
    pnh_.param("meas_noise_pos_m", meas_noise_pos_m_, 0.005);
    pnh_.param("meas_noise_ang_deg", meas_noise_ang_deg_, 1.5);
    pnh_.param("timeout_sec", timeout_sec_, 0.5);

    ROS_INFO_STREAM("[Fusion] faces_yaml: " << faces_yaml_path_);
    ROS_INFO_STREAM("[Fusion] fiducial_topic: " << fiducial_topic_);
    ROS_INFO_STREAM("[Fusion] cube_frame: " << cube_frame_);
    ROS_INFO_STREAM("[Fusion] robot_id=%d, robot_stride=%d, face_id_base=%d", robot_id_, robot_stride_, face_id_base_);

    if (faces_yaml_path_.empty() || !cube_geometry::loadFacesYaml(faces_yaml_path_, face2cube_))
    {
      ROS_ERROR("[Fusion] Failed to load faces YAML. Exiting.");
      ros::shutdown();
      return;
    }
    applyRobotOffset();

    sub_ = nh_.subscribe<fiducial_msgs::FiducialTransformArray>(
        fiducial_topic_, 1, &CubeMultiFaceFusion::callback, this);
    pub_ = nh_.advertise<geometry_msgs::PoseStamped>("cube_center_fused", 1);
    stats_pub_ = nh_.advertise<jaka_close_contro::CubeFusionStats>("cube_fusion_stats", 1);
  }

private:
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

  static Eigen::Matrix3d skew(const Eigen::Vector3d &w)
  {
    Eigen::Matrix3d W;
    W << 0, -w.z(), w.y(),
        w.z(), 0, -w.x(),
        -w.y(), w.x(), 0;
    return W;
  }

  static Eigen::Matrix3d expSO3(const Eigen::Vector3d &phi)
  {
    double theta = phi.norm();
    if (theta < 1e-8)
    {
      return Eigen::Matrix3d::Identity() + skew(phi);
    }
    Eigen::Vector3d a = phi / theta;
    Eigen::Matrix3d A = skew(a);
    return Eigen::Matrix3d::Identity() + std::sin(theta) * A + (1.0 - std::cos(theta)) * (A * A);
  }

  static Eigen::Vector3d logSO3(const Eigen::Matrix3d &R)
  {
    double cos_theta = (R.trace() - 1.0) * 0.5;
    cos_theta = clamp(cos_theta, -1.0, 1.0);
    double theta = std::acos(cos_theta);
    if (theta < 1e-8)
    {
      Eigen::Vector3d v;
      v << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
      return 0.5 * v;
    }
    double sin_theta = std::sin(theta);
    Eigen::Vector3d v;
    v << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
    return (theta / (2.0 * sin_theta)) * v;
  }

  static Eigen::Matrix<double, 6, 1> se3LogError(
      const Eigen::Vector3d &t_pred, const Eigen::Matrix3d &R_pred,
      const Eigen::Vector3d &t_meas, const Eigen::Matrix3d &R_meas)
  {
    Eigen::Matrix3d R_rel = R_pred.transpose() * R_meas;
    Eigen::Vector3d phi = logSO3(R_rel);

    Eigen::Vector3d t_rel = R_pred.transpose() * (t_meas - t_pred);

    Eigen::Matrix<double, 6, 1> xi;
    xi.head<3>() = t_rel;
    xi.tail<3>() = phi;
    return xi;
  }

  static void se3ApplyError(
      const Eigen::Vector3d &t_pred, const Eigen::Matrix3d &R_pred,
      const Eigen::Matrix<double, 6, 1> &delta,
      Eigen::Vector3d &t_new, Eigen::Matrix3d &R_new)
  {
    Eigen::Vector3d rho = delta.head<3>();
    Eigen::Vector3d phi = delta.tail<3>();

    R_new = R_pred * expSO3(phi);
    t_new = t_pred + R_pred * rho;
  }

  double rotationAngularDistanceRad(const Eigen::Matrix3d &Ra, const Eigen::Matrix3d &Rb) const
  {
    Eigen::Matrix3d R_err = Ra.transpose() * Rb;
    Eigen::Vector3d phi = logSO3(R_err);
    return phi.norm();
  }

  Eigen::Matrix3d weightedAverageSO3(const std::vector<Observation> &obs, const std::vector<double> &weights) const
  {
    Eigen::Matrix3d R_mean = Eigen::Matrix3d::Identity();
    bool initialized = false;
    for (size_t i = 0; i < obs.size(); ++i)
    {
      if (weights[i] > 0.0)
      {
        R_mean = obs[i].R;
        initialized = true;
        break;
      }
    }
    if (!initialized)
    {
      return R_mean;
    }

    for (int iter = 0; iter < 5; ++iter)
    {
      Eigen::Vector3d delta = Eigen::Vector3d::Zero();
      double sum_w = 0.0;
      for (size_t i = 0; i < obs.size(); ++i)
      {
        if (weights[i] <= 0.0)
          continue;
        Eigen::Matrix3d R_err = R_mean.transpose() * obs[i].R;
        Eigen::Vector3d phi = logSO3(R_err);
        delta += weights[i] * phi;
        sum_w += weights[i];
      }
      if (sum_w <= 1e-9)
        break;
      delta /= sum_w;
      R_mean = R_mean * expSO3(delta);
      if (delta.norm() < 1e-6)
        break;
    }
    return R_mean;
  }

  double computeWeight(const fiducial_msgs::FiducialTransform &tfm) const
  {
    double err = (tfm.object_error > 0.0) ? tfm.object_error : tfm.image_error;
    if (err <= 0.0)
      err = 1.0;
    return 1.0 / err;
  }

  tf2::Transform toTf2(const Eigen::Vector3d &t, const Eigen::Matrix3d &R) const
  {
    tf2::Transform T;
    T.setOrigin(tf2::Vector3(t.x(), t.y(), t.z()));
    Eigen::Quaterniond q(R);
    q.normalize();
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

    auto selectBestObs = [&](const Eigen::Matrix3d &R_ref) -> size_t {
      size_t best = 0;
      double best_ang = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < obs.size(); ++i)
      {
        double ang = rotationAngularDistanceRad(R_ref, obs[i].R);
        if (ang < best_ang)
        {
          best_ang = ang;
          best = i;
        }
      }
      return best;
    };

    Eigen::Matrix3d R_ref;
    if (has_iekf_state_)
    {
      tf2::Quaternion q = iekf_X_.getRotation();
      q.normalize();
      Eigen::Quaterniond q_ref(q.w(), q.x(), q.y(), q.z());
      R_ref = q_ref.toRotationMatrix();
    }
    else
    {
      R_ref = obs.front().R;
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
        // 退化：全部被 gate，选择与参考最接近的单面作为结果
        size_t best = selectBestObs(R_ref);
        result.T = toTf2(obs[best].t, obs[best].R);
        result.inliers = 1;
        result.valid = (result.inliers >= static_cast<size_t>(min_inliers_));
        result.drop_logs.push_back("All obs gated, fallback to best single");
        return result;
      }

      t_mean /= sum_w;
      Eigen::Matrix3d R_mean = weightedAverageSO3(obs, weights);

      std::vector<double> new_w(obs.size(), 0.0);
      result.dropped_faces.clear();
      result.drop_logs.clear();

      for (size_t i = 0; i < obs.size(); ++i)
      {
        double ang_rad = rotationAngularDistanceRad(R_mean, obs[i].R);
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
      size_t best = selectBestObs(R_ref);
      result.T = toTf2(obs[best].t, obs[best].R);
      result.inliers = 1;
      result.valid = (result.inliers >= static_cast<size_t>(min_inliers_));
      result.drop_logs.push_back("All obs rejected after weighting, fallback to best single");
      return result;
    }

    t_mean /= sum_w;
    Eigen::Matrix3d R_mean = weightedAverageSO3(obs, weights);

    // 统计有效观测
    size_t inliers = 0;
    for (double w : weights)
    {
      if (w > 0.0)
        ++inliers;
    }

    result.T = toTf2(t_mean, R_mean);
    result.inliers = inliers;
    result.valid = (inliers >= static_cast<size_t>(min_inliers_));
    return result;
  }

  // --------------- IEKF ---------------
  tf2::Transform iekfUpdate(const tf2::Transform &Z_tf, const ros::Time &stamp, bool has_measurement)
  {
    if (!has_iekf_state_)
    {
      if (has_measurement)
      {
        iekf_X_ = Z_tf;
      }
      else
      {
        iekf_X_.setIdentity();
      }
      iekf_P_.setIdentity();
      iekf_P_ *= 1e-4;
      iekf_last_stamp_ = stamp;
      has_iekf_state_ = true;
      return iekf_X_;
    }

    double dt = (stamp - iekf_last_stamp_).toSec();
    if (dt <= 0.0 || dt > timeout_sec_)
    {
      if (has_measurement)
      {
        iekf_X_ = Z_tf;
      }
      iekf_P_.setIdentity();
      iekf_P_ *= 1e-4;
      iekf_last_stamp_ = stamp;
      return iekf_X_;
    }

    double dt_clamped = std::max(dt, 1e-3);
    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
    double q_pos = proc_noise_pos_m_;
    double q_ang = proc_noise_ang_deg_ * M_PI / 180.0;
    for (int i = 0; i < 3; ++i)
      Q(i, i) = q_pos * q_pos * dt_clamped;
    for (int i = 3; i < 6; ++i)
      Q(i, i) = q_ang * q_ang * dt_clamped;
    Eigen::Matrix<double, 6, 6> P_pred = iekf_P_ + Q;

    tf2::Vector3 p_pred_tf = iekf_X_.getOrigin();
    tf2::Quaternion q_pred_tf = iekf_X_.getRotation();
    q_pred_tf.normalize();
    Eigen::Vector3d t_pred(p_pred_tf.x(), p_pred_tf.y(), p_pred_tf.z());
    Eigen::Quaterniond q_pred_eig(q_pred_tf.w(), q_pred_tf.x(), q_pred_tf.y(), q_pred_tf.z());
    Eigen::Matrix3d R_pred = q_pred_eig.toRotationMatrix();

    if (!has_measurement)
    {
      iekf_P_ = P_pred;
      iekf_last_stamp_ = stamp;
      return iekf_X_;
    }

    tf2::Vector3 p_meas_tf = Z_tf.getOrigin();
    tf2::Quaternion q_meas_tf = Z_tf.getRotation();
    q_meas_tf.normalize();
    Eigen::Vector3d t_meas(p_meas_tf.x(), p_meas_tf.y(), p_meas_tf.z());
    Eigen::Quaterniond q_meas_eig(q_meas_tf.w(), q_meas_tf.x(), q_meas_tf.y(), q_meas_tf.z());
    Eigen::Matrix3d R_meas = q_meas_eig.toRotationMatrix();

    Eigen::Matrix<double, 6, 1> y = se3LogError(t_pred, R_pred, t_meas, R_meas);

    Eigen::Matrix<double, 6, 6> R_cov = Eigen::Matrix<double, 6, 6>::Zero();
    double r_pos = meas_noise_pos_m_;
    double r_ang = meas_noise_ang_deg_ * M_PI / 180.0;
    for (int i = 0; i < 3; ++i)
      R_cov(i, i) = r_pos * r_pos;
    for (int i = 3; i < 6; ++i)
      R_cov(i, i) = r_ang * r_ang;

    Eigen::Matrix<double, 6, 6> S = P_pred + R_cov;
    Eigen::Matrix<double, 6, 6> K = P_pred * S.inverse();

    Eigen::Matrix<double, 6, 1> delta = K * y;

    Eigen::Vector3d t_new;
    Eigen::Matrix3d R_new;
    se3ApplyError(t_pred, R_pred, delta, t_new, R_new);

    Eigen::Matrix<double, 6, 6> I6 = Eigen::Matrix<double, 6, 6>::Identity();
    iekf_P_ = (I6 - K) * P_pred * (I6 - K).transpose() + K * R_cov * K.transpose();

    iekf_X_ = toTf2(t_new, R_new);
    iekf_last_stamp_ = stamp;
    return iekf_X_;
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

      tf2::Vector3 p_cc = T_cc.getOrigin();
      tf2::Quaternion q_cc = T_cc.getRotation();
      q_cc.normalize();
      Eigen::Quaterniond q_cc_eig(q_cc.w(), q_cc.x(), q_cc.y(), q_cc.z());

      Observation o;
      o.face_id = tfm.fiducial_id;
      o.t = Eigen::Vector3d(p_cc.x(), p_cc.y(), p_cc.z());
      o.R = q_cc_eig.toRotationMatrix();
      o.base_weight = computeWeight(tfm);
      obs.push_back(o);
    }

    if (obs.empty())
    {
      ROS_DEBUG_THROTTLE(1.0, "[Fusion] No valid faces detected");
      return;
    }

    FuseResult fused = robustFuseSE3(obs);

    auto publishStats = [&](const tf2::Transform &T_use) {
      jaka_close_contro::CubeFusionStats stats_msg;
      stats_msg.valid = fused.valid;
      stats_msg.n_obs = static_cast<int32_t>(obs.size());
      stats_msg.inliers = static_cast<int32_t>(fused.inliers);
      if (!fused.dropped_faces.empty())
      {
        std::ostringstream oss;
        for (size_t i = 0; i < fused.dropped_faces.size(); ++i)
        {
          if (i)
            oss << ",";
          oss << fused.dropped_faces[i];
        }
        stats_msg.dropped_faces = oss.str();
      }

      tf2::Vector3 t_f = T_use.getOrigin();
      tf2::Quaternion q_f = T_use.getRotation();
      q_f.normalize();
      Eigen::Quaterniond qf(q_f.w(), q_f.x(), q_f.y(), q_f.z());
      Eigen::Matrix3d R_f = qf.toRotationMatrix();
      Eigen::Vector3d tf_v(t_f.x(), t_f.y(), t_f.z());

      for (const auto &o : obs)
      {
        stats_msg.pos_err_m.push_back(static_cast<float>((o.t - tf_v).norm()));
        double ang = rotationAngularDistanceRad(R_f, o.R) * 180.0 / M_PI;
        stats_msg.ang_err_deg.push_back(static_cast<float>(ang));
      }

      stats_pub_.publish(stats_msg);
    };

    std::string cam_frame = msg->header.frame_id.empty() ? camera_frame_default_ : msg->header.frame_id;
    ros::Time stamp = msg->header.stamp;

    if (!fused.valid)
    {
      ROS_WARN("[Fusion] HOLD_LAST_GOOD: no valid inliers (n_obs=%zu, dropped=%zu)",
               obs.size(), fused.dropped_faces.size());

      tf2::Transform use_tf;
      if (has_iekf_state_)
      {
        use_tf = iekfUpdate(iekf_X_, stamp, false);
      }
      else
      {
        use_tf = fused.T;
        iekf_X_ = fused.T;
        iekf_P_.setIdentity();
        iekf_P_ *= 1e-4;
        iekf_last_stamp_ = stamp;
        has_iekf_state_ = true;
      }

      publishStats(use_tf);

      if (has_iekf_state_)
      {
        publish(use_tf, msg->header.stamp, cam_frame);
      }
      return;
    }

    tf2::Transform T_out = iekfUpdate(fused.T, stamp, true);

    publishStats(T_out);

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

    geometry_msgs::PoseStamped pmsg;
    pmsg.header.stamp = stamp;
    pmsg.header.frame_id = cam_frame;
    tf2::Vector3 t = Tn.getOrigin();
    tf2::Quaternion q_msg = Tn.getRotation();
    geometry_msgs::Pose pose_msg;
    pose_msg.position.x = t.x();
    pose_msg.position.y = t.y();
    pose_msg.position.z = t.z();
    pose_msg.orientation = tf2::toMsg(q_msg);
    pmsg.pose = pose_msg;
    pub_.publish(pmsg);
  }

  // --------------- 成员 ---------------
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Publisher stats_pub_;

  std::map<int, tf2::Transform> face2cube_;
  int robot_id_{1};
  int robot_stride_{100};
  int face_id_base_{10};
  std::string faces_yaml_path_;
  std::string fiducial_topic_;
  std::string camera_frame_default_;
  std::string cube_frame_;

  double sigma_ang_deg_;
  double sigma_pos_m_;
  double gate_ang_single_deg_;
  double gate_ang_multi_deg_;
  double gate_pos_single_m_;
  double gate_pos_multi_m_;
  int min_inliers_;

  double proc_noise_pos_m_;
  double proc_noise_ang_deg_;
  double meas_noise_pos_m_;
  double meas_noise_ang_deg_;
  double timeout_sec_;

  bool has_iekf_state_;
  tf2::Transform iekf_X_;
  ros::Time iekf_last_stamp_;
  Eigen::Matrix<double, 6, 6> iekf_P_;

  void applyRobotOffset()
  {
    std::map<int, tf2::Transform> shifted;
    int robot_off = (robot_id_ - 1) * robot_stride_;
    for (const auto &kv : face2cube_)
    {
      int new_id = kv.first + robot_off;
      shifted[new_id] = kv.second;
      tf2::Vector3 t = kv.second.getOrigin();
      tf2::Quaternion q = kv.second.getRotation();
      ROS_INFO("[Fusion] face id %d -> %d (robot_off=%d), trans=[%.3f, %.3f, %.3f]",
               kv.first, new_id, robot_off, t.x(), t.y(), t.z());
    }
    face2cube_.swap(shifted);
  }
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
