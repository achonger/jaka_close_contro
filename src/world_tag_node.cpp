// world_tag_node.cpp
// 订阅 world_fiducials（camera->fiducial），融合 world_ids 多个标记，发布 world->camera TF

#include <ros/ros.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <sstream>

class WorldTagNode
{
public:
  WorldTagNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
      : nh_(nh), pnh_(pnh), has_prev_(false)
  {
    if (pnh_.hasParam("world_ids"))
    {
      pnh_.getParam("world_ids", world_ids_);
    }
    else if (pnh_.hasParam("world_id"))
    {
      int id_tmp = 500;
      pnh_.param("world_id", id_tmp, 500);
      world_ids_.push_back(id_tmp);
    }
    else
    {
      int id_tmp = 500;
      pnh_.param("world_fiducial_id", id_tmp, 500);
      world_ids_.push_back(id_tmp);
    }
    if (world_ids_.empty())
    {
      world_ids_.push_back(500);
    }

    pnh_.param<std::string>("fiducial_topic", fiducial_topic_, "/world_fiducials");
    pnh_.param<std::string>("world_frame", world_frame_, "world");
    pnh_.param<std::string>("camera_frame", camera_frame_, "zed2i_left_camera_optical_frame");
    pnh_.param("alpha_pos", alpha_pos_, 0.3);
    pnh_.param("alpha_rot", alpha_rot_, 0.3);

    ROS_INFO_STREAM("[WorldTag] world_ids = " << vecToStr(world_ids_));
    ROS_INFO_STREAM("[WorldTag] fiducial_topic    = " << fiducial_topic_);
    ROS_INFO_STREAM("[WorldTag] world_frame       = " << world_frame_);
    ROS_INFO_STREAM("[WorldTag] camera_frame      = " << camera_frame_);

    sub_ = nh_.subscribe(fiducial_topic_, 1, &WorldTagNode::fiducialCb, this);
  }

private:
  void fiducialCb(const fiducial_msgs::FiducialTransformArray::ConstPtr &msg)
  {
    std::vector<tf2::Transform> observations;
    std::vector<double> weights;
    std::vector<int> detected_ids;

    for (const auto &ft : msg->transforms)
    {
      detected_ids.push_back(ft.fiducial_id);
      if (std::find(world_ids_.begin(), world_ids_.end(), ft.fiducial_id) == world_ids_.end())
      {
        continue;
      }
      tf2::Quaternion q(ft.transform.rotation.x, ft.transform.rotation.y, ft.transform.rotation.z, ft.transform.rotation.w);
      q.normalize();
      tf2::Transform T_cam_world(q, tf2::Vector3(ft.transform.translation.x, ft.transform.translation.y, ft.transform.translation.z));
      tf2::Transform T_world_cam = T_cam_world.inverse();
      observations.push_back(T_world_cam);

      double w = 1.0;
      if (ft.object_error > 1e-6)
      {
        w = 1.0 / ft.object_error;
      }
      else if (ft.fiducial_area > 1e-6)
      {
        w = ft.fiducial_area;
      }
      weights.push_back(w);
    }

    if (observations.empty())
    {
      ROS_WARN_THROTTLE(2.0, "[WorldTag] World fiducial not found. Detected IDs: %s", vecToStr(detected_ids).c_str());
      return;
    }

    tf2::Transform T_world_cam = meanTransform(observations, weights);

    if (!has_prev_)
    {
      prev_T_world_cam_ = T_world_cam;
      has_prev_ = true;
    }
    else
    {
      tf2::Vector3 p_new = T_world_cam.getOrigin();
      tf2::Vector3 p_old = prev_T_world_cam_.getOrigin();
      tf2::Vector3 p_f = alpha_pos_ * p_new + (1.0 - alpha_pos_) * p_old;

      tf2::Quaternion q_new = T_world_cam.getRotation();
      tf2::Quaternion q_old = prev_T_world_cam_.getRotation();
      q_new.normalize();
      q_old.normalize();
      tf2::Quaternion q_f = q_old.slerp(q_new, alpha_rot_);
      q_f.normalize();

      prev_T_world_cam_.setOrigin(p_f);
      prev_T_world_cam_.setRotation(q_f);
    }

    geometry_msgs::TransformStamped ts;
    ts.header.stamp = msg->header.stamp;
    ts.header.frame_id = world_frame_;
    ts.child_frame_id = camera_frame_;

    const auto &p = prev_T_world_cam_.getOrigin();
    const auto &q = prev_T_world_cam_.getRotation();
    ts.transform.translation.x = p.x();
    ts.transform.translation.y = p.y();
    ts.transform.translation.z = p.z();
    ts.transform.rotation.x = q.x();
    ts.transform.rotation.y = q.y();
    ts.transform.rotation.z = q.z();
    ts.transform.rotation.w = q.w();

    tf_broadcaster_.sendTransform(ts);
  }

  static std::string vecToStr(const std::vector<int> &vec)
  {
    std::ostringstream ss;
    ss << "[";
    for (size_t i = 0; i < vec.size(); ++i)
    {
      ss << vec[i];
      if (i + 1 < vec.size())
        ss << ", ";
    }
    ss << "]";
    return ss.str();
  }

  static tf2::Transform meanTransform(const std::vector<tf2::Transform> &Ts,
                                      const std::vector<double> &weights)
  {
    if (Ts.empty())
    {
      return tf2::Transform::getIdentity();
    }
    const size_t N = Ts.size();
    std::vector<double> w = weights;
    if (w.size() != N)
    {
      w.assign(N, 1.0);
    }

    tf2::Transform T_ref = Ts.front();
    tf2::Vector3 t_ref = T_ref.getOrigin();
    tf2::Quaternion q_ref = T_ref.getRotation();
    q_ref.normalize();

    Eigen::Matrix3d R_ref = tf2ToEigen(tf2::Matrix3x3(q_ref));

    for (int iter = 0; iter < 10; ++iter)
    {
      Eigen::Vector3d t_acc = Eigen::Vector3d::Zero();
      Eigen::Vector3d phi_acc = Eigen::Vector3d::Zero();
      double w_sum = 0.0;
      for (size_t i = 0; i < N; ++i)
      {
        const tf2::Transform &T = Ts[i];
        tf2::Vector3 t = T.getOrigin();
        tf2::Quaternion q = T.getRotation();
        q.normalize();
        Eigen::Matrix3d R = tf2ToEigen(tf2::Matrix3x3(q));

        Eigen::Matrix3d R_err = R_ref.transpose() * R;
        Eigen::Vector3d phi = logSO3(R_err);
        t_acc += w[i] * Eigen::Vector3d(t.x() - t_ref.x(), t.y() - t_ref.y(), t.z() - t_ref.z());
        phi_acc += w[i] * phi;
        w_sum += w[i];
      }

      if (w_sum < 1e-9)
        break;
      Eigen::Vector3d dt = t_acc / w_sum;
      Eigen::Vector3d dphi = phi_acc / w_sum;
      if (dphi.norm() < 1e-6 && dt.norm() < 1e-6)
      {
        break;
      }
      t_ref += tf2::Vector3(dt.x(), dt.y(), dt.z());
      Eigen::Matrix3d R_update = expSO3(dphi);
      R_ref = R_ref * R_update;
    }

    tf2::Quaternion q_mean;
    tf2::Matrix3x3 R_tf = eigenToTf2(R_ref);
    R_tf.getRotation(q_mean);
    q_mean.normalize();

    tf2::Transform T_out;
    T_out.setOrigin(t_ref);
    T_out.setRotation(q_mean);
    return T_out;
  }

  static Eigen::Matrix3d tf2ToEigen(const tf2::Matrix3x3 &R_tf)
  {
    Eigen::Matrix3d R;
    for (int r = 0; r < 3; ++r)
    {
      for (int c = 0; c < 3; ++c)
      {
        R(r, c) = R_tf[r][c];
      }
    }
    return R;
  }

  static tf2::Matrix3x3 eigenToTf2(const Eigen::Matrix3d &R)
  {
    tf2::Matrix3x3 R_tf;
    for (int r = 0; r < 3; ++r)
    {
      for (int c = 0; c < 3; ++c)
      {
        R_tf[r][c] = R(r, c);
      }
    }
    return R_tf;
  }

  static Eigen::Vector3d logSO3(const Eigen::Matrix3d &R)
  {
    double cos_theta = (R.trace() - 1.0) * 0.5;
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    double theta = std::acos(cos_theta);
    if (theta < 1e-8)
    {
      Eigen::Vector3d v;
      v << R(2, 1) - R(1, 2),
          R(0, 2) - R(2, 0),
          R(1, 0) - R(0, 1);
      return 0.5 * v;
    }
    double sin_theta = std::sin(theta);
    Eigen::Vector3d v;
    v << R(2, 1) - R(1, 2),
        R(0, 2) - R(2, 0),
        R(1, 0) - R(0, 1);
    return (theta / (2.0 * sin_theta)) * v;
  }

  static Eigen::Matrix3d expSO3(const Eigen::Vector3d &phi)
  {
    double theta = phi.norm();
    if (theta < 1e-8)
    {
      Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
      Eigen::Matrix3d A;
      A << 0, -phi.z(), phi.y(),
          phi.z(), 0, -phi.x(),
          -phi.y(), phi.x(), 0;
      return R + A;
    }
    Eigen::Vector3d a = phi / theta;
    double s = std::sin(theta);
    double c = std::cos(theta);
    Eigen::Matrix3d A;
    A << 0, -a.z(), a.y(),
        a.z(), 0, -a.x(),
        -a.y(), a.x(), 0;
    Eigen::Matrix3d A2 = A * A;
    return Eigen::Matrix3d::Identity() + s * A + (1 - c) * A2;
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::vector<int> world_ids_;
  std::string fiducial_topic_, world_frame_, camera_frame_;
  double alpha_pos_, alpha_rot_;
  bool has_prev_;
  tf2::Transform prev_T_world_cam_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "world_tag_node");
  ros::NodeHandle nh, pnh("~");

  try
  {
    WorldTagNode node(nh, pnh);
    ROS_INFO("[WorldTag] Started successfully");
    ros::spin();
    return 0;
  }
  catch (const std::exception &e)
  {
    ROS_FATAL("[WorldTag] Initialization failed: %s", e.what());
    return 1;
  }
}
