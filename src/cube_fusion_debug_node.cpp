#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind/bind.hpp>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <jaka_close_contro/cube_geometry_utils.h>

namespace
{
struct StatSnapshot
{
  double mean{0.0};
  double rms{0.0};
  double max{0.0};
  size_t count{0};
};

struct FaceSample
{
  int face_id{0};
  tf2::Transform T_cam_face;
  tf2::Transform T_cam_cube;
  double pos_err{0.0};
  double ang_err_deg{0.0};
  double image_error{0.0};
  double object_error{0.0};
  double weight{0.0};
};

class ErrorAccumulator
{
public:
  explicit ErrorAccumulator(size_t window_size = 0)
      : window_size_(window_size)
  {
  }

  void addSample(double pos_err, double ang_err)
  {
    if (window_size_ > 0)
    {
      if (pos_vals_.size() >= window_size_)
      {
        popFront();
      }
      pos_vals_.push_back(pos_err);
      ang_vals_.push_back(ang_err);
      pos_sum_ += pos_err;
      pos_sumsq_ += pos_err * pos_err;
      ang_sum_ += ang_err;
      ang_sumsq_ += ang_err * ang_err;
      pos_max_ = std::max(pos_max_, pos_err);
      ang_max_ = std::max(ang_max_, ang_err);
    }
    else
    {
      ++count_;
      pos_sum_ += pos_err;
      pos_sumsq_ += pos_err * pos_err;
      ang_sum_ += ang_err;
      ang_sumsq_ += ang_err * ang_err;
      pos_max_ = std::max(pos_max_, pos_err);
      ang_max_ = std::max(ang_max_, ang_err);
    }
  }

  StatSnapshot snapshotPos() const
  {
    return snapshot(pos_sum_, pos_sumsq_, pos_max_);
  }

  StatSnapshot snapshotAng() const
  {
    return snapshot(ang_sum_, ang_sumsq_, ang_max_);
  }

  void reset()
  {
    pos_vals_.clear();
    ang_vals_.clear();
    pos_sum_ = pos_sumsq_ = ang_sum_ = ang_sumsq_ = 0.0;
    pos_max_ = ang_max_ = 0.0;
    count_ = 0;
  }

private:
  void popFront()
  {
    if (window_size_ == 0 || pos_vals_.empty())
    {
      return;
    }
    const double old_pos = pos_vals_.front();
    const double old_ang = ang_vals_.front();
    pos_vals_.pop_front();
    ang_vals_.pop_front();
    pos_sum_ -= old_pos;
    pos_sumsq_ -= old_pos * old_pos;
    ang_sum_ -= old_ang;
    ang_sumsq_ -= old_ang * old_ang;

    if (old_pos >= pos_max_ - 1e-12)
    {
      pos_max_ = 0.0;
      for (double v : pos_vals_)
      {
        pos_max_ = std::max(pos_max_, v);
      }
    }
    if (old_ang >= ang_max_ - 1e-12)
    {
      ang_max_ = 0.0;
      for (double v : ang_vals_)
      {
        ang_max_ = std::max(ang_max_, v);
      }
    }
  }

  StatSnapshot snapshot(double sum, double sumsq, double max_val) const
  {
    StatSnapshot out;
    const size_t n = (window_size_ > 0) ? pos_vals_.size() : count_;
    if (n == 0)
    {
      return out;
    }
    const double denom = static_cast<double>(n);
    out.count = n;
    out.mean = sum / denom;
    out.rms = std::sqrt(sumsq / denom);
    out.max = max_val;
    return out;
  }

  size_t window_size_{0};
  std::deque<double> pos_vals_;
  std::deque<double> ang_vals_;
  double pos_sum_{0.0};
  double pos_sumsq_{0.0};
  double ang_sum_{0.0};
  double ang_sumsq_{0.0};
  double pos_max_{0.0};
  double ang_max_{0.0};
  size_t count_{0};
};
} // namespace

class CubeFusionDebugNode
{
public:
  CubeFusionDebugNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
      : nh_(nh), pnh_(pnh), tf_listener_(tf_buffer_)
  {
    const std::string default_faces_yaml = ros::package::getPath("jaka_close_contro") + "/config/cube_faces_current.yaml";
    pnh_.param<std::string>("faces_yaml", faces_yaml_path_, default_faces_yaml);
    pnh_.param<std::string>("fiducial_topic", fiducial_topic_, std::string("/tool_fiducials"));
    pnh_.param<std::string>("fused_pose_topic", fused_pose_topic_, std::string("/cube_center_fused"));
    pnh_.param<std::string>("fused_frame", fused_frame_, std::string("cube_center_fused"));
    pnh_.param<std::string>("camera_frame", camera_frame_param_, std::string(""));
    pnh_.param<bool>("fused_from_tf", fused_from_tf_, false);
    pnh_.param<double>("print_rate", print_rate_hz_, 10.0);
    pnh_.param<std::string>("csv_path", csv_path_, std::string(""));
    pnh_.param<int>("stats_window", stats_window_size_, 0);
    pnh_.param<bool>("broadcast_tags", broadcast_tags_, true);
    pnh_.param<bool>("broadcast_faces", broadcast_faces_, true);
    pnh_.param<bool>("broadcast_fused", broadcast_fused_, true);

    if (faces_yaml_path_.empty() || !cube_geometry::loadFacesYaml(faces_yaml_path_, face_to_cube_))
    {
      throw std::runtime_error("faces_yaml missing or failed to load");
    }

    if (!csv_path_.empty())
    {
      csv_.open(csv_path_, std::ios::out | std::ios::trunc);
      if (csv_.is_open())
      {
        csv_ << "stamp,face_id,fused_x,fused_y,fused_z,fused_qx,fused_qy,fused_qz,fused_qw,pred_x,pred_y,pred_z,pred_qx,pred_qy,pred_qz,pred_qw,pos_err,ang_err_deg,image_error,object_error,weight\n";
        ROS_INFO("[FusionDebug] Writing CSV to %s", csv_path_.c_str());
      }
      else
      {
        ROS_WARN("[FusionDebug] Failed to open CSV: %s", csv_path_.c_str());
      }
    }

    if (print_rate_hz_ <= 0.0)
    {
      print_rate_hz_ = 10.0;
    }

    print_timer_ = nh_.createTimer(ros::Duration(1.0 / print_rate_hz_), &CubeFusionDebugNode::printTimer, this);

    if (fused_from_tf_)
    {
      fiducial_sub_tf_ = nh_.subscribe(fiducial_topic_, 1, &CubeFusionDebugNode::fiducialOnlyCallback, this);
      ROS_INFO("[FusionDebug] Using TF for fused pose. Listening to fiducials on %s", fiducial_topic_.c_str());
    }
    else
    {
      fid_sub_.reset(new message_filters::Subscriber<fiducial_msgs::FiducialTransformArray>(nh_, fiducial_topic_, 1));
      fused_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, fused_pose_topic_, 1));
      sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(20), *fid_sub_, *fused_sub_));
      sync_->registerCallback(boost::bind(&CubeFusionDebugNode::syncedCallback, this,
                                          boost::placeholders::_1, boost::placeholders::_2));
      ROS_INFO("[FusionDebug] Syncing fiducials(%s) with fused pose(%s)", fiducial_topic_.c_str(), fused_pose_topic_.c_str());
    }
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<fiducial_msgs::FiducialTransformArray, geometry_msgs::PoseStamped>;

  tf2::Quaternion normalizeQuat(const tf2::Quaternion &qin) const
  {
    tf2::Quaternion q = qin;
    q.normalize();
    return q;
  }

  double angularErrorDeg(const tf2::Quaternion &qa, const tf2::Quaternion &qb) const
  {
    const double dot = std::abs(qa.x() * qb.x() + qa.y() * qb.y() + qa.z() * qb.z() + qa.w() * qb.w());
    const double ang = 2.0 * std::acos(std::min(1.0, std::max(-1.0, dot)));
    return ang * 180.0 / M_PI;
  }

  geometry_msgs::TransformStamped toTransformMsg(const tf2::Transform &tf, const ros::Time &stamp,
                                                 const std::string &parent, const std::string &child) const
  {
    geometry_msgs::TransformStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = parent;
    msg.child_frame_id = child;
    msg.transform = tf2::toMsg(tf);
    return msg;
  }

  void broadcastIfNeeded(const tf2::Transform &tf, const ros::Time &stamp, const std::string &parent,
                         const std::string &child, bool enabled)
  {
    if (!enabled)
    {
      return;
    }
    tf2::Transform Tn = tf;
    Tn.setRotation(normalizeQuat(Tn.getRotation()));
    tf_broadcaster_.sendTransform(toTransformMsg(Tn, stamp, parent, child));
  }

  bool getFusedFromTf(const ros::Time &stamp, const std::string &cam_frame, tf2::Transform &out)
  {
    try
    {
      const ros::Duration timeout(0.1);
      geometry_msgs::TransformStamped tf_msg = tf_buffer_.lookupTransform(cam_frame, fused_frame_, stamp, timeout);
      tf2::fromMsg(tf_msg.transform, out);
      out.setRotation(normalizeQuat(out.getRotation()));
      return true;
    }
    catch (const std::exception &e)
    {
      ROS_WARN_THROTTLE(1.0, "[FusionDebug] lookupTransform failed: %s", e.what());
      return false;
    }
  }

  void fiducialOnlyCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr &fid_msg)
  {
    const std::string cam_frame = !camera_frame_param_.empty() ? camera_frame_param_ : fid_msg->header.frame_id;
    tf2::Transform fused;
    if (!getFusedFromTf(fid_msg->header.stamp, cam_frame, fused))
    {
      return;
    }
    process(fid_msg, fused, cam_frame);
  }

  void syncedCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr &fid_msg,
                      const geometry_msgs::PoseStamped::ConstPtr &fused_msg)
  {
    tf2::Transform fused;
    tf2::fromMsg(fused_msg->pose, fused);
    fused.setRotation(normalizeQuat(fused.getRotation()));
    const std::string cam_frame = !camera_frame_param_.empty() ? camera_frame_param_ : fused_msg->header.frame_id;
    process(fid_msg, fused, cam_frame, fused_msg->header.stamp);
  }

  void process(const fiducial_msgs::FiducialTransformArray::ConstPtr &fid_msg, const tf2::Transform &fused,
               const std::string &cam_frame, const ros::Time &stamp_override = ros::Time(0))
  {
    const ros::Time stamp = stamp_override.isZero() ? fid_msg->header.stamp : stamp_override;
    if (fid_msg->transforms.empty())
    {
      ROS_WARN_THROTTLE(1.0, "[FusionDebug] No fiducials detected");
      return;
    }

    std::vector<FaceSample> frame_faces;
    frame_faces.reserve(fid_msg->transforms.size());

    for (const auto &ft : fid_msg->transforms)
    {
      const auto it = face_to_cube_.find(ft.fiducial_id);
      if (it == face_to_cube_.end())
      {
        continue;
      }

      tf2::Transform T_cam_face;
      tf2::fromMsg(ft.transform, T_cam_face);
      T_cam_face.setRotation(normalizeQuat(T_cam_face.getRotation()));

      broadcastIfNeeded(T_cam_face, stamp, cam_frame, "tag_" + std::to_string(ft.fiducial_id), broadcast_tags_);

      tf2::Transform T_cam_cube = T_cam_face * it->second;
      T_cam_cube.setRotation(normalizeQuat(T_cam_cube.getRotation()));

      const tf2::Vector3 dp = T_cam_cube.getOrigin() - fused.getOrigin();
      const double pos_err = dp.length();
      const double ang_err = angularErrorDeg(T_cam_cube.getRotation(), fused.getRotation());

      auto it_stat = stats_.find(ft.fiducial_id);
      if (it_stat == stats_.end())
      {
        it_stat = stats_.emplace(ft.fiducial_id,
                                 ErrorAccumulator(static_cast<size_t>(std::max(0, stats_window_size_))))
                      .first;
      }
      ErrorAccumulator &acc = it_stat->second;
      acc.addSample(pos_err, ang_err);

      const double err_raw = (ft.object_error > 0.0) ? ft.object_error : ft.image_error;
      const double weight = (err_raw > 1e-9) ? (1.0 / err_raw) : 1.0;

      FaceSample sample;
      sample.face_id = ft.fiducial_id;
      sample.T_cam_face = T_cam_face;
      sample.T_cam_cube = T_cam_cube;
      sample.pos_err = pos_err;
      sample.ang_err_deg = ang_err;
      sample.image_error = ft.image_error;
      sample.object_error = ft.object_error;
      sample.weight = weight;
      frame_faces.push_back(sample);

      broadcastIfNeeded(T_cam_cube, stamp, cam_frame, "cube_center_face_" + std::to_string(ft.fiducial_id), broadcast_faces_);

      if (csv_.is_open())
      {
        tf2::Quaternion qf = fused.getRotation();
        tf2::Quaternion qp = T_cam_cube.getRotation();
        csv_ << std::fixed << std::setprecision(6) << stamp.toSec() << ',' << ft.fiducial_id << ','
             << fused.getOrigin().x() << ',' << fused.getOrigin().y() << ',' << fused.getOrigin().z() << ','
             << qf.x() << ',' << qf.y() << ',' << qf.z() << ',' << qf.w() << ','
             << T_cam_cube.getOrigin().x() << ',' << T_cam_cube.getOrigin().y() << ',' << T_cam_cube.getOrigin().z() << ','
             << qp.x() << ',' << qp.y() << ',' << qp.z() << ',' << qp.w() << ','
             << pos_err << ',' << ang_err << ',' << ft.image_error << ',' << ft.object_error << ',' << weight << '\n';
      }
    }

    if (frame_faces.empty())
    {
      ROS_WARN_THROTTLE(1.0, "[FusionDebug] No faces match faces_yaml IDs");
      return;
    }

    last_frame_faces_ = frame_faces;
    last_fused_pose_ = fused;
    last_cam_frame_ = cam_frame;
    last_stamp_ = stamp;
    has_last_frame_ = true;

    broadcastIfNeeded(fused, stamp, cam_frame, fused_frame_, broadcast_fused_ && !fused_from_tf_);
  }

  void printTimer(const ros::TimerEvent &)
  {
    if (!has_last_frame_)
    {
      return;
    }

    tf2::Vector3 t = last_fused_pose_.getOrigin();
    tf2::Quaternion q = last_fused_pose_.getRotation();
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "[FusionDebug] fused pose (" << last_cam_frame_ << "): t=[" << t.x() << ", " << t.y() << ", " << t.z()
        << "], q=[" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << "]";
    ROS_INFO_STREAM(oss.str());

    for (const auto &sample : last_frame_faces_)
    {
      const auto it = stats_.find(sample.face_id);
      StatSnapshot pos_stat;
      StatSnapshot ang_stat;
      if (it != stats_.end())
      {
        pos_stat = it->second.snapshotPos();
        ang_stat = it->second.snapshotAng();
      }

      const tf2::Vector3 tp = sample.T_cam_cube.getOrigin();
      const tf2::Quaternion qp = sample.T_cam_cube.getRotation();

      std::ostringstream line;
      line << std::fixed << std::setprecision(3);
      line << "[FusionDebug] face=" << sample.face_id
           << " pos_err=" << sample.pos_err << " m, ang_err=" << sample.ang_err_deg << " deg"
           << " | cube_from_face t=[" << tp.x() << ", " << tp.y() << ", " << tp.z() << "]"
           << " q=[" << qp.x() << ", " << qp.y() << ", " << qp.z() << ", " << qp.w() << "]";
      if (pos_stat.count > 0)
      {
        line << " | stats pos(mean/rms/max)= " << pos_stat.mean << "/" << pos_stat.rms << "/" << pos_stat.max
             << " m, ang(mean/rms/max)= " << ang_stat.mean << "/" << ang_stat.rms << "/" << ang_stat.max;
      }
      ROS_INFO_STREAM(line.str());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::string faces_yaml_path_;
  std::string fiducial_topic_;
  std::string fused_pose_topic_;
  std::string fused_frame_;
  std::string camera_frame_param_;
  bool fused_from_tf_{false};
  double print_rate_hz_{10.0};
  std::string csv_path_;
  int stats_window_size_{0};
  bool broadcast_tags_{true};
  bool broadcast_faces_{true};
  bool broadcast_fused_{true};

  std::map<int, tf2::Transform> face_to_cube_;
  std::map<int, ErrorAccumulator> stats_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::unique_ptr<message_filters::Subscriber<fiducial_msgs::FiducialTransformArray>> fid_sub_;
  std::unique_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> fused_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  ros::Subscriber fiducial_sub_tf_;

  ros::Timer print_timer_;
  std::ofstream csv_;

  tf2::Transform last_fused_pose_;
  std::string last_cam_frame_;
  ros::Time last_stamp_;
  std::vector<FaceSample> last_frame_faces_;
  bool has_last_frame_{false};
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cube_fusion_debug_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try
  {
    CubeFusionDebugNode node(nh, pnh);
    ros::spin();
    return 0;
  }
  catch (const std::exception &e)
  {
    ROS_FATAL("[FusionDebug] Failed to start: %s", e.what());
    return 1;
  }
}

