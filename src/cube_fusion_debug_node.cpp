#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>
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

class CubeFusionDebugNode
{
public:
  CubeFusionDebugNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
      : nh_(nh), pnh_(pnh), sync_(SyncPolicy(10), fid_sub_, fused_sub_)
  {
    const std::string default_faces_yaml = ros::package::getPath("jaka_close_contro") + "/config/cube_faces_current.yaml";
    pnh_.param<std::string>("faces_yaml", faces_yaml_path_, default_faces_yaml);
    pnh_.param<std::string>("fiducial_topic", fiducial_topic_, std::string("/tool_fiducials"));
    pnh_.param<std::string>("fused_topic", fused_topic_, std::string("/cube_center_fused"));
    pnh_.param<std::string>("cube_frame", cube_frame_, std::string("cube_center"));
    pnh_.param<std::string>("camera_frame", camera_frame_param_, std::string(""));
    pnh_.param<std::string>("output_csv", output_csv_path_, std::string(""));
    pnh_.param<bool>("publish_predicted_pose", publish_predicted_pose_, true);
    pnh_.param<double>("sync_slop_sec", sync_slop_sec_, 0.03);

    sync_.setMaxIntervalDuration(ros::Duration(sync_slop_sec_));

    if (faces_yaml_path_.empty() || !loadFacesYaml(faces_yaml_path_))
    {
      ROS_FATAL("[FusionDebug] faces_yaml 未配置或解析失败");
      throw std::runtime_error("faces_yaml missing");
    }

    fid_sub_.subscribe(nh_, fiducial_topic_, 1);
    fused_sub_.subscribe(nh_, fused_topic_, 1);
    sync_.registerCallback(boost::bind(&CubeFusionDebugNode::callback, this, _1, _2));

    ROS_INFO("[FusionDebug] faces_yaml=%s, fid_topic=%s, fused_topic=%s, cube_frame=%s", faces_yaml_path_.c_str(),
             fiducial_topic_.c_str(), fused_topic_.c_str(), cube_frame_.c_str());
    if (!output_csv_path_.empty())
    {
      csv_.open(output_csv_path_, std::ios::out | std::ios::trunc);
      if (csv_.is_open())
      {
        csv_ << "stamp,face_id,pos_err_m,ang_err_deg,fused_x,fused_y,fused_z,pred_x,pred_y,pred_z\n";
        ROS_INFO("[FusionDebug] 将结果写入 CSV: %s", output_csv_path_.c_str());
      }
      else
      {
        ROS_WARN("[FusionDebug] 无法打开 CSV: %s", output_csv_path_.c_str());
      }
    }
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<fiducial_msgs::FiducialTransformArray, geometry_msgs::PoseStamped>;

  bool loadFacesYaml(const std::string &path)
  {
    try
    {
      YAML::Node root = YAML::LoadFile(path);
      if (!root["faces"])
      {
        ROS_ERROR("[FusionDebug] faces_yaml 缺少 faces 段");
        return false;
      }
      for (const auto &f : root["faces"])
      {
        const int id = f["id"].as<int>();
        const auto tr = f["translation"];
        const auto rpy = f["rpy_deg"];
        const double tx = tr[0].as<double>();
        const double ty = tr[1].as<double>();
        const double tz = tr[2].as<double>();
        const double rr = rpy[0].as<double>() * M_PI / 180.0;
        const double pp = rpy[1].as<double>() * M_PI / 180.0;
        const double yy = rpy[2].as<double>() * M_PI / 180.0;
        tf2::Matrix3x3 R;
        R.setRPY(rr, pp, yy);
        tf2::Transform T_cube_face(R, tf2::Vector3(tx, ty, tz));
        face_to_cube_[id] = T_cube_face.inverse();
        ROS_INFO("[FusionDebug] face %d loaded: trans[%.3f %.3f %.3f], rpy_deg[%.1f %.1f %.1f]", id, tx, ty, tz,
                 rpy[0].as<double>(), rpy[1].as<double>(), rpy[2].as<double>());
      }
      return true;
    }
    catch (const std::exception &e)
    {
      ROS_ERROR("[FusionDebug] faces_yaml 解析失败: %s", e.what());
      return false;
    }
  }

  void callback(const fiducial_msgs::FiducialTransformArray::ConstPtr &fid_msg,
                const geometry_msgs::PoseStamped::ConstPtr &fused_msg)
  {
    if (fid_msg->transforms.empty())
    {
      ROS_WARN_THROTTLE(1.0, "[FusionDebug] 未检测到 fiducial，跳过");
      return;
    }

    tf2::Transform T_cam_cube_fused;
    tf2::fromMsg(fused_msg->pose, T_cam_cube_fused);
    T_cam_cube_fused.getRotation().normalize();
    const std::string cam_frame = !camera_frame_param_.empty() ? camera_frame_param_ : fused_msg->header.frame_id;

    std::vector<double> pos_errs;
    std::vector<double> ang_errs;

    for (const auto &ft : fid_msg->transforms)
    {
      auto it = face_to_cube_.find(ft.fiducial_id);
      if (it == face_to_cube_.end())
      {
        continue;
      }

      tf2::Transform T_cam_face;
      tf2::fromMsg(ft.transform, T_cam_face);
      tf2::Quaternion qf = T_cam_face.getRotation();
      qf.normalize();
      T_cam_face.setRotation(qf);

      tf2::Transform T_cam_cube_pred = T_cam_face * it->second;

      const tf2::Vector3 dp = T_cam_cube_pred.getOrigin() - T_cam_cube_fused.getOrigin();
      const double pos_err = dp.length();

      tf2::Quaternion q_pred = T_cam_cube_pred.getRotation();
      q_pred.normalize();
      tf2::Quaternion q_fused = T_cam_cube_fused.getRotation();
      q_fused.normalize();
      tf2::Quaternion q_err = q_fused.inverse() * q_pred;
      q_err.normalize();
      const double ang_err = q_err.getAngle() * 180.0 / M_PI;

      pos_errs.push_back(pos_err);
      ang_errs.push_back(ang_err);

      ROS_INFO_STREAM("[FusionDebug] face=" << ft.fiducial_id << " pos_err=" << pos_err << " m, ang_err=" << ang_err << " deg");

      if (publish_predicted_pose_)
      {
        auto pub = getPredPublisher(ft.fiducial_id);
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = fused_msg->header.stamp;
        pose.header.frame_id = cam_frame;
        pose.pose = transformToPoseMsg(T_cam_cube_pred);
        pub.publish(pose);
      }

      if (csv_.is_open())
      {
        csv_ << fused_msg->header.stamp.toSec() << "," << ft.fiducial_id << "," << pos_err << "," << ang_err << ","
             << T_cam_cube_fused.getOrigin().x() << "," << T_cam_cube_fused.getOrigin().y() << "," << T_cam_cube_fused.getOrigin().z() << ","
             << T_cam_cube_pred.getOrigin().x() << "," << T_cam_cube_pred.getOrigin().y() << "," << T_cam_cube_pred.getOrigin().z() << "\n";
      }
    }

    if (pos_errs.empty())
    {
      ROS_WARN_THROTTLE(1.0, "[FusionDebug] 没有与 faces_yaml 匹配的 fiducial，无法计算误差");
      return;
    }

    const StatResult pos_stat = computeStats(pos_errs);
    const StatResult ang_stat = computeStats(ang_errs);
    ROS_INFO_STREAM("[FusionDebug] 本帧统计 (faces=" << pos_errs.size() << ")\n"
                    << "  pos_err  mean=" << pos_stat.mean << " m, rms=" << pos_stat.rms << " m, std=" << pos_stat.stddev
                    << " m, max=" << pos_stat.max << " m\n"
                    << "  ang_err  mean=" << ang_stat.mean << " deg, rms=" << ang_stat.rms << " deg, std="
                    << ang_stat.stddev << " deg, max=" << ang_stat.max << " deg");

    if (pos_stat.max < 0.01 && ang_stat.max < 5.0)
    {
      ROS_INFO("[FusionDebug] 几何自洽检查通过：max(pos_err)=%.4f m, max(ang_err)=%.2f deg", pos_stat.max, ang_stat.max);
    }
    else
    {
      ROS_WARN("[FusionDebug] 几何自洽可能存在问题：max(pos_err)=%.4f m, max(ang_err)=%.2f deg", pos_stat.max, ang_stat.max);
    }
  }

  ros::Publisher getPredPublisher(int face_id)
  {
    auto it = pred_pubs_.find(face_id);
    if (it != pred_pubs_.end())
    {
      return it->second;
    }
    std::string topic = "/cube_center_pred/face_" + std::to_string(face_id);
    ros::Publisher pub = nh_.advertise<geometry_msgs::PoseStamped>(topic, 1);
    pred_pubs_[face_id] = pub;
    return pred_pubs_[face_id];
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::string faces_yaml_path_;
  std::string fiducial_topic_;
  std::string fused_topic_;
  std::string cube_frame_;
  std::string camera_frame_param_;
  std::string output_csv_path_;
  bool publish_predicted_pose_{true};
  double sync_slop_sec_{0.03};

  std::map<int, tf2::Transform> face_to_cube_;
  std::map<int, ros::Publisher> pred_pubs_;

  message_filters::Subscriber<fiducial_msgs::FiducialTransformArray> fid_sub_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> fused_sub_;
  message_filters::Synchronizer<SyncPolicy> sync_;

  std::ofstream csv_;
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
  }
  catch (const std::exception &e)
  {
    ROS_FATAL("[FusionDebug] 节点启动失败: %s", e.what());
    return 1;
  }

  return 0;
}

