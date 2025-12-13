#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

#include <jaka_close_contro/WorldRobotCollectSample.h>
#include <jaka_close_contro/WorldRobotCalibration.h>
#include <jaka_close_contro/CubeFusionStats.h>

#include <string>
#include <vector>
#include <cmath>
#include <fstream>
#include <iostream>

class WorldRobotAutocalibManager
{
public:
  WorldRobotAutocalibManager(ros::NodeHandle &nh, ros::NodeHandle &pnh)
      : nh_(nh), pnh_(pnh), tf_listener_(tf_buffer_)
  {
    pnh_.param<std::string>("world_frame", world_frame_, std::string("world"));
    pnh_.param<std::string>("camera_frame", camera_frame_, std::string("zed2i_left_camera_optical_frame"));
    pnh_.param<std::string>("cube_frame", cube_frame_, std::string("cube_center"));
    pnh_.param<std::string>("robot_base_frame", robot_base_frame_, std::string("Link_0"));
    pnh_.param<std::string>("tool_frame", tool_frame_, tool_frame_);
    pnh_.param<std::string>("cube_topic", cube_topic_, std::string("/cube_center_fused"));
    pnh_.param<std::string>("fusion_stats_topic", stats_topic_, stats_topic_);

    pnh_.param<double>("min_translation", min_translation_, 0.05);
    pnh_.param<double>("min_angle_deg", min_angle_deg_, 15.0);
    pnh_.param<int>("min_samples", min_samples_, 16);
    pnh_.param<bool>("auto_solve", auto_solve_, true);

    const std::string default_yaml = ros::package::getPath("jaka_close_contro") + "/config/world_robot_extrinsic.yaml";
    pnh_.param<std::string>("extrinsic_yaml", extrinsic_yaml_, default_yaml);
    const std::string default_joint_csv = ros::package::getPath("jaka_close_contro") + "/config/joint_calib_samples.csv";
    pnh_.param<std::string>("joint_sample_csv", joint_sample_csv_, default_joint_csv);
    const std::string default_cube_faces_template = ros::package::getPath("jaka_close_contro") + "/config/cube_faces_ideal.yaml";
    const std::string default_cube_faces_output = ros::package::getPath("jaka_close_contro") + "/config/cube_faces_current.yaml";
    pnh_.param<std::string>("cube_faces_yaml_template", cube_faces_yaml_template_, default_cube_faces_template);
    pnh_.param<std::string>("cube_faces_yaml_output", cube_faces_yaml_output_, default_cube_faces_output);

    // 重置上一轮的 CSV 记录，防止残留导致统计混淆
    std::ofstream(joint_sample_csv_, std::ios::out | std::ios::trunc).close();

    collect_client_ = nh_.serviceClient<jaka_close_contro::WorldRobotCollectSample>("/collect_world_robot_sample");
    solve_client_ = nh_.serviceClient<jaka_close_contro::WorldRobotCalibration>("/solve_world_robot_calibration");

    cube_sub_ = nh_.subscribe(cube_topic_, 1, &WorldRobotAutocalibManager::cubeCallback, this);
    stats_sub_ = nh_.subscribe(stats_topic_, 1, &WorldRobotAutocalibManager::statsCallback, this);
    timer_ = nh_.createTimer(ros::Duration(0.1), &WorldRobotAutocalibManager::timerCallback, this);

    ROS_INFO("[AutoCalib] 自动世界-机器人标定管理器启动：world=%s, base=%s, cube=%s, camera=%s", world_frame_.c_str(), robot_base_frame_.c_str(), cube_frame_.c_str(), camera_frame_.c_str());
    ROS_INFO("[AutoCalib] 阈值：Δp >= %.3f m 或 Δθ >= %.1f 度 触发采样，目标样本数=%d", min_translation_, min_angle_deg_, min_samples_);
    ROS_INFO("[AutoCalib] cube_topic=%s, stats_topic=%s, extrinsic_yaml=%s", cube_topic_.c_str(), stats_topic_.c_str(),
             extrinsic_yaml_.c_str());
    ROS_INFO("[AutoCalib] 联合标定样本 CSV: %s", joint_sample_csv_.c_str());
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::ServiceClient collect_client_;
  ros::ServiceClient solve_client_;
  ros::Subscriber cube_sub_;
  ros::Subscriber stats_sub_;
  ros::Timer timer_;

  geometry_msgs::PoseStamped latest_cube_cam_;
  bool has_cube_pose_ = false;

  jaka_close_contro::CubeFusionStats latest_stats_;
  bool has_stats_ = false;

  geometry_msgs::PoseStamped last_sample_world_;
  bool has_last_sample_world_ = false;

  size_t sample_count_ = 0;
  bool solved_ = false;

  std::string world_frame_;
  std::string camera_frame_;
  std::string cube_frame_;
  std::string robot_base_frame_;
  std::string tool_frame_ = "Link_6";
  std::string cube_topic_;
  std::string stats_topic_ = "/cube_fusion_stats";
  std::string joint_sample_csv_;
  bool csv_initialized_ = false;
  double min_translation_ = 0.05;
  double min_angle_deg_ = 15.0;
  int min_samples_ = 16;
  bool auto_solve_ = true;
  std::string extrinsic_yaml_;
  std::string cube_faces_yaml_template_;
  std::string cube_faces_yaml_output_;
  jaka_close_contro::WorldRobotCalibration::Response latest_result_;

  void cubeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    latest_cube_cam_ = *msg;
    has_cube_pose_ = true;
  }

  void statsCallback(const jaka_close_contro::CubeFusionStats::ConstPtr &msg)
  {
    latest_stats_ = *msg;
    has_stats_ = true;
  }

  void timerCallback(const ros::TimerEvent &)
  {
    if (solved_)
    {
      return;
    }

    if (!has_cube_pose_)
    {
      return;
    }

    if (!has_stats_ || !latest_stats_.valid || latest_stats_.inliers < 1)
    {
      return;
    }

    geometry_msgs::PoseStamped cube_world;
    if (!transformToWorld(latest_cube_cam_, cube_world))
    {
      return;
    }

    if (shouldSample(cube_world))
    {
      callSample(cube_world);
    }

    if (auto_solve_ && !solved_ && sample_count_ >= static_cast<size_t>(min_samples_))
    {
      callSolve();
    }
  }

  bool transformToWorld(const geometry_msgs::PoseStamped &in, geometry_msgs::PoseStamped &out)
  {
    try
    {
      tf_buffer_.transform(in, out, world_frame_, ros::Duration(0.2));
      return true;
    }
    catch (const tf2::TransformException &ex)
    {
      ROS_WARN_THROTTLE(1.0, "[AutoCalib] 无法转换 cube 到 world：%s", ex.what());
      return false;
    }
  }

  bool shouldSample(const geometry_msgs::PoseStamped &cube_world)
  {
    if (!has_last_sample_world_)
    {
      return true;
    }

    const double dp = positionDelta(cube_world, last_sample_world_);
    const double dth = orientationDeltaDeg(cube_world, last_sample_world_);

    return (dp >= min_translation_) || (dth >= min_angle_deg_);
  }

  double positionDelta(const geometry_msgs::PoseStamped &a, const geometry_msgs::PoseStamped &b) const
  {
    const double dx = a.pose.position.x - b.pose.position.x;
    const double dy = a.pose.position.y - b.pose.position.y;
    const double dz = a.pose.position.z - b.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  double orientationDeltaDeg(const geometry_msgs::PoseStamped &a, const geometry_msgs::PoseStamped &b) const
  {
    tf2::Quaternion qa, qb;
    tf2::fromMsg(a.pose.orientation, qa);
    tf2::fromMsg(b.pose.orientation, qb);
    qa.normalize();
    qb.normalize();
    const tf2::Quaternion dq = qb.inverse() * qa;
    const double angle = dq.getAngle();
    return angle * 180.0 / M_PI;
  }

  void callSample(const geometry_msgs::PoseStamped &cube_world)
  {
    if (!collect_client_.exists())
    {
      collect_client_.waitForExistence();
    }

    jaka_close_contro::WorldRobotCollectSample srv;
    if (!collect_client_.call(srv))
    {
      ROS_ERROR("[AutoCalib] /collect_world_robot_sample 调用失败（通信错误）");
      return;
    }

    if (!srv.response.success)
    {
      ROS_WARN("[AutoCalib] 采样失败：%s", srv.response.message.c_str());
      return;
    }

    sample_count_ = parseSampleCount(srv.response.message, sample_count_ + 1);
    has_last_sample_world_ = true;
    last_sample_world_ = cube_world;

    // 记录联合标定样本
    geometry_msgs::TransformStamped T_base_tool;
    try
    {
      T_base_tool = tf_buffer_.lookupTransform(robot_base_frame_, tool_frame_, ros::Time(0));
    }
    catch (const tf2::TransformException &ex)
    {
      ROS_WARN_THROTTLE(1.0, "[AutoCalib] 无法获取 %s->%s TF：%s", robot_base_frame_.c_str(), tool_frame_.c_str(), ex.what());
    }
    if (!latest_cube_cam_.header.frame_id.empty() && has_stats_ && latest_stats_.valid && latest_stats_.inliers > 0)
    {
      appendCsvSample(latest_cube_cam_, T_base_tool, latest_stats_);
    }

    ROS_INFO_STREAM("[AutoCalib] 采样成功 #" << sample_count_
                    << "\n  world_cube_meas.pos  = (" << srv.response.world_cube_pos_x << ", "
                    << srv.response.world_cube_pos_y << ", " << srv.response.world_cube_pos_z << ")"
                    << "\n  world_cube_meas.quat = (" << srv.response.world_cube_quat_x << ", "
                    << srv.response.world_cube_quat_y << ", " << srv.response.world_cube_quat_z << ", "
                    << srv.response.world_cube_quat_w << ")"
                    << "\n  base_cube_pred.pos   = (" << srv.response.base_cube_pos_x << ", "
                    << srv.response.base_cube_pos_y << ", " << srv.response.base_cube_pos_z << ")"
                    << "\n  base_cube_pred.quat  = (" << srv.response.base_cube_quat_x << ", "
                    << srv.response.base_cube_quat_y << ", " << srv.response.base_cube_quat_z << ", "
                    << srv.response.base_cube_quat_w << ")");
  }

  size_t parseSampleCount(const std::string &msg, size_t fallback) const
  {
    const std::size_t pos = msg.find_last_of('=');
    if (pos == std::string::npos || pos + 1 >= msg.size())
    {
      return fallback;
    }

    try
    {
      return static_cast<size_t>(std::stoul(msg.substr(pos + 1)));
    }
    catch (const std::exception &)
    {
      return fallback;
    }
  }

  void appendCsvSample(const geometry_msgs::PoseStamped &cube_cam,
                       const geometry_msgs::TransformStamped &base_tool,
                       const jaka_close_contro::CubeFusionStats &stats)
  {
    if (base_tool.header.frame_id.empty())
    {
      ROS_WARN_THROTTLE(2.0, "[AutoCalib] 未获取到 base->tool TF，跳过 CSV 记录");
      return;
    }

    std::ofstream fout;
    fout.setf(std::ios::fixed);
    fout.precision(6);
    if (!csv_initialized_)
    {
      fout.open(joint_sample_csv_, std::ios::out | std::ios::trunc);
      fout << "stamp,base_tx,base_ty,base_tz,base_qx,base_qy,base_qz,base_qw,cube_tx,cube_ty,cube_tz,cube_qx,cube_qy,cube_qz,cube_qw,n_obs,inliers,dropped_faces\n";
      csv_initialized_ = true;
    }
    else
    {
      fout.open(joint_sample_csv_, std::ios::out | std::ios::app);
    }

    tf2::Transform T_bt;
    tf2::fromMsg(base_tool.transform, T_bt);
    tf2::Vector3 tb = T_bt.getOrigin();
    tf2::Quaternion qb = T_bt.getRotation();
    qb.normalize();

    tf2::Transform T_ct;
    tf2::fromMsg(cube_cam.pose, T_ct);
    tf2::Vector3 tc = T_ct.getOrigin();
    tf2::Quaternion qc = T_ct.getRotation();
    qc.normalize();

    fout << cube_cam.header.stamp.toSec() << ","
         << tb.x() << "," << tb.y() << "," << tb.z() << ","
         << qb.x() << "," << qb.y() << "," << qb.z() << "," << qb.w() << ","
         << tc.x() << "," << tc.y() << "," << tc.z() << ","
         << qc.x() << "," << qc.y() << "," << qc.z() << "," << qc.w() << ","
         << stats.n_obs << "," << stats.inliers << "," << stats.dropped_faces << "\n";

    fout.close();
  }

  void callSolve()
  {
    if (!solve_client_.exists())
    {
      solve_client_.waitForExistence();
    }

    jaka_close_contro::WorldRobotCalibration srv;
    if (!solve_client_.call(srv))
    {
      ROS_ERROR("[AutoCalib] /solve_world_robot_calibration 调用失败（通信错误）");
      return;
    }

    if (!srv.response.success)
    {
      ROS_WARN("[AutoCalib] 标定失败：%s", srv.response.message.c_str());
      return;
    }

    solved_ = true;

    latest_result_ = srv.response;

    ROS_INFO("[AutoCalib] 标定成功：translation = [%.3f, %.3f, %.3f], rotation(xyzw) = [%.4f, %.4f, %.4f, %.4f]",
             srv.response.translation.x, srv.response.translation.y, srv.response.translation.z,
             srv.response.rotation.x, srv.response.rotation.y, srv.response.rotation.z, srv.response.rotation.w);

    ROS_INFO("[AutoCalib] 残差统计：pos RMSE=%.4f m, rot RMSE=%.3f deg", srv.response.position_rmse_m,
             srv.response.orientation_rmse_deg);
    ROS_INFO("[AutoCalib] 外参离散度：pos RMSE=%.4f m, rot RMSE=%.3f deg", srv.response.extrinsic_pos_rmse_m,
             srv.response.extrinsic_rot_rmse_deg);

    saveExtrinsic(srv.response, sample_count_);
  }

  void saveExtrinsic(const jaka_close_contro::WorldRobotCalibration::Response &res, size_t sample_count)
  {
    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "world_frame" << YAML::Value << world_frame_;
    emitter << YAML::Key << "robot_base_frame" << YAML::Value << robot_base_frame_;
    emitter << YAML::Key << "translation" << YAML::Value;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "x" << YAML::Value << res.translation.x;
    emitter << YAML::Key << "y" << YAML::Value << res.translation.y;
    emitter << YAML::Key << "z" << YAML::Value << res.translation.z;
    emitter << YAML::EndMap;
    emitter << YAML::Key << "rotation" << YAML::Value;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "x" << YAML::Value << res.rotation.x;
    emitter << YAML::Key << "y" << YAML::Value << res.rotation.y;
    emitter << YAML::Key << "z" << YAML::Value << res.rotation.z;
    emitter << YAML::Key << "w" << YAML::Value << res.rotation.w;
    emitter << YAML::EndMap;
    emitter << YAML::EndMap;

    try
    {
      std::ofstream fout(extrinsic_yaml_);
      fout.setf(std::ios::fixed);
      fout.precision(6);

      const ros::Time now = ros::Time::now();
      fout << "# World-robot extrinsic calibration\n";
      fout << "# Generated at: " << now << " (ROS time)\n";
      fout << "# Samples used: " << sample_count << "\n";
      fout << "# Residuals (measurement vs prediction):\n";
      fout << "#   position_rmse_m      = " << res.position_rmse_m << " m\n";
      fout << "#   orientation_rmse_deg = " << res.orientation_rmse_deg << " deg\n";
      fout << "# Extrinsic dispersion (over calibration runs):\n";
      fout << "#   extrinsic_pos_rmse_m  = " << res.extrinsic_pos_rmse_m << " m\n";
      fout << "#   extrinsic_rot_rmse_deg= " << res.extrinsic_rot_rmse_deg << " deg\n\n";

      fout << emitter.c_str() << std::endl;
      fout.close();
      ROS_INFO("[AutoCalib] 已写入外参文件：%s", extrinsic_yaml_.c_str());
    }
    catch (const std::exception &e)
    {
      ROS_ERROR("[AutoCalib] 保存外参失败：%s", e.what());
    }

    try
    {
      YAML::Node node = YAML::LoadFile(cube_faces_yaml_template_);
      YAML::Node tool_offset = node["tool_offset"];
      if (!tool_offset)
      {
        tool_offset = node["tool_offset"] = YAML::Node(YAML::NodeType::Map);
      }

      YAML::Node translation = tool_offset["translation"];
      if (!translation || !translation.IsSequence())
      {
        translation = tool_offset["translation"] = YAML::Node(YAML::NodeType::Sequence);
      }
      translation[0] = res.tool_extrinsic.position.x;
      translation[1] = res.tool_extrinsic.position.y;
      translation[2] = res.tool_extrinsic.position.z;

      YAML::Node rotation = tool_offset["rotation"];
      if (!rotation || !rotation.IsSequence())
      {
        rotation = tool_offset["rotation"] = YAML::Node(YAML::NodeType::Sequence);
      }
      rotation[0] = res.tool_extrinsic.orientation.x;
      rotation[1] = res.tool_extrinsic.orientation.y;
      rotation[2] = res.tool_extrinsic.orientation.z;
      rotation[3] = res.tool_extrinsic.orientation.w;

      YAML::Emitter cube_emitter;
      cube_emitter << node;
      std::ofstream fout(cube_faces_yaml_output_);
      fout << cube_emitter.c_str();
      fout.close();
      ROS_INFO("[AutoCalib] 已更新立方体配置：%s", cube_faces_yaml_output_.c_str());
    }
    catch (const std::exception &e)
    {
      ROS_ERROR("[AutoCalib] 保存立方体配置失败：%s", e.what());
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "world_robot_autocalib_manager_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  WorldRobotAutocalibManager node(nh, pnh);
  ros::spin();
  return 0;
}

