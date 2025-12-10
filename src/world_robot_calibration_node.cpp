#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Trigger.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

#include <jaka_close_contro/WorldRobotCalibration.h>

class WorldRobotCalibrationNode
{
public:
  WorldRobotCalibrationNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
      : tf_listener_(tf_buffer_)
  {
    pnh.param<std::string>("world_frame", world_frame_, std::string("world"));
    pnh.param<std::string>("robot_base_frame", robot_base_frame_, std::string("Link_0"));
    pnh.param<std::string>("tool_frame", tool_frame_, std::string("Link_6"));
    pnh.param<std::string>("camera_frame", camera_frame_, std::string("zed2i_left_camera_optical_frame"));
    pnh.param<std::string>("cube_frame", cube_frame_, std::string("cube_center"));
    pnh.param<std::string>("cube_topic", cube_topic_, std::string("/cube_center_fused"));
    pnh.param<int>("min_samples", min_samples_, 10);
    pnh.param<bool>("publish_tf", publish_tf_, false);

    std::vector<double> offset_xyz;
    if (pnh.getParam("cube_offset_xyz_m", offset_xyz) && offset_xyz.size() == 3)
    {
      cube_offset_ = Eigen::Vector3d(offset_xyz[0], offset_xyz[1], offset_xyz[2]);
    }
    else
    {
      double offset_z = 0.077;
      pnh.param<double>("cube_offset_z_m", offset_z, 0.077);
      cube_offset_ = Eigen::Vector3d(0.0, 0.0, offset_z);
    }

    const std::string default_output = ros::package::getPath("jaka_close_contro") + "/config/world_robot_extrinsic.yaml";
    pnh.param<std::string>("output_yaml", output_yaml_path_, default_output);

    cube_sub_ = nh.subscribe(cube_topic_, 1, &WorldRobotCalibrationNode::cubeCallback, this);
    collect_srv_ = nh.advertiseService("/collect_world_robot_sample", &WorldRobotCalibrationNode::collectSample, this);
    solve_srv_ = nh.advertiseService("/solve_world_robot_calibration", &WorldRobotCalibrationNode::solveCalibration, this);

    ROS_INFO("[WorldRobot] 世界-机器人标定节点已启动");
    ROS_INFO("[WorldRobot] world_frame=%s, robot_base_frame=%s, tool_frame=%s, camera_frame=%s, cube_frame=%s",
             world_frame_.c_str(), robot_base_frame_.c_str(), tool_frame_.c_str(), camera_frame_.c_str(), cube_frame_.c_str());
    ROS_INFO("[WorldRobot] 立方体偏移（法兰系）= [%.3f, %.3f, %.3f] m", cube_offset_.x(), cube_offset_.y(), cube_offset_.z());
    ROS_INFO("[WorldRobot] 输出 YAML: %s", output_yaml_path_.c_str());

    loadExistingCalibration();

    if (publish_tf_)
    {
      tf_timer_ = nh.createTimer(ros::Duration(0.05), &WorldRobotCalibrationNode::publishSolution, this);
    }
  }

private:
  ros::Subscriber cube_sub_;
  ros::ServiceServer collect_srv_;
  ros::ServiceServer solve_srv_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ros::Timer tf_timer_;

  geometry_msgs::PoseStamped latest_cube_pose_;
  bool has_cube_pose_ = false;

  std::vector<Eigen::Vector3d> base_cube_positions_;
  std::vector<Eigen::Quaterniond> base_cube_rotations_;
  std::vector<Eigen::Vector3d> world_cube_positions_;
  std::vector<Eigen::Quaterniond> world_cube_rotations_;

  std::string world_frame_;
  std::string robot_base_frame_;
  std::string tool_frame_;
  std::string camera_frame_;
  std::string cube_frame_;
  std::string cube_topic_;
  std::string output_yaml_path_;
  int min_samples_ = 10;
  bool publish_tf_ = false;

  Eigen::Vector3d cube_offset_{0.0, 0.0, 0.077};

  bool has_solution_ = false;
  geometry_msgs::TransformStamped solution_tf_;

  void cubeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    latest_cube_pose_ = *msg;
    has_cube_pose_ = true;
  }

  bool collectSample(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
  {
    if (!has_cube_pose_)
    {
      res.success = false;
      res.message = "尚未接收到立方体融合位姿";
      ROS_WARN("[WorldRobot] 未收到 /cube_center_fused，无法采样");
      return true;
    }

    const ros::Time stamp = latest_cube_pose_.header.stamp;

    geometry_msgs::TransformStamped T_world_cam_msg;
    geometry_msgs::TransformStamped T_base_tool_msg;
    if (!lookupTransform(world_frame_, camera_frame_, stamp, T_world_cam_msg))
    {
      res.success = false;
      res.message = "缺少 world->camera TF";
      return true;
    }
    if (!lookupTransform(robot_base_frame_, tool_frame_, stamp, T_base_tool_msg))
    {
      res.success = false;
      res.message = "缺少 base->tool TF";
      return true;
    }

    Eigen::Isometry3d T_world_cam = transformToEigen(T_world_cam_msg.transform);
    Eigen::Isometry3d T_cam_cube = poseToEigen(latest_cube_pose_.pose);
    Eigen::Isometry3d T_world_cube_meas = T_world_cam * T_cam_cube;

    Eigen::Isometry3d T_base_tool = transformToEigen(T_base_tool_msg.transform);
    Eigen::Isometry3d T_tool_cube = Eigen::Isometry3d::Identity();
    T_tool_cube.translation() = cube_offset_;
    Eigen::Isometry3d T_base_cube_pred = T_base_tool * T_tool_cube;

    base_cube_positions_.push_back(T_base_cube_pred.translation());
    base_cube_rotations_.push_back(Eigen::Quaterniond(T_base_cube_pred.rotation()));
    world_cube_positions_.push_back(T_world_cube_meas.translation());
    world_cube_rotations_.push_back(Eigen::Quaterniond(T_world_cube_meas.rotation()));

    res.success = true;
    res.message = "采样成功，当前样本数=" + std::to_string(base_cube_positions_.size());
    ROS_INFO("[WorldRobot] 采集样本 #%zu", base_cube_positions_.size());
    return true;
  }

  bool solveCalibration(jaka_close_contro::WorldRobotCalibration::Request &,
                        jaka_close_contro::WorldRobotCalibration::Response &res)
  {
    const size_t n = base_cube_positions_.size();
    if (n < static_cast<size_t>(min_samples_))
    {
      res.success = false;
      res.message = "样本不足，至少需要 " + std::to_string(min_samples_) + " 组";
      ROS_ERROR("[WorldRobot] 样本不足：%zu/%d", n, min_samples_);
      return true;
    }

    Eigen::Quaterniond q_wb = averageRotationPairs();
    Eigen::Matrix3d R_wb = q_wb.toRotationMatrix();

    Eigen::Vector3d mean_base = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_world = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < n; ++i)
    {
      mean_base += base_cube_positions_[i];
      mean_world += world_cube_positions_[i];
    }
    mean_base /= static_cast<double>(n);
    mean_world /= static_cast<double>(n);

    Eigen::Vector3d t_wb = mean_world - R_wb * mean_base;

    Eigen::Isometry3d T_wb = Eigen::Isometry3d::Identity();
    T_wb.linear() = R_wb;
    T_wb.translation() = t_wb;

    solution_tf_ = eigenToTransform(T_wb);
    solution_tf_.header.frame_id = world_frame_;
    solution_tf_.child_frame_id = robot_base_frame_;
    solution_tf_.header.stamp = ros::Time::now();
    has_solution_ = true;

    saveCalibration(solution_tf_);

    ROS_INFO("[WorldRobot] 求解完成，world->%s: translation = [%.3f, %.3f, %.3f], rotation(xyzw) = [%.4f, %.4f, %.4f, %.4f]",
             robot_base_frame_.c_str(),
             t_wb.x(), t_wb.y(), t_wb.z(),
             q_wb.x(), q_wb.y(), q_wb.z(), q_wb.w());
    ROS_INFO("[WorldRobot] 如需静态发布，可在 launch 中添加：");
    ROS_INFO("static_transform_publisher %.6f %.6f %.6f %.6f %.6f %.6f %.6f %s %s 10",
             t_wb.x(), t_wb.y(), t_wb.z(),
             q_wb.x(), q_wb.y(), q_wb.z(), q_wb.w(),
             world_frame_.c_str(), robot_base_frame_.c_str());

    res.success = true;
    res.message = "标定完成";
    res.translation.x = t_wb.x();
    res.translation.y = t_wb.y();
    res.translation.z = t_wb.z();
    res.rotation.x = q_wb.x();
    res.rotation.y = q_wb.y();
    res.rotation.z = q_wb.z();
    res.rotation.w = q_wb.w();

    return true;
  }

  bool lookupTransform(const std::string &parent,
                       const std::string &child,
                       const ros::Time &stamp,
                       geometry_msgs::TransformStamped &out)
  {
    const ros::Time query_time = stamp.isZero() ? ros::Time(0) : stamp;
    try
    {
      out = tf_buffer_.lookupTransform(parent, child, query_time, ros::Duration(1.0));
      return true;
    }
    catch (const tf2::TransformException &ex)
    {
      ROS_ERROR_THROTTLE(1.0, "[WorldRobot] TF 查询失败 %s -> %s: %s", parent.c_str(), child.c_str(), ex.what());
      return false;
    }
  }

  Eigen::Isometry3d transformToEigen(const geometry_msgs::Transform &tf_msg)
  {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(tf_msg.translation.x, tf_msg.translation.y, tf_msg.translation.z);
    Eigen::Quaterniond q(tf_msg.rotation.w, tf_msg.rotation.x, tf_msg.rotation.y, tf_msg.rotation.z);
    q.normalize();
    T.linear() = q.toRotationMatrix();
    return T;
  }

  Eigen::Isometry3d poseToEigen(const geometry_msgs::Pose &pose)
  {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    q.normalize();
    T.linear() = q.toRotationMatrix();
    return T;
  }

  geometry_msgs::TransformStamped eigenToTransform(const Eigen::Isometry3d &T)
  {
    geometry_msgs::TransformStamped msg;
    msg.transform.translation.x = T.translation().x();
    msg.transform.translation.y = T.translation().y();
    msg.transform.translation.z = T.translation().z();

    Eigen::Quaterniond q(T.rotation());
    q.normalize();
    msg.transform.rotation.x = q.x();
    msg.transform.rotation.y = q.y();
    msg.transform.rotation.z = q.z();
    msg.transform.rotation.w = q.w();
    return msg;
  }

  Eigen::Quaterniond averageRotationPairs()
  {
    Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
    for (size_t i = 0; i < base_cube_rotations_.size(); ++i)
    {
      // R_wb_i = R_world_cube_i * R_base_cube_i^T
      Eigen::Quaterniond q = world_cube_rotations_[i] * base_cube_rotations_[i].conjugate();
      if (q.w() < 0)
      {
        q.coeffs() *= -1.0; // 避免符号翻转带来的平均失真
      }
      Eigen::Vector4d v(q.w(), q.x(), q.y(), q.z());
      A += v * v.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(A);
    Eigen::Vector4d eigvec = solver.eigenvectors().col(3);
    Eigen::Quaterniond q_avg(eigvec(0), eigvec(1), eigvec(2), eigvec(3));
    q_avg.normalize();
    return q_avg;
  }

  void publishSolution(const ros::TimerEvent &)
  {
    if (!publish_tf_ || !has_solution_)
    {
      return;
    }
    solution_tf_.header.stamp = ros::Time::now();
    tf_broadcaster_.sendTransform(solution_tf_);
  }

  void saveCalibration(const geometry_msgs::TransformStamped &tf_msg)
  {
    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "world_frame" << YAML::Value << world_frame_;
    emitter << YAML::Key << "robot_base_frame" << YAML::Value << robot_base_frame_;
    emitter << YAML::Key << "translation" << YAML::Value;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "x" << YAML::Value << tf_msg.transform.translation.x;
    emitter << YAML::Key << "y" << YAML::Value << tf_msg.transform.translation.y;
    emitter << YAML::Key << "z" << YAML::Value << tf_msg.transform.translation.z;
    emitter << YAML::EndMap;
    emitter << YAML::Key << "rotation" << YAML::Value;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "x" << YAML::Value << tf_msg.transform.rotation.x;
    emitter << YAML::Key << "y" << YAML::Value << tf_msg.transform.rotation.y;
    emitter << YAML::Key << "z" << YAML::Value << tf_msg.transform.rotation.z;
    emitter << YAML::Key << "w" << YAML::Value << tf_msg.transform.rotation.w;
    emitter << YAML::EndMap;
    emitter << YAML::EndMap;

    try
    {
      std::ofstream fout(output_yaml_path_);
      fout << emitter.c_str();
      fout.close();
      ROS_INFO("[WorldRobot] 标定结果已保存至 %s", output_yaml_path_.c_str());
    }
    catch (const std::exception &e)
    {
      ROS_ERROR("[WorldRobot] 保存标定文件失败: %s", e.what());
    }
  }

  void loadExistingCalibration()
  {
    try
    {
      YAML::Node node = YAML::LoadFile(output_yaml_path_);
      if (!node["translation"] || !node["rotation"])
      {
        return;
      }

      geometry_msgs::TransformStamped tf_msg;
      tf_msg.header.frame_id = node["world_frame"] ? node["world_frame"].as<std::string>() : world_frame_;
      tf_msg.child_frame_id = node["robot_base_frame"] ? node["robot_base_frame"].as<std::string>() : robot_base_frame_;
      tf_msg.transform.translation.x = node["translation"]["x"].as<double>();
      tf_msg.transform.translation.y = node["translation"]["y"].as<double>();
      tf_msg.transform.translation.z = node["translation"]["z"].as<double>();
      tf_msg.transform.rotation.x = node["rotation"]["x"].as<double>();
      tf_msg.transform.rotation.y = node["rotation"]["y"].as<double>();
      tf_msg.transform.rotation.z = node["rotation"]["z"].as<double>();
      tf_msg.transform.rotation.w = node["rotation"]["w"].as<double>();
      tf_msg.header.stamp = ros::Time::now();

      solution_tf_ = tf_msg;
      has_solution_ = true;
      ROS_INFO("[WorldRobot] 已加载历史标定结果并可选发布 TF");
    }
    catch (const std::exception &e)
    {
      ROS_WARN("[WorldRobot] 未找到已保存的标定文件或解析失败（%s），将从零开始采样", e.what());
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "world_robot_calibration_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  WorldRobotCalibrationNode node(nh, pnh);
  ros::spin();
  return 0;
}
