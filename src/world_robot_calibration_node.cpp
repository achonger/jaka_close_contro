#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <fstream>
#include <vector>
#include <map>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <ceres/ceres.h>

#include <jaka_close_contro/WorldRobotCalibration.h>
#include <jaka_close_contro/WorldRobotCollectSample.h>
#include <jaka_close_contro/CubeFusionStats.h>

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
    pnh.param<std::string>("fusion_stats_topic", stats_topic_, std::string("/cube_fusion_stats"));
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

    const std::string base_path = ros::package::getPath("jaka_close_contro");
    const std::string default_output = base_path + "/config/world_robot_extrinsic.yaml";
    const std::string default_faces_output = base_path + "/config/cube_faces_current.yaml";
    const std::string default_tool_output = base_path + "/config/tool_offset_current.yaml";
    const std::string default_faces_input = base_path + "/config/cube_faces_ideal.yaml";
    pnh.param<std::string>("output_yaml", output_yaml_path_, default_output);
    pnh.param<std::string>("faces_output_yaml", faces_output_yaml_path_, default_faces_output);
    pnh.param<std::string>("tool_offset_output_yaml", tool_offset_output_yaml_path_, default_tool_output);
    pnh.param<std::string>("faces_input_yaml", faces_input_yaml_path_, default_faces_input);

    cube_sub_ = nh.subscribe(cube_topic_, 1, &WorldRobotCalibrationNode::cubeCallback, this);
    stats_sub_ = nh.subscribe(stats_topic_, 1, &WorldRobotCalibrationNode::statsCallback, this);
    collect_srv_ = nh.advertiseService("/collect_world_robot_sample", &WorldRobotCalibrationNode::collectSample, this);
    solve_srv_ = nh.advertiseService("/solve_world_robot_calibration", &WorldRobotCalibrationNode::solveCalibration, this);

    ROS_INFO("[WorldRobot] 世界-机器人标定节点已启动");
    ROS_INFO("[WorldRobot] world_frame=%s, robot_base_frame=%s, tool_frame=%s, camera_frame=%s, cube_frame=%s",
             world_frame_.c_str(), robot_base_frame_.c_str(), tool_frame_.c_str(), camera_frame_.c_str(), cube_frame_.c_str());
    ROS_INFO("[WorldRobot] 立方体偏移（法兰系）= [%.3f, %.3f, %.3f] m", cube_offset_.x(), cube_offset_.y(), cube_offset_.z());
    ROS_INFO("[WorldRobot] 输出 YAML: %s", output_yaml_path_.c_str());
    ROS_INFO("[WorldRobot] faces_input=%s, faces_output=%s, tool_offset_output=%s", faces_input_yaml_path_.c_str(),
             faces_output_yaml_path_.c_str(), tool_offset_output_yaml_path_.c_str());

    loadExistingCalibration();

    loadFacesYaml();

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

  struct FaceParam
  {
    int id;
    Eigen::Vector3d t;
    Eigen::Vector3d rpy; // rad
  };

  struct ResidualStats
  {
    size_t sample_count = 0;
    double position_mean_m = 0.0;
    double position_std_m = 0.0;
    double position_rmse_m = 0.0;
    double position_max_m = 0.0;

    double orientation_mean_deg = 0.0;
    double orientation_std_deg = 0.0;
    double orientation_rmse_deg = 0.0;
    double orientation_max_deg = 0.0;

    double extrinsic_pos_mean_m = 0.0;
    double extrinsic_pos_std_m = 0.0;
    double extrinsic_pos_rmse_m = 0.0;
    double extrinsic_pos_max_m = 0.0;

    double extrinsic_rot_mean_deg = 0.0;
    double extrinsic_rot_std_deg = 0.0;
    double extrinsic_rot_rmse_deg = 0.0;
    double extrinsic_rot_max_deg = 0.0;
  };

  struct JointCalibResidual
  {
    JointCalibResidual(const Eigen::Isometry3d &T_world_cube_meas,
                       const Eigen::Isometry3d &T_base_tool,
                       double pos_sigma,
                       double rot_sigma)
        : T_world_cube_meas_(T_world_cube_meas), T_base_tool_(T_base_tool), pos_sigma_(pos_sigma), rot_sigma_(rot_sigma) {}

    template <typename T>
    bool operator()(const T *const world_base, const T *const flange_cube, T *residuals) const
    {
      Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_wb(world_base);
      Eigen::Quaternion<T> q_wb(world_base[6], world_base[3], world_base[4], world_base[5]);
      q_wb.normalize();

      Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_fc(flange_cube);
      Eigen::Quaternion<T> q_fc(flange_cube[6], flange_cube[3], flange_cube[4], flange_cube[5]);
      q_fc.normalize();

      Eigen::Transform<T, 3, Eigen::Isometry> T_wb = Eigen::Transform<T, 3, Eigen::Isometry>::Identity();
      T_wb.linear() = q_wb.toRotationMatrix();
      T_wb.translation() = t_wb;

      Eigen::Transform<T, 3, Eigen::Isometry> T_fc = Eigen::Transform<T, 3, Eigen::Isometry>::Identity();
      T_fc.linear() = q_fc.toRotationMatrix();
      T_fc.translation() = t_fc;

      Eigen::Transform<T, 3, Eigen::Isometry> T_bt = T_base_tool_.cast<T>();

      Eigen::Transform<T, 3, Eigen::Isometry> T_wc_pred = T_wb * T_bt * T_fc;
      Eigen::Transform<T, 3, Eigen::Isometry> T_err = T_world_cube_meas_.inverse().cast<T>() * T_wc_pred;

      Eigen::Matrix<T, 3, 1> t_err = T_err.translation();
      Eigen::AngleAxis<T> aa(T_err.linear());
      Eigen::Matrix<T, 3, 1> r_err = aa.axis() * aa.angle();

      residuals[0] = t_err.x() / static_cast<T>(pos_sigma_);
      residuals[1] = t_err.y() / static_cast<T>(pos_sigma_);
      residuals[2] = t_err.z() / static_cast<T>(pos_sigma_);
      residuals[3] = r_err.x() / static_cast<T>(rot_sigma_);
      residuals[4] = r_err.y() / static_cast<T>(rot_sigma_);
      residuals[5] = r_err.z() / static_cast<T>(rot_sigma_);
      return true;
    }

    Eigen::Isometry3d T_world_cube_meas_;
    Eigen::Isometry3d T_base_tool_;
    double pos_sigma_;
    double rot_sigma_;
  };

  bool computeInitialWorldBase(Eigen::Isometry3d &T_wb_init)
  {
    if (base_cube_positions_.empty() || world_cube_positions_.size() != base_cube_positions_.size())
    {
      ROS_ERROR("[WorldRobot] 无法计算初始值，样本为空或维度不匹配");
      return false;
    }

    Eigen::Quaterniond q_wb = averageRotationPairs();
    Eigen::Matrix3d R_wb = q_wb.toRotationMatrix();

    Eigen::Vector3d mean_base = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_world = Eigen::Vector3d::Zero();
    const size_t n = base_cube_positions_.size();
    for (size_t i = 0; i < n; ++i)
    {
      mean_base += base_cube_positions_[i];
      mean_world += world_cube_positions_[i];
    }
    mean_base /= static_cast<double>(n);
    mean_world /= static_cast<double>(n);

    Eigen::Vector3d t_wb = mean_world - R_wb * mean_base;

    T_wb_init = Eigen::Isometry3d::Identity();
    T_wb_init.linear() = R_wb;
    T_wb_init.translation() = t_wb;
    return true;
  }

  void resetSamples()
  {
    base_cube_positions_.clear();
    base_cube_rotations_.clear();
    world_cube_positions_.clear();
    world_cube_rotations_.clear();
    base_tool_positions_.clear();
    base_tool_rotations_.clear();
  }

  geometry_msgs::PoseStamped latest_cube_pose_;
  bool has_cube_pose_ = false;

  jaka_close_contro::CubeFusionStats latest_stats_;
  bool has_stats_ = false;

  ros::Subscriber stats_sub_;

  std::vector<Eigen::Vector3d> base_cube_positions_;
  std::vector<Eigen::Quaterniond> base_cube_rotations_;
  std::vector<Eigen::Vector3d> base_tool_positions_;
  std::vector<Eigen::Quaterniond> base_tool_rotations_;
  std::vector<Eigen::Vector3d> world_cube_positions_;
  std::vector<Eigen::Quaterniond> world_cube_rotations_;

  std::string world_frame_;
  std::string robot_base_frame_;
  std::string tool_frame_;
  std::string camera_frame_;
  std::string cube_frame_;
  std::string cube_topic_;
  std::string stats_topic_;
  std::string output_yaml_path_;
  std::string faces_output_yaml_path_;
  std::string tool_offset_output_yaml_path_;
  std::string faces_input_yaml_path_;
  int min_samples_ = 10;
  bool publish_tf_ = false;

  Eigen::Vector3d cube_offset_{0.0, 0.0, 0.077};

  bool has_solution_ = false;
  geometry_msgs::TransformStamped solution_tf_;

  std::map<int, FaceParam> faces_;

  void cubeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    latest_cube_pose_ = *msg;
    has_cube_pose_ = true;

    geometry_msgs::TransformStamped T_world_cam_msg;
    if (!lookupTransform(world_frame_, camera_frame_, msg->header.stamp, T_world_cam_msg))
    {
      return;
    }

    Eigen::Isometry3d T_world_cam = transformToEigen(T_world_cam_msg.transform);
    Eigen::Isometry3d T_cam_cube = poseToEigen(msg->pose);
    Eigen::Isometry3d T_world_cube = T_world_cam * T_cam_cube;

    Eigen::Vector3d p_w = T_world_cube.translation();
    Eigen::Quaterniond q_w(T_world_cube.rotation());
    q_w.normalize();

    ROS_INFO_STREAM_THROTTLE(0.5, "[WorldRobotCalib] cube in world:\n"
                                   << "  pos  = (" << p_w.x() << ", " << p_w.y() << ", " << p_w.z() << ")\n"
                                   << "  quat = (" << q_w.x() << ", " << q_w.y() << ", " << q_w.z() << ", " << q_w.w() << ")");
  }

  void statsCallback(const jaka_close_contro::CubeFusionStats::ConstPtr &msg)
  {
    latest_stats_ = *msg;
    has_stats_ = true;
  }

  bool collectSample(jaka_close_contro::WorldRobotCollectSample::Request &,
                     jaka_close_contro::WorldRobotCollectSample::Response &res)
  {
    if (!has_cube_pose_)
    {
      res.success = false;
      res.message = "尚未接收到立方体融合位姿";
      ROS_WARN("[WorldRobot] 未收到 /cube_center_fused，无法采样");
      return true;
    }

    if (!has_stats_ || !latest_stats_.valid || latest_stats_.inliers < 1)
    {
      res.success = false;
      res.message = "融合无效或内点不足，跳过采样";
      ROS_WARN("[WorldRobot] 融合状态无效(valid=%s, inliers=%d)，本次不采样", has_stats_ ? (latest_stats_.valid ? "true" : "false") : "unset", latest_stats_.inliers);
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
    Eigen::Quaterniond q_base_tool(T_base_tool.rotation());
    q_base_tool.normalize();
    base_tool_positions_.push_back(T_base_tool.translation());
    base_tool_rotations_.push_back(q_base_tool);

    Eigen::Isometry3d T_tool_cube = Eigen::Isometry3d::Identity();
    T_tool_cube.translation() = cube_offset_;
    Eigen::Isometry3d T_base_cube_pred = T_base_tool * T_tool_cube;

    Eigen::Quaterniond q_base_cube(T_base_cube_pred.rotation());
    q_base_cube.normalize();
    base_cube_positions_.push_back(T_base_cube_pred.translation());
    base_cube_rotations_.push_back(q_base_cube);

    Eigen::Quaterniond q_world_cube(T_world_cube_meas.rotation());
    q_world_cube.normalize();
    world_cube_positions_.push_back(T_world_cube_meas.translation());
    world_cube_rotations_.push_back(q_world_cube);

    const size_t idx = base_cube_positions_.size();
    const Eigen::Vector3d p_w = T_world_cube_meas.translation();
    Eigen::Quaterniond q_w(T_world_cube_meas.rotation());
    q_w.normalize();
    const Eigen::Vector3d p_b = T_base_cube_pred.translation();
    Eigen::Quaterniond q_b(T_base_cube_pred.rotation());
    q_b.normalize();

    ROS_INFO_STREAM("[WorldRobot] 采集样本 #" << idx
                                            << "\n  world_cube_meas.pos  = ("
                                            << p_w.x() << ", " << p_w.y() << ", " << p_w.z() << ")"
                                            << "\n  world_cube_meas.quat = ("
                                            << q_w.x() << ", " << q_w.y() << ", " << q_w.z() << ", " << q_w.w() << ")"
                                            << "\n  base_cube_pred.pos   = ("
                                            << p_b.x() << ", " << p_b.y() << ", " << p_b.z() << ")"
                                            << "\n  base_cube_pred.quat  = ("
                                            << q_b.x() << ", " << q_b.y() << ", " << q_b.z() << ", " << q_b.w() << ")");

    res.world_cube_pos_x = p_w.x();
    res.world_cube_pos_y = p_w.y();
    res.world_cube_pos_z = p_w.z();
    res.world_cube_quat_x = q_w.x();
    res.world_cube_quat_y = q_w.y();
    res.world_cube_quat_z = q_w.z();
    res.world_cube_quat_w = q_w.w();

    res.base_cube_pos_x = p_b.x();
    res.base_cube_pos_y = p_b.y();
    res.base_cube_pos_z = p_b.z();
    res.base_cube_quat_x = q_b.x();
    res.base_cube_quat_y = q_b.y();
    res.base_cube_quat_z = q_b.z();
    res.base_cube_quat_w = q_b.w();

    res.success = true;
    res.message = "采样成功，当前样本数=" + std::to_string(base_cube_positions_.size());
    return true;
  }

  bool solveCalibration(jaka_close_contro::WorldRobotCalibration::Request &,
                        jaka_close_contro::WorldRobotCalibration::Response &res)
  {
    const size_t n = base_cube_positions_.size();
    if (n != base_tool_positions_.size() || n != base_tool_rotations_.size())
    {
      res.success = false;
      res.message = "采样数据尺寸不一致，请重新采样";
      ROS_ERROR("[WorldRobot] 样本数量不一致：base_cube=%zu, base_tool=%zu, base_tool_rot=%zu", n, base_tool_positions_.size(),
                base_tool_rotations_.size());
      resetSamples();
      return true;
    }

    if (n < static_cast<size_t>(min_samples_))
    {
      res.success = false;
      res.message = "样本不足，至少需要 " + std::to_string(min_samples_) + " 组";
      ROS_ERROR("[WorldRobot] 样本不足：%zu/%d", n, min_samples_);
      return true;
    }
    Eigen::Isometry3d T_wb_init;
    if (!computeInitialWorldBase(T_wb_init))
    {
      res.success = false;
      res.message = "初始值求解失败";
      return true;
    }

    Eigen::Quaterniond q_wb_init(T_wb_init.rotation());
    q_wb_init.normalize();
    double world_base_param[7] = {T_wb_init.translation().x(), T_wb_init.translation().y(), T_wb_init.translation().z(),
                                  q_wb_init.x(), q_wb_init.y(), q_wb_init.z(), q_wb_init.w()};

    double flange_cube_param[7] = {cube_offset_.x(), cube_offset_.y(), cube_offset_.z(), 0.0, 0.0, 0.0, 1.0};

    ceres::Problem problem;
    ceres::LocalParameterization *se3_param_world = new ceres::ProductParameterization(
        new ceres::IdentityParameterization(3), new ceres::QuaternionParameterization());
    ceres::LocalParameterization *se3_param_flange = new ceres::ProductParameterization(
        new ceres::IdentityParameterization(3), new ceres::QuaternionParameterization());
    problem.AddParameterBlock(world_base_param, 7, se3_param_world);
    problem.AddParameterBlock(flange_cube_param, 7, se3_param_flange);

    const double pos_sigma = 0.01;
    const double rot_sigma = 5.0 * M_PI / 180.0;

    for (size_t i = 0; i < n; ++i)
    {
      Eigen::Isometry3d T_world_cube_meas = Eigen::Isometry3d::Identity();
      Eigen::Quaterniond q_wc = world_cube_rotations_[i];
      q_wc.normalize();
      T_world_cube_meas.linear() = q_wc.toRotationMatrix();
      T_world_cube_meas.translation() = world_cube_positions_[i];

      Eigen::Isometry3d T_base_tool = Eigen::Isometry3d::Identity();
      Eigen::Quaterniond q_bt = base_tool_rotations_[i];
      q_bt.normalize();
      T_base_tool.linear() = q_bt.toRotationMatrix();
      T_base_tool.translation() = base_tool_positions_[i];

      ceres::CostFunction *cost = new ceres::AutoDiffCostFunction<JointCalibResidual, 6, 7, 7>(
          new JointCalibResidual(T_world_cube_meas, T_base_tool, pos_sigma, rot_sigma));
      problem.AddResidualBlock(cost, nullptr, world_base_param, flange_cube_param);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 100;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (!summary.IsSolutionUsable())
    {
      res.success = false;
      res.message = "Ceres 求解失败: " + summary.BriefReport();
      ROS_ERROR("[WorldRobot] 联合标定失败：%s", summary.FullReport().c_str());
      return true;
    }

    Eigen::Quaterniond q_wb(world_base_param[6], world_base_param[3], world_base_param[4], world_base_param[5]);
    q_wb.normalize();
    Eigen::Vector3d t_wb(world_base_param[0], world_base_param[1], world_base_param[2]);
    Eigen::Isometry3d T_wb_opt = Eigen::Isometry3d::Identity();
    T_wb_opt.linear() = q_wb.toRotationMatrix();
    T_wb_opt.translation() = t_wb;

    Eigen::Quaterniond q_fc(flange_cube_param[6], flange_cube_param[3], flange_cube_param[4], flange_cube_param[5]);
    q_fc.normalize();
    Eigen::Vector3d t_fc(flange_cube_param[0], flange_cube_param[1], flange_cube_param[2]);
    Eigen::Isometry3d T_fc_opt = Eigen::Isometry3d::Identity();
    T_fc_opt.linear() = q_fc.toRotationMatrix();
    T_fc_opt.translation() = t_fc;

    solution_tf_ = eigenToTransform(T_wb_opt);
    solution_tf_.header.frame_id = world_frame_;
    solution_tf_.child_frame_id = robot_base_frame_;
    solution_tf_.header.stamp = ros::Time::now();
    has_solution_ = true;

    saveCalibration(solution_tf_);

    ROS_INFO("[WorldRobot] 联合标定完成，world->%s: translation = [%.3f, %.3f, %.3f], rotation(xyzw) = [%.4f, %.4f, %.4f, %.4f]",
             robot_base_frame_.c_str(),
             t_wb.x(), t_wb.y(), t_wb.z(),
             q_wb.x(), q_wb.y(), q_wb.z(), q_wb.w());
    ROS_INFO("[WorldRobot] 工具外参（flange->cube_center）: translation = [%.3f, %.3f, %.3f], rotation(xyzw) = [%.4f, %.4f, %.4f, %.4f]",
             t_fc.x(), t_fc.y(), t_fc.z(), q_fc.x(), q_fc.y(), q_fc.z(), q_fc.w());

    res.success = true;
    res.message = "标定完成";
    res.translation.x = t_wb.x();
    res.translation.y = t_wb.y();
    res.translation.z = t_wb.z();
    res.rotation.x = q_wb.x();
    res.rotation.y = q_wb.y();
    res.rotation.z = q_wb.z();
    res.rotation.w = q_wb.w();

    res.tool_extrinsic.position.x = t_fc.x();
    res.tool_extrinsic.position.y = t_fc.y();
    res.tool_extrinsic.position.z = t_fc.z();
    res.tool_extrinsic.orientation.x = q_fc.x();
    res.tool_extrinsic.orientation.y = q_fc.y();
    res.tool_extrinsic.orientation.z = q_fc.z();
    res.tool_extrinsic.orientation.w = q_fc.w();

    tf2::Transform T_WB_est;
    tf2::Quaternion q_tf(q_wb.x(), q_wb.y(), q_wb.z(), q_wb.w());
    q_tf.normalize();
    T_WB_est.setRotation(q_tf);
    T_WB_est.setOrigin(tf2::Vector3(t_wb.x(), t_wb.y(), t_wb.z()));

    tf2::Transform T_FC_est;
    tf2::Quaternion q_fc_tf(q_fc.x(), q_fc.y(), q_fc.z(), q_fc.w());
    q_fc_tf.normalize();
    T_FC_est.setRotation(q_fc_tf);
    T_FC_est.setOrigin(tf2::Vector3(t_fc.x(), t_fc.y(), t_fc.z()));

    const ResidualStats stats = computeResidualStats(T_WB_est, T_FC_est);

    writeToolOffsetYaml(tool_offset_output_yaml_path_, T_FC_est);
    if (!faces_.empty())
    {
      writeCubeFacesYaml(faces_output_yaml_path_, faces_);
    }
    else
    {
      ROS_WARN("[WorldRobot] 未加载面信息，跳过面配置写出");
    }

    res.position_rmse_m = stats.position_rmse_m;
    res.position_mean_m = stats.position_mean_m;
    res.position_std_m = stats.position_std_m;
    res.position_max_m = stats.position_max_m;

    res.orientation_rmse_deg = stats.orientation_rmse_deg;
    res.orientation_mean_deg = stats.orientation_mean_deg;
    res.orientation_std_deg = stats.orientation_std_deg;
    res.orientation_max_deg = stats.orientation_max_deg;

    res.extrinsic_pos_rmse_m = stats.extrinsic_pos_rmse_m;
    res.extrinsic_pos_mean_m = stats.extrinsic_pos_mean_m;
    res.extrinsic_pos_std_m = stats.extrinsic_pos_std_m;
    res.extrinsic_pos_max_m = stats.extrinsic_pos_max_m;

    res.extrinsic_rot_rmse_deg = stats.extrinsic_rot_rmse_deg;
    res.extrinsic_rot_mean_deg = stats.extrinsic_rot_mean_deg;
    res.extrinsic_rot_std_deg = stats.extrinsic_rot_std_deg;
    res.extrinsic_rot_max_deg = stats.extrinsic_rot_max_deg;

    resetSamples();

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

  bool loadFacesYaml()
  {
    try
    {
      YAML::Node root = YAML::LoadFile(faces_input_yaml_path_);
      if (!root["faces"])
      {
        ROS_WARN("[WorldRobot] faces_input_yaml 缺少 faces 字段: %s", faces_input_yaml_path_.c_str());
        return false;
      }

      faces_.clear();
      for (const auto &f : root["faces"])
      {
        if (!f["id"] || !f["translation"] || !f["rpy_deg"])
        {
          ROS_WARN("[WorldRobot] faces_input_yaml 条目缺字段，跳过一项");
          continue;
        }
        FaceParam fp;
        fp.id = f["id"].as<int>();
        auto tr = f["translation"];
        auto rpy = f["rpy_deg"];
        fp.t = Eigen::Vector3d(tr[0].as<double>(), tr[1].as<double>(), tr[2].as<double>());
        fp.rpy = Eigen::Vector3d(rpy[0].as<double>() * M_PI / 180.0,
                                 rpy[1].as<double>() * M_PI / 180.0,
                                 rpy[2].as<double>() * M_PI / 180.0);
        faces_[fp.id] = fp;
      }

      ROS_INFO("[WorldRobot] 已加载 %zu 个面定义用于输出: %s", faces_.size(), faces_input_yaml_path_.c_str());
      return !faces_.empty();
    }
    catch (const std::exception &e)
    {
      ROS_WARN("[WorldRobot] faces_input_yaml 读取失败: %s", e.what());
      return false;
    }
  }

  void writeToolOffsetYaml(const std::string &path, const tf2::Transform &T_fc)
  {
    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "tool_offset" << YAML::Value;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "translation" << YAML::Value << YAML::Flow << YAML::BeginSeq << T_fc.getOrigin().x()
            << T_fc.getOrigin().y() << T_fc.getOrigin().z() << YAML::EndSeq;
    emitter << YAML::Key << "rotation" << YAML::Value << YAML::Flow << YAML::BeginSeq << T_fc.getRotation().x()
            << T_fc.getRotation().y() << T_fc.getRotation().z() << T_fc.getRotation().w() << YAML::EndSeq;
    emitter << YAML::EndMap;
    emitter << YAML::EndMap;

    try
    {
      std::ofstream fout(path);
      fout << emitter.c_str();
      fout.close();
      ROS_INFO("[WorldRobot] 已写入工具外参: %s", path.c_str());
    }
    catch (const std::exception &e)
    {
      ROS_ERROR("[WorldRobot] 写入工具外参失败: %s", e.what());
    }
  }

  void writeCubeFacesYaml(const std::string &path, const std::map<int, FaceParam> &faces)
  {
    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "faces" << YAML::Value << YAML::BeginSeq;

    std::vector<int> ids;
    ids.reserve(faces.size());
    for (const auto &kv : faces)
    {
      ids.push_back(kv.first);
    }
    std::sort(ids.begin(), ids.end());

    for (int id : ids)
    {
      const FaceParam &fp = faces.at(id);
      emitter << YAML::BeginMap;
      emitter << YAML::Key << "id" << YAML::Value << fp.id;
      emitter << YAML::Key << "translation" << YAML::Value << YAML::Flow << YAML::BeginSeq << fp.t.x() << fp.t.y()
              << fp.t.z() << YAML::EndSeq;
      emitter << YAML::Key << "rpy_deg" << YAML::Value << YAML::Flow << YAML::BeginSeq << fp.rpy.x() * 180.0 / M_PI
              << fp.rpy.y() * 180.0 / M_PI << fp.rpy.z() * 180.0 / M_PI << YAML::EndSeq;
      emitter << YAML::EndMap;
    }

    emitter << YAML::EndSeq;
    emitter << YAML::EndMap;

    try
    {
      std::ofstream fout(path);
      fout << emitter.c_str();
      fout.close();
      ROS_INFO("[WorldRobot] 已写入面配置: %s", path.c_str());
    }
    catch (const std::exception &e)
    {
      ROS_ERROR("[WorldRobot] 写入面配置失败: %s", e.what());
    }
  }

  ResidualStats computeResidualStats(const tf2::Transform &T_WB_est, const tf2::Transform &T_FC_est)
  {
    ResidualStats stats;
    const size_t N = world_cube_positions_.size();
    stats.sample_count = N;

    if (N == 0)
    {
      ROS_WARN("[WorldRobot] 无样本可计算残差统计");
      return stats;
    }
    if (base_tool_positions_.size() != N || base_tool_rotations_.size() != N)
    {
      ROS_WARN("[WorldRobot] 残差统计维度不一致，无法计算");
      return stats;
    }
    if (N < 2)
    {
      ROS_WARN("[WorldRobot] 样本不足，无法计算有效统计量 (N=%zu)", N);
    }

    std::vector<double> pos_errs;
    std::vector<double> rot_errs_deg;
    std::vector<double> extrinsic_disp_pos;
    std::vector<double> extrinsic_disp_rot_deg;
    pos_errs.reserve(N);
    rot_errs_deg.reserve(N);
    extrinsic_disp_pos.reserve(N);
    extrinsic_disp_rot_deg.reserve(N);

    for (size_t i = 0; i < N; ++i)
    {
      tf2::Transform T_W_cube_meas;
      tf2::Quaternion q_w(world_cube_rotations_[i].x(), world_cube_rotations_[i].y(), world_cube_rotations_[i].z(), world_cube_rotations_[i].w());
      q_w.normalize();
      T_W_cube_meas.setOrigin(tf2::Vector3(world_cube_positions_[i].x(), world_cube_positions_[i].y(), world_cube_positions_[i].z()));
      T_W_cube_meas.setRotation(q_w);

      tf2::Transform T_B_tool;
      tf2::Quaternion q_bt(base_tool_rotations_[i].x(), base_tool_rotations_[i].y(), base_tool_rotations_[i].z(), base_tool_rotations_[i].w());
      q_bt.normalize();
      T_B_tool.setOrigin(tf2::Vector3(base_tool_positions_[i].x(), base_tool_positions_[i].y(), base_tool_positions_[i].z()));
      T_B_tool.setRotation(q_bt);

      tf2::Transform T_W_cube_pred = T_WB_est * T_B_tool * T_FC_est;
      tf2::Transform T_err = T_W_cube_pred.inverse() * T_W_cube_meas;

      tf2::Vector3 t_err = T_err.getOrigin();
      double e_t = t_err.length();
      tf2::Quaternion q_err = T_err.getRotation();
      q_err.normalize();
      double e_R = q_err.getAngle() * 180.0 / M_PI;

      tf2::Transform T_WB_i = T_W_cube_meas * (T_B_tool * T_FC_est).inverse();
      tf2::Transform T_disp = T_WB_est.inverse() * T_WB_i;
      tf2::Vector3 t_disp = T_disp.getOrigin();
      double e_t_disp = t_disp.length();
      tf2::Quaternion q_disp = T_disp.getRotation();
      q_disp.normalize();
      double e_R_disp = q_disp.getAngle() * 180.0 / M_PI;

      pos_errs.push_back(e_t);
      rot_errs_deg.push_back(e_R);
      extrinsic_disp_pos.push_back(e_t_disp);
      extrinsic_disp_rot_deg.push_back(e_R_disp);
    }

    auto computeStats = [](const std::vector<double> &v,
                           double &mean, double &stddev,
                           double &rmse, double &maxv)
    {
      const size_t n = v.size();
      if (n == 0)
      {
        mean = stddev = rmse = maxv = 0.0;
        return;
      }
      double sum = 0.0;
      double sum_sq = 0.0;
      maxv = 0.0;
      for (double x : v)
      {
        sum += x;
        sum_sq += x * x;
        if (x > maxv)
        {
          maxv = x;
        }
      }
      mean = sum / static_cast<double>(n);
      rmse = std::sqrt(sum_sq / static_cast<double>(n));
      if (n > 1)
      {
        double var = 0.0;
        for (double x : v)
        {
          double d = x - mean;
          var += d * d;
        }
        var /= static_cast<double>(n - 1);
        stddev = std::sqrt(var);
      }
      else
      {
        stddev = 0.0;
      }
    };

    computeStats(pos_errs, stats.position_mean_m, stats.position_std_m, stats.position_rmse_m, stats.position_max_m);
    computeStats(rot_errs_deg, stats.orientation_mean_deg, stats.orientation_std_deg, stats.orientation_rmse_deg, stats.orientation_max_deg);
    computeStats(extrinsic_disp_pos, stats.extrinsic_pos_mean_m, stats.extrinsic_pos_std_m, stats.extrinsic_pos_rmse_m, stats.extrinsic_pos_max_m);
    computeStats(extrinsic_disp_rot_deg, stats.extrinsic_rot_mean_deg, stats.extrinsic_rot_std_deg, stats.extrinsic_rot_rmse_deg, stats.extrinsic_rot_max_deg);

    if (N >= 1)
    {
      ROS_INFO_STREAM("=== World-Robot Calibration Residuals (N=" << N << ") ===");
      ROS_INFO_STREAM("Position residual | RMSE = " << stats.position_rmse_m << " m"
                                                   << ", mean = " << stats.position_mean_m << " m"
                                                   << ", std = " << stats.position_std_m << " m"
                                                   << ", max = " << stats.position_max_m << " m");
      ROS_INFO_STREAM("Orientation residual | RMSE = " << stats.orientation_rmse_deg << " deg"
                                                       << ", mean = " << stats.orientation_mean_deg << " deg"
                                                       << ", std = " << stats.orientation_std_deg << " deg"
                                                       << ", max = " << stats.orientation_max_deg << " deg");
      ROS_INFO_STREAM("Extrinsic dispersion (world->base) - translation | RMSE = " << stats.extrinsic_pos_rmse_m << " m"
                                                                                   << ", mean = " << stats.extrinsic_pos_mean_m << " m"
                                                                                   << ", std = " << stats.extrinsic_pos_std_m << " m"
                                                                                   << ", max = " << stats.extrinsic_pos_max_m << " m");
      ROS_INFO_STREAM("Extrinsic dispersion (world->base) - orientation | RMSE = " << stats.extrinsic_rot_rmse_deg << " deg"
                                                                                   << ", mean = " << stats.extrinsic_rot_mean_deg << " deg"
                                                                                   << ", std = " << stats.extrinsic_rot_std_deg << " deg"
                                                                                   << ", max = " << stats.extrinsic_rot_max_deg << " deg");
    }

    return stats;
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
