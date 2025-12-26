// cube_multi_face_fusion_node.cpp
#include <ros/ros.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <yaml-cpp/yaml.h>

// 新增缺失的头文件
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Ceres 依赖
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/jet.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/cost_function.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/loss_function.h>
#include <ceres/local_parameterization.h>

// Eigen 依赖
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <map>
#include <cmath>
#include <deque>
#include <algorithm>
#include <glog/logging.h>
#include <gflags/gflags.h>

struct Observation {
  tf2::Transform T_cam_cube;
  double weight;
};

struct PoseResidual {
  PoseResidual(const Eigen::Vector3d& obs_t, const Eigen::Quaterniond& obs_q, double w)
    : t_(obs_t), q_(obs_q), weight_(w) {}

  template <typename T>
  bool operator()(const T* const cam_cube_rot, // quaternion (w, x, y, z)
                  const T* const cam_cube_t,   // translation (x, y, z)
                  T* residuals) const {
    // translation residual
    residuals[0] = T(weight_) * (cam_cube_t[0] - T(t_(0)));
    residuals[1] = T(weight_) * (cam_cube_t[1] - T(t_(1)));
    residuals[2] = T(weight_) * (cam_cube_t[2] - T(t_(2)));

    // rotation residual
    T est_q[4] = { cam_cube_rot[0], cam_cube_rot[1], cam_cube_rot[2], cam_cube_rot[3] };
    T obs_q_t[4] = { T(q_.w()), T(q_.x()), T(q_.y()), T(q_.z()) };

    T est_q_inv[4] = { est_q[0], -est_q[1], -est_q[2], -est_q[3] };
    T q_err[4];
    ceres::QuaternionProduct(est_q_inv, obs_q_t, q_err);

    T angle_axis[3];
    ceres::QuaternionToAngleAxis(q_err, angle_axis);

    residuals[3] = T(weight_) * angle_axis[0];
    residuals[4] = T(weight_) * angle_axis[1];
    residuals[5] = T(weight_) * angle_axis[2];

    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector3d& t, const Eigen::Quaterniond& q, double w) {
    return (new ceres::AutoDiffCostFunction<PoseResidual, 6, 4, 3>(
      new PoseResidual(t, q, w)));
  }

  Eigen::Vector3d t_;
  Eigen::Quaterniond q_;
  double weight_;
};

class CubeMultiFaceFusion {
public:
  CubeMultiFaceFusion(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh),
    camera_frame_default_("zed2i_left_camera_optical_frame"),
    has_tool_offset_(false),
    tf_buffer_(),
    tf_listener_(tf_buffer_),
    tf_broadcaster_()  // 明确初始化
  {
    // 读取参数
    pnh_.param<std::string>("faces_yaml", faces_yaml_path_, "");
    pnh_.param<std::string>("fiducial_topic", fiducial_topic_, "/tool_fiducials");
    pnh_.param<std::string>("camera_frame_default", camera_frame_default_, "zed2i_left_camera_optical_frame");
    pnh_.param<std::string>("cube_frame_prefix", cube_frame_prefix_, "cube_center_");
    pnh_.param<std::string>("tool_frame_prefix", tool_frame_prefix_, "vision_tool_");
    pnh_.param<std::string>("robots_config", robots_config_path_, "");
    pnh_.param("alpha_pos", alpha_pos_, 0.3);
    pnh_.param("alpha_rot", alpha_rot_, 0.3);
    pnh_.param("max_step_pos", max_step_pos_, 0.03);
    double max_deg = 15.0;
    pnh_.param("max_step_deg", max_deg, max_deg);
    max_step_ang_ = max_deg * M_PI / 180.0;
    pnh_.param("timeout_sec", timeout_sec_, 0.5);
    pnh_.param("publish_frequency", publish_frequency_, 30.0);
    pnh_.param("robot_stride", robot_stride_, 100);
    pnh_.param("face_id_base", face_id_base_, 10);
    pnh_.param("face_count", face_count_, 4);
    loadRobotIds();
    loadRobotsConfig();

    ROS_INFO_STREAM("[Fusion] faces_yaml: " << faces_yaml_path_);
    ROS_INFO_STREAM("[Fusion] fiducial_topic: " << fiducial_topic_);
    ROS_INFO_STREAM("[Fusion] cube_frame prefix: " << cube_frame_prefix_);
    ROS_INFO("[Fusion] Node started. Listening on %s", fiducial_topic_.c_str());
    if (faces_yaml_path_.empty() || !loadFacesYaml(faces_yaml_path_)) {
      ROS_ERROR("[Fusion] Failed to load faces YAML. Exiting.");
      ros::shutdown();
      return;
    }

    // 订阅与发布
    sub_ = nh_.subscribe<fiducial_msgs::FiducialTransformArray>(
      fiducial_topic_, 1,
      &CubeMultiFaceFusion::callback, this);
  }

private:
  static const size_t POSE_HISTORY_SIZE = 5;
  struct FusionState {
    std::deque<tf2::Transform> history_poses;
    tf2::Transform stable_pose;
    bool has_stable_pose{false};
    bool has_prev{false};
    tf2::Transform prev;
    ros::Time last_stamp;
    tf2::Quaternion prev_cube_q;
    bool has_prev_q{false};
    ros::Time last_publish;
    bool has_last_publish{false};
  };

  std::map<int, FusionState> robot_states_;
  std::map<int, ros::Publisher> publishers_;

  // 新增 TF 监听器
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  bool loadFacesYaml(const std::string& path) {
    try {
      ROS_INFO("[Fusion] Loading YAML from: %s", path.c_str());
      YAML::Node root = YAML::LoadFile(path);
      
      // 加载面定义
      if (!root["faces"]) {
        ROS_ERROR("[Fusion] YAML missing 'faces' section");
        return false;
      }
      for (const auto &f : root["faces"]) {
        int id = 0;
        try {
          id = f["id"].as<int>();
        } catch (...) {
          id = std::stoi(f["id"].as<std::string>());
        }
        auto tr = f["translation"];
        auto rpy = f["rpy_deg"];
        double tx = tr[0].as<double>();
        double ty = tr[1].as<double>();
        double tz = tr[2].as<double>();
        double rr = rpy[0].as<double>() * M_PI/180.0;
        double pp = rpy[1].as<double>() * M_PI/180.0;
        double yy = rpy[2].as<double>() * M_PI/180.0;
        tf2::Matrix3x3 R; R.setRPY(rr, pp, yy);
        tf2::Vector3 t(tx, ty, tz);
        tf2::Transform T_cube_face(R, t);
        face2cube_[id] = T_cube_face.inverse();
        
      ROS_INFO("[Fusion] Loaded face ID=%d: trans=[%.3f,%.3f,%.3f], rpy_deg=[%.1f,%.1f,%.1f]",
               id, tx, ty, tz, 
               rpy[0].as<double>(), rpy[1].as<double>(), rpy[2].as<double>());
      }

      // 加载工具偏移
      if (root["tool_offset"]) {
        auto to = root["tool_offset"];
        auto trans = to["translation"];
        double tx = trans[0].as<double>();
        double ty = trans[1].as<double>();
        double tz = trans[2].as<double>();
        
        tool_offset_.setOrigin(tf2::Vector3(tx, ty, tz));
        tool_offset_.setRotation(tf2::Quaternion(0, 0, 0, 1));
        has_tool_offset_ = true;
        
        ROS_INFO("[Fusion] Tool offset loaded: trans=[%.3f,%.3f,%.3f]", tx, ty, tz);
      } else {
        ROS_WARN("[Fusion] No 'tool_offset' in YAML. Using default [0,0,0.077]");
        tool_offset_.setOrigin(tf2::Vector3(0, 0, 0.077));
        tool_offset_.setRotation(tf2::Quaternion(0, 0, 0, 1));
        has_tool_offset_ = true;
      }

      return true;
    } catch (const std::exception& e) {
      ROS_ERROR("[Fusion] YAML load error: %s", e.what());
      return false;
    }
  }

  void callback(const fiducial_msgs::FiducialTransformArray::ConstPtr &msg) {
    ROS_INFO("[Fusion] Received %zu fiducials on topic %s",
             msg->transforms.size(), fiducial_topic_.c_str());
    
    std::map<int, std::vector<Observation>> obs_per_robot;
    for (const auto &tfm : msg->transforms) {
      auto it = face2cube_.find(tfm.fiducial_id);
      if (it == face2cube_.end()) continue;
      int rid = robotIdFromFace(tfm.fiducial_id);
      if (!shouldUseRobot(rid)) continue;
      
      tf2::Transform T_cf;
      tf2::fromMsg(tfm.transform, T_cf);
      tf2::Transform T_cc = T_cf * it->second;
      
      Eigen::Vector3d t(Eigen::Vector3d(
        T_cc.getOrigin().x(),
        T_cc.getOrigin().y(),
        T_cc.getOrigin().z()));
      tf2::Quaternion q2 = T_cc.getRotation();
      Eigen::Quaterniond q(q2.w(), q2.x(), q2.y(), q2.z());
      double w = computeWeight(tfm);

      ROS_DEBUG("[Fusion] Face ID=%d, weight=%.3f, pos=[%.3f,%.3f,%.3f]", 
                tfm.fiducial_id, w, t(0), t(1), t(2));

      obs_per_robot[rid].push_back({T_cc, w});
    }
    
    if (obs_per_robot.empty()) {
      ROS_DEBUG_THROTTLE(1.0, "[Fusion] No valid faces detected");
      return;
    }

    ros::Time stamp = msg->header.stamp;
    std::string cam_frame = msg->header.frame_id.empty()
      ? camera_frame_default_ : msg->header.frame_id;

    for (const auto& kv : obs_per_robot) {
      int rid = kv.first;
      const auto& obs = kv.second;
      tf2::Transform T_est;
      if (obs.size() >= 2) {
        T_est = optimizePose(obs);
      } else {
        T_est = obs[0].T_cam_cube;
      }

      tf2::Transform T_out = temporalFilter(T_est, stamp, rid);
      tf2::Transform pose_to_publish = smoothAndValidate(T_out, rid);
      FusionState& state = robot_states_[rid];
      if (!state.has_last_publish || (stamp - state.last_publish).toSec() >= 1.0 / publish_frequency_) {
        publish(pose_to_publish, stamp, cam_frame, rid);
        state.last_publish = stamp;
        state.has_last_publish = true;
      }
    }
  }

  double computeWeight(const fiducial_msgs::FiducialTransform &tfm) {
    double err = (tfm.object_error > 0.0) ? tfm.object_error : tfm.image_error;
    if (err <= 0.0) err = 1.0;
    return 1.0 / err;
  }

  tf2::Transform optimizePose(const std::vector<Observation>& obs) {
    Eigen::Quaterniond q0;
    tf2::Transform T0 = obs[0].T_cam_cube;
    tf2::Quaternion q2 = T0.getRotation(); q2.normalize();
    q0 = Eigen::Quaterniond(q2.w(), q2.x(), q2.y(), q2.z());
    Eigen::Matrix<double,3,1> t0;
    tf2::Vector3 tvec = T0.getOrigin();
    t0 << tvec.x(), tvec.y(), tvec.z();

    double cam_cube_rot[4] = { q0.w(), q0.x(), q0.y(), q0.z() };
    double cam_cube_t[3]   = { t0(0), t0(1), t0(2) };

    ceres::Problem problem;
    for (const auto &o : obs) {
      Eigen::Vector3d t_obs(o.T_cam_cube.getOrigin().x(),
                            o.T_cam_cube.getOrigin().y(),
                            o.T_cam_cube.getOrigin().z());
      tf2::Quaternion qq = o.T_cam_cube.getRotation();
      qq.normalize();
      Eigen::Quaterniond q_obs(qq.w(), qq.x(), qq.y(), qq.z());
      ceres::CostFunction* cost = PoseResidual::Create(t_obs, q_obs, o.weight);
      problem.AddResidualBlock(cost,
        new ceres::HuberLoss(1.0),
        cam_cube_rot,
        cam_cube_t);
    }

    problem.SetParameterization(cam_cube_rot,
      new ceres::EigenQuaternionParameterization());
    ceres::Solver::Options options;
    options.max_num_iterations = 50;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    Eigen::Quaterniond q_opt(cam_cube_rot[0], cam_cube_rot[1], cam_cube_rot[2], cam_cube_rot[3]);
    q_opt.normalize();
    tf2::Transform Topt;
    Topt.setOrigin(tf2::Vector3(cam_cube_t[0], cam_cube_t[1], cam_cube_t[2]));
    Topt.setRotation(tf2::Quaternion(q_opt.x(), q_opt.y(), q_opt.z(), q_opt.w()));
    return Topt;
  }

  tf2::Transform temporalFilter(const tf2::Transform& T_new, const ros::Time& stamp, int robot_id) {
    FusionState& state = robot_states_[robot_id];
    if (!state.has_prev) {
      state.prev = T_new; state.last_stamp = stamp; state.has_prev = true;
      return T_new;
    }
    double dt = (stamp - state.last_stamp).toSec();
    if (dt > timeout_sec_) {
      state.prev = T_new; state.last_stamp = stamp;
      return T_new;
    }
    tf2::Vector3 p_prev = state.prev.getOrigin();
    tf2::Vector3 p_new = T_new.getOrigin();
    tf2::Vector3 dp = p_new - p_prev;
    if (dp.length() > max_step_pos_) {
      dp *= max_step_pos_ / dp.length();
      p_new = p_prev + dp;
    }
    tf2::Vector3 p_f = p_prev * (1.0 - alpha_pos_) + p_new * alpha_pos_;

    tf2::Quaternion q_prev = state.prev.getRotation(); q_prev.normalize();
    tf2::Quaternion q_new = T_new.getRotation(); q_new.normalize();
    tf2::Quaternion dq = q_prev.inverse() * q_new; dq.normalize();
    double ang = dq.getAngle();
    if (ang > max_step_ang_) {
      tf2::Vector3 axis = dq.getAxis();
      dq.setRotation(axis, max_step_ang_); dq.normalize();
    }
    tf2::Quaternion q_step = q_prev * dq; q_step.normalize();
    tf2::Quaternion q_f = q_prev.slerp(q_step, alpha_rot_); q_f.normalize();

    tf2::Transform Tf;
    Tf.setOrigin(p_f);
    Tf.setRotation(q_f);
    state.prev = Tf; state.last_stamp = stamp;
    return Tf;
  }

  void publish(const tf2::Transform& T_cube, const ros::Time& stamp, const std::string& cam_frame, int robot_id) {
    const RobotInfo info = robotInfo(robot_id);
    const std::string cube_frame = cube_frame_prefix_ + info.name;
    const std::string tool_frame = tool_frame_prefix_ + info.name;
    const std::string topic = "/vision/" + info.name + "/cube_center";

    ROS_INFO("[Fusion] Publishing TF: %s -> %s (robot %s)", cam_frame.c_str(), cube_frame.c_str(), info.name.c_str());
    ROS_INFO("[Fusion] Position: [%.3f, %.3f, %.3f]",
             T_cube.getOrigin().x(), T_cube.getOrigin().y(), T_cube.getOrigin().z());

    auto pub_it = publishers_.find(robot_id);
    if (pub_it == publishers_.end()) {
      publishers_[robot_id] = nh_.advertise<geometry_msgs::PoseStamped>(topic, 1);
      pub_it = publishers_.find(robot_id);
      ROS_INFO("[Fusion] Advertised publisher %s for robot %s", topic.c_str(), info.name.c_str());
    }

    // 1. 发布 cube_center (立方体中心)
    {
      tf2::Transform Tn = T_cube;
      tf2::Quaternion q = Tn.getRotation(); q.normalize();
      Tn.setRotation(q);

      geometry_msgs::TransformStamped tf_msg;
      tf_msg.header.stamp = stamp;
      tf_msg.header.frame_id = cam_frame;
      tf_msg.child_frame_id = cube_frame;
      tf_msg.transform = tf2::toMsg(Tn);
      tf_broadcaster_.sendTransform(tf_msg);

      geometry_msgs::PoseStamped pmsg;
      pmsg.header = tf_msg.header;
      pmsg.pose.position.x = tf_msg.transform.translation.x;
      pmsg.pose.position.y = tf_msg.transform.translation.y;
      pmsg.pose.position.z = tf_msg.transform.translation.z;
      pmsg.pose.orientation = tf_msg.transform.rotation;
      pub_it->second.publish(pmsg);
    }

    // 2. 发布 jaka_tool (机械臂法兰 = 立方体中心 + tool_offset)
    if (has_tool_offset_) {
      tf2::Transform T_tool = T_cube * tool_offset_;  // T_cam_tool = T_cam_cube * T_cube_tool
      
      geometry_msgs::TransformStamped tf_tool;
      tf_tool.header.stamp = stamp;
      tf_tool.header.frame_id = cam_frame;
      tf_tool.child_frame_id = tool_frame;
      tf_tool.transform = tf2::toMsg(T_tool);
      tf_broadcaster_.sendTransform(tf_tool);
      
      ROS_DEBUG("[Fusion] Published tool frame at [%.3f,%.3f,%.3f]",
                T_tool.getOrigin().x(), T_tool.getOrigin().y(), T_tool.getOrigin().z());
    }

    // 验证 TF 是否发布成功
    static int counter = 0;
    if (++counter % 10 == 0) {  // 每10次发布验证一次
      std::string error_msg;
      if (tf_buffer_.canTransform(cube_frame, cam_frame, ros::Time(0), &error_msg)) {
        ROS_DEBUG("[Fusion] TF verified: %s -> %s", cam_frame.c_str(), cube_frame.c_str());
      } else {
        ROS_WARN("[Fusion] TF verification failed: %s", error_msg.c_str());
      }
    }
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::map<int, tf2::Transform> face2cube_;
  tf2::Transform tool_offset_;
  bool has_tool_offset_;

  std::string faces_yaml_path_;
  std::string fiducial_topic_;
  std::string camera_frame_default_;
  std::string cube_frame_prefix_;
  std::string tool_frame_prefix_;
  std::string robots_config_path_;
  int robot_stride_{100};
  int face_id_base_{10};
  int face_count_{4};
  std::vector<int> robot_ids_;

  double alpha_pos_, alpha_rot_;
  double max_step_pos_, max_step_ang_, timeout_sec_;
  double publish_frequency_;

  struct RobotInfo {
    int id;
    std::string name;
    std::string ns;
    std::string base_frame;
    std::string tool_frame;
  };
  std::map<int, RobotInfo> robots_;

  void loadRobotIds() {
    XmlRpc::XmlRpcValue rids_param;
    if (pnh_.hasParam("robot_ids") && pnh_.getParam("robot_ids", rids_param)) {
      if (rids_param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < rids_param.size(); ++i) {
          if (rids_param[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
            robot_ids_.push_back(static_cast<int>(rids_param[i]));
          } else if (rids_param[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
            try { robot_ids_.push_back(std::stoi(static_cast<std::string>(rids_param[i]))); } catch (...) {}
          }
        }
      }
    }
    if (robot_ids_.empty()) {
      robot_ids_.push_back(1);
    }
  }

  void loadRobotsConfig() {
    if (robots_config_path_.empty()) {
      for (int rid : robot_ids_) {
        RobotInfo info;
        info.id = rid;
        info.name = "jaka" + std::to_string(rid);
        info.ns = "/" + info.name;
        info.base_frame = "Link_0";
        info.tool_frame = "Link_6";
        robots_[rid] = info;
      }
      return;
    }

    try {
      YAML::Node root = YAML::LoadFile(robots_config_path_);
      if (root["robots"]) {
        for (const auto& r : root["robots"]) {
          RobotInfo info;
          info.id = r["id"].as<int>();
          info.name = r["name"].as<std::string>();
          info.ns = r["ns"] ? r["ns"].as<std::string>() : "/" + info.name;
          info.base_frame = r["base_frame"] ? r["base_frame"].as<std::string>() : "Link_0";
          info.tool_frame = r["tool_frame"] ? r["tool_frame"].as<std::string>() : "Link_6";
          robots_[info.id] = info;
        }
      }
    } catch (const std::exception& e) {
      ROS_WARN("[Fusion] Failed to load robots config %s: %s", robots_config_path_.c_str(), e.what());
    }

    for (int rid : robot_ids_) {
      if (robots_.find(rid) == robots_.end()) {
        RobotInfo info;
        info.id = rid;
        info.name = "jaka" + std::to_string(rid);
        info.ns = "/" + info.name;
        info.base_frame = "Link_0";
        info.tool_frame = "Link_6";
        robots_[rid] = info;
      }
    }
  }

  RobotInfo robotInfo(int rid) const {
    auto it = robots_.find(rid);
    if (it != robots_.end()) return it->second;
    RobotInfo info;
    info.id = rid;
    info.name = "jaka" + std::to_string(rid);
    info.ns = "/" + info.name;
    info.base_frame = "Link_0";
    info.tool_frame = "Link_6";
    return info;
  }

  bool shouldUseRobot(int rid) const {
    return std::find(robot_ids_.begin(), robot_ids_.end(), rid) != robot_ids_.end();
  }

  int robotIdFromFace(int face_id) const {
    if (face_id < face_id_base_) return -1;
    int offset = face_id - face_id_base_;
    int rid = offset / robot_stride_ + 1;
    int face_offset = offset % robot_stride_;
    if (face_offset >= face_count_) return rid; // allow custom spacing; rely on face list
    return rid;
  }

  tf2::Transform smoothAndValidate(const tf2::Transform& T_in, int rid) {
    FusionState& state = robot_states_[rid];

    tf2::Transform T_out = T_in;
    // Quaternion sign-flip correction
    tf2::Quaternion q_curr = T_out.getRotation();
    q_curr.normalize();
    if (state.has_prev_q) {
      double dot = state.prev_cube_q.x()*q_curr.x()
                 + state.prev_cube_q.y()*q_curr.y()
                 + state.prev_cube_q.z()*q_curr.z()
                 + state.prev_cube_q.w()*q_curr.w();
      if (dot < 0.0) {
        q_curr = tf2::Quaternion(-q_curr.x(), -q_curr.y(), -q_curr.z(), -q_curr.w());
        ROS_DEBUG("[Fusion] Quaternion sign flipped for robot %d", rid);
      }
    }
    T_out.setRotation(q_curr);
    state.prev_cube_q = q_curr;
    state.has_prev_q = true;

    // 历史姿态缓冲
    state.history_poses.push_back(T_out);
    if (state.history_poses.size() > POSE_HISTORY_SIZE) {
      state.history_poses.pop_front();
    }

    if (state.history_poses.size() == POSE_HISTORY_SIZE) {
      // 计算平滑姿态
      tf2::Vector3 sum_p(0, 0, 0);
      Eigen::Quaterniond acc_q(0, 0, 0, 0);
      for (const auto &T : state.history_poses) {
        sum_p += T.getOrigin();
        tf2::Quaternion qq = T.getRotation();
        qq.normalize();
        Eigen::Quaterniond qe(qq.w(), qq.x(), qq.y(), qq.z());
        acc_q.coeffs() += qe.coeffs();
      }
      tf2::Vector3 avg_p = sum_p * (1.0 / state.history_poses.size());
      acc_q.normalize();

      tf2::Transform T_smooth;
      T_smooth.setOrigin(avg_p);
      tf2::Quaternion q_smooth(acc_q.x(), acc_q.y(), acc_q.z(), acc_q.w());
      q_smooth.normalize();
      T_smooth.setRotation(q_smooth);

      bool accept = false;
      if (!state.has_stable_pose) {
        accept = true;
      } else {
        tf2::Vector3 dp = T_smooth.getOrigin() - state.stable_pose.getOrigin();
        double pos_diff = dp.length();

        tf2::Quaternion q_prev = state.stable_pose.getRotation();
        tf2::Quaternion q_cur = T_smooth.getRotation();
        q_prev.normalize(); q_cur.normalize();
        tf2::Quaternion dq = q_prev.inverse() * q_cur;
        dq.normalize();
        double ang = dq.getAngle();

        const double MAX_POS_JUMP = 0.05;
        const double MAX_ANG_JUMP = 30.0 * M_PI/180.0;

        if (pos_diff < MAX_POS_JUMP && ang < MAX_ANG_JUMP) {
          accept = true;
        } else {
          ROS_WARN("[Fusion] Pose rejected: pos jump %.3f m, ang jump %.1f deg for robot %d",
                   pos_diff, ang * 180.0 / M_PI, rid);
        }
      }

      if (accept) {
        state.stable_pose = T_smooth;
        state.has_stable_pose = true;
        return T_smooth;
      } else {
        return state.has_stable_pose ? state.stable_pose : T_out;
      }
    }

    return T_out;
  }
};

int main(int argc, char** argv) {
  // 初始化 glog (Ceres 依赖)
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  
  ros::init(argc, argv, "cube_multi_face_fusion_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    CubeMultiFaceFusion fusion(nh, pnh);
    ROS_INFO("[Fusion] Started successfully with Ceres optimization");
    ros::spin();
    return 0;
  } catch (const std::exception& e) {
    ROS_FATAL("[Fusion] Fatal error: %s", e.what());
    return 1;
  }
}
