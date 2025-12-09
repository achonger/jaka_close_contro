#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/calib3d.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <fstream>

class HandEyeCalibrationNode
{
public:
  HandEyeCalibrationNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : tf_listener_(tf_buffer_)
  {
    // 使用私有命名空间读取参数，确保 launch 中的配置生效
    pnh.param<std::string>("world_fiducial_topic", world_fiducial_topic_, "/world_fiducials");
    pnh.param<int>("world_fiducial_id", world_fiducial_id_, 0);
    pnh.param<std::string>("robot_base_frame", robot_base_frame_, "Link_0");
    pnh.param<std::string>("tool_frame", tool_frame_, "Link_6");
    pnh.param<std::string>("camera_frame", camera_frame_, "zed2i_left_camera_optical_frame");
    pnh.param<std::string>("output_frame", output_frame_, "handeye_calibration");

    const std::string default_output = ros::package::getPath("jaka_close_contro") + "/config/hand_eye_calibration.yaml";
    pnh.param<std::string>("output_yaml", output_yaml_path_, default_output);

    fiducial_sub_ = nh.subscribe(world_fiducial_topic_, 1, &HandEyeCalibrationNode::fiducialCallback, this);
    joint_state_sub_ = nh.subscribe("/joint_states", 1, &HandEyeCalibrationNode::jointStateCallback, this);

    collect_service_ = nh.advertiseService("/collect_calibration_data", &HandEyeCalibrationNode::collectCalibrationData, this);
    calibrate_service_ = nh.advertiseService("/calibrate_hand_eye", &HandEyeCalibrationNode::calibrateHandEye, this);

    result_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/hand_eye_calibration_result", 1);

    ROS_INFO("Hand-eye calibration node ready. Listening on %s for fiducials (ID=%d)",
             world_fiducial_topic_.c_str(), world_fiducial_id_);

    loadExistingCalibration();
  }

private:
  ros::Subscriber fiducial_sub_;
  ros::Subscriber joint_state_sub_;
  ros::ServiceServer collect_service_;
  ros::ServiceServer calibrate_service_;
  ros::Publisher result_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::StaticTransformBroadcaster tf_broadcaster_;

  fiducial_msgs::FiducialTransformArray latest_fiducials_;
  sensor_msgs::JointState latest_joint_state_;
  bool have_fiducial_ = false;
  bool have_joint_state_ = false;

  std::vector<cv::Mat> R_gripper2base_;
  std::vector<cv::Mat> t_gripper2base_;
  std::vector<cv::Mat> R_target2cam_;
  std::vector<cv::Mat> t_target2cam_;

  std::string world_fiducial_topic_;
  int world_fiducial_id_ = 0;
  std::string robot_base_frame_;
  std::string tool_frame_;
  std::string camera_frame_;
  std::string output_frame_;
  std::string output_yaml_path_;

  void fiducialCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg)
  {
    latest_fiducials_ = *msg;
    have_fiducial_ = true;
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    latest_joint_state_ = *msg;
    have_joint_state_ = true;
  }

  bool collectCalibrationData(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    if (!have_fiducial_ || !have_joint_state_) {
      res.success = false;
      res.message = "缺少 fiducial 或关节数据";
      ROS_WARN("[HandEye] Missing fiducial or joint data, cannot collect.");
      return true;
    }

    auto maybe_target = getTargetTransform();
    if (!maybe_target.first) {
      res.success = false;
      res.message = "未检测到指定的世界标记";
      return true;
    }

    geometry_msgs::TransformStamped T_base_tool_msg;
    if (!lookupTransform(robot_base_frame_, tool_frame_, latest_joint_state_.header.stamp, T_base_tool_msg)) {
      res.success = false;
      res.message = "无法获取基座到末端的 TF";
      return true;
    }

    appendSample(T_base_tool_msg.transform, maybe_target.second);

    res.success = true;
    res.message = "采集成功";
    ROS_INFO("[HandEye] 已采集 %zu 组样本", R_gripper2base_.size());
    return true;
  }

  bool calibrateHandEye(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    if (R_gripper2base_.size() < 3) {
      res.success = false;
      res.message = "样本数量不足，至少需要 3 组";
      ROS_ERROR("[HandEye] Not enough samples: %zu", R_gripper2base_.size());
      return true;
    }

    cv::Mat R_cam2gripper, t_cam2gripper;
    try {
      cv::calibrateHandEye(R_gripper2base_, t_gripper2base_, R_target2cam_, t_target2cam_, R_cam2gripper, t_cam2gripper, cv::CALIB_HAND_EYE_TSAI);
    } catch (const cv::Exception& e) {
      res.success = false;
      res.message = std::string("标定计算失败: ") + e.what();
      ROS_ERROR("[HandEye] Calibration failed: %s", e.what());
      return true;
    }

    geometry_msgs::TransformStamped result = toTransform(R_cam2gripper, t_cam2gripper);
    result.header.frame_id = camera_frame_;
    result.child_frame_id = tool_frame_;
    result.header.stamp = ros::Time::now();

    publishAndSaveResult(result);

    res.success = true;
    res.message = "标定完成";
    return true;
  }

  std::pair<bool, geometry_msgs::Transform> getTargetTransform()
  {
    for (const auto& t : latest_fiducials_.transforms) {
      if (t.fiducial_id == world_fiducial_id_) {
        geometry_msgs::Transform tf;
        tf.translation.x = t.transform.translation.x;
        tf.translation.y = t.transform.translation.y;
        tf.translation.z = t.transform.translation.z;
        tf.rotation.x = t.transform.rotation.x;
        tf.rotation.y = t.transform.rotation.y;
        tf.rotation.z = t.transform.rotation.z;
        tf.rotation.w = t.transform.rotation.w;
        return {true, tf};
      }
    }
    ROS_WARN_THROTTLE(1.0, "[HandEye] 未检测到 ID=%d 的世界标记", world_fiducial_id_);
    return {false, geometry_msgs::Transform{}};
  }

  bool lookupTransform(const std::string& parent,
                       const std::string& child,
                       const ros::Time& stamp,
                       geometry_msgs::TransformStamped& out)
  {
    ros::Time query_time = stamp.isZero() ? ros::Time(0) : stamp;
    try {
      out = tf_buffer_.lookupTransform(parent, child, query_time, ros::Duration(1.0));
      return true;
    } catch (const tf2::TransformException& ex) {
      ROS_ERROR_THROTTLE(1.0, "[HandEye] TF %s -> %s 查询失败: %s", parent.c_str(), child.c_str(), ex.what());
      return false;
    }
  }

  void appendSample(const geometry_msgs::Transform& base_to_tool,
                    const geometry_msgs::Transform& target_to_cam)
  {
    Eigen::Matrix3d R_bt = quaternionToMatrix(base_to_tool.rotation);
    Eigen::Vector3d t_bt(base_to_tool.translation.x,
                        base_to_tool.translation.y,
                        base_to_tool.translation.z);

    Eigen::Matrix3d R_tc = quaternionToMatrix(target_to_cam.rotation);
    Eigen::Vector3d t_tc(target_to_cam.translation.x,
                        target_to_cam.translation.y,
                        target_to_cam.translation.z);

    R_gripper2base_.push_back(eigenToCv(R_bt));
    t_gripper2base_.push_back(eigenToCv(t_bt));
    R_target2cam_.push_back(eigenToCv(R_tc));
    t_target2cam_.push_back(eigenToCv(t_tc));
  }

  Eigen::Matrix3d quaternionToMatrix(const geometry_msgs::Quaternion& q_msg)
  {
    Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
    q.normalize();
    return q.toRotationMatrix();
  }

  cv::Mat eigenToCv(const Eigen::Matrix3d& R)
  {
    cv::Mat mat(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        mat.at<double>(r, c) = R(r, c);
    return mat;
  }

  cv::Mat eigenToCv(const Eigen::Vector3d& t)
  {
    cv::Mat mat(3, 1, CV_64F);
    mat.at<double>(0) = t(0);
    mat.at<double>(1) = t(1);
    mat.at<double>(2) = t(2);
    return mat;
  }

  geometry_msgs::TransformStamped toTransform(const cv::Mat& R, const cv::Mat& t)
  {
    Eigen::Matrix3d R_eig;
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        R_eig(r, c) = R.at<double>(r, c);

    Eigen::Quaterniond q(R_eig);
    q.normalize();

    geometry_msgs::TransformStamped tf;
    tf.transform.translation.x = t.at<double>(0);
    tf.transform.translation.y = t.at<double>(1);
    tf.transform.translation.z = t.at<double>(2);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    return tf;
  }

  void publishAndSaveResult(const geometry_msgs::TransformStamped& tf_msg)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = tf_msg.header;
    pose.pose.position.x = tf_msg.transform.translation.x;
    pose.pose.position.y = tf_msg.transform.translation.y;
    pose.pose.position.z = tf_msg.transform.translation.z;
    pose.pose.orientation = tf_msg.transform.rotation;

    result_pub_.publish(pose);

    tf_broadcaster_.sendTransform(tf_msg);

    YAML::Node node;
    node["hand_eye_calibration"]["position"]["x"] = pose.pose.position.x;
    node["hand_eye_calibration"]["position"]["y"] = pose.pose.position.y;
    node["hand_eye_calibration"]["position"]["z"] = pose.pose.position.z;
    node["hand_eye_calibration"]["orientation"]["w"] = pose.pose.orientation.w;
    node["hand_eye_calibration"]["orientation"]["x"] = pose.pose.orientation.x;
    node["hand_eye_calibration"]["orientation"]["y"] = pose.pose.orientation.y;
    node["hand_eye_calibration"]["orientation"]["z"] = pose.pose.orientation.z;

    std::ofstream ofs(output_yaml_path_);
    ofs << node;
    ofs.close();

    ROS_INFO("[HandEye] 标定结果发布到 %s，已保存到 %s",
             pose.header.frame_id.c_str(), output_yaml_path_.c_str());
  }

  void loadExistingCalibration()
  {
    try
    {
      const YAML::Node node = YAML::LoadFile(output_yaml_path_);

      const auto& root = node["hand_eye_calibration"];
      if (!root)
      {
        return;
      }

      geometry_msgs::TransformStamped tf_msg;
      tf_msg.header.frame_id = camera_frame_;
      tf_msg.child_frame_id = tool_frame_;
      tf_msg.header.stamp = ros::Time::now();

      tf_msg.transform.translation.x = root["position"]["x"].as<double>();
      tf_msg.transform.translation.y = root["position"]["y"].as<double>();
      tf_msg.transform.translation.z = root["position"]["z"].as<double>();
      tf_msg.transform.rotation.w = root["orientation"]["w"].as<double>();
      tf_msg.transform.rotation.x = root["orientation"]["x"].as<double>();
      tf_msg.transform.rotation.y = root["orientation"]["y"].as<double>();
      tf_msg.transform.rotation.z = root["orientation"]["z"].as<double>();

      // 发布一次结果，便于重启后无需重新标定即可使用
      result_pub_.publish(toPose(tf_msg));
      tf_broadcaster_.sendTransform(tf_msg);
      ROS_INFO("[HandEye] 已从 %s 载入历史标定并发布 TF", output_yaml_path_.c_str());
    }
    catch (const YAML::Exception& e)
    {
      ROS_INFO("[HandEye] 未找到已有标定文件(%s)，等待新标定", output_yaml_path_.c_str());
    }
  }

  geometry_msgs::PoseStamped toPose(const geometry_msgs::TransformStamped& tf_msg)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = tf_msg.header;
    pose.pose.position.x = tf_msg.transform.translation.x;
    pose.pose.position.y = tf_msg.transform.translation.y;
    pose.pose.position.z = tf_msg.transform.translation.z;
    pose.pose.orientation = tf_msg.transform.rotation;
    return pose;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibration_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  HandEyeCalibrationNode node(nh, pnh);
  ros::spin();
  return 0;
}

