#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <ros/package.h>
#include <jaka_sdk_driver/JointMove.h>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

class WorldRobotCalibMotionNode
{
public:
  WorldRobotCalibMotionNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
      : nh_(nh)
  {
    pnh.param<std::string>("arm_name", arm_name_, std::string("jaka1"));
    pnh.param<std::string>("calib_pose_csv", calib_pose_csv_,
                           ros::package::getPath("jaka_close_contro") + "/config/jaka1_world_robot_calibration_pose.csv");
    pnh.param<std::string>("joint_move_service", joint_move_service_, std::string("/jaka_driver/joint_move"));
    pnh.param<double>("joint_speed_scale", joint_speed_scale_, 0.15);
    pnh.param<double>("goal_tolerance", goal_tolerance_, 0.01);
    pnh.param<double>("hold_time_after_reach", hold_time_after_reach_, 1.5);
    pnh.param<double>("max_wait_per_target", max_wait_per_target_, 30.0);
    pnh.param<double>("check_rate", check_rate_hz_, 20.0);

    joint_state_sub_ = nh_.subscribe("/joint_states", 10, &WorldRobotCalibMotionNode::jointStateCb, this);
    joint_move_client_ = nh_.serviceClient<jaka_sdk_driver::JointMove>(joint_move_service_);

    if (!loadCalibPoses())
    {
      ROS_ERROR("[CalibMotion] 无法加载标定轨迹 CSV，节点退出");
      ros::shutdown();
      return;
    }

    ROS_INFO("[CalibMotion] 臂标识=%s, 标定轨迹姿态数: %zu", arm_name_.c_str(), poses_.size());
    ROS_INFO("[CalibMotion] 速度缩放=%.2f, 收敛阈值=%.3f rad, 到位保持=%.2f s", joint_speed_scale_, goal_tolerance_, hold_time_after_reach_);
    ROS_INFO("[CalibMotion] CSV 文件: %s", calib_pose_csv_.c_str());
    ROS_INFO("[CalibMotion] joint_move 服务: %s", joint_move_service_.c_str());

    executeTrajectory();
  }

private:
  ros::NodeHandle nh_;
  std::string arm_name_;
  std::string calib_pose_csv_;
  std::string joint_move_service_;
  double joint_speed_scale_ = 0.15;
  double goal_tolerance_ = 0.01;
  double hold_time_after_reach_ = 1.5;
  double max_wait_per_target_ = 30.0;
  double check_rate_hz_ = 20.0;

  std::vector<std::vector<double>> poses_;
  std::vector<double> current_joints_;
  bool has_joint_state_ = false;

  ros::Subscriber joint_state_sub_;
  ros::ServiceClient joint_move_client_;

  void jointStateCb(const sensor_msgs::JointState::ConstPtr &msg)
  {
    current_joints_ = msg->position;
    has_joint_state_ = true;
  }

  bool loadCalibPoses()
  {
    std::ifstream fin(calib_pose_csv_);
    if (!fin.is_open())
    {
      ROS_ERROR("[CalibMotion] 无法打开 CSV: %s", calib_pose_csv_.c_str());
      return false;
    }

    std::string line;
    bool header_skipped = false;
    while (std::getline(fin, line))
    {
      if (line.empty() || line[0] == '#')
      {
        continue;
      }
      if (!header_skipped)
      {
        header_skipped = true;
        continue;
      }

      std::vector<double> joints;
      std::stringstream ss(line);
      std::string item;
      while (std::getline(ss, item, ','))
      {
        try
        {
          joints.push_back(std::stod(item));
        }
        catch (const std::exception &e)
        {
          ROS_WARN("[CalibMotion] CSV 解析失败，跳过该行: %s", line.c_str());
          joints.clear();
          break;
        }
      }

      if (joints.size() == 6)
      {
        poses_.push_back(joints);
      }
      else if (!joints.empty())
      {
        ROS_WARN("[CalibMotion] 该行不是 6 轴关节数据，已跳过: %s", line.c_str());
      }
    }

    return !poses_.empty();
  }

  bool sendJointTarget(const std::vector<double> &target)
  {
    if (!joint_move_client_.exists())
    {
      ROS_WARN("[CalibMotion] 等待 /jaka_driver/joint_move 服务...");
      joint_move_client_.waitForExistence();
    }

    jaka_sdk_driver::JointMove srv;
    srv.request.pose = target;
    srv.request.has_ref = false;
    srv.request.ref_joint.clear();
    srv.request.mvvelo = std::max(0.01, joint_speed_scale_); // SDK 速度单位取决于具体接口，这里直接按比例缩放
    srv.request.mvacc = std::max(0.01, 0.5 * joint_speed_scale_);
    srv.request.mvtime = 0.0;
    srv.request.mvradii = 0.0;
    srv.request.coord_mode = 0;
    srv.request.index = 0;

    if (!joint_move_client_.call(srv))
    {
      ROS_ERROR("[CalibMotion] joint_move 调用失败（通信错误）");
      return false;
    }
    if (srv.response.ret != 0)
    {
      ROS_ERROR("[CalibMotion] joint_move SDK 返回错误，ret=%d, message=%s", srv.response.ret, srv.response.message.c_str());
      return false;
    }

    return true;
  }

  bool waitUntilReached(const std::vector<double> &target)
  {
    ros::Rate rate(check_rate_hz_);
    const ros::Time start = ros::Time::now();
    bool within_tolerance = false;
    ros::Time hold_start;

    while (ros::ok())
    {
      ros::spinOnce();
      if (!has_joint_state_)
      {
        rate.sleep();
        continue;
      }

      double max_diff = 0.0;
      for (size_t i = 0; i < target.size() && i < current_joints_.size(); ++i)
      {
        max_diff = std::max(max_diff, std::fabs(target[i] - current_joints_[i]));
      }

      if (max_diff < goal_tolerance_)
      {
        if (!within_tolerance)
        {
          within_tolerance = true;
          hold_start = ros::Time::now();
          ROS_INFO("[CalibMotion] 已到位（误差 %.4f rad），保持 %.2f s 稳定...", max_diff, hold_time_after_reach_);
        }
        if ((ros::Time::now() - hold_start).toSec() >= hold_time_after_reach_)
        {
          return true;
        }
      }
      else
      {
        within_tolerance = false;
      }

      if ((ros::Time::now() - start).toSec() > max_wait_per_target_)
      {
        ROS_ERROR("[CalibMotion] 等待到位超时（%.1f s），最大误差=%.4f rad", max_wait_per_target_, max_diff);
        return false;
      }

      rate.sleep();
    }

    return false;
  }

  void executeTrajectory()
  {
    for (size_t i = 0; i < poses_.size() && ros::ok(); ++i)
    {
      ROS_INFO("[CalibMotion] 执行标定姿态 #%zu", i + 1);
      const std::vector<double> &target = poses_[i];

      if (!sendJointTarget(target))
      {
        ROS_ERROR("[CalibMotion] 发送姿态 #%zu 失败，停止后续执行", i + 1);
        break;
      }

      if (!waitUntilReached(target))
      {
        ROS_ERROR("[CalibMotion] 姿态 #%zu 未能稳定到位，停止后续执行", i + 1);
        break;
      }

      ROS_INFO("[CalibMotion] 姿态 #%zu 完成，等待下一步", i + 1);
    }

    ROS_INFO("[CalibMotion] 标定轨迹执行完成");
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "world_robot_calib_motion_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  WorldRobotCalibMotionNode node(nh, pnh);
  ros::spin();
  return 0;
}

