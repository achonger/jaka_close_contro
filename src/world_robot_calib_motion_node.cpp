#include <ros/ros.h>
#include <ros/package.h>
#include <jaka_sdk_driver/LinearMove.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

struct CalibPoseRow
{
  std::string name;
  double x_mm = 0.0;
  double y_mm = 0.0;
  double z_mm = 0.0;
  double rx_deg = 0.0;
  double ry_deg = 0.0;
  double rz_deg = 0.0;
};

class WorldRobotCalibMotionNode
{
public:
  WorldRobotCalibMotionNode(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh)
  {
    pnh.param<std::string>("arm_name", arm_name_, std::string("jaka1"));
    pnh.param<std::string>("calib_pose_csv", calib_pose_csv_,
                           ros::package::getPath("jaka_close_contro") + "/config/jaka1_world_robot_calibration_pose.csv");
    pnh.param<std::string>("linear_move_service", linear_move_service_, std::string("/jaka_driver/linear_move"));

    pnh.param<double>("speed_scale", speed_scale_, 0.15);
    // 兼容旧参数名 joint_speed_scale
    pnh.param<double>("joint_speed_scale", speed_scale_, speed_scale_);
    pnh.param<double>("linear_speed_mm_s", linear_speed_mm_s_, 80.0);
    pnh.param<double>("linear_acc_mm_s2", linear_acc_mm_s2_, 200.0);
    pnh.param<double>("hold_time_after_reach", hold_time_after_reach_, 1.0);

    linear_move_client_ = nh_.serviceClient<jaka_sdk_driver::LinearMove>(linear_move_service_);

    if (!loadCalibPoses())
    {
      ROS_ERROR("[CalibMotion] 无法加载标定姿态 CSV，节点退出");
      ros::shutdown();
      return;
    }

    ROS_INFO("[CalibMotion] 臂标识=%s, 标定姿态数: %zu", arm_name_.c_str(), poses_.size());
    ROS_INFO("[CalibMotion] 速度缩放=%.2f, 线速度=%.1f mm/s, 线加速度=%.1f mm/s^2, 到位保持=%.2f s", speed_scale_,
             linear_speed_mm_s_, linear_acc_mm_s2_, hold_time_after_reach_);
    ROS_INFO("[CalibMotion] CSV 文件: %s", calib_pose_csv_.c_str());
    ROS_INFO("[CalibMotion] linear_move 服务: %s", linear_move_service_.c_str());

    executeTrajectory();
  }

private:
  ros::NodeHandle nh_;
  std::string arm_name_;
  std::string calib_pose_csv_;
  std::string linear_move_service_;
  double speed_scale_ = 0.15;
  double linear_speed_mm_s_ = 80.0;
  double linear_acc_mm_s2_ = 200.0;
  double hold_time_after_reach_ = 1.0;

  std::vector<CalibPoseRow> poses_;
  ros::ServiceClient linear_move_client_;

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
        continue; // 跳过表头
      }

      std::stringstream ss(line);
      std::string item;
      std::vector<std::string> cols;
      while (std::getline(ss, item, ','))
      {
        cols.push_back(item);
      }

      if (cols.size() != 7)
      {
        ROS_WARN("[CalibMotion] CSV 行格式错误（期望7列），已跳过: %s", line.c_str());
        continue;
      }

      CalibPoseRow row;
      row.name = cols[0];
      try
      {
        row.x_mm = std::stod(cols[1]);
        row.y_mm = std::stod(cols[2]);
        row.z_mm = std::stod(cols[3]);
        row.rx_deg = std::stod(cols[4]);
        row.ry_deg = std::stod(cols[5]);
        row.rz_deg = std::stod(cols[6]);
      }
      catch (const std::exception &e)
      {
        ROS_WARN("[CalibMotion] CSV 数值解析失败，跳过该行: %s", line.c_str());
        continue;
      }

      poses_.push_back(row);
    }

    return !poses_.empty();
  }

  bool sendLinearTarget(const CalibPoseRow &target)
  {
    if (!linear_move_client_.exists())
    {
      ROS_WARN("[CalibMotion] 等待 %s 服务...", linear_move_service_.c_str());
      linear_move_client_.waitForExistence();
    }

    const double DEG2RAD = M_PI / 180.0;
    jaka_sdk_driver::LinearMove srv;
    srv.request.pose = {target.x_mm, target.y_mm, target.z_mm,
                        target.rx_deg * DEG2RAD, target.ry_deg * DEG2RAD, target.rz_deg * DEG2RAD};
    srv.request.mvvelo = linear_speed_mm_s_ * speed_scale_;
    srv.request.mvacc = linear_acc_mm_s2_ * speed_scale_;
    srv.request.mvtime = 0.0;
    srv.request.mvradii = 0.0;
    srv.request.coord_mode = 0; // 基座坐标系
    srv.request.index = 0;

    if (!linear_move_client_.call(srv))
    {
      ROS_ERROR("[CalibMotion] linear_move 通信失败，姿态 %s 未发送", target.name.c_str());
      return false;
    }
    if (srv.response.ret != 0)
    {
      ROS_ERROR("[CalibMotion] linear_move 执行失败，ret=%d, message=%s", srv.response.ret, srv.response.message.c_str());
      return false;
    }

    return true;
  }

  void executeTrajectory()
  {
    for (size_t i = 0; i < poses_.size() && ros::ok(); ++i)
    {
      const auto &p = poses_[i];
      ROS_INFO("[CalibMotion] 执行标定位姿 #%zu (%s): pos(mm)=(%.1f, %.1f, %.1f), rpy(deg)=(%.1f, %.1f, %.1f)", i + 1,
               p.name.c_str(), p.x_mm, p.y_mm, p.z_mm, p.rx_deg, p.ry_deg, p.rz_deg);

      if (!sendLinearTarget(p))
      {
        ROS_ERROR("[CalibMotion] 姿态 %s 发送失败，停止后续执行", p.name.c_str());
        break;
      }

      ros::Duration(hold_time_after_reach_).sleep();
      ROS_INFO("[CalibMotion] 姿态 %s 完成，等待下一步", p.name.c_str());
    }

    ROS_INFO("[CalibMotion] 标定姿态序列执行完成");
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
