#include <ros/ros.h>
#include <ros/package.h>
#include <jaka_msgs/Move.h>
#include <jaka_close_contro/WorldRobotCollectSample.h>

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

    const bool has_wait_before = pnh.getParam("wait_before_sample_sec", wait_before_sample_sec_);
    const bool has_wait_after = pnh.getParam("wait_after_sample_sec", wait_after_sample_sec_);
    const bool has_pre = pnh.getParam("pre_sample_wait_sec", pre_sample_wait_sec_);
    const bool has_post = pnh.getParam("post_sample_wait_sec", post_sample_wait_sec_);

    if (!has_wait_before && has_pre)
    {
      wait_before_sample_sec_ = pre_sample_wait_sec_;
    }
    if (!has_wait_after && has_post)
    {
      wait_after_sample_sec_ = post_sample_wait_sec_;
    }

    if (!has_wait_before && !has_pre)
    {
      // 兼容旧参数：如果未提供新参数，则沿用 hold_time_after_reach
      pnh.param<double>("hold_time_after_reach", hold_time_after_reach_, 1.0);
      wait_before_sample_sec_ = hold_time_after_reach_;
    }

    pnh.param<std::string>("collect_sample_service", collect_sample_service_, std::string("/collect_world_robot_sample"));
    pnh.param<bool>("trigger_sample_service", trigger_sample_service_, true);

    linear_move_client_ = nh_.serviceClient<jaka_msgs::Move>(linear_move_service_);
    collect_sample_client_ = nh_.serviceClient<jaka_close_contro::WorldRobotCollectSample>(collect_sample_service_);

    if (!loadCalibPoses())
    {
      ROS_ERROR("[CalibMotion] 无法加载标定姿态 CSV，节点退出");
      ros::shutdown();
      return;
    }

    ROS_INFO("[CalibMotion] 臂标识=%s, 标定姿态数: %zu", arm_name_.c_str(), poses_.size());
    ROS_INFO("[CalibMotion] 速度缩放=%.2f, 线速度=%.1f mm/s, 线加速度=%.1f mm/s^2", speed_scale_,
             linear_speed_mm_s_, linear_acc_mm_s2_);
    ROS_INFO("[CalibMotion] 到位等待：采样前 %.2f s，采样后 %.2f s", wait_before_sample_sec_, wait_after_sample_sec_);
    ROS_INFO("[CalibMotion] CSV 文件: %s", calib_pose_csv_.c_str());
    ROS_INFO("[CalibMotion] linear_move 服务: %s", linear_move_service_.c_str());
    ROS_INFO("[CalibMotion] collect_sample 服务: %s, 是否主动触发=%s", collect_sample_service_.c_str(),
             trigger_sample_service_ ? "true" : "false");

    waitForService();
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
  double pre_sample_wait_sec_ = 5.0;  // 兼容旧参数
  double post_sample_wait_sec_ = 2.0; // 兼容旧参数
  double wait_before_sample_sec_ = 5.0;
  double wait_after_sample_sec_ = 2.0;
  std::string collect_sample_service_ = "/collect_world_robot_sample";
  bool trigger_sample_service_ = true;

  std::size_t target_index_ = 0;

  std::vector<CalibPoseRow> poses_;
  ros::ServiceClient linear_move_client_;
  ros::ServiceClient collect_sample_client_;

  void waitForService()
  {
    while (ros::ok())
    {
      if (ros::service::waitForService(linear_move_service_, ros::Duration(1.0)))
      {
        ROS_INFO("[CalibMotion] linear_move 服务就绪: %s", linear_move_service_.c_str());
        return;
      }
      ROS_WARN("[CalibMotion] 等待 %s ...", linear_move_service_.c_str());
    }
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
    const double DEG2RAD = M_PI / 180.0;
    jaka_msgs::Move srv;
    srv.request.pose = {static_cast<float>(target.x_mm), static_cast<float>(target.y_mm), static_cast<float>(target.z_mm),
                        static_cast<float>(target.rx_deg * DEG2RAD), static_cast<float>(target.ry_deg * DEG2RAD),
                        static_cast<float>(target.rz_deg * DEG2RAD)};
    srv.request.mvvelo = linear_speed_mm_s_ * speed_scale_;
    srv.request.mvacc = linear_acc_mm_s2_ * speed_scale_;
    srv.request.mvtime = 0.0;
    srv.request.mvradii = 0.0;
    srv.request.coord_mode = 0; // 基座坐标系
    srv.request.index = static_cast<int32_t>(target_index_);

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

    ROS_INFO("[CalibMotion] linear_move ret=%d, message=%s", srv.response.ret, srv.response.message.c_str());

    return true;
  }

  bool triggerSample(std::size_t pose_index, const std::string &pose_name)
  {
    if (!trigger_sample_service_)
    {
      ROS_INFO("[CalibMotion] 采样窗口（被动），当前姿态 #%zu (%s)", pose_index + 1, pose_name.c_str());
      return true;
    }

    if (!collect_sample_client_.exists())
    {
      ROS_WARN("[CalibMotion] collect_sample 服务未就绪，等待中... (%s)", collect_sample_service_.c_str());
      collect_sample_client_.waitForExistence();
    }

    jaka_close_contro::WorldRobotCollectSample srv;
    if (!collect_sample_client_.call(srv))
    {
      ROS_ERROR("[CalibMotion] collect_sample 调用失败（通信错误），姿态 #%zu (%s)", pose_index + 1, pose_name.c_str());
      return false;
    }

    if (!srv.response.success)
    {
      ROS_WARN("[CalibMotion] collect_sample 返回失败：%s", srv.response.message.c_str());
      return false;
    }

    ROS_INFO_STREAM("[CalibMotion] 采样完成 #" << (pose_index + 1)
                    << "\n  world_cube_meas.pos  = (" << srv.response.world_cube_pos_x << ", " << srv.response.world_cube_pos_y
                    << ", " << srv.response.world_cube_pos_z << ")"
                    << "\n  world_cube_meas.quat = (" << srv.response.world_cube_quat_x << ", " << srv.response.world_cube_quat_y
                    << ", " << srv.response.world_cube_quat_z << ", " << srv.response.world_cube_quat_w << ")"
                    << "\n  base_cube_pred.pos   = (" << srv.response.base_cube_pos_x << ", " << srv.response.base_cube_pos_y
                    << ", " << srv.response.base_cube_pos_z << ")"
                    << "\n  base_cube_pred.quat  = (" << srv.response.base_cube_quat_x << ", " << srv.response.base_cube_quat_y
                    << ", " << srv.response.base_cube_quat_z << ", " << srv.response.base_cube_quat_w << ")");
    return true;
  }

  void executeTrajectory()
  {
    const double effective_speed = linear_speed_mm_s_ * speed_scale_;
    const double effective_acc = linear_acc_mm_s2_ * speed_scale_;
    ROS_INFO("[CalibMotion] 实际发送速度=%.1f mm/s, 加速度=%.1f mm/s^2", effective_speed, effective_acc);

    for (size_t i = 0; i < poses_.size() && ros::ok(); ++i)
    {
      const auto &p = poses_[i];
      target_index_ = i;
      ROS_INFO_STREAM("[CalibMotion] [" << ros::Time::now() << "] 发送 linear_move 到标定位姿 #" << (i + 1)
                                           << " (" << p.name << "): pos(mm)=(" << p.x_mm << ", " << p.y_mm << ", "
                                           << p.z_mm << "), rpy(deg)=(" << p.rx_deg << ", " << p.ry_deg << ", "
                                           << p.rz_deg << ")");

      if (!sendLinearTarget(p))
      {
        ROS_ERROR("[CalibMotion] 姿态 %s 发送失败，停止后续执行", p.name.c_str());
        break;
      }

      ROS_INFO_STREAM("[CalibMotion] [" << ros::Time::now() << "] linear_move 返回，采样前静止 "
                                          << wait_before_sample_sec_ << " s");
      ros::Duration(wait_before_sample_sec_).sleep();

      ROS_INFO_STREAM("[CalibMotion] [" << ros::Time::now() << "] 开始采样触发（姿态 #" << (i + 1) << ")");
      triggerSample(i, p.name);

      ROS_INFO_STREAM("[CalibMotion] [" << ros::Time::now() << "] 采样结束，继续静止 "
                                          << wait_after_sample_sec_ << " s");
      ros::Duration(wait_after_sample_sec_).sleep();

      ROS_INFO_STREAM("[CalibMotion] [" << ros::Time::now() << "] 姿态 " << p.name << " 完成，准备下一个点");
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
