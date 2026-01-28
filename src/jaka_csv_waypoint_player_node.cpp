#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <jaka_msgs/Move.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <boost/bind.hpp>
#include <memory>

namespace
{
struct Waypoint
{
  int idx = 0;
  double x_mm = 0.0;
  double y_mm = 0.0;
  double z_mm = 0.0;
  double rx = 0.0;
  double ry = 0.0;
  double rz = 0.0;
};

bool isNumeric(const std::string &s)
{
  char *end = nullptr;
  std::strtod(s.c_str(), &end);
  return end != s.c_str() && *end == '\0';
}

std::string trim(const std::string &s)
{
  size_t start = 0;
  while (start < s.size() && std::isspace(static_cast<unsigned char>(s[start])))
  {
    ++start;
  }
  size_t end = s.size();
  while (end > start && std::isspace(static_cast<unsigned char>(s[end - 1])))
  {
    --end;
  }
  return s.substr(start, end - start);
}
} // namespace

class JakaCsvWaypointPlayer
{
public:
  JakaCsvWaypointPlayer(ros::NodeHandle &nh, ros::NodeHandle &pnh)
      : nh_(nh), pnh_(pnh)
  {
    pnh_.param<std::string>("waypoint_csv", waypoint_csv_,
                            ros::package::getPath("jaka_close_contro") + "/config/Ushape_sample6_points.csv");
    pnh_.param<std::string>("linear_move_service", linear_move_service_, std::string("jaka_driver/linear_move"));
    pnh_.param<std::string>("joint_state_topic", joint_state_topic_, std::string("/joint_states"));
    pnh_.param<double>("linear_speed_mm_s", linear_speed_mm_s_, 80.0);
    pnh_.param<double>("linear_acc_mm_s2", linear_acc_mm_s2_, 200.0);
    pnh_.param<double>("motion_done_timeout_sec", motion_done_timeout_sec_, 120.0);
    pnh_.param<double>("motion_stable_duration_sec", motion_stable_duration_sec_, 0.5);
    pnh_.param<double>("motion_joint_threshold_rad", motion_joint_threshold_rad_, 0.002);
    pnh_.param<double>("dwell_sec", dwell_sec_, 2.0);
    pnh_.param<double>("speed_scale", speed_scale_, 0.15);
    pnh_.param<bool>("angles_in_degrees", angles_in_degrees_, true);
    pnh_.param<int>("coord_mode", coord_mode_, 0);
    pnh_.param<bool>("sort_by_idx", sort_by_idx_, true);
    pnh_.param<bool>("use_driver", use_driver_, true);
    pnh_.param<bool>("do_motion", do_motion_, true);
    pnh_.param<bool>("round_trip", round_trip_, true);
    pnh_.param<bool>("reverse_include_last", reverse_include_last_, false);
    pnh_.param<bool>("require_same_count", require_same_count_, true);
    pnh_.param<bool>("sync_by_idx", sync_by_idx_, true);
    pnh_.param<std::string>("tcp_name", tcp_name_, std::string("megnetic_1"));

    if (speed_scale_ > 0.15)
    {
      ROS_WARN("[CsvWaypoint] speed_scale=%.3f 超过 0.15，已限制为 0.15", speed_scale_);
      speed_scale_ = 0.15;
    }

    loadArmsFromParams();
    if (multi_arm_mode_)
    {
      setupArmInterfaces();
    }
    else
    {
      joint_state_sub_ = nh_.subscribe(joint_state_topic_, 50, &JakaCsvWaypointPlayer::jointStateCallback, this);
      linear_move_client_ = nh_.serviceClient<jaka_msgs::Move>(linear_move_service_);
    }
  }

  bool loadWaypoints()
  {
    if (multi_arm_mode_)
    {
      if (!loadMultiArmWaypoints())
      {
        return false;
      }
      return true;
    }

    if (!loadWaypointsFromCsv(waypoint_csv_, waypoints_))
    {
      return false;
    }
    ROS_INFO("[CsvWaypoint] 已加载 %zu 个路点，TCP=%s", waypoints_.size(), tcp_name_.c_str());
    return true;
  }

  void waitForService()
  {
    if (!use_driver_)
    {
      ROS_WARN("[CsvWaypoint] use_driver=false，跳过服务检查");
      return;
    }
    if (multi_arm_mode_)
    {
      for (auto &arm : arms_)
      {
        if (!arm || !arm->enabled)
        {
          continue;
        }
        while (ros::ok())
        {
          if (ros::service::waitForService(arm->linear_move_service, ros::Duration(1.0)))
          {
            ROS_INFO("[CsvWaypoint] linear_move 服务就绪: %s", arm->linear_move_service.c_str());
            break;
          }
          ROS_WARN("[CsvWaypoint] 等待 %s ...", arm->linear_move_service.c_str());
        }
      }
      return;
    }

    while (ros::ok())
    {
      if (ros::service::waitForService(linear_move_service_, ros::Duration(1.0)))
      {
        ROS_INFO("[CsvWaypoint] linear_move 服务就绪: %s", linear_move_service_.c_str());
        return;
      }
      ROS_WARN("[CsvWaypoint] 等待 %s ...", linear_move_service_.c_str());
    }
  }

  bool sendLinearTarget(const Waypoint &wp, int sequence_index)
  {
    if (!use_driver_ || !do_motion_)
    {
      ROS_INFO("[CsvWaypoint] use_driver/do_motion=false，跳过运动发送");
      return true;
    }
    const double deg2rad = M_PI / 180.0;
    const double scale = angles_in_degrees_ ? deg2rad : 1.0;
    jaka_msgs::Move srv;
    srv.request.pose = {static_cast<float>(wp.x_mm),
                        static_cast<float>(wp.y_mm),
                        static_cast<float>(wp.z_mm),
                        static_cast<float>(wp.rx * scale),
                        static_cast<float>(wp.ry * scale),
                        static_cast<float>(wp.rz * scale)};
    srv.request.mvvelo = static_cast<float>(linear_speed_mm_s_ * speed_scale_);
    srv.request.mvacc = static_cast<float>(linear_acc_mm_s2_ * speed_scale_);
    srv.request.mvtime = 0.0;
    srv.request.mvradii = 0.0;
    srv.request.coord_mode = coord_mode_;
    srv.request.index = static_cast<int32_t>(sequence_index);

    if (!linear_move_client_.call(srv))
    {
      ROS_ERROR("[CsvWaypoint] linear_move 通信失败，idx=%d 未发送", wp.idx);
      return false;
    }
    ROS_INFO("[CsvWaypoint] linear_move ret=%d, message=%s", srv.response.ret, srv.response.message.c_str());
    if (srv.response.ret != 0)
    {
      ROS_ERROR("[CsvWaypoint] linear_move 执行失败，ret=%d, message=%s", srv.response.ret, srv.response.message.c_str());
      return false;
    }
    return true;
  }

  bool waitRobotMotionDone(double timeout_sec)
  {
    const ros::Time start = ros::Time::now();
    ros::Rate rate(20.0);
    ros::Time stable_start;

    while (ros::ok())
    {
      ros::spinOnce();

      if ((ros::Time::now() - start).toSec() > timeout_sec)
      {
        ROS_WARN("[CsvWaypoint] 等待机器人停止运动超时（%.1f s）", timeout_sec);
        return false;
      }

      ros::Time last_motion;
      ros::Time last_stamp;
      {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        last_motion = last_motion_time_;
        last_stamp = last_joint_stamp_;
      }

      if (last_stamp.isZero())
      {
        ROS_WARN_THROTTLE(2.0, "[CsvWaypoint] 尚未收到 joint_state，继续等待...");
        rate.sleep();
        continue;
      }

      const double since_motion = (ros::Time::now() - last_motion).toSec();
      if (since_motion >= motion_stable_duration_sec_)
      {
        if (stable_start.isZero())
        {
          stable_start = ros::Time::now();
        }
        else if ((ros::Time::now() - stable_start).toSec() >= motion_stable_duration_sec_)
        {
          ROS_INFO("[CsvWaypoint] 检测到机器人已停止运动");
          return true;
        }
      }
      else
      {
        stable_start = ros::Time(0);
      }

      rate.sleep();
    }
    return false;
  }

  bool run()
  {
    waitForService();
    if (!loadWaypoints())
    {
      return false;
    }

    if (multi_arm_mode_)
    {
      return runMultiArm();
    }

    int sequence_index = 0;
    for (size_t i = 0; i < waypoints_.size() && ros::ok(); ++i)
    {
      const auto &wp = waypoints_[i];
      ROS_INFO("[CsvWaypoint] [%zu/%zu] idx=%d pos(mm)=[%.3f %.3f %.3f] rpy(%s)=[%.3f %.3f %.3f]",
               i + 1, waypoints_.size(), wp.idx, wp.x_mm, wp.y_mm, wp.z_mm,
               angles_in_degrees_ ? "deg" : "rad", wp.rx, wp.ry, wp.rz);

      if (!sendLinearTarget(wp, sequence_index))
      {
        return false;
      }

      if (do_motion_)
      {
        if (!waitRobotMotionDone(motion_done_timeout_sec_))
        {
          return false;
        }
      }

      if (dwell_sec_ > 0.0)
      {
        ROS_INFO("[CsvWaypoint] 到位后停留 %.2f 秒", dwell_sec_);
        ros::Duration(dwell_sec_).sleep();
      }
      ++sequence_index;
    }

    if (round_trip_ && waypoints_.size() > 1)
    {
      const int start_index = reverse_include_last_ ? static_cast<int>(waypoints_.size()) - 1
                                                    : static_cast<int>(waypoints_.size()) - 2;
      for (int i = start_index; i >= 0 && ros::ok(); --i)
      {
        const auto &wp = waypoints_[static_cast<size_t>(i)];
        ROS_INFO("[CsvWaypoint] [%d/%zu] idx=%d pos(mm)=[%.3f %.3f %.3f] rpy(%s)=[%.3f %.3f %.3f]",
                 sequence_index + 1, waypoints_.size(), wp.idx, wp.x_mm, wp.y_mm, wp.z_mm,
                 angles_in_degrees_ ? "deg" : "rad", wp.rx, wp.ry, wp.rz);

        if (!sendLinearTarget(wp, sequence_index))
        {
          return false;
        }

        if (do_motion_)
        {
          if (!waitRobotMotionDone(motion_done_timeout_sec_))
          {
            return false;
          }
        }

        if (dwell_sec_ > 0.0)
        {
          ROS_INFO("[CsvWaypoint] 到位后停留 %.2f 秒", dwell_sec_);
          ros::Duration(dwell_sec_).sleep();
        }
        ++sequence_index;
      }
    }

    ROS_INFO("[CsvWaypoint] 路点执行完成");
    return true;
  }

private:
  struct ArmContext
  {
    std::string name;
    bool enabled{false};
    std::string tcp_name;
    std::string waypoint_csv;
    std::string linear_move_service;
    std::string joint_state_topic;
    ros::ServiceClient move_client;
    ros::Subscriber joint_sub;
    std::vector<Waypoint> waypoints;
    std::mutex joint_mutex;
    std::vector<double> prev_joint_pos;
    bool has_prev_joint{false};
    ros::Time last_motion_time;
    ros::Time last_joint_stamp;
  };

  void loadArmsFromParams()
  {
    XmlRpc::XmlRpcValue arms_param;
    if (!pnh_.getParam("arms", arms_param) || arms_param.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      multi_arm_mode_ = false;
      return;
    }

    for (auto it = arms_param.begin(); it != arms_param.end(); ++it)
    {
      auto arm = std::make_shared<ArmContext>();
      arm->name = it->first;
      if (it->second.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        continue;
      }
      XmlRpc::XmlRpcValue cfg = it->second;
      if (cfg.hasMember("enabled"))
      {
        arm->enabled = static_cast<bool>(cfg["enabled"]);
      }
      if (cfg.hasMember("tcp_name"))
      {
        arm->tcp_name = static_cast<std::string>(cfg["tcp_name"]);
      }
      if (cfg.hasMember("waypoint_csv"))
      {
        arm->waypoint_csv = static_cast<std::string>(cfg["waypoint_csv"]);
      }
      if (cfg.hasMember("linear_move_service"))
      {
        arm->linear_move_service = static_cast<std::string>(cfg["linear_move_service"]);
      }
      if (cfg.hasMember("joint_state_topic"))
      {
        arm->joint_state_topic = static_cast<std::string>(cfg["joint_state_topic"]);
      }
      arms_.push_back(arm);
    }

    multi_arm_mode_ = std::any_of(arms_.begin(), arms_.end(), [](const std::shared_ptr<ArmContext> &arm) {
      return arm && arm->enabled;
    });
  }

  void setupArmInterfaces()
  {
    for (size_t i = 0; i < arms_.size(); ++i)
    {
      auto &arm = arms_[i];
      if (!arm || !arm->enabled)
      {
        continue;
      }
      arm->move_client = nh_.serviceClient<jaka_msgs::Move>(arm->linear_move_service);
      arm->joint_sub = nh_.subscribe<sensor_msgs::JointState>(
          arm->joint_state_topic, 50, boost::bind(&JakaCsvWaypointPlayer::jointStateCallbackMulti, this, _1, i));
    }
  }

  void jointStateCallbackMulti(const sensor_msgs::JointState::ConstPtr &msg, size_t arm_index)
  {
    if (arm_index >= arms_.size())
    {
      return;
    }
    auto &arm = arms_[arm_index];
    if (!arm || !arm->enabled)
    {
      return;
    }
    if (msg->position.size() < 6)
    {
      ROS_WARN_THROTTLE(5.0, "[CsvWaypoint] %s joint_state 位置元素不足6个，忽略该帧", arm->name.c_str());
      return;
    }

    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    std::lock_guard<std::mutex> lock(arm->joint_mutex);

    if (arm->has_prev_joint)
    {
      double max_diff = 0.0;
      const size_t n = std::min(arm->prev_joint_pos.size(), msg->position.size());
      for (size_t i = 0; i < n; ++i)
      {
        const double diff = std::abs(msg->position[i] - arm->prev_joint_pos[i]);
        max_diff = std::max(max_diff, diff);
      }
      if (max_diff > motion_joint_threshold_rad_)
      {
        arm->last_motion_time = stamp;
      }
    }
    else
    {
      arm->last_motion_time = stamp;
      arm->has_prev_joint = true;
    }

    arm->prev_joint_pos = msg->position;
    arm->last_joint_stamp = stamp;
  }

  bool loadWaypointsFromCsv(const std::string &path, std::vector<Waypoint> &out)
  {
    std::ifstream fin(path);
    if (!fin.is_open())
    {
      ROS_ERROR("[CsvWaypoint] 无法打开 CSV: %s", path.c_str());
      return false;
    }

    std::string line;
    bool header_checked = false;
    while (std::getline(fin, line))
    {
      line = trim(line);
      if (line.empty() || line[0] == '#')
      {
        continue;
      }

      std::stringstream ss(line);
      std::string item;
      std::vector<std::string> cols;
      while (std::getline(ss, item, ','))
      {
        cols.push_back(trim(item));
      }

      if (!header_checked)
      {
        header_checked = true;
        if (cols.empty() || !isNumeric(cols[0]))
        {
          continue;
        }
      }

      if (cols.size() != 7)
      {
        ROS_WARN("[CsvWaypoint] CSV 行格式错误（期望7列），已跳过: %s", line.c_str());
        continue;
      }

      Waypoint wp;
      try
      {
        wp.idx = static_cast<int>(std::stod(cols[0]));
        wp.x_mm = std::stod(cols[1]);
        wp.y_mm = std::stod(cols[2]);
        wp.z_mm = std::stod(cols[3]);
        wp.rx = std::stod(cols[4]);
        wp.ry = std::stod(cols[5]);
        wp.rz = std::stod(cols[6]);
      }
      catch (const std::exception &)
      {
        ROS_WARN("[CsvWaypoint] CSV 数值解析失败，跳过该行: %s", line.c_str());
        continue;
      }

      out.push_back(wp);
    }

    if (sort_by_idx_)
    {
      std::sort(out.begin(), out.end(), [](const Waypoint &a, const Waypoint &b) {
        return a.idx < b.idx;
      });
    }

    if (out.empty())
    {
      ROS_ERROR("[CsvWaypoint] CSV 未加载到有效路点: %s", path.c_str());
      return false;
    }
    return true;
  }

  bool loadMultiArmWaypoints()
  {
    for (auto &arm : arms_)
    {
      if (!arm || !arm->enabled)
      {
        continue;
      }
      if (arm->waypoint_csv.empty())
      {
        ROS_ERROR("[CsvWaypoint] %s 未配置 waypoint_csv", arm->name.c_str());
        return false;
      }
      if (!loadWaypointsFromCsv(arm->waypoint_csv, arm->waypoints))
      {
        ROS_ERROR("[CsvWaypoint] %s 路点加载失败", arm->name.c_str());
        return false;
      }
      ROS_INFO("[CsvWaypoint] %s 已加载 %zu 个路点，TCP=%s", arm->name.c_str(), arm->waypoints.size(),
               arm->tcp_name.c_str());
    }

    if (require_same_count_)
    {
      size_t count = 0;
      bool count_set = false;
      for (const auto &arm : arms_)
      {
        if (!arm || !arm->enabled)
        {
          continue;
        }
        if (!count_set)
        {
          count = arm->waypoints.size();
          count_set = true;
        }
        else if (arm->waypoints.size() != count)
        {
          ROS_ERROR("[CsvWaypoint] 各机械臂路点数量不一致");
          return false;
        }
      }
    }
    return true;
  }

  bool sendLinearTargetMulti(const std::shared_ptr<ArmContext> &arm, const Waypoint &wp, int sequence_index)
  {
    if (!use_driver_ || !do_motion_)
    {
      ROS_INFO("[CsvWaypoint] %s use_driver/do_motion=false，跳过运动发送", arm->name.c_str());
      return true;
    }
    const double deg2rad = M_PI / 180.0;
    const double scale = angles_in_degrees_ ? deg2rad : 1.0;
    jaka_msgs::Move srv;
    srv.request.pose = {static_cast<float>(wp.x_mm),
                        static_cast<float>(wp.y_mm),
                        static_cast<float>(wp.z_mm),
                        static_cast<float>(wp.rx * scale),
                        static_cast<float>(wp.ry * scale),
                        static_cast<float>(wp.rz * scale)};
    srv.request.mvvelo = static_cast<float>(linear_speed_mm_s_ * speed_scale_);
    srv.request.mvacc = static_cast<float>(linear_acc_mm_s2_ * speed_scale_);
    srv.request.mvtime = 0.0;
    srv.request.mvradii = 0.0;
    srv.request.coord_mode = coord_mode_;
    srv.request.index = static_cast<int32_t>(sequence_index);

    if (!arm->move_client.call(srv))
    {
      ROS_ERROR("[CsvWaypoint] %s linear_move 通信失败，idx=%d 未发送", arm->name.c_str(), wp.idx);
      return false;
    }
    ROS_INFO("[CsvWaypoint] %s linear_move ret=%d, message=%s", arm->name.c_str(), srv.response.ret,
             srv.response.message.c_str());
    if (srv.response.ret != 0)
    {
      ROS_ERROR("[CsvWaypoint] %s linear_move 执行失败，ret=%d, message=%s", arm->name.c_str(), srv.response.ret,
                srv.response.message.c_str());
      return false;
    }
    return true;
  }

  bool waitAllRobotsMotionDone(double timeout_sec)
  {
    const ros::Time start = ros::Time::now();
    ros::Rate rate(20.0);
    std::vector<ros::Time> stable_start(arms_.size());

    while (ros::ok())
    {
      ros::spinOnce();

      if ((ros::Time::now() - start).toSec() > timeout_sec)
      {
        ROS_WARN("[CsvWaypoint] 等待机器人停止运动超时（%.1f s）", timeout_sec);
        return false;
      }

      bool all_stable = true;
      for (size_t i = 0; i < arms_.size(); ++i)
      {
        auto &arm = arms_[i];
        if (!arm || !arm->enabled)
        {
          continue;
        }
        ros::Time last_motion;
        ros::Time last_stamp;
        {
          std::lock_guard<std::mutex> lock(arm->joint_mutex);
          last_motion = arm->last_motion_time;
          last_stamp = arm->last_joint_stamp;
        }

        if (last_stamp.isZero())
        {
          ROS_WARN_THROTTLE(2.0, "[CsvWaypoint] %s 尚未收到 joint_state，继续等待...", arm->name.c_str());
          all_stable = false;
          continue;
        }

        const double since_motion = (ros::Time::now() - last_motion).toSec();
        if (since_motion >= motion_stable_duration_sec_)
        {
          if (stable_start[i].isZero())
          {
            stable_start[i] = ros::Time::now();
            all_stable = false;
          }
          else if ((ros::Time::now() - stable_start[i]).toSec() < motion_stable_duration_sec_)
          {
            all_stable = false;
          }
        }
        else
        {
          stable_start[i] = ros::Time(0);
          all_stable = false;
        }
      }

      if (all_stable)
      {
        ROS_INFO("[CsvWaypoint] 检测到所有机械臂已停止运动");
        return true;
      }

      rate.sleep();
    }
    return false;
  }

  bool runMultiArm()
  {
    size_t count = 0;
    bool count_set = false;
    for (const auto &arm : arms_)
    {
      if (!arm || !arm->enabled)
      {
        continue;
      }
      if (!count_set)
      {
        count = arm->waypoints.size();
        count_set = true;
      }
      else if (!require_same_count_)
      {
        count = std::min(count, arm->waypoints.size());
      }
    }
    if (!count_set || count == 0)
    {
      ROS_ERROR("[CsvWaypoint] 没有启用的机械臂路点");
      return false;
    }

    std::vector<int> order;
    order.reserve(round_trip_ ? count * 2 : count);
    for (size_t i = 0; i < count; ++i)
    {
      order.push_back(static_cast<int>(i));
    }
    if (round_trip_ && count > 1)
    {
      const int start_index = reverse_include_last_ ? static_cast<int>(count) - 1
                                                    : static_cast<int>(count) - 2;
      for (int i = start_index; i >= 0; --i)
      {
        order.push_back(i);
      }
    }

    int sequence_index = 0;
    for (size_t step = 0; step < order.size() && ros::ok(); ++step)
    {
      const int idx = order[step];
      bool idx_mismatch = false;
      int ref_idx = 0;
      bool ref_set = false;
      for (const auto &arm : arms_)
      {
        if (!arm || !arm->enabled)
        {
          continue;
        }
        if (!ref_set)
        {
          ref_idx = arm->waypoints[static_cast<size_t>(idx)].idx;
          ref_set = true;
        }
        else if (sync_by_idx_ && arm->waypoints[static_cast<size_t>(idx)].idx != ref_idx)
        {
          idx_mismatch = true;
        }
      }
      if (idx_mismatch)
      {
        ROS_ERROR("[CsvWaypoint] 同步 idx 不一致，终止执行");
        return false;
      }

      ROS_INFO("[CsvWaypoint] step=%zu/%zu %s", step + 1, order.size(),
               step < count ? "forward" : "reverse");

      std::vector<std::thread> threads;
      std::vector<bool> results(arms_.size(), false);
      threads.reserve(arms_.size());
      for (size_t i = 0; i < arms_.size(); ++i)
      {
        auto &arm = arms_[i];
        if (!arm || !arm->enabled)
        {
          continue;
        }
        const Waypoint &wp = arm->waypoints[static_cast<size_t>(idx)];
        ROS_INFO("[CsvWaypoint] %s idx=%d pos(mm)=[%.3f %.3f %.3f] rpy(%s)=[%.3f %.3f %.3f]",
                 arm->name.c_str(), wp.idx, wp.x_mm, wp.y_mm, wp.z_mm,
                 angles_in_degrees_ ? "deg" : "rad", wp.rx, wp.ry, wp.rz);
        threads.emplace_back([this, &arm, &wp, &results, i, sequence_index]() {
          results[i] = sendLinearTargetMulti(arm, wp, sequence_index);
        });
      }
      for (auto &t : threads)
      {
        t.join();
      }
      for (size_t i = 0; i < arms_.size(); ++i)
      {
        if (!arms_[i] || !arms_[i]->enabled)
        {
          continue;
        }
        if (!results[i])
        {
          return false;
        }
      }

      if (do_motion_)
      {
        if (!waitAllRobotsMotionDone(motion_done_timeout_sec_))
        {
          return false;
        }
      }

      if (dwell_sec_ > 0.0)
      {
        ROS_INFO("[CsvWaypoint] 到位后停留 %.2f 秒", dwell_sec_);
        ros::Duration(dwell_sec_).sleep();
      }

      ++sequence_index;
    }
    ROS_INFO("[CsvWaypoint] 路点执行完成");
    return true;
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
  {
    if (msg->position.size() < 6)
    {
      ROS_WARN_THROTTLE(5.0, "[CsvWaypoint] joint_state 位置元素不足6个，忽略该帧");
      return;
    }

    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    std::lock_guard<std::mutex> lock(joint_mutex_);

    if (has_prev_joint_)
    {
      double max_diff = 0.0;
      const size_t n = std::min(prev_joint_pos_.size(), msg->position.size());
      for (size_t i = 0; i < n; ++i)
      {
        const double diff = std::abs(msg->position[i] - prev_joint_pos_[i]);
        max_diff = std::max(max_diff, diff);
      }
      if (max_diff > motion_joint_threshold_rad_)
      {
        last_motion_time_ = stamp;
      }
    }
    else
    {
      last_motion_time_ = stamp;
      has_prev_joint_ = true;
    }

    prev_joint_pos_ = msg->position;
    last_joint_stamp_ = stamp;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber joint_state_sub_;
  ros::ServiceClient linear_move_client_;
  bool multi_arm_mode_{false};

  std::string waypoint_csv_;
  std::string linear_move_service_;
  std::string joint_state_topic_;
  std::string tcp_name_;

  double speed_scale_{0.15};
  double linear_speed_mm_s_{80.0};
  double linear_acc_mm_s2_{200.0};
  double motion_done_timeout_sec_{120.0};
  double motion_stable_duration_sec_{0.5};
  double motion_joint_threshold_rad_{0.002};
  double dwell_sec_{2.0};
  bool angles_in_degrees_{true};
  int coord_mode_{0};
  bool sort_by_idx_{true};
  bool use_driver_{true};
  bool do_motion_{true};
  bool round_trip_{true};
  bool reverse_include_last_{false};
  bool require_same_count_{true};
  bool sync_by_idx_{true};

  std::mutex joint_mutex_;
  std::vector<double> prev_joint_pos_;
  bool has_prev_joint_{false};
  ros::Time last_motion_time_;
  ros::Time last_joint_stamp_;

  std::vector<Waypoint> waypoints_;
  std::vector<std::shared_ptr<ArmContext>> arms_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jaka_csv_waypoint_player_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  JakaCsvWaypointPlayer node(nh, pnh);
  const bool ok = node.run();
  return ok ? 0 : 1;
}
