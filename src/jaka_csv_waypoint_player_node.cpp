#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <jaka_msgs/Move.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

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
    pnh_.param<std::string>("tcp_name", tcp_name_, std::string("megnetic_1"));

    if (speed_scale_ > 0.15)
    {
      ROS_WARN("[CsvWaypoint] speed_scale=%.3f 超过 0.15，已限制为 0.15", speed_scale_);
      speed_scale_ = 0.15;
    }

    joint_state_sub_ = nh_.subscribe(joint_state_topic_, 50, &JakaCsvWaypointPlayer::jointStateCallback, this);
    linear_move_client_ = nh_.serviceClient<jaka_msgs::Move>(linear_move_service_);
  }

  bool loadWaypoints()
  {
    std::ifstream fin(waypoint_csv_);
    if (!fin.is_open())
    {
      ROS_ERROR("[CsvWaypoint] 无法打开 CSV: %s", waypoint_csv_.c_str());
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

      waypoints_.push_back(wp);
    }

    if (sort_by_idx_)
    {
      std::sort(waypoints_.begin(), waypoints_.end(), [](const Waypoint &a, const Waypoint &b) {
        return a.idx < b.idx;
      });
    }

    if (waypoints_.empty())
    {
      ROS_ERROR("[CsvWaypoint] CSV 未加载到有效路点");
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

  bool sendLinearTarget(const Waypoint &wp)
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
    srv.request.index = static_cast<int32_t>(target_index_);

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

    int sequence_index = 0;
    for (size_t i = 0; i < waypoints_.size() && ros::ok(); ++i)
    {
      const auto &wp = waypoints_[i];
      ROS_INFO("[CsvWaypoint] [%zu/%zu] idx=%d pos(mm)=[%.3f %.3f %.3f] rpy(%s)=[%.3f %.3f %.3f]",
               i + 1, waypoints_.size(), wp.idx, wp.x_mm, wp.y_mm, wp.z_mm,
               angles_in_degrees_ ? "deg" : "rad", wp.rx, wp.ry, wp.rz);

      target_index_ = sequence_index;
      if (!sendLinearTarget(wp))
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

        target_index_ = sequence_index;
        if (!sendLinearTarget(wp))
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

  std::mutex joint_mutex_;
  std::vector<double> prev_joint_pos_;
  bool has_prev_joint_{false};
  ros::Time last_motion_time_;
  ros::Time last_joint_stamp_;
  int target_index_{0};

  std::vector<Waypoint> waypoints_;
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
