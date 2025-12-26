// cube_aruco_detector_node.cpp
// 集成到 jaka_close_contro 包
// 支持多尺寸: ID=0 (80mm world tag), ID=1-4 (40mm tool tags)

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <map>
#include <set>
#include <vector>
#include <sstream>
#include <xmlrpcpp/XmlRpcValue.h>

class CubeArucoDetector {
public:
  CubeArucoDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), have_cam_info_(false) {
    // 读取参数
    pnh_.param<std::string>("image_topic", image_topic_, "/zed2i/zed_node/left/image_rect_color");
    pnh_.param<std::string>("camera_info_topic", camera_info_topic_, "/zed2i/zed_node/left/camera_info");
    pnh_.param<std::string>("fiducial_topic", fiducial_topic_, "/fiducial_transforms");
    pnh_.param("default_marker_size", default_marker_size_, 0.04); // 默认40mm
    pnh_.param<std::string>("dictionary", dictionary_name_, "DICT_4X4_50");
    pnh_.param<std::string>("tool_dictionary", tool_dictionary_name_, "DICT_6X6_1000");
    pnh_.param("tool_marker_length_m", tool_marker_length_m_, 0.0425);
    pnh_.param("tool_marker_separation_m", tool_marker_separation_m_, 0.005);
    pnh_.param("tool_board_active_size_m", tool_board_active_size_m_, -1.0);
    pnh_.param("min_gridboard_markers", min_gridboard_markers_, 2);
    pnh_.param("robot_stride", robot_stride_, 100);
    pnh_.param("id_step", id_step_, 10);
    pnh_.param("face_id_base", face_id_base_, 10);
    pnh_.param("num_faces", num_faces_, 4);
    pnh_.param("markers_per_face", markers_per_face_, 4);
    loadRobotIds();

    pnh_.param("world_board_enable", world_board_enable_, true);
    pnh_.param("world_board_output_id", world_board_output_id_, 500);
    pnh_.param("world_board_marker_length_m", world_board_marker_length_m_, 0.0525);
    pnh_.param("world_board_marker_separation_m", world_board_marker_separation_m_, 0.005);
    pnh_.param("world_board_min_markers", world_board_min_markers_, 2);
    pnh_.param("enable_single_world_markers", enable_single_world_markers_, false);

    // 读取多尺寸配置
    XmlRpc::XmlRpcValue marker_sizes_param;
    if (pnh_.hasParam("marker_sizes")) {
      pnh_.getParam("marker_sizes", marker_sizes_param);
      if (marker_sizes_param.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        for (XmlRpc::XmlRpcValue::iterator it = marker_sizes_param.begin();
             it != marker_sizes_param.end(); ++it) {
          try {
            int id = std::stoi(it->first);

            double size = 0.0;
            if (it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
              size = static_cast<double>(it->second);
            } else if (it->second.getType() == XmlRpc::XmlRpcValue::TypeInt) {
              size = static_cast<int>(it->second);
            } else {
              ROS_ERROR("[Detector] marker_sizes[%s] has non-numeric type, skip", it->first.c_str());
              continue;
            }

            marker_sizes_[id] = size;
            ROS_INFO("[Detector] Marker ID=%d size=%.3f m", id, size);
          } catch (const std::exception& e) {
            ROS_ERROR("[Detector] Failed to parse marker_sizes entry '%s': %s", it->first.c_str(), e.what());
            continue;
          }
        }
      }
    }

    ROS_INFO_STREAM("[Detector] image_topic: " << image_topic_);
    ROS_INFO_STREAM("[Detector] camera_info_topic: " << camera_info_topic_);
    ROS_INFO_STREAM("[Detector] fiducial_topic: " << fiducial_topic_);

    // ArUco 配置
    dictionary_world_ = cv::aruco::getPredefinedDictionary(parseDictionary(dictionary_name_));
    dictionary_tool_ = cv::aruco::getPredefinedDictionary(parseDictionary(tool_dictionary_name_));
    detector_params_ = cv::aruco::DetectorParameters::create();
    ROS_INFO("[Detector] 使用 world 字典: %s", dictionary_name_.c_str());
    ROS_INFO("[Detector] 使用 tool 字典: %s", tool_dictionary_name_.c_str());

    if (tool_board_active_size_m_ <= 0.0) {
      tool_board_active_size_m_ = 2.0 * tool_marker_length_m_ + tool_marker_separation_m_;
    }
    if (tool_board_active_size_m_ <= 0.0) {
      ROS_WARN("[Detector] tool_board_active_size_m 非法，回退为 0.09");
      tool_board_active_size_m_ = 0.09;
    }

    loadToolGridboards();
    loadWorldGridboard();

    // 订阅与发布
    caminfo_sub_ = nh_.subscribe(camera_info_topic_, 1, &CubeArucoDetector::cameraInfoCallback, this);
    image_sub_ = nh_.subscribe(image_topic_, 1, &CubeArucoDetector::imageCallback, this);
    fid_pub_ = nh_.advertise<fiducial_msgs::FiducialTransformArray>(fiducial_topic_, 1);
  }

private:
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
    if (have_cam_info_) return;

    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) 
      camera_matrix_.at<double>(i/3, i%3) = msg->K[i];
    
    dist_coeffs_ = cv::Mat(msg->D.size(), 1, CV_64F);
    for (size_t i = 0; i < msg->D.size(); ++i)
      dist_coeffs_.at<double>(i, 0) = msg->D[i];
    
    cam_frame_id_ = msg->header.frame_id;
    have_cam_info_ = true;
    ROS_INFO("[Detector] Got camera info. Frame ID: %s", cam_frame_id_.c_str());
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (!have_cam_info_) {
      ROS_WARN_THROTTLE(1.0, "[Detector] Waiting for camera info...");
      return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception& e) {
      ROS_ERROR("[Detector] cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
    if (gray.empty()) {
      ROS_WARN_THROTTLE(1.0, "[Detector] Empty gray image");
      return;
    }

    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> world_corners;
    std::vector<int> world_ids;
    std::vector<std::vector<cv::Point2f>> tool_corners;
    std::vector<int> tool_ids;
    if (enable_single_world_markers_) {
      cv::aruco::detectMarkers(gray, dictionary_world_, world_corners, world_ids, detector_params_);
    }
    cv::aruco::detectMarkers(gray, dictionary_tool_, tool_corners, tool_ids, detector_params_);

    fiducial_msgs::FiducialTransformArray out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = cam_frame_id_;
    out.image_seq = msg->header.seq;

    if (world_ids.empty() && tool_ids.empty()) {
      ROS_DEBUG_THROTTLE(1.0, "[Detector] No markers detected");
      fid_pub_.publish(out);
      return;
    }

    if (enable_single_world_markers_) {
      for (size_t i = 0; i < world_ids.size(); ++i) {
        int id = world_ids[i];
        if (tool_marker_ids_.count(id) > 0) {
          continue;
        }
        double marker_size = default_marker_size_;

        auto size_it = marker_sizes_.find(id);
        if (size_it != marker_sizes_.end()) {
          marker_size = size_it->second;
        }

        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(
          std::vector<std::vector<cv::Point2f>>{world_corners[i]},
          marker_size,
          camera_matrix_,
          dist_coeffs_,
          rvecs,
          tvecs
        );

        if (rvecs.empty() || tvecs.empty()) continue;

        cv::Mat R_cv;
        cv::Rodrigues(rvecs[0], R_cv);
        Eigen::Matrix3d R;
        for (int r = 0; r < 3; ++r)
          for (int c = 0; c < 3; ++c)
            R(r, c) = R_cv.at<double>(r, c);

        Eigen::Quaterniond q(R);

        fiducial_msgs::FiducialTransform ft;
        ft.fiducial_id = id;
        ft.fiducial_area = static_cast<float>(marker_size * marker_size * 1e6);
        ft.image_error = 0.0f;

        ft.transform.translation.x = tvecs[0][0];
        ft.transform.translation.y = tvecs[0][1];
        ft.transform.translation.z = tvecs[0][2];
        ft.transform.rotation.x = q.x();
        ft.transform.rotation.y = q.y();
        ft.transform.rotation.z = q.z();
        ft.transform.rotation.w = q.w();

        out.transforms.push_back(ft);
      }
    }

    for (const auto& entry : tool_boards_) {
      int face_id = entry.first;
      const auto& board = entry.second;
      cv::Vec3d rvec, tvec;
      int used = cv::aruco::estimatePoseBoard(tool_corners, tool_ids, board, camera_matrix_, dist_coeffs_, rvec, tvec);
      if (used < min_gridboard_markers_) {
        continue;
      }

      cv::Mat R_cv;
      cv::Rodrigues(rvec, R_cv);
      Eigen::Matrix3d R;
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
          R(r, c) = R_cv.at<double>(r, c);

      Eigen::Quaterniond q(R);
      float reproj_rms = computeBoardReprojError(board, tool_corners, tool_ids, rvec, tvec, camera_matrix_, dist_coeffs_);

      fiducial_msgs::FiducialTransform ft;
      ft.fiducial_id = face_id;
      ft.fiducial_area = static_cast<float>(tool_board_active_size_m_ * tool_board_active_size_m_ * 1e6);
      ft.image_error = 0.0f;
      ft.object_error = reproj_rms;

      ft.transform.translation.x = tvec[0];
      ft.transform.translation.y = tvec[1];
      ft.transform.translation.z = tvec[2];
      ft.transform.rotation.x = q.x();
      ft.transform.rotation.y = q.y();
      ft.transform.rotation.z = q.z();
      ft.transform.rotation.w = q.w();

      out.transforms.push_back(ft);
    }

    if (world_board_enable_ && world_board_) {
      cv::Vec3d rvec, tvec;
      int used = cv::aruco::estimatePoseBoard(tool_corners, tool_ids, world_board_,
                                              camera_matrix_, dist_coeffs_, rvec, tvec);
      if (used >= world_board_min_markers_) {
        cv::Mat R_cv;
        cv::Rodrigues(rvec, R_cv);
        Eigen::Matrix3d R;
        for (int r = 0; r < 3; ++r)
          for (int c = 0; c < 3; ++c)
            R(r, c) = R_cv.at<double>(r, c);

        Eigen::Vector3d t(tvec[0], tvec[1], tvec[2]);
        Eigen::Vector3d offset(world_board_center_offset_.x, world_board_center_offset_.y, 0.0);
        Eigen::Vector3d t_center = t + R * offset;
        Eigen::Quaterniond q(R);
        float reproj_rms = computeBoardReprojError(world_board_, tool_corners, tool_ids, rvec, tvec,
                                                   camera_matrix_, dist_coeffs_);

        fiducial_msgs::FiducialTransform ft;
        ft.fiducial_id = world_board_output_id_;
        ft.fiducial_area = static_cast<float>(world_board_active_size_m_ * world_board_active_size_m_ * 1e6);
        ft.image_error = 0.0f;
        ft.object_error = reproj_rms;

        ft.transform.translation.x = t_center.x();
        ft.transform.translation.y = t_center.y();
        ft.transform.translation.z = t_center.z();
        ft.transform.rotation.x = q.x();
        ft.transform.rotation.y = q.y();
        ft.transform.rotation.z = q.z();
        ft.transform.rotation.w = q.w();

        out.transforms.push_back(ft);
      }
    }

    fid_pub_.publish(out);
    ROS_DEBUG_THROTTLE(1.0, "[Detector] Published %zu transforms (world=%zu, tool=%zu)",
                       out.transforms.size(), world_ids.size(), tool_ids.size());

  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber image_sub_, caminfo_sub_;
  ros::Publisher fid_pub_;

  cv::Mat camera_matrix_, dist_coeffs_;
  bool have_cam_info_;
  std::string cam_frame_id_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_world_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_tool_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  std::string dictionary_name_;
  std::string tool_dictionary_name_;

  double default_marker_size_;
  std::map<int, double> marker_sizes_;

  std::string image_topic_, camera_info_topic_, fiducial_topic_;

  double tool_marker_length_m_;
  double tool_marker_separation_m_;
  double tool_board_active_size_m_;
  int min_gridboard_markers_;
  int robot_stride_{100};
  int id_step_{10};
  int face_id_base_{10};
  int num_faces_{4};
  int markers_per_face_{4};
  std::vector<int> robot_ids_param_;

  bool world_board_enable_ = true;
  int world_board_output_id_ = 500;
  double world_board_marker_length_m_ = 0.0525;
  double world_board_marker_separation_m_ = 0.005;
  int world_board_min_markers_ = 2;
  bool enable_single_world_markers_ = false;

  std::map<int, cv::Ptr<cv::aruco::GridBoard>> tool_boards_;
  std::set<int> tool_marker_ids_;

  cv::Ptr<cv::aruco::GridBoard> world_board_;
  std::vector<int> world_board_marker_ids_;
  double world_board_active_size_m_ = 0.0;
  cv::Point2f world_board_center_offset_{0.0f, 0.0f};

  static cv::aruco::PREDEFINED_DICTIONARY_NAME parseDictionary(const std::string& name) {
    if (name == "DICT_4X4_50") return cv::aruco::DICT_4X4_50;
    if (name == "DICT_4X4_100") return cv::aruco::DICT_4X4_100;
    if (name == "DICT_6X6_1000") return cv::aruco::DICT_6X6_1000;
    ROS_WARN("[Detector] 未识别字典 '%s'，回退 DICT_4X4_50", name.c_str());
    return cv::aruco::DICT_4X4_50;
  }

  void loadToolGridboards() {
    tool_boards_.clear();
    tool_marker_ids_.clear();

    std::map<int, std::vector<int>> mappings;
    bool has_param = loadToolGridboardsParam(mappings);
    if (!has_param) {
      mappings = generateDefaultToolBoards();
    }

    for (const auto& kv : mappings) {
      int face_id = kv.first;
      const auto& ids = kv.second;
      if (ids.size() != 4) {
        ROS_WARN("[Detector] face_id=%d 的 GridBoard 内部IDs数量=%zu，期望4个，跳过",
                 face_id, ids.size());
        continue;
      }
      cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(
        2, 2, static_cast<float>(tool_marker_length_m_), static_cast<float>(tool_marker_separation_m_),
        dictionary_tool_, 0);
      board->ids = ids;
      centerBoard(*board, tool_board_active_size_m_);
      tool_boards_[face_id] = board;
      for (int id : ids) {
        tool_marker_ids_.insert(id);
      }
      ROS_INFO("[Detector] GridBoard face_id=%d 内部IDs=%zu", face_id, ids.size());
    }
  }

  void loadWorldGridboard() {
    world_board_.release();
    world_board_marker_ids_.clear();

    XmlRpc::XmlRpcValue param;
    if (pnh_.getParam("world_board_marker_ids", param) && param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int i = 0; i < param.size(); ++i) {
        if (param[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
          world_board_marker_ids_.push_back(static_cast<int>(param[i]));
        }
      }
    }
    if (world_board_marker_ids_.empty()) {
      world_board_marker_ids_ = {500, 501, 502, 503};
    }

    world_board_active_size_m_ = 2.0 * world_board_marker_length_m_ + world_board_marker_separation_m_;
    world_board_center_offset_ = cv::Point2f(static_cast<float>(world_board_active_size_m_ * 0.5),
                                             static_cast<float>(world_board_active_size_m_ * 0.5));

    if (world_board_marker_ids_.size() != 4) {
      ROS_WARN("[Detector] 世界板 marker IDs 数量=%zu，期望 4 个，将禁用世界板输出", world_board_marker_ids_.size());
      return;
    }

    world_board_ = cv::aruco::GridBoard::create(
      2, 2, static_cast<float>(world_board_marker_length_m_),
      static_cast<float>(world_board_marker_separation_m_),
      dictionary_tool_, 0);
    world_board_->ids = world_board_marker_ids_;
  }

  bool loadToolGridboardsParam(std::map<int, std::vector<int>>& mappings) {
    XmlRpc::XmlRpcValue param;
    if (!pnh_.getParam("tool_gridboards", param)) {
      return false;
    }
    if (param.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_WARN("[Detector] tool_gridboards 不是字典类型，忽略");
      return false;
    }
    for (XmlRpc::XmlRpcValue::iterator it = param.begin(); it != param.end(); ++it) {
      int face_id = 0;
      try {
        face_id = std::stoi(it->first);
      } catch (const std::exception& e) {
        ROS_WARN("[Detector] tool_gridboards 键解析失败: %s", it->first.c_str());
        continue;
      }
      if (it->second.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_WARN("[Detector] tool_gridboards[%s] 不是数组", it->first.c_str());
        continue;
      }
      std::vector<int> ids;
      for (int i = 0; i < it->second.size(); ++i) {
        if (it->second[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
          ids.push_back(static_cast<int>(it->second[i]));
        } else {
          ROS_WARN("[Detector] tool_gridboards[%s] 存在非整型元素", it->first.c_str());
        }
      }
      if (ids.empty()) {
        continue;
      }
      mappings[face_id] = ids;
    }
    return !mappings.empty();
  }

  void loadRobotIds() {
    robot_ids_param_.clear();
    XmlRpc::XmlRpcValue param;
    if (pnh_.hasParam("robot_ids") && pnh_.getParam("robot_ids", param)) {
      if (param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < param.size(); ++i) {
          if (param[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
            robot_ids_param_.push_back(static_cast<int>(param[i]));
          } else if (param[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
            try {
              robot_ids_param_.push_back(std::stoi(static_cast<std::string>(param[i])));
            } catch (...) {
              ROS_WARN("[Detector] robot_ids 元素解析失败，已跳过");
            }
          }
        }
      } else if (param.getType() == XmlRpc::XmlRpcValue::TypeString) {
        std::string ids_str = static_cast<std::string>(param);
        std::istringstream iss(ids_str);
        std::string token;
        while (iss >> token) {
          try {
            robot_ids_param_.push_back(std::stoi(token));
          } catch (...) {
            ROS_WARN("[Detector] robot_ids 字符串解析失败: %s", token.c_str());
          }
        }
      }
    }

    if (robot_ids_param_.empty()) {
      robot_ids_param_.push_back(1);
    }
  }

  std::map<int, std::vector<int>> generateDefaultToolBoards() {
    std::map<int, std::vector<int>> mappings;
    std::vector<int> robots;
    robots.reserve(robot_ids_param_.size());
    for (const auto &v : robot_ids_param_) {
      robots.push_back(v);
    }
    if (robots.empty()) {
      robots.push_back(1);
    }

    for (int rid : robots) {
      int robot_off = (rid - 1) * robot_stride_;
      for (int k = 0; k < num_faces_; ++k) {
        int face_id = face_id_base_ + k + robot_off;
        int marker_base = k * id_step_ + robot_off;
        std::vector<int> ids;
        for (int m = 0; m < markers_per_face_; ++m) {
          ids.push_back(marker_base + m);
        }
        mappings[face_id] = ids;
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < ids.size(); ++i) {
          oss << ids[i];
          if (i + 1 < ids.size()) oss << ",";
        }
        oss << "]";
        ROS_INFO("[Detector] robot=%d face_id=%d ids=%s", rid, face_id, oss.str().c_str());
      }
    }
    return mappings;
  }

  static void centerBoard(cv::aruco::GridBoard& board, double active_size) {
    const float shift = static_cast<float>(active_size * 0.5);
    for (auto& marker_pts : board.objPoints) {
      for (auto& pt : marker_pts) {
        pt.x -= shift;
        pt.y -= shift;
      }
    }
  }

  static float computeBoardReprojError(const cv::Ptr<cv::aruco::GridBoard>& board,
                                       const std::vector<std::vector<cv::Point2f>>& corners,
                                       const std::vector<int>& ids,
                                       const cv::Vec3d& rvec,
                                       const cv::Vec3d& tvec,
                                       const cv::Mat& camera_matrix,
                                       const cv::Mat& dist_coeffs) {
    double total_sq = 0.0;
    int total_points = 0;
    for (size_t i = 0; i < ids.size(); ++i) {
      int id = ids[i];
      auto it = std::find(board->ids.begin(), board->ids.end(), id);
      if (it == board->ids.end()) {
        continue;
      }
      size_t idx = static_cast<size_t>(std::distance(board->ids.begin(), it));
      std::vector<cv::Point2f> proj;
      cv::projectPoints(board->objPoints[idx], rvec, tvec, camera_matrix, dist_coeffs, proj);
      if (proj.size() != corners[i].size()) {
        continue;
      }
      for (size_t k = 0; k < proj.size(); ++k) {
        cv::Point2f diff = proj[k] - corners[i][k];
        total_sq += diff.dot(diff);
        total_points++;
      }
    }
    if (total_points == 0) {
      return 0.0f;
    }
    return static_cast<float>(std::sqrt(total_sq / total_points));
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "cube_aruco_detector_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    CubeArucoDetector detector(nh, pnh);
    ros::spin();
    return 0;
  } catch (const std::exception& e) {
    ROS_FATAL("Detector failed: %s", e.what());
    return 1;
  }
}
