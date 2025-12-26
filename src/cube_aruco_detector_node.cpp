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
#include <map>
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
    pnh_.param<std::string>("dict_name", dict_name_, "DICT_4X4_50");
    pnh_.param("default_marker_size", default_marker_size_, 0.04); // 默认40mm
    pnh_.param("marker_separation", marker_separation_, 0.01);
    pnh_.param("marker_length", marker_length_, default_marker_size_);
    pnh_.param("world_marker_length", world_marker_length_, 0.08);
    pnh_.param("world_marker_separation", world_marker_separation_, 0.02);
    pnh_.param("world_board_id", world_board_id_, 500);
    pnh_.param("robot_stride", robot_stride_, 100);
    pnh_.param("id_step", id_step_, 10);
    pnh_.param("face_id_base", face_id_base_, 10);
    pnh_.param("face_count", face_count_, 4);
    loadRobotIds();

    // 读取多尺寸配置
    XmlRpc::XmlRpcValue marker_sizes_param;
    if (pnh_.hasParam("marker_sizes")) {
      pnh_.getParam("marker_sizes", marker_sizes_param);
      if (marker_sizes_param.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        for (XmlRpc::XmlRpcValue::iterator it = marker_sizes_param.begin();
             it != marker_sizes_param.end(); ++it) {
          int id = std::stoi(it->first);
          double size = static_cast<double>(it->second);
          marker_sizes_[id] = size;
          ROS_INFO("[Detector] Marker ID=%d size=%.3f m", id, size);
        }
      }
    }

    ROS_INFO_STREAM("[Detector] image_topic: " << image_topic_);
    ROS_INFO_STREAM("[Detector] camera_info_topic: " << camera_info_topic_);
    ROS_INFO_STREAM("[Detector] fiducial_topic: " << fiducial_topic_);

    // ArUco 配置
    dictionary_ = cv::aruco::getPredefinedDictionary(dictionaryFromName(dict_name_));
    detector_params_ = cv::aruco::DetectorParameters::create();
    ROS_INFO_STREAM("[Detector] Using ArUco dictionary " << dict_name_);

    // 订阅与发布
    caminfo_sub_ = nh_.subscribe(camera_info_topic_, 1, &CubeArucoDetector::cameraInfoCallback, this);
    image_sub_ = nh_.subscribe(image_topic_, 1, &CubeArucoDetector::imageCallback, this);
    fid_pub_ = nh_.advertise<fiducial_msgs::FiducialTransformArray>(fiducial_topic_, 1);

    if (!loadCustomBoards()) {
      generateDefaultBoards();
    }
    buildWorldBoard();
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
    cv::aruco::detectMarkers(gray, dictionary_, corners, ids, detector_params_);

    fiducial_msgs::FiducialTransformArray out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = cam_frame_id_;
    out.image_seq = msg->header.seq;

    // 单独的 marker 位姿（保留原有输出，便于调试）
    if (!ids.empty()) {
      for (size_t i = 0; i < ids.size(); ++i) {
        int id = ids[i];
        double marker_size = markerSizeFor(id);

        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(
          std::vector<std::vector<cv::Point2f>>{corners[i]},
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

    // 计算各个面的 GridBoard 位姿，输出 fiducial_id=face_id
    for (const auto& board_def : board_defs_) {
      fiducial_msgs::FiducialTransform ft;
      if (estimateBoard(board_def.board, corners, ids, ft)) {
        ft.fiducial_id = board_def.face_id;
        ft.fiducial_area = static_cast<float>(board_def.marker_length * board_def.marker_length * 1e6 * 4.0);
        out.transforms.push_back(ft);
      }
    }

    // 世界板中心（2x2 GridBoard，原点在几何中心）
    if (world_board_) {
      fiducial_msgs::FiducialTransform ft;
      if (estimateBoard(world_board_, corners, ids, ft)) {
        ft.fiducial_id = world_board_id_;
        ft.fiducial_area = static_cast<float>(world_marker_length_ * world_marker_length_ * 1e6 * 4.0);
        out.transforms.push_back(ft);
      }
    }

    fid_pub_.publish(out);
    ROS_DEBUG_THROTTLE(1.0, "[Detector] Published %zu fiducials (markers + faces)", out.transforms.size());

    if (!ids.empty()) {
      std::stringstream ss;
      ss << "Detected markers: ";
      for (size_t i = 0; i < ids.size(); ++i) {
        ss << "ID=" << ids[i];
        if (i < ids.size()-1) ss << ", ";
      }
      ROS_INFO("[Detector] %s", ss.str().c_str());
    }

  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber image_sub_, caminfo_sub_;
  ros::Publisher fid_pub_;

  cv::Mat camera_matrix_, dist_coeffs_;
  bool have_cam_info_;
  std::string cam_frame_id_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

  double default_marker_size_;
  std::map<int, double> marker_sizes_;
  double marker_separation_{0.01};
  double marker_length_{0.04};
  double world_marker_length_{0.08};
  double world_marker_separation_{0.02};
  int world_board_id_{500};
  int robot_stride_{100};
  int id_step_{10};
  int face_id_base_{10};
  int face_count_{4};
  std::vector<int> robot_ids_;

  struct BoardDefinition {
    int face_id;
    cv::Ptr<cv::aruco::GridBoard> board;
    double marker_length;
  };

  std::vector<BoardDefinition> board_defs_;
  cv::Ptr<cv::aruco::GridBoard> world_board_;

  std::string image_topic_, camera_info_topic_, fiducial_topic_;
  std::string dict_name_;

  void loadRobotIds() {
    XmlRpc::XmlRpcValue rid_param;
    if (pnh_.hasParam("robot_ids") && pnh_.getParam("robot_ids", rid_param)) {
      if (rid_param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < rid_param.size(); ++i) {
          if (rid_param[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
            robot_ids_.push_back(static_cast<int>(rid_param[i]));
          } else if (rid_param[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
            try {
              robot_ids_.push_back(std::stoi(static_cast<std::string>(rid_param[i])));
            } catch (...) {
              ROS_WARN("[Detector] Failed to parse robot_id");
            }
          }
        }
      }
    }
    if (robot_ids_.empty()) {
      robot_ids_.push_back(1);
    }
  }

  bool loadCustomBoards() {
    XmlRpc::XmlRpcValue boards_param;
    if (!pnh_.hasParam("tool_gridboards") || !pnh_.getParam("tool_gridboards", boards_param)) {
      return false;
    }

    if (boards_param.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_WARN("[Detector] tool_gridboards is not an array, fallback to auto generation");
      return false;
    }

    board_defs_.clear();
    for (int i = 0; i < boards_param.size(); ++i) {
      auto entry = boards_param[i];
      if (entry.getType() != XmlRpc::XmlRpcValue::TypeStruct) continue;
      if (!entry.hasMember("face_id") || !entry.hasMember("ids")) continue;
      int face_id = static_cast<int>(entry["face_id"]);

      int x_count = entry.hasMember("x_markers") ? static_cast<int>(entry["x_markers"]) : 2;
      int y_count = entry.hasMember("y_markers") ? static_cast<int>(entry["y_markers"]) : 2;
      double m_len = entry.hasMember("marker_length") ? static_cast<double>(entry["marker_length"]) : marker_length_;
      double m_sep = entry.hasMember("marker_separation") ? static_cast<double>(entry["marker_separation"]) : marker_separation_;

      std::vector<int> ids_raw;
      XmlRpc::XmlRpcValue ids_param = entry["ids"];
      if (ids_param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int j = 0; j < ids_param.size(); ++j) {
          if (ids_param[j].getType() == XmlRpc::XmlRpcValue::TypeInt) {
            ids_raw.push_back(static_cast<int>(ids_param[j]));
          } else if (ids_param[j].getType() == XmlRpc::XmlRpcValue::TypeString) {
            try { ids_raw.push_back(std::stoi(static_cast<std::string>(ids_param[j]))); } catch (...) {}
          }
        }
      }
      if ((int)ids_raw.size() != x_count * y_count) {
        ROS_WARN("[Detector] tool_gridboards entry %d has mismatched ids, skipping", i);
        continue;
      }

      cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(x_count, y_count, m_len, m_sep, dictionary_);
      board->ids = ids_raw;
      board_defs_.push_back({face_id, board, m_len});
      ROS_INFO("[Detector] Loaded custom board face_id=%d", face_id);
    }

    return !board_defs_.empty();
  }

  void generateDefaultBoards() {
    board_defs_.clear();
    for (int rid : robot_ids_) {
      int robot_off = (rid - 1) * robot_stride_;
      for (int face_idx = 0; face_idx < face_count_; ++face_idx) {
        int marker_base = face_idx * id_step_ + robot_off;
        std::vector<int> ids{
          marker_base + 0,
          marker_base + 1,
          marker_base + 2,
          marker_base + 3
        };
        cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(
          2, 2, marker_length_, marker_separation_, dictionary_);
        board->ids = ids;
        int face_id = face_id_base_ + face_idx + robot_off;
        board_defs_.push_back({face_id, board, marker_length_});
        ROS_INFO("[Detector] Auto board rid=%d face=%d ids=[%d,%d,%d,%d] face_id=%d",
                 rid, face_idx, ids[0], ids[1], ids[2], ids[3], face_id);
      }
    }
  }

  void buildWorldBoard() {
    std::vector<int> ids{500, 501, 502, 503};
    world_board_ = cv::aruco::GridBoard::create(2, 2, world_marker_length_, world_marker_separation_, dictionary_);
    world_board_->ids = ids;
    for (int wid : ids) {
      marker_sizes_[wid] = world_marker_length_;
    }
    ROS_INFO("[Detector] World board ready with ids 500..503, output id=%d", world_board_id_);
  }

  double markerSizeFor(int id) const {
    auto it = marker_sizes_.find(id);
    if (it != marker_sizes_.end()) return it->second;
    return default_marker_size_;
  }

  bool estimateBoard(const cv::Ptr<cv::aruco::GridBoard>& board,
                     const std::vector<std::vector<cv::Point2f>>& corners,
                     const std::vector<int>& ids,
                     fiducial_msgs::FiducialTransform& out) {
    cv::Vec3d rvec, tvec;
    int valid = cv::aruco::estimatePoseBoard(corners, ids, board, camera_matrix_, dist_coeffs_, rvec, tvec);
    if (valid <= 0) return false;

    cv::Mat R_cv;
    cv::Rodrigues(rvec, R_cv);
    Eigen::Matrix3d R;
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        R(r, c) = R_cv.at<double>(r, c);
    Eigen::Quaterniond q(R);

    out.transform.translation.x = tvec[0];
    out.transform.translation.y = tvec[1];
    out.transform.translation.z = tvec[2];
    out.transform.rotation.x = q.x();
    out.transform.rotation.y = q.y();
    out.transform.rotation.z = q.z();
    out.transform.rotation.w = q.w();
    out.image_error = 0.0f;
    return true;
  }

  int dictionaryFromName(const std::string& name) {
    if (name == "DICT_4X4_50") return cv::aruco::DICT_4X4_50;
    if (name == "DICT_5X5_50") return cv::aruco::DICT_5X5_50;
    if (name == "DICT_6X6_50") return cv::aruco::DICT_6X6_50;
    ROS_WARN("[Detector] Unknown dict_name %s, fallback to DICT_4X4_50", name.c_str());
    return cv::aruco::DICT_4X4_50;
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
