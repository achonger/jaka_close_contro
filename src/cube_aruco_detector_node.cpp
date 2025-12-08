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

class CubeArucoDetector {
public:
  CubeArucoDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), have_cam_info_(false) {
    // 读取参数
    pnh_.param<std::string>("image_topic", image_topic_, "/zed2i/zed_node/left/image_rect_color");
    pnh_.param<std::string>("camera_info_topic", camera_info_topic_, "/zed2i/zed_node/left/camera_info");
    pnh_.param<std::string>("fiducial_topic", fiducial_topic_, "/fiducial_transforms");
    pnh_.param("default_marker_size", default_marker_size_, 0.04); // 默认40mm

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

    // ArUco 配置 (固定使用 DICT_4X4_50)
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    detector_params_ = cv::aruco::DetectorParameters::create();
    ROS_INFO("[Detector] Using ArUco dictionary DICT_4X4_50");

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
    cv::aruco::detectMarkers(gray, dictionary_, corners, ids, detector_params_);

    fiducial_msgs::FiducialTransformArray out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = cam_frame_id_;
    out.image_seq = msg->header.seq;

    if (ids.empty()) {
      ROS_DEBUG_THROTTLE(1.0, "[Detector] No markers detected");
      fid_pub_.publish(out);
      return;
    }

    for (size_t i = 0; i < ids.size(); ++i) {
      int id = ids[i];
      double marker_size = default_marker_size_;
      
      auto size_it = marker_sizes_.find(id);
      if (size_it != marker_sizes_.end()) {
        marker_size = size_it->second;
      }

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

    fid_pub_.publish(out);
    ROS_DEBUG_THROTTLE(1.0, "[Detector] Published %zu markers", out.transforms.size());

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

  std::string image_topic_, camera_info_topic_, fiducial_topic_;
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