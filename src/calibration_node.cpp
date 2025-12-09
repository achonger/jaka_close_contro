#include <ros/ros.h>
#include <aruco_msgs/MarkerArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Trigger.h>
#include <jaka_sdk_driver/JointMove.h>
#include <vector>
#include <fstream>
#include <yaml-cpp/yaml.h>

class CalibrationNode
{
public:
    CalibrationNode(ros::NodeHandle& nh) : tf_listener_(tf_buffer_)
    {
        // 订阅ArUco标记检测结果
        marker_sub_ = nh.subscribe("/aruco/markers", 1, &CalibrationNode::markerCallback, this);
        
        // 订阅关节状态
        joint_state_sub_ = nh.subscribe("/joint_states", 1, &CalibrationNode::jointStateCallback, this);
        
        // 服务客户端 - 用于发送关节运动命令
        joint_move_client_ = nh.serviceClient<jaka_sdk_driver::JointMove>("/jaka_driver/joint_move");
        
        // 服务服务器 - 用于触发标定
        calibrate_service_ = nh.advertiseService("/calibrate_hand_eye", &CalibrationNode::calibrateHandEye, this);
        collect_data_service_ = nh.advertiseService("/collect_calibration_data", &CalibrationNode::collectCalibrationData, this);
        
        // 发布标定结果
        calibration_result_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/calibration_result", 1);
        
        ROS_INFO("Calibration Node initialized");
    }

private:
    ros::Subscriber marker_sub_;
    ros::Subscriber joint_state_sub_;
    ros::ServiceServer calibrate_service_;
    ros::ServiceServer collect_data_service_;
    ros::Publisher calibration_result_pub_;
    ros::ServiceClient joint_move_client_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    std::vector<std::vector<double>> robot_poses_;  // 机械臂末端位姿
    std::vector<geometry_msgs::Pose> marker_poses_; // 标记位姿
    sensor_msgs::JointState current_joint_state_;
    aruco_msgs::MarkerArray current_markers_;
    
    bool has_new_marker_data_ = false;
    bool has_new_joint_data_ = false;

    void markerCallback(const aruco_msgs::MarkerArray::ConstPtr& msg)
    {
        current_markers_ = *msg;
        has_new_marker_data_ = true;
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        current_joint_state_ = *msg;
        has_new_joint_data_ = true;
    }

    bool collectCalibrationData(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
    {
        ROS_INFO("Collecting calibration data...");
        
        if (!has_new_marker_data_ || !has_new_joint_data_) {
            ROS_WARN("No marker or joint data available");
            res.success = false;
            res.message = "No data available";
            return true;
        }

        // 获取机械臂末端位姿（从关节角度计算）
        std::vector<double> robot_pose = calculateRobotPose(current_joint_state_);
        
        // 获取ArUco标记位姿
        if (current_markers_.markers.empty()) {
            ROS_WARN("No markers detected");
            res.success = false;
            res.message = "No markers detected";
            return true;
        }
        
        geometry_msgs::Pose marker_pose = current_markers_.markers[0].pose.pose; // 假设使用第一个标记
        
        // 存储数据
        robot_poses_.push_back(robot_pose);
        marker_poses_.push_back(marker_pose);
        
        ROS_INFO("Collected calibration data point. Total: %ld", robot_poses_.size());
        
        res.success = true;
        res.message = "Data collected successfully";
        return true;
    }

    bool calibrateHandEye(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
    {
        ROS_INFO("Starting hand-eye calibration...");
        
        if (robot_poses_.size() < 3) {
            ROS_ERROR("Need at least 3 data points for calibration");
            res.success = false;
            res.message = "Insufficient data points";
            return true;
        }
        
        // 执行手眼标定（简化实现，实际应使用更复杂的算法）
        geometry_msgs::PoseStamped calibration_result = performHandEyeCalibration();
        
        // 保存标定结果
        saveCalibrationResult(calibration_result);
        
        // 发布标定结果
        calibration_result_pub_.publish(calibration_result);
        
        // 清空数据
        robot_poses_.clear();
        marker_poses_.clear();
        
        res.success = true;
        res.message = "Hand-eye calibration completed";
        return true;
    }

    std::vector<double> calculateRobotPose(const sensor_msgs::JointState& joint_state)
    {
        // 简化：直接返回关节角度作为位姿表示
        // 实际应用中需要使用正运动学计算末端位姿
        std::vector<double> pose;
        for (const auto& position : joint_state.position) {
            pose.push_back(position);
        }
        return pose;
    }

    geometry_msgs::PoseStamped performHandEyeCalibration()
    {
        // 简化的手眼标定实现
        // 实际应用中应该使用专门的手眼标定库或算法
        geometry_msgs::PoseStamped result;
        result.header.frame_id = "base_link"; // 机械臂基坐标系
        result.header.stamp = ros::Time::now();
        
        // 这里应该实现真正的手眼标定算法
        // 暂时返回一个示例位姿
        result.pose.position.x = 0.1;
        result.pose.position.y = 0.0;
        result.pose.position.z = 0.2;
        result.pose.orientation.w = 1.0;
        result.pose.orientation.x = 0.0;
        result.pose.orientation.y = 0.0;
        result.pose.orientation.z = 0.0;
        
        ROS_INFO("Hand-eye calibration completed (simplified implementation)");
        return result;
    }

    void saveCalibrationResult(const geometry_msgs::PoseStamped& result)
    {
        YAML::Node config;
        config["calibration_transform"]["position"]["x"] = result.pose.position.x;
        config["calibration_transform"]["position"]["y"] = result.pose.position.y;
        config["calibration_transform"]["position"]["z"] = result.pose.position.z;
        config["calibration_transform"]["orientation"]["w"] = result.pose.orientation.w;
        config["calibration_transform"]["orientation"]["x"] = result.pose.orientation.x;
        config["calibration_transform"]["orientation"]["y"] = result.pose.orientation.y;
        config["calibration_transform"]["orientation"]["z"] = result.pose.orientation.z;
        
        std::ofstream file("/tmp/hand_eye_calibration.yaml");
        file << config;
        file.close();
        
        ROS_INFO("Calibration result saved to /tmp/hand_eye_calibration.yaml");
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibration_node");
    ros::NodeHandle nh("~");
    
    CalibrationNode cal_node(nh);
    
    ros::spin();
    return 0;
}