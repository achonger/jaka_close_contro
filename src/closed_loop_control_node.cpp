#include <ros/ros.h>
#include <aruco_msgs/MarkerArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Trigger.h>
#include <jaka_sdk_driver/JointMove.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <string>

class ClosedLoopControlNode
{
public:
    ClosedLoopControlNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : tf_listener_(tf_buffer_)
    {
        // 使用私有命名空间读取参数，避免与其他节点冲突
        pnh.param<std::string>("base_frame", base_frame_, "Link_0");
        pnh.param<std::string>("world_frame", world_frame_, "world");
        pnh.param<std::string>("camera_frame", camera_frame_, "zed2i_left_camera_optical_frame");

        const std::string default_output = ros::package::getPath("jaka_close_contro") + "/config/world_robot_calibration.yaml";
        pnh.param<std::string>("world_robot_calibration_yaml", world_robot_yaml_path_, default_output);

        // 订阅ArUco标记检测结果
        marker_sub_ = nh.subscribe("/aruco/markers", 1, &ClosedLoopControlNode::markerCallback, this);
        
        // 订阅关节状态
        joint_state_sub_ = nh.subscribe("/joint_states", 1, &ClosedLoopControlNode::jointStateCallback, this);
        
        // 服务客户端 - 用于发送关节运动命令
        joint_move_client_ = nh.serviceClient<jaka_sdk_driver::JointMove>("/jaka_driver/joint_move");
        
        // 服务服务器 - 用于触发闭环控制
        control_service_ = nh.advertiseService("/start_closed_loop_control", &ClosedLoopControlNode::startClosedLoopControl, this);
        
        // 发布控制目标位姿
        target_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/target_pose", 1);
        
        // 发布当前物体位姿
        current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_object_pose", 1);
        
        // 加载标定结果
        loadCalibrationResult();
        
        // 设置默认目标位姿
        target_pose_.header.frame_id = base_frame_;
        target_pose_.pose.position.x = 0.3;
        target_pose_.pose.position.y = 0.0;
        target_pose_.pose.position.z = 0.2;
        target_pose_.pose.orientation.w = 1.0;
        target_pose_.pose.orientation.x = 0.0;
        target_pose_.pose.orientation.y = 0.0;
        target_pose_.pose.orientation.z = 0.0;
        
        ROS_INFO("Closed Loop Control Node initialized");
    }

private:
    ros::Subscriber marker_sub_;
    ros::Subscriber joint_state_sub_;
    ros::ServiceServer control_service_;
    ros::Publisher target_pose_pub_;
    ros::Publisher current_pose_pub_;
    ros::ServiceClient joint_move_client_;

    std::string base_frame_;
    std::string world_frame_;
    std::string camera_frame_;
    std::string world_robot_yaml_path_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    geometry_msgs::PoseStamped target_pose_;
    geometry_msgs::PoseStamped current_object_pose_;
    Eigen::Isometry3d world_to_base_tf_ = Eigen::Isometry3d::Identity(); // 世界到机械臂基座
    bool has_world_to_base_tf_ = false;
    sensor_msgs::JointState current_joint_state_;
    aruco_msgs::MarkerArray current_markers_;
    
    double position_tolerance_ = 0.01; // 位置容差 1cm
    double orientation_tolerance_ = 0.05; // 姿态容差

    void markerCallback(const aruco_msgs::MarkerArray::ConstPtr& msg)
    {
        current_markers_ = *msg;
        
        if (!current_markers_.markers.empty()) {
            // 将检测到的标记位姿转换为物体中心位姿
            geometry_msgs::PoseStamped marker_pose;
            marker_pose.header = current_markers_.markers[0].header;
            marker_pose.pose = current_markers_.markers[0].pose.pose;
            
            // 转换到世界坐标系
            current_object_pose_ = transformPoseToRobotBase(marker_pose);
            
            // 发布当前物体位姿
            current_pose_pub_.publish(current_object_pose_);
        }
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        current_joint_state_ = *msg;
    }

    bool startClosedLoopControl(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
    {
        ROS_INFO("Starting closed-loop control...");
        
        if (current_markers_.markers.empty()) {
            ROS_WARN("No markers detected, cannot start control");
            res.success = false;
            res.message = "No markers detected";
            return true;
        }
        
        // 开始闭环控制循环
        ros::Rate rate(10); // 10Hz控制频率
        int max_iterations = 100; // 最大迭代次数
        int iteration = 0;
        
        while (ros::ok() && iteration < max_iterations) {
            // 检查是否达到目标位姿
            if (isAtTargetPose()) {
                ROS_INFO("Reached target pose!");
                res.success = true;
                res.message = "Successfully reached target pose";
                return true;
            }
            
            // 计算控制命令
            geometry_msgs::PoseStamped control_command = calculateControlCommand();
            
            // 发送控制命令
            if (!sendControlCommand(control_command)) {
                ROS_ERROR("Failed to send control command");
                res.success = false;
                res.message = "Failed to send control command";
                return true;
            }
            
            iteration++;
            rate.sleep();
            ros::spinOnce();
        }
        
        if (iteration >= max_iterations) {
            ROS_WARN("Maximum iterations reached, stopping control");
            res.success = false;
            res.message = "Maximum iterations reached";
        }
        
        return true;
    }

    geometry_msgs::PoseStamped transformPoseToRobotBase(const geometry_msgs::PoseStamped& camera_pose)
    {
        geometry_msgs::PoseStamped robot_pose;
        robot_pose.header = camera_pose.header;
        robot_pose.header.frame_id = base_frame_;

        if (!has_world_to_base_tf_)
        {
            ROS_WARN_THROTTLE(5.0, "[ClosedLoop] 尚未加载世界-机器人标定，使用单位变换");
        }

        try
        {
            // 获取世界到相机的在线 TF（由大世界板提供）
            const geometry_msgs::TransformStamped T_world_camera_msg =
                tf_buffer_.lookupTransform(world_frame_, camera_frame_, ros::Time(0), ros::Duration(1.0));

            // Eigen 转换
            Eigen::Isometry3d world_to_camera_tf = Eigen::Isometry3d::Identity();
            world_to_camera_tf.translation() = Eigen::Vector3d(
                T_world_camera_msg.transform.translation.x,
                T_world_camera_msg.transform.translation.y,
                T_world_camera_msg.transform.translation.z);
            Eigen::Quaterniond world_cam_quat(
                T_world_camera_msg.transform.rotation.w,
                T_world_camera_msg.transform.rotation.x,
                T_world_camera_msg.transform.rotation.y,
                T_world_camera_msg.transform.rotation.z);
            world_to_camera_tf.rotate(world_cam_quat);

            // 相机观测的立方体位姿
            Eigen::Isometry3d camera_pose_tf = Eigen::Isometry3d::Identity();
            camera_pose_tf.translation() = Eigen::Vector3d(
                camera_pose.pose.position.x,
                camera_pose.pose.position.y,
                camera_pose.pose.position.z);
            Eigen::Quaterniond camera_rot(
                camera_pose.pose.orientation.w,
                camera_pose.pose.orientation.x,
                camera_pose.pose.orientation.y,
                camera_pose.pose.orientation.z);
            camera_pose_tf.rotate(camera_rot);

            // world -> cube
            Eigen::Isometry3d world_to_cube_tf = world_to_camera_tf * camera_pose_tf;

            // base -> cube = (base<-world) * (world->cube)
            Eigen::Isometry3d base_to_cube_tf = world_to_base_tf_.inverse() * world_to_cube_tf;

            robot_pose.pose.position.x = base_to_cube_tf.translation().x();
            robot_pose.pose.position.y = base_to_cube_tf.translation().y();
            robot_pose.pose.position.z = base_to_cube_tf.translation().z();

            Eigen::Quaterniond robot_quat(base_to_cube_tf.rotation());
            robot_pose.pose.orientation.w = robot_quat.w();
            robot_pose.pose.orientation.x = robot_quat.x();
            robot_pose.pose.orientation.y = robot_quat.y();
            robot_pose.pose.orientation.z = robot_quat.z();
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_ERROR_THROTTLE(5.0, "[ClosedLoop] 查询 TF world->camera 失败: %s", ex.what());
            // 保持零位姿，提示上层不要继续使用
            robot_pose.pose.orientation.w = 1.0;
        }

        return robot_pose;
    }

    bool isAtTargetPose()
    {
        if (current_object_pose_.header.frame_id.empty()) {
            return false;
        }
        
        // 计算位置误差
        double pos_error = std::sqrt(
            std::pow(current_object_pose_.pose.position.x - target_pose_.pose.position.x, 2) +
            std::pow(current_object_pose_.pose.position.y - target_pose_.pose.position.y, 2) +
            std::pow(current_object_pose_.pose.position.z - target_pose_.pose.position.z, 2)
        );
        
        // 计算姿态误差（使用四元数差）
        Eigen::Quaterniond current_quat(current_object_pose_.pose.orientation.w,
                                       current_object_pose_.pose.orientation.x,
                                       current_object_pose_.pose.orientation.y,
                                       current_object_pose_.pose.orientation.z);
        Eigen::Quaterniond target_quat(target_pose_.pose.orientation.w,
                                      target_pose_.pose.orientation.x,
                                      target_pose_.pose.orientation.y,
                                      target_pose_.pose.orientation.z);
        
        double orient_error = current_quat.angularDistance(target_quat);
        
        ROS_INFO("Position error: %f, Orientation error: %f", pos_error, orient_error);
        
        return (pos_error < position_tolerance_) && (orient_error < orientation_tolerance_);
    }

    geometry_msgs::PoseStamped calculateControlCommand()
    {
        geometry_msgs::PoseStamped command;
        command.header.frame_id = base_frame_;
        command.header.stamp = ros::Time::now();
        
        // 简单的比例控制器
        double kp_pos = 0.5;  // 位置比例增益
        double kp_orient = 0.5;  // 姿态比例增益
        
        // 计算位置误差
        double dx = (target_pose_.pose.position.x - current_object_pose_.pose.position.x) * kp_pos;
        double dy = (target_pose_.pose.position.y - current_object_pose_.pose.position.y) * kp_pos;
        double dz = (target_pose_.pose.position.z - current_object_pose_.pose.position.z) * kp_pos;
        
        // 计算姿态误差（简化处理）
        Eigen::Quaterniond current_quat(current_object_pose_.pose.orientation.w,
                                       current_object_pose_.pose.orientation.x,
                                       current_object_pose_.pose.orientation.y,
                                       current_object_pose_.pose.orientation.z);
        Eigen::Quaterniond target_quat(target_pose_.pose.orientation.w,
                                      target_pose_.pose.orientation.x,
                                      target_pose_.pose.orientation.y,
                                      target_pose_.pose.orientation.z);
        
        // 简化的姿态误差计算
        Eigen::Vector3d rotation_error = quaternionToEuler(target_quat) - quaternionToEuler(current_quat);
        
        command.pose.position.x = current_joint_state_.position[0] + dx * 0.1; // 简化映射
        command.pose.position.y = current_joint_state_.position[1] + dy * 0.1;
        command.pose.position.z = current_joint_state_.position[2] + dz * 0.1;
        command.pose.orientation = target_pose_.pose.orientation;
        
        return command;
    }

    Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& q)
    {
        Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
        return euler;
    }

    bool sendControlCommand(const geometry_msgs::PoseStamped& command)
    {
        // 将位姿命令转换为关节命令
        // 这里需要逆运动学求解，简化处理
        jaka_sdk_driver::JointMove srv;
        
        // 简化：直接使用当前关节位置加上一个小的调整
        srv.request.coord_mode = 0; // 0: 关节空间
        srv.request.mvvelo = 0.1;   // 速度
        srv.request.mvacc = 0.1;    // 加速度
        srv.request.pose.resize(6);
        srv.request.pose[0] = command.pose.position.x;
        srv.request.pose[1] = command.pose.position.y;
        srv.request.pose[2] = command.pose.position.z;
        srv.request.pose[3] = 0.0; // 简化
        srv.request.pose[4] = 0.0;
        srv.request.pose[5] = 0.0;
        
        if (joint_move_client_.call(srv)) {
            ROS_INFO("Control command sent successfully");
            return true;
        } else {
            ROS_ERROR("Failed to call joint move service");
            return false;
        }
    }

    void loadCalibrationResult()
    {
        try {
            YAML::Node config = YAML::LoadFile(world_robot_yaml_path_);
            const auto& root = config["world_robot_calibration"];
            if (!root)
            {
                throw YAML::Exception(YAML::Mark::null_mark(), "缺少 world_robot_calibration 字段");
            }

            double x = root["position"]["x"].as<double>();
            double y = root["position"]["y"].as<double>();
            double z = root["position"]["z"].as<double>();
            double qw = root["orientation"]["w"].as<double>();
            double qx = root["orientation"]["x"].as<double>();
            double qy = root["orientation"]["y"].as<double>();
            double qz = root["orientation"]["z"].as<double>();

            world_to_base_tf_.translation() = Eigen::Vector3d(x, y, z);
            Eigen::Quaterniond quat(qw, qx, qy, qz);
            world_to_base_tf_.linear() = quat.matrix();
            has_world_to_base_tf_ = true;

            ROS_INFO("已从 %s 载入世界→基座标定", world_robot_yaml_path_.c_str());
        } catch (const YAML::Exception& e) {
            ROS_WARN("无法加载标定文件 %s，使用单位变换", world_robot_yaml_path_.c_str());
            world_to_base_tf_ = Eigen::Isometry3d::Identity();
            has_world_to_base_tf_ = false;
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "closed_loop_control_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ClosedLoopControlNode control_node(nh, pnh);
    
    ros::spin();
    return 0;
}