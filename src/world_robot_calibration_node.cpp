#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Trigger.h>
#include <jaka_sdk_driver/JointMove.h>
#include <vector>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>

class WorldRobotCalibrationNode
{
public:
    WorldRobotCalibrationNode(ros::NodeHandle& nh) : tf_listener_(tf_buffer_)
    {
        // 订阅立方体中心位姿（从相机坐标系）
        cube_pose_sub_ = nh.subscribe("/cube_center_fused", 1, &WorldRobotCalibrationNode::cubePoseCallback, this);
        
        // 订阅关节状态
        joint_state_sub_ = nh.subscribe("/joint_states", 1, &WorldRobotCalibrationNode::jointStateCallback, this);
        
        // 服务客户端 - 用于发送关节运动命令
        joint_move_client_ = nh.serviceClient<jaka_sdk_driver::JointMove>("/jaka_driver/joint_move");
        
        // 服务服务器 - 用于触发标定
        calibrate_service_ = nh.advertiseService("/calibrate_world_robot", &WorldRobotCalibrationNode::calibrateWorldRobot, this);
        collect_data_service_ = nh.advertiseService("/collect_world_robot_calibration_data", &WorldRobotCalibrationNode::collectCalibrationData, this);
        
        // 发布标定结果
        calibration_result_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/world_robot_calibration_result", 1);
        
        // 读取参数
        nh.param<std::string>("world_frame", world_frame_, "world");
        nh.param<std::string>("robot_base_frame", robot_base_frame_, "Link_0");
        nh.param<std::string>("tool_frame", tool_frame_, "Link_6");
        nh.param<std::string>("camera_frame", camera_frame_, "zed2i_left_camera_optical_frame");
        nh.param<std::string>("cube_frame", cube_frame_, "cube_center");
        
        ROS_INFO("World-Robot Calibration Node initialized");
        ROS_INFO("World frame: %s", world_frame_.c_str());
        ROS_INFO("Robot base frame: %s", robot_base_frame_.c_str());
        ROS_INFO("Tool frame: %s", tool_frame_.c_str());
        ROS_INFO("Camera frame: %s", camera_frame_.c_str());
        ROS_INFO("Cube frame: %s", cube_frame_.c_str());
    }

private:
    ros::Subscriber cube_pose_sub_;
    ros::Subscriber joint_state_sub_;
    ros::ServiceServer calibrate_service_;
    ros::ServiceServer collect_data_service_;
    ros::Publisher calibration_result_pub_;
    ros::ServiceClient joint_move_client_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    std::vector<Eigen::Matrix4d> robot_poses_;  // 机械臂末端位姿（相对于基座）
    std::vector<Eigen::Matrix4d> cube_poses_in_world_; // 立方体在世界坐标系中的位姿
    
    geometry_msgs::PoseStamped current_cube_pose_;
    sensor_msgs::JointState current_joint_state_;
    bool has_new_cube_data_ = false;
    bool has_new_joint_data_ = false;
    
    std::string world_frame_;
    std::string robot_base_frame_;
    std::string tool_frame_;
    std::string camera_frame_;
    std::string cube_frame_;

    void cubePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        current_cube_pose_ = *msg;
        has_new_cube_data_ = true;
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        current_joint_state_ = *msg;
        has_new_joint_data_ = true;
    }

    bool collectCalibrationData(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
    {
        ROS_INFO("Collecting world-robot calibration data...");
        
        if (!has_new_cube_data_ || !has_new_joint_data_) {
            ROS_WARN("No cube pose or joint data available");
            res.success = false;
            res.message = "No data available";
            return true;
        }

        try {
            // 获取立方体在相机坐标系中的位姿
            geometry_msgs::TransformStamped T_cam_cube_msg;
            T_cam_cube_msg = tf_buffer_.lookupTransform(camera_frame_, cube_frame_, ros::Time(0), ros::Duration(1.0));
            
            // 获取相机在世界坐标系中的位姿
            geometry_msgs::TransformStamped T_world_cam_msg;
            T_world_cam_msg = tf_buffer_.lookupTransform(world_frame_, camera_frame_, ros::Time(0), ros::Duration(1.0));
            
            // 转换为Eigen矩阵
            Eigen::Matrix4d T_cam_cube = transformToEigen(T_cam_cube_msg.transform);
            Eigen::Matrix4d T_world_cam = transformToEigen(T_world_cam_msg.transform);
            
            // 计算立方体在世界坐标系中的位姿
            Eigen::Matrix4d T_world_cube = T_world_cam * T_cam_cube;
            
            // 获取机械臂末端在基座坐标系中的位姿（从关节角度计算）
            Eigen::Matrix4d T_robot_base_tool = calculateRobotPose(current_joint_state_);
            
            // 存储数据
            robot_poses_.push_back(T_robot_base_tool);
            cube_poses_in_world_.push_back(T_world_cube);
            
            ROS_INFO("Collected world-robot calibration data point. Total: %ld", robot_poses_.size());
            ROS_INFO("Cube position in world: [%.3f, %.3f, %.3f]", 
                     T_world_cube(0, 3), T_world_cube(1, 3), T_world_cube(2, 3));
            ROS_INFO("Tool position in robot base: [%.3f, %.3f, %.3f]", 
                     T_robot_base_tool(0, 3), T_robot_base_tool(1, 3), T_robot_base_tool(2, 3));
            
            res.success = true;
            res.message = "Data collected successfully";
            return true;
        }
        catch (tf2::TransformException& ex) {
            ROS_ERROR("Transform lookup failed: %s", ex.what());
            res.success = false;
            res.message = "Transform lookup failed: " + std::string(ex.what());
            return true;
        }
    }

    bool calibrateWorldRobot(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
    {
        ROS_INFO("Starting world-robot calibration...");
        
        if (robot_poses_.size() < 3) {
            ROS_ERROR("Need at least 3 data points for calibration, current: %ld", robot_poses_.size());
            res.success = false;
            res.message = "Insufficient data points";
            return true;
        }
        
        // 执行世界-机器人标定
        geometry_msgs::PoseStamped calibration_result = performWorldRobotCalibration();
        
        // 保存标定结果
        saveCalibrationResult(calibration_result);
        
        // 发布标定结果
        calibration_result_pub_.publish(calibration_result);
        
        // 清空数据
        robot_poses_.clear();
        cube_poses_in_world_.clear();
        
        res.success = true;
        res.message = "World-robot calibration completed";
        return true;
    }

    Eigen::Matrix4d transformToEigen(const geometry_msgs::Transform& t)
    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        
        // 位置
        T(0, 3) = t.translation.x;
        T(1, 3) = t.translation.y;
        T(2, 3) = t.translation.z;
        
        // 四元数转旋转矩阵
        Eigen::Quaterniond q(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
        q.normalize();
        Eigen::Matrix3d R = q.toRotationMatrix();
        
        T.block<3, 3>(0, 0) = R;
        
        return T;
    }

Eigen::Matrix4d calculateRobotPose(const sensor_msgs::JointState& joint_state)
{
    // 使用 TF 查询机器人基座到工具末端的位姿，并尽量与最新的关节状态同步。
    // 关节状态由仓库自带的 jaka_sdk_driver_node 发布，配合 robot_state_publisher 生成 TF。
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    // 若关节时间戳为空，则退回最新 TF；否则按关节时间戳查询，提升同步性。
    const ros::Time query_time = joint_state.header.stamp.isZero() ? ros::Time(0) : joint_state.header.stamp;

    try
    {
        geometry_msgs::TransformStamped T_base_tool_msg =
            tf_buffer_.lookupTransform(robot_base_frame_,    // 源：机器人基座
                                       tool_frame_,          // 目标：工具末端
                                       query_time,
                                       ros::Duration(1.0));

        T = transformToEigen(T_base_tool_msg.transform);
    }
    catch (tf2::TransformException& ex)
    {
        ROS_ERROR_THROTTLE(1.0,
                           "Failed to lookup TF %s -> %s for FK: %s",
                           robot_base_frame_.c_str(),
                           tool_frame_.c_str(),
                           ex.what());
        ROS_WARN_THROTTLE(5.0,
                          "Using Identity for T_robot_base_tool due to missing TF. "
                          "Calibration accuracy will be very poor!");
    }

    return T;
}


    geometry_msgs::PoseStamped performWorldRobotCalibration()
    {
        ROS_INFO("Performing world-robot calibration with %ld data points", robot_poses_.size());
        
        if (robot_poses_.size() < 2) {
            ROS_ERROR("Need at least 2 poses for calibration");
            geometry_msgs::PoseStamped result;
            result.header.frame_id = robot_base_frame_;
            result.header.stamp = ros::Time::now();
            return result;
        }

        // 将问题转化为 AX = XB 的形式
        // A: 机器人基座到工具的变换变化 (T_robot_base_tool_i * T_robot_base_tool_j^(-1))
        // B: 世界到立方体的变换变化 (T_world_cube_i * T_world_cube_j^(-1))
        // X: 世界到机器人基座的变换 (T_world_robot_base)
        
        std::vector<Eigen::Matrix4d> A_transforms;
        std::vector<Eigen::Matrix4d> B_transforms;
        
        // 计算相邻位姿之间的变换差
        for (size_t i = 1; i < robot_poses_.size(); ++i) {
            Eigen::Matrix4d T_robot_base_tool_i = robot_poses_[i-1];
            Eigen::Matrix4d T_robot_base_tool_j = robot_poses_[i];
            
            Eigen::Matrix4d T_robot_base_tool_diff = T_robot_base_tool_j * T_robot_base_tool_i.inverse();
            
            Eigen::Matrix4d T_world_cube_i = cube_poses_in_world_[i-1];
            Eigen::Matrix4d T_world_cube_j = cube_poses_in_world_[i];
            
            Eigen::Matrix4d T_world_cube_diff = T_world_cube_j * T_world_cube_i.inverse();
            
            A_transforms.push_back(T_robot_base_tool_diff);
            B_transforms.push_back(T_world_cube_diff);
        }
        
        if (A_transforms.empty()) {
            ROS_ERROR("No transform differences calculated");
            geometry_msgs::PoseStamped result;
            result.header.frame_id = robot_base_frame_;
            result.header.stamp = ros::Time::now();
            return result;
        }

        // 使用Tsai手眼标定算法求解旋转部分
        Eigen::Matrix3d R_world_robot_base = solveRotationAXB(A_transforms, B_transforms);
        
        // 求解平移部分
        Eigen::Vector3d t_world_robot_base = solveTranslationAXB(A_transforms, B_transforms, R_world_robot_base);
        
        // 构造最终的变换矩阵
        Eigen::Matrix4d T_world_robot_base = Eigen::Matrix4d::Identity();
        T_world_robot_base.block<3, 3>(0, 0) = R_world_robot_base;
        T_world_robot_base.block<3, 1>(0, 3) = t_world_robot_base;
        
        // 转换为四元数
        Eigen::Quaterniond quat(R_world_robot_base);
        quat.normalize();
        
        geometry_msgs::PoseStamped result;
        // 使用参数指定的世界坐标系名称，避免硬编码导致的 TF 不一致
        result.header.frame_id = world_frame_;
        result.header.stamp = ros::Time::now();
        result.pose.position.x = t_world_robot_base.x();
        result.pose.position.y = t_world_robot_base.y();
        result.pose.position.z = t_world_robot_base.z();
        result.pose.orientation.w = quat.w();
        result.pose.orientation.x = quat.x();
        result.pose.orientation.y = quat.y();
        result.pose.orientation.z = quat.z();
        
        ROS_INFO("World-robot calibration completed");
        ROS_INFO("Result: pos=[%.3f,%.3f,%.3f], quat=[%.3f,%.3f,%.3f,%.3f]",
                 result.pose.position.x, result.pose.position.y, result.pose.position.z,
                 result.pose.orientation.w, result.pose.orientation.x, 
                 result.pose.orientation.y, result.pose.orientation.z);
        
        return result;
    }

    // 使用Tsai算法求解AX=XB问题的旋转部分
    Eigen::Matrix3d solveRotationAXB(const std::vector<Eigen::Matrix4d>& A, const std::vector<Eigen::Matrix4d>& B)
    {
        std::vector<Eigen::Vector3d> a_vectors; // 从A变换提取的旋转轴
        std::vector<Eigen::Vector3d> b_vectors; // 从B变换提取的旋转轴
        
        for (size_t i = 0; i < A.size(); ++i) {
            Eigen::Matrix3d Ra = A[i].block<3, 3>(0, 0);
            Eigen::Matrix3d Rb = B[i].block<3, 3>(0, 0);
            
            // 计算旋转轴和角度
            Eigen::AngleAxisd aa_a(Ra);
            Eigen::AngleAxisd aa_b(Rb);
            
            if (std::abs(aa_a.angle()) > 1e-6 && std::abs(aa_b.angle()) > 1e-6) {
                a_vectors.push_back(aa_a.axis() * aa_a.angle());
                b_vectors.push_back(aa_b.axis() * aa_b.angle());
            }
        }
        
        if (a_vectors.size() < 2) {
            ROS_WARN("Not enough rotation data points for calibration, using identity rotation");
            return Eigen::Matrix3d::Identity();
        }
        
        // 构建方程组求解旋转矩阵
        Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
        for (size_t i = 0; i < a_vectors.size(); ++i) {
            M += b_vectors[i] * a_vectors[i].transpose();
        }
        
        // 使用SVD分解求解旋转矩阵
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
        
        // 确保行列式为正（避免镜像）
        if (R.determinant() < 0) {
            Eigen::Matrix3d tmp = Eigen::Matrix3d::Identity();
            tmp(2, 2) = -1.0;
            R = svd.matrixU() * tmp * svd.matrixV().transpose();
        }
        
        return R;
    }

   // 求解 AX = XB 问题的平移部分
Eigen::Vector3d solveTranslationAXB(const std::vector<Eigen::Matrix4d>& A,
                                   const std::vector<Eigen::Matrix4d>& B,
                                   const Eigen::Matrix3d& R_world_robot_base)
{
    // 由 AX = XB 在 SE(3) 上展开可得：
    //   R_a t_x + t_a = R_x t_b + t_x
    // => (R_a - I) t_x = R_x t_b - t_a
    // 其中 R_x 就是已由 solveRotationAXB 求得的 R_world_robot_base。
    const size_t N = A.size();

    Eigen::MatrixXd C(3 * N, 3);
    Eigen::VectorXd d(3 * N);

    for (size_t i = 0; i < N; ++i)
    {
        Eigen::Vector3d ta = A[i].block<3,1>(0,3);
        Eigen::Vector3d tb = B[i].block<3,1>(0,3);
        Eigen::Matrix3d Ra = A[i].block<3,3>(0,0);

        // 右端项：R_x * t_b - t_a
        Eigen::Vector3d rhs = R_world_robot_base * tb - ta;

        // 系数矩阵：R_a - I
        C.block<3,3>(3 * i, 0) = Ra - Eigen::Matrix3d::Identity();
        d.segment<3>(3 * i)    = rhs;
    }

    // 使用最小二乘法求解 t_world_robot_base
    Eigen::Vector3d t_world_robot_base = C.colPivHouseholderQr().solve(d);

    return t_world_robot_base;
}


    void saveCalibrationResult(const geometry_msgs::PoseStamped& result)
    {
        YAML::Node config;
        config["world_robot_calibration"]["position"]["x"] = result.pose.position.x;
        config["world_robot_calibration"]["position"]["y"] = result.pose.position.y;
        config["world_robot_calibration"]["position"]["z"] = result.pose.position.z;
        config["world_robot_calibration"]["orientation"]["w"] = result.pose.orientation.w;
        config["world_robot_calibration"]["orientation"]["x"] = result.pose.orientation.x;
        config["world_robot_calibration"]["orientation"]["y"] = result.pose.orientation.y;
        config["world_robot_calibration"]["orientation"]["z"] = result.pose.orientation.z;
        
        std::ofstream file("/tmp/world_robot_calibration.yaml");
        file << config;
        file.close();
        
        ROS_INFO("Calibration result saved to /tmp/world_robot_calibration.yaml");
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "world_robot_calibration_node");
    ros::NodeHandle nh("~");
    
    WorldRobotCalibrationNode cal_node(nh);
    
    ros::spin();
    return 0;
}