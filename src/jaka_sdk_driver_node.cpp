#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>

#include <thread>
#include <atomic>
#include <cstring>   // for memset, memcpy

#include "jaka_sdk_driver/JointMove.h"

// 使用 JAKA ROS 工程里自带的 SDK 头文件
#include "jaka_driver/JAKAZuRobot.h"

class JakaSdkDriver
{
public:
    JakaSdkDriver(ros::NodeHandle& nh)
        : nh_(nh), nh_priv_("~"), running_(false)
    {
        // 1. 读取参数
        nh_priv_.param<double>("status_hz", status_hz_, 50.0);

        std::string ip_default = "192.168.1.100";
        // Prefer namespace param "ip", then private "~ip", then "~robot_ip"
        if (!nh_.getParam("ip", robot_ip_))
        {
            nh_priv_.param<std::string>("ip", robot_ip_, ip_default);
        }
        nh_priv_.param<std::string>("robot_ip", robot_ip_, robot_ip_);

        // 2. JointState publisher
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);

        // 3. 建立 SDK 连接（这里只做 login_in，不再自动 power_on / enable_robot）
        if (!connectRobot())
        {
            ROS_ERROR("[JakaSdkDriver] Failed to connect to robot. Node will exit.");
            ros::shutdown();
            return;
        }

        // 4. joint_move service，名字沿用 /jaka_driver/joint_move，方便复用原来的代码/launch
        joint_move_srv_ = nh_.advertiseService("jaka_driver/joint_move",
                                               &JakaSdkDriver::jointMoveCb, this);

        // 5. 启动状态发布线程
        running_ = true;
        status_thread_ = std::thread(&JakaSdkDriver::statusLoop, this);
    }

    ~JakaSdkDriver()
    {
        running_ = false;
        if (status_thread_.joinable())
            status_thread_.join();

        // 根据需要可以在这里调用 SDK 的关闭接口
        // 例如：
        // robot_.disable_robot();
        // robot_.power_off();
        // robot_.logout();
    }

private:
    //===================== 连接控制柜（仅 login_in） =====================//
    bool connectRobot()
    {
        ROS_INFO("[JakaSdkDriver] Connecting to robot at %s ...", robot_ip_.c_str());

        errno_t ret = 0;

        // 只做登录，不自动上电/使能
        ret = robot_.login_in(robot_ip_.c_str());
        if (ret != 0)
        {
            ROS_ERROR("[JakaSdkDriver] login_in failed, ret=%d", ret);
            return false;
        }

        ROS_INFO("[JakaSdkDriver] login_in OK. 请在示教器 / JAKA APP 上手动完成【上电】和【使能】，再运行运动命令。");

        // 不在这里调用 power_on() / enable_robot()
        // 保持 JAKA 官方推荐的人机流程：人手动确认安全后上电、使能，
        // SDK 只负责状态轮询 + 运动命令。
        return true;
    }

    //===================== 状态发布线程 =====================//
    void statusLoop()
    {
        ros::Rate rate(status_hz_);

        while (ros::ok() && running_)
        {
            RobotStatus status;   // 类型名来自 JAKAZuRobot.h
            errno_t ret = robot_.get_robot_status(&status);

            if (ret == 0)
            {
                publishJointState(status);
            }
            else
            {
                ROS_WARN_THROTTLE(5.0,
                    "[JakaSdkDriver] get_robot_status failed, ret=%d", ret);
            }

            rate.sleep();
        }
    }

void publishJointState(const RobotStatus& status)
{
    sensor_msgs::JointState js;
    js.header.stamp = ros::Time::now();

    js.name.resize(6);
    js.position.resize(6);
    js.velocity.resize(6);
    js.effort.resize(6);

    // ★★ 这里改成和 URDF 一致的命名 ★★
    js.name[0] = "joint_1";
    js.name[1] = "joint_2";
    js.name[2] = "joint_3";
    js.name[3] = "joint_4";
    js.name[4] = "joint_5";
    js.name[5] = "joint_6";

    for (size_t i = 0; i < 6; ++i)
    {
        js.position[i] = status.joint_position[i];
        js.velocity[i] = 0.0;
        js.effort[i]   = 0.0;
    }

    joint_state_pub_.publish(js);
}


    //===================== joint_move 服务回调 =====================//
    bool jointMoveCb(jaka_sdk_driver::JointMove::Request& req,
                     jaka_sdk_driver::JointMove::Response& res)
    {
        if (req.pose.size() != 6)
        {
            res.ret = -1;
            res.message = "pose size must be 6";
            ROS_ERROR("[JakaSdkDriver] joint_move: pose size != 6");
            return true;
        }

        // 构造 JointValue：不依赖具体字段名，直接按内存写入前 6 个 double
        JointValue target;
        std::memset(&target, 0, sizeof(target));

        double* p = reinterpret_cast<double*>(&target);
        for (size_t i = 0; i < 6; ++i)
        {
            p[i] = req.pose[i];
        }

        // joint_move 实际签名：
        // errno_t joint_move(const JointValue *joint_pos,
        //                    MoveMode move_mode,
        //                    BOOL is_block,
        //                    double speed,
        //                    double acc = 90,
        //                    double tol = 0,
        //                    const OptionalCond *option_cond = nullptr);
        //
        // 这里选一个“绝对关节空间 + 阻塞式”的组合：
        MoveMode move_mode = static_cast<MoveMode>(0);      // 一般 0 对应 ABSJ（绝对关节），具体可查 JAKA 文档
        BOOL     is_block  = static_cast<BOOL>(1);          // 阻塞直到到位
        double   speed     = req.mvvelo;                    // 速度
        double   acc       = req.mvacc;                     // 加速度
        double   tol       = 0.0;
        const OptionalCond* opt = nullptr;

        errno_t ret = robot_.joint_move(&target, move_mode, is_block,
                                        speed, acc, tol, opt);

        if (ret != 0)
        {
            res.ret = ret;
            res.message = "joint_move failed from SDK";
            ROS_ERROR("[JakaSdkDriver] joint_move SDK failed, ret=%d", ret);
        }
        else
        {
            res.ret = 0;
            res.message = "success";
            ROS_INFO("[JakaSdkDriver] joint_move success.");
        }

        return true;
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;

    std::string robot_ip_;
    double status_hz_;

    ros::Publisher   joint_state_pub_;
    ros::ServiceServer joint_move_srv_;

    std::thread       status_thread_;
    std::atomic<bool> running_;

    // JAKA SDK 机器人对象
    JAKAZuRobot robot_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "jaka_sdk_driver_node");
    ros::NodeHandle nh;

    JakaSdkDriver driver(nh);

    ros::spin();
    return 0;
}
