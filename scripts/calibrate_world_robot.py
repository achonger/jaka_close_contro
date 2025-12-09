#!/usr/bin/env python3
"""
世界-机器人标定脚本
用于收集标定数据并执行标定过程
"""

import rospy
import sys
import time
from std_srvs.srv import Trigger, TriggerRequest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from jaka_sdk_driver.srv import JointMove, JointMoveRequest
import math

class WorldRobotCalibrationClient:
    def __init__(self):
        rospy.init_node('world_robot_calibration_client', anonymous=True)
        
        # 服务代理
        self.collect_data_srv = rospy.ServiceProxy('/collect_world_robot_calibration_data', Trigger)
        self.calibrate_srv = rospy.ServiceProxy('/calibrate_world_robot', Trigger)
        self.joint_move_srv = rospy.ServiceProxy('/jaka_driver/joint_move', JointMove)
        
        # 订阅标定结果
        self.calibration_result_sub = rospy.Subscriber('/world_robot_calibration_result', PoseStamped, self.calibration_result_callback)
        
        self.calibration_result = None
        print("World-Robot Calibration Client initialized")
    
    def calibration_result_callback(self, msg):
        self.calibration_result = msg
        print(f"Received calibration result: pos=({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}), "
              f"quat=({msg.pose.orientation.w:.3f}, {msg.pose.orientation.x:.3f}, {msg.pose.orientation.y:.3f}, {msg.pose.orientation.z:.3f})")
    
    def move_to_pose(self, joint_positions, velocity=0.5, acceleration=0.5):
        """移动机械臂到指定关节角度"""
        try:
            req = JointMoveRequest()
            req.pose = joint_positions
            req.mvvelo = velocity
            req.mvacc = acceleration
            req.mvradius = 0.0  # 精确定位
            
            response = self.joint_move_srv(req)
            if response.ret == 0:
                print(f"Successfully moved to pose: {joint_positions}")
                return True
            else:
                print(f"Failed to move to pose: {response.message}")
                return False
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False
    
    def collect_calibration_data(self):
        """收集标定数据"""
        print("Starting calibration data collection...")
        
        # 定义几个不同的机械臂位姿用于标定
        calibration_poses = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],           # 初始位姿
            [0.2, 0.1, 0.0, 0.0, 0.1, 0.05],         # 位姿1
            [-0.1, 0.15, 0.05, 0.05, -0.1, 0.0],     # 位姿2
            [0.0, -0.1, -0.1, 0.1, 0.05, -0.05],     # 位姿3
            [0.15, 0.05, -0.15, -0.05, 0.15, 0.1],   # 位姿4
        ]
        
        for i, pose in enumerate(calibration_poses):
            print(f"Moving to calibration pose {i+1}/{len(calibration_poses)}: {pose}")
            
            # 移动到指定位姿
            if not self.move_to_pose(pose):
                print(f"Failed to move to pose {i+1}, skipping...")
                continue
            
            # 等待机械臂稳定
            print("Waiting for robot to stabilize...")
            time.sleep(3.0)
            
            # 确保视觉系统有时间检测立方体
            print("Waiting for vision system...")
            time.sleep(2.0)
            
            # 收集数据点
            try:
                response = self.collect_data_srv(TriggerRequest())
                if response.success:
                    print(f"Successfully collected data point {i+1}")
                else:
                    print(f"Failed to collect data point {i+1}: {response.message}")
            except rospy.ServiceException as e:
                print(f"Service call failed for data point {i+1}: {e}")
            
            # 短暂暂停
            time.sleep(1.0)
        
        print("Finished collecting calibration data")
    
    def perform_calibration(self):
        """执行标定"""
        print("Starting world-robot calibration...")
        
        try:
            response = self.calibrate_srv(TriggerRequest())
            if response.success:
                print("Calibration completed successfully")
                return True
            else:
                print(f"Calibration failed: {response.message}")
                return False
        except rospy.ServiceException as e:
            print(f"Calibration service call failed: {e}")
            return False
    
    def run_calibration_procedure(self):
        """运行完整的标定流程"""
        print("Starting world-robot calibration procedure...")
        
        # 1. 收集标定数据
        self.collect_calibration_data()
        
        # 2. 执行标定
        success = self.perform_calibration()
        
        if success:
            print("World-robot calibration procedure completed successfully!")
        else:
            print("World-robot calibration procedure failed!")
        
        return success

if __name__ == '__main__':
    client = WorldRobotCalibrationClient()
    
    if len(sys.argv) > 1 and sys.argv[1] == 'auto':
        # 自动运行完整流程
        client.run_calibration_procedure()
    else:
        # 交互式模式
        print("\nWorld-Robot Calibration Options:")
        print("1. Collect calibration data")
        print("2. Perform calibration")
        print("3. Run full procedure")
        print("4. Exit")
        
        while not rospy.is_shutdown():
            try:
                choice = input("\nEnter your choice (1-4): ").strip()
                
                if choice == '1':
                    client.collect_calibration_data()
                elif choice == '2':
                    client.perform_calibration()
                elif choice == '3':
                    client.run_calibration_procedure()
                elif choice == '4':
                    print("Exiting...")
                    break
                else:
                    print("Invalid choice. Please enter 1-4.")
            except KeyboardInterrupt:
                print("\nExiting...")
                break
            except EOFError:
                print("\nExiting...")
                break