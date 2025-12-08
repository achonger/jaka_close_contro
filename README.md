# 立方体ArUco标记检测与世界标签系统

这个ROS包提供了一个用于检测和跟踪立方体上ArUco标记的系统，同时具备机器人应用的世界标签功能。

## 概述

该软件包包含多个协同工作的节点，用于：
- 检测立方体面上的ArUco标记
- 融合多面信息以实现稳定的立方体跟踪
- 中继基准标记信息
- 处理空间定位的世界标签

## 节点

### cube_aruco_detector_node
从相机输入中检测立方体面上的ArUco标记。

### cube_multi_face_fusion_node
融合来自多个立方体面的信息，提供稳定的立方体姿态估计。

### fiducial_relay_node
在系统的不同部分之间中继基准标记信息。

### world_tag_node
处理空间定位和映射的世界标签信息。

## 配置文件

配置文件位于 `config/` 目录下：
- `cube_faces_current.yaml`：当前立方体面配置
- `cube_faces_ideal.yaml`：理想立方体面配置

## 启动文件

该软件包在 `launch/` 目录中包含以下启动文件：
- `full_system.launch`：启动完整系统
- `world_tag.launch`：启动世界标签处理节点
- `world_tag_bringup.launch`：启动世界标签初始化序列

## 依赖项

此软件包需要：
- ROS（机器人操作系统）
- OpenCV
- ArUco标记检测库
- cv_bridge
- image_transport

## 安装

1. 将此软件包克隆到您的ROS工作区：
   ```bash
   cd ~/catkin_ws/src
   git clone <repository-url>
   ```

2. 构建工作区：
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

3. 源化工作区：
   ```bash
   source devel/setup.bash
   ```

## 使用方法

运行完整系统：
```bash
roslaunch cube_aruco_detector full_system.launch
```

运行世界标签处理：
```bash
roslaunch cube_aruco_detector world_tag.launch
```

## 许可证

[在此处指定许可证]