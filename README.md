# JAKA机械臂视觉伺服闭环控制系统

## 系统概述

本项目实现了基于视觉反馈的JAKA机械臂闭环控制系统，能够将正方体中心位姿精确控制到世界坐标系的指定位姿。系统主要包括：

1. **视觉检测模块** - 使用ArUco标记检测物体位姿
2. **手眼标定模块** - 标定相机坐标系到机械臂基坐标系的转换
3. **世界-机器人标定模块** - 标定世界坐标系到机械臂基坐标系的转换
4. **闭环控制模块** - 基于视觉反馈的实时控制
5. **机械臂驱动模块** - JAKA机械臂底层驱动

## 系统架构

```
相机 → ArUco检测 → 坐标转换 → 闭环控制器 → JAKA驱动 → 机械臂
          ↑                                    ↓
       物体位姿                            实际位姿反馈
```

## 标定流程

### 1. 世界坐标系定义

首先通过`world_tag_node`定义世界坐标系，使用固定的ArUco标记作为世界坐标系原点。

#### 世界坐标系标定步骤：

1. **放置世界坐标系标靶**：在工作区域放置一个固定且可见的ArUco标记作为世界坐标系原点
2. **配置世界坐标系节点**：
   ```bash
   roslaunch jaka_close_contro world_tag_bringup.launch
   ```
3. **验证世界坐标系**：确保相机-世界坐标系的TF变换正确建立

### 2. 手眼标定

手眼标定用于建立相机坐标系与机械臂末端执行器之间的关系。

#### 标定步骤：

1. **准备标定工具**：将ArUco标记固定在机械臂末端，或使用立方体标靶
2. **启动系统**：
   ```bash
   roslaunch jaka_close_contro full_system.launch
   ```
3. **收集数据点**：
   - 将机械臂移动到不同的位姿
   - 确保相机能清晰看到标定工具
   - 每个位姿下运行服务收集数据：
   ```bash
   rosservice call /collect_calibration_data "{}"
   ```
   - 重复此过程至少10次，覆盖工作空间的不同区域

4. **执行标定**：
   ```bash
   rosservice call /calibrate_hand_eye "{}"
   ```

### 3. 世界-机器人标定

新增了世界坐标系与机械臂基座标系之间的标定功能，这是实现精确全局定位的关键步骤。

#### 世界-机器人标定步骤：

1. **确保系统正常运行**：
   - 世界坐标系已通过`world_tag_node`正确定义
   - 机械臂末端的立方体标靶位姿可通过`cube_multi_face_fusion_node`获取

2. **启动世界-机器人标定节点**：
   ```bash
   roslaunch jaka_close_contro world_robot_calibration.launch
   ```

3. **收集标定数据**：
   - 移动机械臂到不同的位姿
   - 运行服务收集数据点：
   ```bash
   rosservice call /collect_world_robot_calibration_data "{}"
   ```
   - 重复此过程至少5次，确保覆盖工作空间的不同区域

4. **执行标定**：
   ```bash
   rosservice call /calibrate_world_robot "{}"
   ```

5. **验证标定结果**：
   - 检查标定结果的合理性
   - 通过移动机械臂并检查世界坐标系下的位姿一致性来验证

## 闭环控制流程

### 1. 设置目标位姿

在`closed_loop_control_node.cpp`中设置目标位姿：

```cpp
target_pose_.pose.position.x = 0.3;  // 目标位置x (m)
target_pose_.pose.position.y = 0.0;  // 目标位置y (m)
target_pose_.pose.position.z = 0.2;  // 目标位置z (m)
target_pose_.pose.orientation.w = 1.0;  // 目标姿态
// ... 其他姿态参数
```

### 2. 启动闭环控制

```bash
rosservice call /start_closed_loop_control "{}"
```

### 3. 控制算法

系统采用基于位置的视觉伺服控制算法：

1. **视觉反馈**：实时检测物体当前位姿
2. **误差计算**：计算当前位姿与目标位姿的偏差
3. **控制律**：使用比例控制器计算控制命令
4. **执行控制**：将控制命令发送给机械臂
5. **迭代更新**：重复上述过程直到达到目标精度

## 系统节点说明

### calibration_node
- **功能**：执行手眼标定
- **订阅**：
  - `/aruco/markers` - ArUco标记检测结果
  - `/joint_states` - 机械臂关节状态
- **发布**：
  - `/calibration_result` - 标定结果
- **服务**：
  - `/collect_calibration_data` - 收集标定数据
  - `/calibrate_hand_eye` - 执行手眼标定

### world_robot_calibration_node
- **功能**：执行世界坐标系-机械臂基座标系标定
- **订阅**：
  - `/cube_center_fused` - 立方体中心位姿
  - `/joint_states` - 机械臂关节状态
- **发布**：
  - `/world_robot_calibration_result` - 标定结果
- **服务**：
  - `/collect_world_robot_calibration_data` - 收集标定数据
  - `/calibrate_world_robot` - 执行世界-机器人标定
- **服务客户端**：
  - `/jaka_driver/joint_move` - 机械臂运动控制

### closed_loop_control_node
- **功能**：执行闭环控制
- **订阅**：
  - `/aruco/markers` - ArUco标记检测结果
  - `/joint_states` - 机械臂关节状态
- **发布**：
  - `/target_pose` - 目标位姿
  - `/current_object_pose` - 当前物体位姿
- **服务**：
  - `/start_closed_loop_control` - 启动闭环控制
- **服务客户端**：
  - `/jaka_driver/joint_move` - 机械臂运动控制

### 其他节点
- `cube_aruco_detector_node` - ArUco标记检测
- `world_tag_node` - 世界坐标系定义
- `cube_multi_face_fusion_node` - 多面位姿融合

## 使用方法

### 1. 自动标定流程
运行标定脚本：
```bash
/workspace/scripts/calibration_procedure.sh
```

### 2. 世界-机器人标定流程
使用Python脚本进行标定：
```bash
python3 /workspace/scripts/calibrate_world_robot.py
```

### 3. 手动标定流程
1. 启动所有节点
2. 手动移动机械臂到不同位姿
3. 收集标定数据
4. 执行手眼标定或世界-机器人标定
5. 验证标定结果

### 4. 闭环控制
1. 设置目标位姿
2. 启动闭环控制服务
3. 监控控制过程
4. 验证最终结果

## 参数配置

### 控制参数
- `position_tolerance_`：位置容差 (默认0.01m)
- `orientation_tolerance_`：姿态容差 (默认0.05rad)
- 控制频率：10Hz

### 标定参数
- 数据点数量：建议≥10个（手眼标定）或≥5个（世界-机器人标定）
- 标定板尺寸：根据实际ArUco标记大小设置

## 注意事项

1. **安全第一**：在自动控制前确保工作区域无障碍物
2. **标定精度**：标定质量直接影响控制精度
3. **视觉检测**：确保相机视野内始终有可见的标记
4. **机械臂状态**：确保机械臂处于安全工作状态
5. **坐标系一致性**：确保所有坐标系定义一致

## 故障排除

1. **检测不到标记**：检查相机参数和标记大小设置
2. **控制精度低**：重新执行手眼标定或世界-机器人标定
3. **系统不稳定**：调整控制参数和频率
4. **通信失败**：检查网络连接和IP配置

## 文件结构

```
/workspace/
├── src/
│   ├── calibration_node.cpp              # 标定节点
│   ├── world_robot_calibration_node.cpp  # 世界-机器人标定节点
│   └── closed_loop_control_node.cpp      # 闭环控制节点
├── launch/
│   ├── closed_loop_system.launch         # 系统启动文件
│   └── world_robot_calibration.launch    # 世界-机器人标定启动文件
├── scripts/
│   ├── calibration_procedure.sh          # 标定流程脚本
│   └── calibrate_world_robot.py          # 世界-机器人标定脚本
├── CMakeLists.txt                        # 编译配置
└── README.md                            # 本说明文档
```