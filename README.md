# JAKA 机械臂视觉伺服闭环控制系统

## 系统概述

本项目基于 ROS1 Noetic，为 JAKA Zu3 提供完整的视觉伺服链路：

- **视觉检测管线**：多尺寸 ArUco 检测 → 世界大板滤波 → 末端正方体多面融合。
- **世界-机器人标定**：直接求解 `world -> robot_base` 的静态外参，不再需要传统相机-末端手眼标定。
- **闭环控制**：利用实时 `world -> camera` TF 与已标定的 `world -> base` 变换，计算 `base -> cube` 后驱动机械臂闭环到达目标位姿。
- **驱动与 TF**：官方 URDF 通过 `robot_state_publisher` 发布 TF，依赖 JAKA SDK 的 `/joint_states`。

## 官方 URDF 与 TF 发布

- 仓库包含官方 **JAKA Zu3** URDF（`urdf/jaka_zu3.urdf`）。
- `launch/jaka_sdk_bringup.launch` 会启动 SDK 驱动与 `robot_state_publisher`，默认帧：`Link_0`（基座）、`Link_6`（法兰）。如需自定义，可在 launch 参数中覆盖。

## 快速上手：一键世界-机器人标定

1. 启动完整标定链路（相机、检测、融合、标定）：
   ```bash
   roslaunch jaka_close_contro world_robot_calibration.launch
   ```
2. 手动移动机械臂，让末端正方体（法兰 +Z 方向 77mm 处）在相机视野内采集多组姿态，调用采样服务：
   ```bash
   rosservice call /collect_world_robot_sample "{}"
   ```
3. 样本达到阈值（默认 ≥10）后求解外参：
   ```bash
   rosservice call /solve_world_robot_calibration "{}"
   ```
   - 结果保存到 `config/world_robot_extrinsic.yaml`（可通过 `~output_yaml` 覆盖）。
   - 日志会打印 static_transform_publisher 命令示例，可在其他 launch 中固化 TF。
   - 若 `~publish_tf` 为 true，节点会自动周期性发布 `world -> robot_base`。

## 快速上手：自动轨迹 + 自动采样标定

无需手动移动机械臂，直接运行预定义标定位姿（TCP 笛卡尔位姿）并自动采样/求解：

1. 启动全自动标定管线：
   ```bash
   roslaunch jaka_close_contro world_robot_autocalib_motion.launch
   ```
   - 启动内容：驱动 + TF、ZED2i 相机、Aruco 检测/分流、世界板滤波、多面融合、世界-机器人标定节点、自动采样管理节点、自动标定轨迹执行节点。
   - 轨迹文件使用 `config/jaka1_world_robot_calibration_pose.csv`，字段为 `name,x_mm,y_mm,z_mm,rx_deg,ry_deg,rz_deg`，单位分别为 mm / deg（基座坐标系下的 TCP 位姿）。
   - `world_robot_calib_motion_node` 通过 `/jaka_driver/linear_move` 调用官方驱动的笛卡尔直线运动接口，默认速度倍率 15%，线速度 80 mm/s、线加速度 200 mm/s²。到达每个姿态后自动**等待 5 s（采样前）+ 触发采样 + 再等待 2 s**，确保视觉稳定再切换下一个点。
2. 节点会按 CSV 顺序依次发送目标 TCP 位姿，运动过程中自动判断位姿变化并调用 `/collect_world_robot_sample`（或由自动管理节点触发）。
3. 样本数量达到 `min_samples`（默认 16）后，管理节点自动调用 `/solve_world_robot_calibration`，在终端打印残差统计并写入 `config/world_robot_extrinsic.yaml`。
4. 标定完成后即可直接切换到闭环观察/控制（`world_robot_closedloop.launch`），无需再次求解外参。

## 快速上手：一键闭环观察/控制

闭环阶段直接使用标定生成的 `config/world_robot_extrinsic.yaml`，无需再次求解外参。推荐流程：

1. **准备外参**：先按上一节完成世界-机器人标定，确认 `config/world_robot_extrinsic.yaml` 已生成且数值正确。
2. **一键启动闭环管线**：
   ```bash
   roslaunch jaka_close_contro world_robot_closedloop.launch
   ```
   - 启动内容：机器人驱动 + TF、ZED2i 相机、多尺寸 ArUco 检测与分流、世界大板滤波、多面融合（只发 camera->cube TF）、外参广播器（从 YAML 发布 world->Link_0）、闭环观测/控制节点。
   - 如需修改机器人 IP 或 URDF，使用 `ip`、`urdf_file` 参数覆盖。
3. **观察世界系下的正方体位姿**：`cube_world_closedloop_node` 会在终端持续打印世界坐标系下 `cube_center` 的位置与四元数，可用来监测闭环输入。
4. **设置目标（可选）**：可在 launch 中添加 `target_world_x/y/z` 与 `target_world_qx/qy/qz/qw` 参数，节点会计算对应的 `base` 系末端目标姿态（当前示例仅打印，可按需求对接控制接口）。
5. **控制接口集成**：保持 `cube_multi_face_fusion_node` 的 `publish_tool_tf=false`，避免覆盖 URDF 的 `Link_6` TF；根据需要在 `cube_world_closedloop_node` 中补充具体的 JAKA 控制调用，即可实现基于世界坐标系的闭环移动。

## 世界-机器人标定数据链路

- **视觉测量**：
  - `world_tag_node` 平滑输出 `world -> camera`。
  - `cube_multi_face_fusion_node` 输出 `camera -> cube_center`（`/cube_center_fused`）。
  - 计算得到 `world -> cube_center(meas) = (world -> camera) * (camera -> cube_center)`。
- **机器人预测**：
  - 从 TF 读取 `robot_base -> tool`。
  - 结合固定偏移 `tool -> cube_center`（默认 +Z 0.077 m）得到 `robot_base -> cube_center(pred)`。
- **求解**：
  - 收集若干对 `cube_center` 在 base 系与 world 系下的位姿，使用 SVD/四元数平均拟合刚体变换，得到 `world -> robot_base`。

## 主要节点与话题

- `cube_aruco_detector_node`
  - 输入：`/zed2i/zed_node/left/image_rect_color`、`/zed2i/zed_node/left/camera_info`
  - 输出：`/fiducial_transforms`
- `fiducial_relay_node`
  - 输入：`/fiducial_transforms`
  - 输出：`/world_fiducials`（ID=0），`/tool_fiducials`（ID=10/11/12/13）
- `world_tag_node`
  - 输入：`/world_fiducials`
  - 输出：平滑 TF `world -> zed2i_left_camera_optical_frame`
- `cube_multi_face_fusion_node`
  - 输入：`/tool_fiducials`
  - 输出：`/cube_center_fused`（PoseStamped，frame_id 为相机光学系）
- `world_robot_calibration_node`
  - 输入：`/cube_center_fused`，TF 查询 `world -> camera`、`robot_base -> tool`
  - 服务：`/collect_world_robot_sample`、`/solve_world_robot_calibration`
  - 输出：标定结果 YAML，可选 TF 广播 `world -> robot_base`
- `closed_loop_control_node`
  - 使用 `world -> base` 标定与实时 `world -> camera` TF 计算 `base -> cube`，并调用 `/jaka_driver/joint_move` 进行闭环控制。

## 使用提示

- 世界基准板：id=0、边长 80mm 的平面 ArUco，固定不动并始终在相机视野内。
- 末端正方体：法兰坐标系 +Z 方向 77mm 处固定五面标签，偏移可通过 `~cube_offset_z_m` 或 `~cube_offset_xyz_m` 调整。
- 建议采集时覆盖足够多姿态与空间分布，提升拟合稳定性。
- 标定结果保存在 YAML 中，重启后无需重新标定（除非硬件装配变化）。

## 融合与外参自检实验工具

为快速验证几何与外参质量，提供三组可复现实验：

1. **逐面反推 vs 融合残差（cube_fusion_debug）**
   - 启动：`roslaunch jaka_close_contro cube_fusion_debug.launch`
   - 功能：逐个 tag 反算 `camera->cube_center` 并与融合结果对比，输出每面误差与 mean/rms/max，支持 CSV。
   - 通过阈值（静止场景）：`max(pos_err) < 0.01 m` 且 `max(ang_err) < 5 deg`。若出现 0.02~0.03 m 或 10~20 deg 需检查 faces_yaml 或坐标约定。
2. **RViz 几何一致性检查**
   - 启动：`roslaunch jaka_close_contro cube_fusion_rviz_check.launch`
   - 内容：仅启动相机 + ArUco + 世界板 + 融合（可选 debug 节点），自动加载 `rviz/cube_fusion_debug.rviz` 查看 `cube_center` 与各面预测位置。
   - 判定：肉眼观察 `cube_center` 是否落在立方体中心附近、不会穿出立方体或明显偏离。
3. **机器人理论 cube vs 视觉 cube 残差（cube_nominal_compare）**
   - 启动：`roslaunch jaka_close_contro cube_nominal_compare.launch`
   - 逻辑：用 TF 查询 `base->tool`，结合 `tool_to_cube`（支持平移+旋转）得到理论 `base->cube`，与视觉 `base->cube` 对比，输出 pos/ang 的 mean/rms/max，可写 CSV，发布 `/cube_center_nominal` 供 RViz。
   - 预期：静止时 pos_err 约毫米~1cm 量级，ang_err 不应出现几十度系统偏差；若异常优先检查 `tool_to_cube` 旋转或 faces_yaml 坐标定义。

`tool_to_cube` 约定：如果立方体中心位于工具 +Z 方向 0.077 m，则 `T_tool_cube.tz = +0.077`，`T_cube_tool.tz = -0.077`；不要混用两者方向。

## 文件结构

```
├── launch/
│   ├── world_robot_calibration.launch   # 一键世界-机器人标定
│   ├── world_robot_closedloop.launch    # 一键闭环观察/控制
│   ├── world_robot_autocalib_motion.launch # 自动轨迹 + 自动采样标定
│   ├── cube_fusion_debug.launch         # 实验1：逐面 vs 融合残差
│   ├── cube_fusion_rviz_check.launch    # 实验2：RViz 几何检查
│   ├── cube_nominal_compare.launch      # 实验3：机器人几何 vs 视觉残差
│   ├── world_robot_selfcheck.launch     # 仅自检视觉/外参 TF 链
│   ├── hand_eye_calibration.launch      # 兼容入口，内部直接 include 新标定
│   ├── closed_loop_system.launch        # 示例系统启动，已移除旧手眼节点
│   └── jaka_sdk_bringup.launch
├── src/
│   ├── cube_aruco_detector_node.cpp
│   ├── fiducial_relay_node.cpp
│   ├── world_tag_node.cpp
│   ├── cube_multi_face_fusion_node.cpp
│   ├── world_robot_calibration_node.cpp
│   ├── world_robot_autocalib_manager_node.cpp
│   ├── world_robot_calib_motion_node.cpp
│   ├── world_robot_extrinsic_broadcaster_node.cpp
│   ├── cube_world_closedloop_node.cpp
│   ├── cube_fusion_debug_node.cpp
│   ├── cube_nominal_compare_node.cpp
│   └── closed_loop_control_node.cpp
├── srv/
│   └── WorldRobotCalibration.srv
├── config/
│   ├── cube_faces_current.yaml
│   ├── cube_faces_ideal.yaml
│   └── jaka1_world_robot_calibration_pose.csv   # arm1 标定轨迹（可扩展 arm2/arm3/arm4）
├── rviz/
│   └── cube_fusion_debug.rviz                   # RViz 默认视角，用于检查融合几何
└── urdf/
    └── jaka_zu3.urdf
```

## 源码节点说明（src/*.cpp）

- **cube_aruco_detector_node.cpp**：启动 fiducial_ros 包的 ArUco 检测器，订阅相机图像与相机内参，发布 `/fiducial_transforms` 供后续分流使用。输入：彩色图像、CameraInfo；输出：`fiducial_transforms`。
- **fiducial_relay_node.cpp**：根据 ID 将 ArUco 检测结果分流到世界板与末端立方体两路；输入：`/fiducial_transforms`；输出：`/world_fiducials`（ID0 大板）与 `/tool_fiducials`（立方体多面）。
- **world_tag_node.cpp**：对世界板位姿进行滤波与发布，输入 `/world_fiducials`，输出平滑 TF `world -> camera`（frame 由相机光学系决定）。
- **cube_multi_face_fusion_node.cpp**：对末端立方体多面 ArUco 进行鲁棒融合与时间滤波，输入 `/tool_fiducials`，输出 `/cube_center_fused` PoseStamped（frame 为相机光学系），并可选发布 `camera -> cube_center` TF。
- **world_robot_calibration_node.cpp**：采集世界/机器人样本并执行 Ceres 联合标定，同时估计 `world -> base` 与 `flange -> cube_center`，提供服务 `/collect_world_robot_sample`、`/solve_world_robot_calibration`，输出 YAML 与可选 TF。
- **world_robot_autocalib_manager_node.cpp**：管理自动采样流程与保存标定结果。输入：采样/求解服务结果，参数：CSV 模板与输出路径；输出：更新 `config/world_robot_extrinsic.yaml` 与 `config/cube_faces_current.yaml`，并维护采样状态。
- **world_robot_calib_motion_node.cpp**：按 CSV 轨迹驱动机器人做自动标定位姿，调用 `/jaka_driver/linear_move` 发送笛卡尔轨迹；输入：轨迹 CSV；输出：机器人运动和采样触发。
- **world_robot_extrinsic_broadcaster_node.cpp**：从 YAML 读取 `world -> base` 外参并周期性广播 TF，供闭环/调试使用。
- **cube_world_closedloop_node.cpp**：在闭环阶段计算 `base -> cube` 目标与观察值，打印或发布用于控制的位姿；输入：`world -> camera`、`world -> base` TF 与 `/cube_center_fused`。
- **cube_fusion_debug_node.cpp**：调试多面融合质量，逐面反推 `cube_center` 与融合结果对比，输出误差统计与可选 CSV。
- **cube_nominal_compare_node.cpp**：对比机器人理论 `base -> cube` 与视觉测量，输出 pos/ang 残差统计，并可发布 `/cube_center_nominal` 供 RViz。
- **closed_loop_control_node.cpp**：示例闭环控制器，结合已标定外参与实时视觉，计算并调用 `/jaka_driver/joint_move` 或自定义接口实现闭环移动。

