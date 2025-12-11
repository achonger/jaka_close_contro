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

## 文件结构

```
├── launch/
│   ├── world_robot_calibration.launch   # 一键世界-机器人标定
│   ├── world_robot_closedloop.launch    # 一键闭环观察/控制
│   ├── hand_eye_calibration.launch      # 兼容入口，内部直接 include 新标定
│   ├── closed_loop_system.launch        # 示例系统启动，已移除旧手眼节点
│   └── jaka_sdk_bringup.launch
├── src/
│   ├── cube_aruco_detector_node.cpp
│   ├── fiducial_relay_node.cpp
│   ├── world_tag_node.cpp
│   ├── cube_multi_face_fusion_node.cpp
│   ├── world_robot_calibration_node.cpp
│   ├── world_robot_extrinsic_broadcaster_node.cpp
│   ├── cube_world_closedloop_node.cpp
│   └── closed_loop_control_node.cpp
├── srv/
│   └── WorldRobotCalibration.srv
├── config/
│   ├── cube_faces_current.yaml
│   └── cube_faces_ideal.yaml
└── urdf/
    └── jaka_zu3.urdf
```

