# JAKA 视觉采集与离线标定工具包

本仓库提供基于 ROS1 Noetic 的 JAKA Zu3 视觉检测与标定数据录制管线。核心特点：

- **多面融合**：`cube_multi_face_fusion_node` 读取 ArUco 棱方体各面的 YAML 几何，输出相机系下的 `/cube_center_fused` (`PoseStamped`)，默认不广播 TF、也不依赖工具外参。
- **同步采样录制**：`world_robot_calib_recorder_node` 按 CSV 轨迹驱动机械臂，使用融合结果的时间戳查 `base->tool` TF，生成离线标定数据集，遇到无时间戳/TF 不可用的样本会直接丢弃并计数。
- **离线求解入口**：`scripts/world_robot_calib_offline.py` 保留了世界-机器人/工具外参离线求解的模板（需自行填入算法）。
- **官方 URDF**：`launch/jaka_sdk_bringup.launch` 启动 JAKA SDK 驱动与 `robot_state_publisher`，默认帧 `Link_0`（基座）、`Link_6`（法兰）。

## 快速上手

1. **启动录制链路（采集，不在线解算）**
   ```bash
   roslaunch jaka_close_contro world_robot_calib_record.launch
   ```
   - 输出数据：`config/world_robot_calib_dataset_<time>.csv`（可用 `~output_dataset_csv` 覆盖）。
   - 节点依次运动到 `config/jaka1_world_robot_calibration_pose.csv` 中的姿态，在稳定窗口内对 `/cube_center_fused` 做李群均值，并用同一时间戳查询 `base->tool` TF；若视觉时间戳缺失或 TF 查询失败则跳过该样本并累加丢弃计数。

2. **离线求解（模板）**
   ```bash
   rosrun jaka_close_contro world_robot_calib_offline.py \
       --input config/world_robot_calib_dataset_xxx.csv \
       --output-world-robot config/world_robot_extrinsic.yaml \
       --output-tool-offset config/tool_offset_current.yaml
   ```
   脚本仅是占位符，需要自行实现手眼 / 世界-机器人外参求解，再写回 YAML。

3. **多面融合 Debug 一键启动**
   ```bash
   roslaunch jaka_close_contro cube_fusion_debug.launch
   ```
   操作步骤：
   - 启动后会依次拉起 ZED2i、`cube_aruco_detector_node`、`fiducial_relay_node`、`cube_multi_face_fusion_node`、`cube_fusion_debug_node`。
   - 在终端查看融合输出 `/cube_center_fused`，以及 `cube_fusion_debug_node` 打印的逐面误差统计。
   - 若需保存 CSV，可在 launch 中设置 `output_csv` 参数；默认输出到 `config/cube_fusion_debug.csv`。
   - 若需要从 TF 获取融合位姿，可将 `fused_from_tf` 置为 `true`，并设置 `fused_frame` 为对应 TF 子坐标系名。

## 主要节点与话题

- `cube_aruco_detector_node`
  - 输入：图像与相机内参（示例使用 ZED2i 左目话题）。
  - 输出：`/fiducial_transforms`。
- `fiducial_relay_node`
  - 输入：`/fiducial_transforms`。
  - 输出：`/world_fiducials`（ID=0）、`/tool_fiducials`（ID=10/11/12/13）。
- `world_tag_node`
  - 输入：`/world_fiducials`。
  - 输出：平滑的 `world -> camera` TF。
- `cube_multi_face_fusion_node`
  - 输入：`/tool_fiducials`；参数：`faces_yaml`（默认 `config/cube_faces_current.yaml`）。
  - 输出：`/cube_center_fused` (`PoseStamped`，frame_id 为相机光学系) 及融合统计 `/cube_fusion_stats`。不再广播 TF，也不依赖 `tool_offset`。
- `world_robot_calib_recorder_node`
  - 输入：`/cube_center_fused`、`/cube_fusion_stats`、TF `base->tool`、`/joint_states`；参数：轨迹 CSV、采样窗口、TF 查询超时等。
  - 输出：离线标定数据集 CSV（按融合时间戳查询 TF，同步写入）。
- `cube_fusion_debug_node`
  - 功能：逐面反推 `camera->cube_center`、与融合结果对比，发布调试 TF（tag、per-face cube、fused）并可写 CSV。

## 配置文件

- `config/cube_faces_current.yaml`：立方体各面几何（`id`、`translation`、`rpy_deg`）。
- `config/tool_offset_current.yaml`：保留工具外参输出模板，融合节点不再依赖该文件。
- `config/jaka1_world_robot_calibration_pose.csv`：录制用的末端笛卡尔姿态列表。

## 注意事项

- 融合节点不广播 TF，订阅方请直接使用 `/cube_center_fused` 的 `PoseStamped` 数据。
- 录制节点严格使用融合消息的时间戳查 TF，`lookupTransform` 设置了 `~tf_lookup_timeout_sec`（默认 0.1 s）；时间戳缺失或 TF 不可用的样本会被丢弃并计数。
- 仓库已移除“一键自动标定 / 闭环控制”相关节点与 launch，仅保留检测、融合、录制与调试功能。
