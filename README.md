# JAKA 视觉伺服与世界-机器人标定

本仓库提供 1~4 台 JAKA 机械臂的视觉链路、世界板定义、末端立方体多面融合以及世界→机器人离线标定工具链。所有组件在只连接部分机械臂时均可降级运行（缺失的机器人仅打印警告，不会阻塞节点）。

## 关键规则与配置

- **统一机器人注册表**：`config/robots.yaml` 列出 4 台机械臂的 `name/id/ns/base_frame/tool_frame/ip`。Launch/脚本会据此自动找到 TF 名称与 service 名称。
- **Cube 面 ID 规则**（保持 jaka1 旧编号）：
  - jaka1：face1~4 → 10,11,12,13（面内 marker 0..3,10..13,20..23,30..33）
  - 其余机械臂整体偏移 `robot_stride=100`：jaka2 110..113，jaka3 210..213，jaka4 310..313。
- **世界板 GridBoard**：2x2、marker id 500/501/502/503，世界坐标系原点为整块棋盘的几何中心。
- **默认 TF/帧**：
  - 世界：`world`，相机：`zed2i_left_camera_optical_frame`
  - 立方体中心 TF：`cube_center_<robot_name>`，数据话题：`/vision/<robot_name>/cube_center`
  - 融合节点附带发布工具虚拟帧：`vision_tool_<robot_name>`（来自 cube_center + tool_offset）

## 运行逻辑（节点/话题/TF）

1. **ArUco 检测 `cube_aruco_detector_node`**
   - 输入：相机图像 + 相机内参
   - 自动为每个 `robot_ids` 生成 4 个 GridBoard：面内 marker 遵循 0/10/20/30 偏移 + `robot_stride`，输出 fiducial_id=face_id（10..13/110..113/...）；同时输出单 marker TF（便于调试）。
   - 世界板：识别 id 500/501/502/503 组成的 2x2 Board，输出 fiducial_id=500，对应原点即世界中心。
   - 输出话题：`/fiducial_transforms`（包含单 marker 与 face_id/世界板 TF）。

2. **话题分流 `fiducial_relay_node`**
   - world_ids 默认 `[500,501,502,503]`，全部转发到 `/world_fiducials`；其他 fiducial 默认全放行到 `/tool_fiducials`（如需白名单，可设置 `generate_tool_ids=true` 自动生成 10/11/12/13 + stride）。

3. **世界板 TF `world_tag_node`**
   - 订阅 `/world_fiducials`，优先使用 fiducial_id=500（GridBoard 中心），缺失时回退 500/501/502/503 任意一枚。
   - 发布 TF：`world` → `zed2i_left_camera_optical_frame`，支持平滑滤波。

4. **多面融合 `cube_multi_face_fusion_node`**
   - 读取 `config/cube_faces_4robots.yaml` 的 face→cube 几何，按 `robot_ids` 将 fiducial_id 映射到机器人实例。
   - 每个机器人独立滤波与优化，输出：
     - 话题 `/vision/<robot_name>/cube_center` (`geometry_msgs/PoseStamped`)
     - TF：`<camera_frame>` → `cube_center_<robot_name>`
     - TF：`<camera_frame>` → `vision_tool_<robot_name>`（cube_center 加 tool_offset）
   - 即使某机器人面缺失也不会报错，仅跳过输出。

## 启动示例

### 仅视觉链路（多机器人并存，支持 500 世界板）
```bash
roslaunch jaka_close_contro full_system.launch robot_ids:="[1,2]"
```
- 仅看到 jaka1：输出 `/vision/jaka1/cube_center` 与 TF `cube_center_jaka1`
- 同时看到 jaka1+jaka2：两路输出并存，不互相覆盖
- 未连接的机器人不会阻塞，相关输出缺失时仅有 warn。

### 快速闭环/调试（复用同一参数）
```bash
roslaunch jaka_close_contro closed_loop_system.launch robot_ids:="[1]"
```
（如需多台机械臂闭环，可复制/修改 bringup include，让不同 IP 的 driver 启动在各自 namespace）。

## 自动录制世界→基座标定数据集

### 录制前准备
1. 启动视觉链路，确保能看到：
   - 世界板（id 500/501/502/503）
   - 目标机械臂的立方体面（对应 face_id 段）
2. 确认 TF 树存在 `Link_0`→`Link_6`（或 robots.yaml 中配置的 base/tool）。
3. 准备姿态 CSV（每行 6 个关节值，逗号分隔）。

### 录制命令
```bash
# 先启动视觉/TF
roslaunch jaka_close_contro full_system.launch robot_ids:="[1,2]"

# 录制 jaka2，速度≤15%，结果保存在 data/world_robot_calib_dataset_jaka2_*.csv
python3 scripts/record_world_robot_dataset.py \
  --robot-id 2 \
  --pose-csv config/poses_jaka2.csv \
  --out-dir data \
  --robots-config config/robots.yaml \
  --world-frame world \
  --camera-frame zed2i_left_camera_optical_frame
```
- `robot-id`/`robot-name` 任选其一指定目标机械臂
- `cube_frame_prefix` 默认 `cube_center_`，会自动拼接机器人名
- 录制过程中若世界板或 cube_center 丢失，当前样本会被跳过并打印 warn（不会写入 CSV）。

### 数据集字段（每行）
- `world_cam_*`：world→camera 变换
- `world_cube_*`：world→cube_center_<robot> 变换（来自融合输出）
- `base_tool_*`：base→tool 变换（来自 TF）
- 额外元信息：robot_id/robot_name/pose_idx/各帧名

## 离线求解 world→Link_0 外参

```bash
python3 scripts/offline_calibrate_world_robot.py data/world_robot_calib_dataset_jaka2_XXXX.csv \
  --robot-name jaka2 \
  --output-dir data
```
- 默认使用 tool_offset 平移 `[0,0,0.077]`，如需调整可传 `--tool-offset x y z`
- 输出 `data/world_robot_extrinsic_offline_jaka2.yaml`，不会覆盖其他机器人结果
- 支持多机器人各自独立运行，dataset/结果互不冲突

## 常见 ID/文件速查
- 机器人注册表：`config/robots.yaml`
- Cube 面几何：`config/cube_faces_4robots.yaml`
- 世界板 ID：500/501/502/503（原点在棋盘中心）
- 融合输出话题：`/vision/<robot>/cube_center`
- 融合 TF：`cube_center_<robot>`、`vision_tool_<robot>`

## 开发备注
- `cube_aruco_detector_node` 支持旧 `tool_gridboards` 参数：如显式提供则优先使用，否则按 `robot_ids`/stride 自动生成。
- `fiducial_relay_node` 默认全放行非 world_id；如需白名单，将 `generate_tool_ids` 设为 true 即可按 robot_ids 自动填充 10/11/12/13 + 偏移。
- 若仅有 jaka1 连接，其余机器人缺席不会造成节点退出；所有 TF/话题按可见信息独立产出。
