# JAKA 多机视觉检测、融合与离线标定工具包

本仓库提供基于 ROS1 Noetic 的多机视觉检测、棱方体多面融合、离线世界—机器人标定采集与求解流程。管线设计遵循“面向 1~4 台机械臂的动态子集”原则：只连 jaka1 也能单独运行；`robot_ids:=[1,2,3,4]` 时缺失的机械臂会被安全跳过并输出告警。

## 运行逻辑总览

1. **检测 (`cube_aruco_detector_node`)**  
   - 自动按 `robot_ids`、`robot_stride=100`、`id_step=10` 生成四个面（0/10/20/30 起步）对应的 GridBoard，并输出 **face_id**（10/11/12/13 + stride）而非面内 marker id。  
   - 世界板使用 2×2 GridBoard，ID=500/501/502/503，原点平移到整个图案的几何中心。
2. **分流 (`fiducial_relay_node`)**  
   - 默认将 world_ids（500~503）转发到 `/world_fiducials`，其余全部透传到 `/tool_fiducials`，不会因缺少白名单丢弃 110/210/...。
3. **世界系 (`world_tag_node`)**  
   - 订阅 `/world_fiducials`，融合 world_ids，发布 `world -> camera` TF，支持多 marker 同时存在。
4. **多面融合 (`cube_multi_face_fusion_node`)**  
   - 每台机械臂启动一份融合节点，过滤自身 face_id 集合（jaka1：10..13；jaka2：110..113；…）。  
   - 输出 `/vision/jakaX/cube_center` (`PoseStamped`) 与 `/vision/jakaX/cube_fusion_stats`，并广播 `cube_center_jakaX` TF。缺失观测时保持运行、保持上一状态。
5. **自动录制 (`world_robot_calib_recorder_node`)**  
   - 仅对选定的机械臂下指令，订阅对应融合主题，按姿态 CSV 采样世界/相机/基座/工具关系，生成离线标定数据集。
6. **离线求解 (`scripts/world_robot_calib_offline.py`)**  
   - 逐机器人读取数据集，假设棱方体坐标系与 tool_frame 重合，使用 `world_cube * inv(base_tool)` 平均求得 world->base，输出 `world_robot_extrinsic_offline_<robot>.yaml`，不会覆盖其他机器人的结果。

## ID 编码与标板规则

- **工具棱方体 face_id（保持 jaka1 旧编号）**  
  - jaka1：face1~4 = 10,11,12,13（面内 marker：0..3,10..13,20..23,30..33）  
  - 其余机械臂整体平移 `robot_stride=100`：jaka2=110..113、jaka3=210..213、jaka4=310..313。
- **世界板**：2×2，marker IDs=500/501/502/503，世界原点位于整块图案几何中心。
- **几何参数**（四面同构，单位 m，边长 0.10 m => face->center 偏移 0.05 m）  
  `+X` [0.05, 0, 0], rpy[90, 0, 90]; `-X` [-0.05, 0, 0], rpy[90, 0, -90];  
  `+Y` [0, 0.05, 0], rpy[90, 0, 180]; `-Y` [0, -0.05, 0], rpy[90, 0, 0]。  
  配置位于 `config/cube_faces_4robots.yaml`。

## 配置文件

- `config/robots.yaml`：4 台机械臂的 id/ip/ns/base_frame/tool_frame/姿态 CSV/默认输出目录与 YAML 路径。
- `config/cube_faces_4robots.yaml`：四台机械臂的 face_id 与几何。
- `config/jaka*_world_robot_calibration_pose.csv`：每台机械臂的标定位姿列表（可在运行时覆盖）。

## 视觉/调试启动

仅视觉链路（多机融合 + 可选调试/RViz）：
```bash
# 可通过 robot_ids 控制生成哪些 face_id，默认加载 robots.yaml 中的 4 台
roslaunch jaka_close_contro cube_fusion_rviz_check.launch robot_ids:="[1,2]"

# 需要逐面误差/CSV 时使用调试版
roslaunch jaka_close_contro cube_fusion_debug.launch robot_ids:="[1,2,3]" output_csv:=/tmp/fusion_dbg.csv
```
- 画面中同时出现 jaka1+jaka2 时，将看到 `/vision/jaka1/cube_center` 与 `/vision/jaka2/cube_center` 同时输出；缺失的机械臂只会提示 warn，不会阻塞。

## 按立方体选择的多面融合 Debug（推荐）

- 编译并加载环境：`catkin build jaka_close_contro` 或 `catkin_make`，然后 `source devel/setup.bash`。  
- 单立方体一键调试（默认使用包内 `config/cube_faces_4robots.yaml`，仅关注该机器人面）
  ```bash
  roslaunch jaka_close_contro cube_fusion_debug_select_cube.launch robot_name:=jaka2
  ```
  - 常用可选参数：`svo_file:=<bag或svo路径>`（离线回放）、`output_csv:=/tmp/jaka2_fusion_debug.csv`、`fused_from_tf:=true`（改用 TF 里的融合结果进行对比）、`print_rate:=5.0`、`stats_window:=50`。
  - 几何单位与边长：`cube_faces_4robots.yaml` 的 translation 以米为单位，当前立方体边长 0.10 m，因此面到中心偏移为 0.05 m。
  - 过滤验证：即便视野同时出现其他机器人 face_id，调试节点日志/CSV 也只会出现对应机器人（如 jaka2）的一组 face_id（110..113），不会混入 10/210/... 的误差统计。

## 自动录制世界—基座标定（逐机器人）

1. **选择机器人与姿态列表**  
   - 确认 `config/robots.yaml` 中的 ip/base_frame/tool_frame/姿态 CSV。  
   - 如需自定义，直接传参 `robot_name`、`robot_id`、`calib_pose_csv`。
2. **启动采集链路（相机 + 棱方体检测 + 分流 + world->camera + 多机融合 + 录制）**  
   ```bash
   # 使用默认包内 CSV，推荐直接运行
   roslaunch jaka_close_contro world_robot_calib_record.launch \
       robot_name:=jaka1 \
       robot_id:=1 \
       robot_ip:=192.168.1.100 \
       robot_ids:="[1]"
   ```
   - 示例：仅录制 jaka1（只启一份融合节点，避免 load_yaml 报错）
     ```bash
     roslaunch jaka_close_contro world_robot_calib_record.launch \
       robot_name:=jaka1 robot_id:=1 robot_ip:=192.168.1.100 robot_ids:="[1]"
     ```
   - 示例：录制 jaka2
     ```bash
     roslaunch jaka_close_contro world_robot_calib_record.launch \
       robot_name:=jaka2 robot_id:=2 robot_ip:=192.168.1.101 robot_ids:="[2]"
     ```
   - 请勿使用 `/abs/path/to/...` 占位符路径；节点会提示并退出，请直接填写真实路径或使用默认包内 CSV。
   - `robot_ids` 控制 detector/fusion 生成哪些 face_id；即便只连 jaka2，其他节点也会“空跑”但不会崩溃。  
   - 速度上限锁定 15%：`speed_scale` > 0.15 会被自动截断并告警。
3. **采样/过滤规则**  
   - 每个姿态在窗口内对 `/vision/jakaX/cube_center` 做均值，并用同一时间戳查询 `base->tool`、`world->camera`。  
   - 若世界板丢失或目标机器人棱方体丢失，样本被拒绝并打印 warn（不写入 CSV）。  
   - 输出 CSV 位于 `dataset_out_dir`（默认 `<pkg>/config`），文件名包含机器人名，例如 `world_robot_calib_dataset_<stamp>_jaka2.csv`。
   - CSV 列包含：`robot_name/robot_id`、`world_cam_*`、`world_cube_*`、`base_tool_*`、面数/内点统计、姿态备注等。
4. **离线求解 world->base（独立文件，互不覆盖）**  
   ```bash
   python3 scripts/world_robot_calib_offline.py --mode world_base --robot_name jaka2 \
       --source_csv <dataset_csv> \
       --output_yaml config/world_robot_extrinsic_offline_jaka2.yaml
   ```
   - 假设棱方体坐标系与 tool_frame 重合，使用 `world_cube * inv(base_tool)` 均值估计 world->base。  
   - 结果写入独立 YAML（含 robot_name/robot_id/dataset 路径），不会覆盖其他机器人文件；`--output_dir` 可批量指定输出目录。
5. **多轮录制**  
   - 重复步骤 1~4 选择不同 `robot_name`/`calib_pose_csv`，即可得到 `world_robot_extrinsic_offline_jaka1.yaml`、`..._jaka2.yaml` 等互不冲突的结果。

## 主要节点与话题对照

- `cube_aruco_detector_node`  
  输入：相机图像/内参。输出：`/fiducial_transforms`（face_id=10/11/.../310/311/...，world_id=500）。参数：`robot_ids`、`robot_stride`、`id_step`、`face_id_base`、世界板 2×2 IDs=500..503。
- `fiducial_relay_node`  
  输入：`/fiducial_transforms`。输出：`/world_fiducials`（默认 500..503）、`/tool_fiducials`（未指定则转发所有非 world）。支持 `robot_ids` 自动生成 tool_ids。
- `world_tag_node`  
  输入：`/world_fiducials`。输出：平滑 `world->camera` TF（默认世界原点在 2×2 图案中心）。
- `cube_multi_face_fusion_node`（多实例）  
  输入：`/tool_fiducials`。输出：`/vision/jakaX/cube_center`、`/vision/jakaX/cube_fusion_stats` 与 TF `cube_center_jakaX`。参数：`robot_id`、`robot_stride`、`face_id_base`、`faces_yaml=config/cube_faces_4robots.yaml`。
- `world_robot_calib_recorder_node`  
  输入：选定机器人融合结果、TF `base->tool`、TF `world->camera`、`/joint_states`。输出：`world_robot_calib_dataset_<stamp>_<robot>.csv`，当缺少世界/棱方体/TF 时拒绝写入并累加丢弃计数。
- `world_robot_calib_offline.py`  
  输入：单机器人 CSV。输出：`world_robot_extrinsic_offline_<robot>.yaml`，包含 world->base 平移+四元数与数据来源路径。

## 多臂位姿级闭环（方案B）使用指南

### (0) 编译与环境准备
```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
roscore  # 如未运行，请先启动
```

### (1) 启动多臂或单臂（统一入口：multi_arm_closed_loop.launch）
- 示例 1：仅启 jaka1+jaka2，允许连接机器人（需要 set_target 才会运动）  
  ```bash
  roslaunch jaka_close_contro multi_arm_closed_loop.launch \
    enable_jaka1:=true enable_jaka2:=true enable_jaka3:=false enable_jaka4:=false \
    connect_jaka1:=true connect_jaka2:=true
  ```
- 示例 2：仅启 jaka2（单臂用法），且不连接机器人（绝对不运动，验证视觉与状态）  
  ```bash
  roslaunch jaka_close_contro multi_arm_closed_loop.launch \
    enable_jaka1:=false enable_jaka2:=true enable_jaka3:=false enable_jaka4:=false \
    connect_jaka2:=false
  ```
- 四臂都启用，但只连接 jaka1+jaka2（其余 DISCONNECTED、不影响）  
  ```bash
  roslaunch jaka_close_contro multi_arm_closed_loop.launch \
    enable_jaka1:=true enable_jaka2:=true enable_jaka3:=true enable_jaka4:=true \
    connect_jaka1:=true connect_jaka2:=true connect_jaka3:=false connect_jaka4:=false
  ```
- 单臂启动即为“仅 enable 一个机器人 + multi_arm_closed_loop.launch”，无需其他 launch。  
- 常用参数覆盖：  
  - 每臂 `calib_yaml`：`calib_yaml_jakaX:=/abs/path/to/world_robot_extrinsic_offline_jakaX.yaml`  
  - 每臂 `cube_pose_topic`：`cube_pose_topic_jakaX:=/custom/jakaX/world_cube`  
  - 控制/阈值：`kp_pos`/`kp_ang`/`v_max`/`w_max_deg`/`eps_pos_m`/`eps_ang_deg` 等 rosparam，可在 launch 覆盖。  
- 前提：不设置目标则不会运动；若想确保绝对不动，使用 `connect_jakaX:=false`。

### (2) 发送目标（world 下 tool/link6 位姿）
- 推荐脚本（service 调用）：  
  ```bash
  rosrun jaka_close_contro send_world_target.py --robot jaka2 \
    --x 0.20 --y -0.10 --z 0.35 --roll_deg 0 --pitch_deg 0 --yaw_deg 90
  rosrun jaka_close_contro send_world_target.py --robot jaka4 \
    --x 0.25 --y -0.05 --z 0.33 --qx 0 --qy 0 --qz 0.7071 --qw 0.7071
  ```
- 直接 service（等价于脚本内部调用）：  
  ```bash
  rosservice call /jaka2/pose_servo_world/set_target "{target: {header: {frame_id: 'world'}, pose: {position: {x: 0.20, y: -0.10, z: 0.35}, orientation: {x: 0.0, y: 0.0, z: 0.7071, w: 0.7071}}}}"
  rosservice call /jaka4/pose_servo_world/set_target "{target: {header: {frame_id: 'world'}, pose: {position: {x: 0.25, y: -0.05, z: 0.33}, orientation: {x: 0.0, y: 0.0, z: 0.7071, w: 0.7071}}}}"
  ```

### (3) 停止 / 清除目标
- stop：立即进入 HOLD/停止更新命令  
  ```bash
  rosservice call /jaka2/pose_servo_world/stop "{}"
  rosservice call /jaka4/pose_servo_world/stop "{}"
  ```
- clear_target：清空目标，回到 IDLE  
  ```bash
  rosservice call /jaka2/pose_servo_world/clear_target "{}"
  rosservice call /jaka4/pose_servo_world/clear_target "{}"
  ```

### (4) 查看状态
- status（字符串：state=RUN/IDLE/HOLD/DISCONNECTED, ep(m), etheta(rad), cube_age(s)）：  
  ```bash
  rostopic echo /jaka2/pose_servo_world/status
  rostopic info /jaka2/pose_servo_world/status
  ```
- err（Twist：linear=ep(m), angular=etheta(rad)）：  
  ```bash
  rostopic echo /jaka2/pose_servo_world/err
  ```
- 如无输出，先 `rostopic list` 检查话题是否存在、namespace 是否正确。

### (5) 查询 cube 在 world 下的位姿（本次新增）
- 直接 service：  
  ```bash
  rosservice call /jaka2/pose_servo_world/get_cube_pose_world "{}"
  ```
- 脚本：  
  ```bash
  rosrun jaka_close_contro get_cube_pose_world.py --robot jaka2
  ```

## 一键闭环（视觉链路 + 多臂驱动 + TF->Pose 桥 + 闭环控制）

> 适用于一次性启动相机、ArUco 棱方体检测、world tag、每臂多面融合、TF→Pose 桥接、驱动以及多臂闭环控制。默认启动视觉链路，可通过 `start_vision:=false` 关闭（用于已单独启动视觉时）。

### (0) 编译与环境
```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
roscore
```

### (1) 一键启动
```bash
roslaunch jaka_close_contro multi_arm_closed_loop.launch \
  enable_jaka1:=true enable_jaka2:=true enable_jaka3:=false enable_jaka4:=false \
  connect_jaka1:=true connect_jaka2:=true \
  start_vision:=true
```

- 关键参数  
  - `start_vision`（默认 `true`）：是否启动相机 + 棱方体检测 + relay + world tag + 多面融合。  
  - `robot_ids`：传递给检测/relay，用于生成 face_id；默认 `[1,2,3,4]`。  
  - `world_frame` / `camera_frame`：视觉链路与 TF→Pose 桥接使用的坐标系。  
  - 每臂 IP：`jaka1_ip`~`jaka4_ip`（默认 192.168.1.100/101/102/103）。  
  - `strict_cube_frame`：闭环节点是否只接受 `world_frame` 的 Pose（默认 `true`）。  
  - 驱动 URDF：`urdf_file`。  
  - 控制参数：`control_rate_hz`、`kp_pos`、`kp_ang`、`v_max`、`w_max_deg`、`eps_pos_m`、`eps_ang_deg`、`vision_timeout_s`、`servo_timeout_s`。  
  - 闭环订阅：`/vision/jakaX/cube_center_world`（由 TF→Pose 桥接节点发布，强制填充时间戳）。  
  - 驱动服务名：默认相对名 `jaka_driver/linear_move`，在 namespace 下自动变为 `/jakaX/jaka_driver/linear_move`，避免多臂冲突。

### (2) 验证视觉输出与闭环输入
```bash
rostopic info /vision/jaka1/cube_center_world | grep Publishers
rostopic info /vision/jaka2/cube_center_world | grep Publishers
```
应看到非 None 的 Publisher（来自 `tf_to_pose_stamped_node`）。

### (3) 查询闭环输入状态
```bash
rosservice call /jaka1/pose_servo_world/get_cube_pose_world "{}"
rosservice call /jaka2/pose_servo_world/get_cube_pose_world "{}"
```
- 正常输出 `ok: True`，`age_sec` 为正数且小于 `vision_timeout_s`，`pose.header.frame_id=world` 且 `stamp` 非零（无 `/clock` 时使用 wall time 计算超时）。

### (4) 命名空间与驱动隔离
- 驱动服务路径为 `/jakaX/jaka_driver/linear_move`，两臂不会冲突。  
- 闭环节点自动在各自 namespace 下工作（`cube_pose_topic`、service、status/err 话题均带 `jakaX/` 前缀）。
- 返回字段：`ok` / `message` / `pose`（PoseStamped，frame_id=world）/ `age_sec`（当前时间与 pose 时间差，s）。若未收到或超时则 `ok=false` 并给出提示。

### (6) 安全与注意事项
- 只有 `connect_jakaX:=true` 且设置了目标时才会运动；`connect=false` 可确保绝对不运动。  
- 视觉超时（`vision_timeout_s`）自动 HOLD；`servo_timeout_s` 超时未收敛也会 HOLD 并告警。  
- 所有内部计算使用米/弧度，下发前已转换为 driver 所需 mm/弧度；`w_max_deg`/`eps_ang_deg` 会自动转为弧度。  
- 目标与视觉输入必须是 world frame（frame_id=world），否则会被拒绝或导致异常。  
- 每臂的 calib_yaml/cube_pose_topic 可独立覆盖；日志会打印加载的外参与订阅 topic。

## 兼容性与降级

- `robot_ids` 可自由裁剪，未出现的机器人只会输出 warn，不会终止节点。  
- 仅连接 jaka1 时，`world_robot_calib_record.launch`/`cube_fusion_*` 仍可正常运行；多机器人画面同时出现时会得到各自的 `/vision/jakaX/cube_center`。  
- `fiducial_relay_node` 默认放行所有非 world_id，因此不会因未配置 tool_ids 而丢失 110/210/... face_id。
