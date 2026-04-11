# README_autocalib_linear_move_analysis

> 范围声明：本报告**仅**聚焦自动标定链路（`world_robot_calib_recorder_node.cpp`）与其直线运动调用路径，不展开到无关模块。分析方式为静态代码阅读（未做设备联调）。

---

## 1. 功能概述

### 1.1 自动标定功能目的
自动标定节点 `world_robot_calib_recorder_node` 的目标是：
1. 按预设标定位姿序列驱动机械臂运动；
2. 在每个位姿稳定后，采集视觉融合结果（`cam->cube`）与机器人/TF状态（`base->tool`、`world->camera`）；
3. 将采样窗口的统计结果写入数据集 CSV，供后续离线外参求解使用。

从实现上看，这是一条“**运动 -> 静止判定 -> 采样 -> 写数据集**”的流水链。

### 1.2 在项目中的角色
它是“世界坐标-机器人标定”中的**数据录制端**（recorder），并不直接求解外参；求解通常交给离线脚本（例如历史提交中出现的 `scripts/world_robot_calib_offline.py`，本报告不展开脚本算法）。

### 1.3 与机械臂直线运动关系
关系是直接且核心的：每个标定点通过 `jaka_msgs/Move` 服务请求发到 `linear_move` 服务；位置/姿态目标来自 CSV，速度与加速度来自参数。

### 1.4 是否通过 linear_move 实现直线运动
是。节点参数 `linear_move_service` 默认值为 `jaka_driver/linear_move`，并使用 `ros::ServiceClient<jaka_msgs::Move>` 调用。调用函数是 `sendLinearTarget()`。

---

## 2. 入口与启动链路

### 2.1 入口 node 与源码
- 入口 node：`world_robot_calib_recorder_node`
- 源码路径：`src/world_robot_calib_recorder_node.cpp`
- CMake 注册：`add_executable(world_robot_calib_recorder_node src/world_robot_calib_recorder_node.cpp)`

### 2.2 主要 launch
- 直接启动它的 launch：`launch/world_robot_calib_record.launch`
- 该 launch 在“步骤4”中启动 recorder node，并传入标定相关参数。

### 2.3 启动调用链（按文件）

```text
launch/world_robot_calib_record.launch
  -> include launch/jaka_sdk_bringup.launch
      -> 启动外部包节点 jaka_driver/jaka_driver (提供 /jaka_driver/linear_move 等)
      -> 启动 src/jaka_state_adapter_node.cpp (jaka_driver/joint_position -> /joint_states)
      -> 启动 robot_state_publisher (URDF + /joint_states 发布 TF)
  -> 启动视觉链节点（cube_aruco_detector_node / fiducial_relay_node / world_tag_node / cube_multi_face_fusion_node）
      -> 发布 /vision/<robot>/cube_center 与 /vision/<robot>/cube_fusion_stats
  -> 启动 world_robot_calib_recorder_node
      -> 读取 calib_pose_csv
      -> 调用 linear_move service
      -> 订阅 joint_states/fused_topic/stats_topic
      -> 采样并写 output_dataset_csv
```

### 2.4 shell 脚本参与情况
- 在当前仓库中未发现专门用于启动该自动标定链的 shell 启动脚本（静态检索结果）。
- 若现场有外部 orchestration 脚本（如上位机启动脚本），当前仓库无法直接确认。

### 2.5 找不到/需继续检查项
- `jaka_driver/linear_move` 的服务实现不在本仓库（应在外部 `jaka_driver` 包）。
- 若需完整闭环到 SDK API，需继续检查工作空间中的外部包：`jaka_driver`、`jaka_msgs`、`jaka_sdk_driver`。

---

## 3. world_robot_calib_recorder_node.cpp 的直线运动实现

> 本节只展开与 linear_move 直接相关逻辑。

### 3.1 参数读取（与线性运动直接相关）
- `linear_move_service`（默认 `jaka_driver/linear_move`）
- `use_driver`（默认 true）
- `do_motion`（默认 true）
- `speed_scale`（默认读取 0.15，并被上限钳制到 0.15）
- `linear_speed_mm_s`（默认 80.0）
- `linear_acc_mm_s2`（默认 200.0）
- `motion_done_timeout_sec`（默认 120.0）
- `motion_stable_duration_sec`（默认 0.5）
- `motion_joint_threshold_rad`（默认 0.002）
- `joint_state_topic`（默认 `/joint_states`）

### 3.2 标定位姿来源
- 来源参数：`calib_pose_csv`（可被 `pose_csv` 覆盖）。
- 默认路径构造：`ros::package::getPath("jaka_close_contro") + /config/jaka1_world_robot_calibration_pose.csv`。
- launch 中实际常用：`$(find jaka_close_contro)/config/$(arg robot_name)_world_robot_calibration_pose.csv`。

### 3.3 位姿数据格式 / 单位
CSV 列格式严格 7 列：
`name,x_mm,y_mm,z_mm,rx_deg,ry_deg,rz_deg`

单位：
- 位置：mm
- 姿态：RPY 角度（deg）

### 3.4 Move 请求构造细节
在 `sendLinearTarget(const CalibPoseRow&)` 中：

1. `request.pose` 六维向量：
   - `[0] = x_mm`
   - `[1] = y_mm`
   - `[2] = z_mm`
   - `[3] = rx_deg * DEG2RAD`
   - `[4] = ry_deg * DEG2RAD`
   - `[5] = rz_deg * DEG2RAD`

2. 运动学参数：
   - `mvvelo = linear_speed_mm_s * speed_scale`
   - `mvacc = linear_acc_mm_s2 * speed_scale`
   - `mvtime = 0.0`
   - `mvradii = 0.0`
   - `coord_mode = 0`
   - `index = target_index_`

3. `has_ref / ref_joint`：
   - **未设置**（说明 recorder 侧未显式使用参考关节字段；最终是否被 driver 使用取决于 driver 实现）。

4. service 名称：
   - 来自参数 `linear_move_service`，默认 `jaka_driver/linear_move`。
   - launch 常在全局命名空间下以 `/jaka_driver/linear_move` 暴露（相对/绝对名解析视运行命名空间而定）。

### 3.5 每次发送后是否等待机器人停止
是。
- `executeTrajectory()` 中发送成功后调用 `waitRobotMotionDone(motion_done_timeout_sec_)`。
- 停止判据来自 `/joint_states`：
  - 在 `jointStateCallback()` 对比当前帧与上一帧关节差值；
  - 若任一关节差值超过 `motion_joint_threshold_rad`，更新 `last_motion_time_`；
  - `waitRobotMotionDone()` 持续检查“距最后运动时间”是否超过 `motion_stable_duration_sec_` 且稳定持续满足，才判定“停止”。

### 3.6 自动标定中“运动/采样”交替机制
单个位姿流程：
1. `sendLinearTarget()` 发 linear_move；
2. `waitRobotMotionDone()` 等静止；
3. `wait_before_sample_sec` 前等待；
4. `collectSampleWindow()` 采样 `sample_duration_sec`；
5. `wait_after_sample_sec` 后等待；
6. 下一标定位姿。

---

## 4. 外部依赖文件清单（重点）

> 下表按类别列出 `world_robot_calib_recorder_node.cpp` 的外部依赖；标注“直接/间接”。

### A. 启动文件

1) `launch/world_robot_calib_record.launch`
- 类型：launch
- 作用：自动标定总入口；设置 robot_name、calib_pose_csv、fused/stats topic、输出目录；启动 recorder。
- 依赖关系：**直接**（启动 recorder 的主 launch）。

2) `launch/jaka_sdk_bringup.launch`
- 类型：launch
- 作用：被上面 include；启动 `jaka_driver`、`jaka_state_adapter_node`、`robot_state_publisher`。
- 依赖关系：**间接**（通过 include 影响 linear_move 与 joint_states/TF 可用性）。

3) （未发现）自动标定专用 shell 启动脚本
- 类型：script（未定位）
- 作用：未找到。
- 依赖关系：不确定。

### B. 配置与数据文件

1) `config/jaka1_world_robot_calibration_pose.csv`
2) `config/jaka2_world_robot_calibration_pose.csv`
3) `config/jaka3_world_robot_calibration_pose.csv`
4) `config/jaka4_world_robot_calibration_pose.csv`
- 类型：csv
- 作用：标定位姿序列输入。
- 依赖关系：**直接**（由 `calib_pose_csv/pose_csv` 参数解析到具体文件）。

5) `config/` 目录（输出数据集默认目录）
- 类型：目录/数据路径
- 作用：`output_dir` 默认在包内 config；输出 `world_robot_calib_dataset_*.csv`。
- 依赖关系：**直接**（输出路径）。

6) `config/cube_faces_4robots.yaml`
- 类型：yaml
- 作用：供 `cube_multi_face_fusion_node` 配置各面几何/ID，影响 `fused_topic` 来源质量。
- 依赖关系：**间接**（视觉融合前置）。

7) `urdf/jaka_zu3.urdf`
- 类型：urdf
- 作用：`robot_state_publisher` 用于发布 TF，支撑 recorder 查询 `base->tool`。
- 依赖关系：**间接**（TF前置）。

### C. 接口定义文件

1) `jaka_msgs/Move.srv`（外部包，当前仓库未包含）
- 类型：srv
- 作用：linear_move service 请求/响应定义。
- 依赖关系：**直接**（recorder 直接构造该请求）。

2) `msg/CubeFusionStats.msg`
- 类型：msg
- 作用：`stats_topic` 统计信息输入（n_obs/inliers/误差数组）。
- 依赖关系：**直接**（recorder 订阅并写数据集统计字段）。

3) `sensor_msgs/JointState`（ROS标准消息）
- 类型：msg
- 作用：机器人运动停止判定输入。
- 依赖关系：**直接**。

4) `geometry_msgs/PoseStamped`（ROS标准消息）
- 类型：msg
- 作用：融合位姿输入（`fused_topic`）。
- 依赖关系：**直接**。

5) `srv/SetPoseTarget.srv`、`srv/GetCubePoseWorld.srv`
- 类型：srv
- 作用：主要供 `pose_servo_world_node` 使用；不属于 recorder 直线运动主链。
- 依赖关系：对 recorder 为**非直接链路**（同包内接口）。

### D. 驱动与底层控制文件

1) 外部节点：`jaka_driver`（包：`jaka_driver`）
- 类型：外部可执行/driver
- 作用：提供 `/jaka_driver/linear_move` 服务。
- 依赖关系：**直接（运行时）**。

2) 外部源码：`jaka_driver.cpp`（用户提到）
- 路径：当前仓库未找到。
- 作用：推测为 linear_move callback 实现所在。
- 依赖关系：**直接（若存在于外部包）**。

3) 外部 SDK 头/库：`jaka_driver/JAKAZuRobot.h`（在本仓库 `src/jaka_sdk_driver_node.cpp` 被 include）
- 类型：SDK 头
- 作用：底层 API 入口（示例节点里使用 `joint_move/get_robot_status/login_in`）。
- 依赖关系：对 recorder->linear_move 链路是**间接**（通过 driver）。

4) `src/jaka_sdk_driver_node.cpp`（本仓库）
- 类型：cpp
- 作用：当前实现了 `jaka_driver/joint_move` 服务，不是 recorder 默认调用的 `linear_move`。
- 依赖关系：对本次自动标定 linear_move 为**参考/旁路**。

### E. 话题 / TF / 状态相关文件

1) `src/jaka_state_adapter_node.cpp`
- 类型：cpp
- 作用：`jaka_driver/joint_position -> joint_states`，支撑停止判定与 TF 链。
- 依赖关系：**间接关键依赖**。

2) `src/cube_multi_face_fusion_node.cpp`
- 类型：cpp
- 作用：发布 `/vision/<robot>/cube_center` 与 `/vision/<robot>/cube_fusion_stats`。
- 依赖关系：**间接关键依赖**（采样输入）。

3) `src/world_tag_node.cpp`、`src/cube_aruco_detector_node.cpp`、`src/fiducial_relay_node.cpp`
- 类型：cpp
- 作用：视觉检测与 world 对齐前置链。
- 依赖关系：**间接**。

4) `robot_state_publisher`（外部ROS包节点）+ `urdf/jaka_zu3.urdf`
- 类型：node/urdf
- 作用：生成 TF；recorder 查询 `base_frame -> tool_frame` 与 `world_frame -> camera_frame`。
- 依赖关系：**间接关键依赖**。

### 推导链（参数到文件）示例
- `calib_pose_csv`：launch 传参 `$(find jaka_close_contro)/config/$(arg robot_name)_world_robot_calibration_pose.csv` -> recorder `resolveCalibPosePath()` 验证/绝对化路径。
- `output_dir`：launch `dataset_out_dir` -> recorder 若 `output_dataset_csv` 为空则拼接 `output_dir/output_prefix_timestamp.csv`。
- `fused_topic/stats_topic`：launch 用 `/vision/$(arg robot_name)/...` -> recorder 订阅。
- `joint_state_topic`：recorder 默认 `/joint_states`；来源由 `jaka_state_adapter_node` 提供。

---

## 5. 直线运动调用链完整追踪

> 受限于仓库内容：driver 内部 `linear_move` callback 源码未在本仓库定位到，链路后半段按“已确认/待确认”分层给出。

### 5.1 已确认链路（仓库内可证实）

1. CSV 取行：
   - 文件：`config/<robot>_world_robot_calibration_pose.csv`
   - 函数：`loadCalibPoses()`
   - 行解析为 `CalibPoseRow{name,x_mm,y_mm,z_mm,rx_deg,ry_deg,rz_deg}`

2. 发运动命令：
   - 函数：`executeTrajectory()` -> `sendLinearTarget()`
   - 构造 `jaka_msgs::Move`：
     - `pose[0..2]=mm`
     - `pose[3..5]=deg->rad`
     - `mvvelo/mvacc` 已乘 `speed_scale`
     - `mvtime=0,mvradii=0,coord_mode=0,index=target_index`

3. 调服务：
   - `linear_move_client_.call(srv)`
   - service 名：`linear_move_service`（默认 `jaka_driver/linear_move`）

4. 返回判定：
   - 通信失败：`call()` 返回 false -> 失败
   - 服务返回失败：`srv.response.ret != 0` -> 失败
   - `ret == 0` -> success

5. 运动完成等待：
   - `waitRobotMotionDone()` 通过 `joint_states` 差分阈值判定“静止”

### 5.2 待确认链路（仓库外）

6. `/jaka_driver/linear_move` service callback：
   - 文件：**未找到（需在外部 `jaka_driver` 包查找）**
   - 需确认：
     - `coord_mode/index/mvtime/mvradii` 是否被实际使用；
     - `has_ref/ref_joint` 缺省时的处理；
     - 错误码 `ERR_FUCTION_CALL_ERROR` 在什么条件触发。

7. callback 到 JAKA SDK API：
   - 文件：**未找到（外部包）**
   - 需确认是调用 `linear_move` / `movel` / 其他API，及其参数语义（绝对位姿or增量位姿）。

### 5.3 单位转换跨层说明
- recorder 层：位置 mm，姿态 rad（由 deg 转换）后写入 `pose[6]`。
- driver/sdk 层：**不确定**（需看外部 `Move.srv` 注释与 driver 实现）。
- 当前可确认：recorder 已按“位置mm + 角度rad”发送。

### 5.4 可能被忽略字段
- recorder 设置了 `mvtime/mvradii/coord_mode/index`，但 driver 是否使用不确定。
- recorder 未设置 `has_ref/ref_joint`，若 driver 强依赖该字段可能引发错误（需外部 driver 代码验证）。

---

## 6. 成功项目中为什么能正常运行（基于当前链路推断）

1) 目标位姿来源稳定：
- 来自离线配置 CSV 的预设绝对目标，不依赖实时反馈 pose 拼接，输入可控。

2) 运动参数保守：
- 默认 `speed_scale=0.15` 且上限钳制；`linear_speed_mm_s=80`、`linear_acc_mm_s2=200`，相比激进参数更稳。

3) 每次运动后等待停止：
- 有明确关节阈值+稳定时长判据，减少“未停稳即采样/再运动”的时序问题。

4) 前置条件依赖明显：
- 依赖 driver 服务可用、joint_states连续、TF链完整、视觉融合数据可用。
- 对于上电/使能/TCP/user frame/payload/collision level 等，recorder本身不配置，依赖 driver/现场状态。

5) 返回值语义简单明确：
- 以 `call()` 是否成功 + `ret==0` 判定成功，失败即停止后续姿态。

6) 与 threshold_listener_jaka openloop 思路的核心差异（原则层）：
- recorder 是“**固定标定点 + 每步稳态等待 + 采样窗口**”流程。
- openloop 常见问题是“连续指令/反馈耦合构造目标/时序不足”，更易触发 SDK 功能调用错误。

---

## 7. 与当前 threshold_listener_jaka 的详细对比（只谈直线运动）

> 说明：本仓库中未找到 `openloop_move_jaka4.cpp` 与你提到的 `jaka_driver.cpp`，因此以下对比分为“可确认”和“待补证”。

### 7.1 可确认部分（来自 recorder）
- 目标位姿来源：CSV 绝对目标。
- 是否依赖当前反馈 pose：不依赖（运动目标直接来自 CSV）。
- 单位处理：位置 mm，姿态 deg->rad。
- pose 六维构造：严格 `[x,y,z,rx,ry,rz]`。
- 成功判定：`call==true && ret==0`。

### 7.2 需要你当前项目文件补证的对比点（本仓库缺失）
请补充以下文件后可给出逐行对比：
- `threshold_listener_jaka/openloop_move_jaka4.cpp`
- 对应 driver 的 `jaka_driver.cpp`（含 linear_move callback）
- `jaka_msgs/Move.srv` 实际定义（尤其 has_ref/ref_joint/coord_mode/index 注释）

### 7.3 对 `ERR_FUCTION_CALL_ERROR` 的最可能原因（静态推断，非定论）
按优先级推测：
1. **单位或姿态语义不匹配**（例如角度应为度却传弧度，或反之）。
2. **服务字段组合不符合 driver/sdk 期望**（如 `coord_mode/index/has_ref/ref_joint` 组合非法）。
3. **运动前置状态未满足**（未使能、模式不对、TCP/user frame/payload 不匹配）。

---

## 8. 结论与建议

### 8.1 自动标定成功路径的最小可工作闭环
最小闭环：
`world_robot_calib_record.launch -> recorder读取CSV -> call /jaka_driver/linear_move -> wait joint_states静止 -> 采样fused/stats + TF -> 写dataset CSV`

### 8.2 threshold_listener_jaka 最值得优先借鉴的点
优先借鉴 recorder 的三点：
1. 固定绝对目标（去掉即时反馈拼接）；
2. 每步后显式静止判定；
3. 失败即停（`ret!=0` 不继续连发）。

### 8.3 threshold_listener_jaka 最可能有问题的 3 点（优先级）
1. 指令单位/语义与 driver 不一致；
2. 请求字段未按 driver 约定设置（尤其参考关节/坐标模式字段）；
3. 运动时序过紧，没有“停止确认-再发下一条”的门控。

### 8.4 下一步最小验证实验（建议）
实验目标：验证“最小字段集+单位”是否可稳定通过。

建议实验：
1. 仅复用 recorder 的 `sendLinearTarget()` 字段组合；
2. 只发 1~2 个低风险姿态（小位移）；
3. 每条后等待 `joint_states` 静止；
4. 记录 `ret/message` 与 driver 日志，确认是否还出现 `ERR_FUCTION_CALL_ERROR`。

### 8.5 迁移到 threshold_listener_jaka 的最小改动路径
1. 先把目标生成改成“CSV绝对位姿”；
2. 完全对齐 recorder 的 Move 请求字段（含 speed_scale 限幅策略）；
3. 增加运动完成门控（joint_states 阈值+稳定时长）；
4. 保留原业务逻辑，其余不动；
5. 逐步恢复高级策略（反馈闭环/动态目标）并做 A/B 比较。

---

## 建议继续检查的文件列表（为完成 100% 调用链）

1. 外部包 `jaka_msgs`：`Move.srv` 定义与字段注释。
2. 外部包 `jaka_driver`：`linear_move` service callback 源文件（通常 `jaka_driver.cpp` 或同名实现）。
3. 外部包 `jaka_driver` 与 SDK 适配层：linear_move 最终调用到的 SDK API 位置。
4. 你当前项目 `threshold_listener_jaka/openloop_move_jaka4.cpp`：目标构造与单位处理细节。
5. 任何现场启动脚本（若存在）：确认命名空间重映射是否改变 service/topic 实际全名。

