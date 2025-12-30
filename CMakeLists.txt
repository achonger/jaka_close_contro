cmake_minimum_required(VERSION 3.0.2)
project(jaka_close_contro)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

# 查找所有依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  fiducial_msgs
  zed_wrapper
  std_msgs
  jaka_msgs
  jaka_sdk_driver
  std_srvs
  message_generation
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_message_files(FILES
  CubeFusionStats.msg
)

add_service_files(FILES
  WorldRobotCalibration.srv
  WorldRobotCollectSample.srv
  SetPoseTarget.srv
  GetCubePoseWorld.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  sensor_msgs
  geometry_msgs
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
    cv_bridge
    fiducial_msgs
    zed_wrapper
    std_msgs
    jaka_msgs
    jaka_sdk_driver
    std_srvs
    message_runtime
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

# 1. 多尺寸 ArUco 检测器
add_executable(cube_aruco_detector_node src/cube_aruco_detector_node.cpp)
target_link_libraries(cube_aruco_detector_node 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS}
)

# 2. 话题分流器
add_executable(fiducial_relay_node src/fiducial_relay_node.cpp)
target_link_libraries(fiducial_relay_node 
  ${catkin_LIBRARIES}
)

# 3. 世界坐标系定义
add_executable(world_tag_node src/world_tag_node.cpp)
target_link_libraries(world_tag_node
  ${catkin_LIBRARIES}
)

# 4. 末端正方体多面融合 (关键修正)
add_executable(cube_multi_face_fusion_node src/cube_multi_face_fusion_node.cpp)
target_link_libraries(cube_multi_face_fusion_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${YAML_CPP_LIBRARIES}
)
add_dependencies(cube_multi_face_fusion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



# 5. 标定数据记录（离线求解）
add_executable(world_robot_calib_recorder_node src/world_robot_calib_recorder_node.cpp)
target_link_libraries(world_robot_calib_recorder_node
  ${catkin_LIBRARIES}
)
add_dependencies(world_robot_calib_recorder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 9. 关节状态适配（jaka_driver -> /joint_states）
add_executable(jaka_state_adapter_node src/jaka_state_adapter_node.cpp)
target_link_libraries(jaka_state_adapter_node
  ${catkin_LIBRARIES}
)
add_dependencies(jaka_state_adapter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 10. 融合几何自检与标定对比工具
add_executable(cube_fusion_debug_node src/cube_fusion_debug_node.cpp)
target_link_libraries(cube_fusion_debug_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
add_dependencies(cube_fusion_debug_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(pose_servo_world_node src/pose_servo_world_node.cpp)
target_link_libraries(pose_servo_world_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
add_dependencies(pose_servo_world_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 安装配置文件
install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
