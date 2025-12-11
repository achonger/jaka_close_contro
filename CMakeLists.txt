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
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  fiducial_msgs
  zed_wrapper
  jaka_msgs
  jaka_sdk_driver
  std_srvs
  message_generation
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)

add_service_files(FILES
  WorldRobotCalibration.srv
  WorldRobotCollectSample.srv
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
    tf2
    tf2_ros
    tf2_geometry_msgs
    cv_bridge
    fiducial_msgs
    zed_wrapper
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
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
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
  ${CERES_LIBRARIES}
  glog
  gflags
)



# 5. 闭环控制节点
add_executable(closed_loop_control_node src/closed_loop_control_node.cpp)
target_link_libraries(closed_loop_control_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# 6. 世界-机器人外参发布器
add_executable(world_robot_extrinsic_broadcaster_node src/world_robot_extrinsic_broadcaster_node.cpp)
target_link_libraries(world_robot_extrinsic_broadcaster_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# 6+. 世界系闭环观测/控制节点
add_executable(cube_world_closedloop_node src/cube_world_closedloop_node.cpp)
target_link_libraries(cube_world_closedloop_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# 7. 世界-机器人标定节点
add_executable(world_robot_calibration_node src/world_robot_calibration_node.cpp)
target_link_libraries(world_robot_calibration_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
add_dependencies(world_robot_calibration_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 8. 自动标定管理与运动执行
add_executable(world_robot_autocalib_manager_node src/world_robot_autocalib_manager_node.cpp)
target_link_libraries(world_robot_autocalib_manager_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
add_dependencies(world_robot_autocalib_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(world_robot_calib_motion_node src/world_robot_calib_motion_node.cpp)
target_link_libraries(world_robot_calib_motion_node
  ${catkin_LIBRARIES}
)
add_dependencies(world_robot_calib_motion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 安装配置文件
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)