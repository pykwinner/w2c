cmake_minimum_required(VERSION 3.0.2)
project(shoot_robot)

# 当前代码使用C++11以上特性
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_srvs
  serial
  apriltag_ros
  actionlib
  move_base_msgs
  tf
  tf2
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    std_srvs
    serial
    apriltag_ros
    actionlib
    move_base_msgs
    tf
    tf2
)

# 本项目没有include目录，只添加catkin头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# ============================================================
# 激光串口控制节点
# ============================================================

add_executable(
  shoot_node
  src/shoot.cpp
)

# ============================================================
# AprilTag识别、位置调整和射击节点
# ============================================================

add_executable(
  apriltag_detect_node
  src/apriltag_detect.cpp
)

# ============================================================
# TF坐标查询节点
# ============================================================

add_executable(
  w2u_tf_echo_node
  src/tf_echo.cpp
)

# ============================================================
# 单次move_base导航测试节点
# ============================================================

add_executable(
  movebase_client_node
  src/movebase_client.cpp
)

# ============================================================
# 比赛打靶主流程节点
# ============================================================

add_executable(
  shoot_robot_node
  src/shoot_robot.cpp
)

# ============================================================
# 基础打靶流程节点
# ============================================================

add_executable(
  shoot_robot_base_node
  src/shoot_robot_base.cpp
)

# ============================================================
# 任务流程节点
# ============================================================

add_executable(
  w2u_task_flow_node
  src/task_flow.cpp
)

# 本项目生成的所有可执行目标
set(SHOOT_ROBOT_TARGETS
  shoot_node
  apriltag_detect_node
  w2u_tf_echo_node
  movebase_client_node
  shoot_robot_node
  shoot_robot_base_node
  w2u_task_flow_node
)

# 链接ROS和其他catkin库
foreach(target ${SHOOT_ROBOT_TARGETS})
  target_link_libraries(
    ${target}
    ${catkin_LIBRARIES}
  )
endforeach()

# 保证依赖包生成的消息先于本项目节点编译
if(catkin_EXPORTED_TARGETS)
  foreach(target ${SHOOT_ROBOT_TARGETS})
    add_dependencies(
      ${target}
      ${catkin_EXPORTED_TARGETS}
    )
  endforeach()
endif()

# ============================================================
# 安装规则
# 仅执行catkin_make install时使用
# ============================================================

# 安装所有可执行节点
install(
  TARGETS
    ${SHOOT_ROBOT_TARGETS}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装launch启动文件
install(
  DIRECTORY
    launch/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# 安装YAML配置文件
install(
  DIRECTORY
    config/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)