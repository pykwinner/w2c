cmake_minimum_required(VERSION 3.0.2)
project(upros_core_move)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  actionlib
  actionlib_msgs
  tf2_geometry_msgs
  tf2_ros
  upros_message
)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(core_move_node
        src/core_move_node.cpp
        src/core_move.cpp
        src/go_straight_planner.cpp
        src/base_planner.cpp
        src/real_pid.cpp
)
target_link_libraries(core_move_node
        ${catkin_LIBRARIES}
)

add_executable(core_move_client_node
        src/core_move_client.cpp
)
target_link_libraries(core_move_client_node
        ${catkin_LIBRARIES}
)


