cmake_minimum_required(VERSION 3.0.2)
project(upros_arm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  upros_message
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS upros_message tf2_ros
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(claw_test_node src/claw_test.cpp)
target_link_libraries(claw_test_node ${catkin_LIBRARIES})

add_executable(go_home_node src/go_home.cpp)
target_link_libraries(go_home_node ${catkin_LIBRARIES})

add_executable(inverse_move_node src/inverse_move.cpp)
target_link_libraries(inverse_move_node ${catkin_LIBRARIES})

add_executable(grab_tof_node src/tof_grab.cpp)
target_link_libraries(grab_tof_node ${catkin_LIBRARIES})

add_executable(apriltag_grab_node src/apriltag_grab.cpp)
target_link_libraries(apriltag_grab_node ${catkin_LIBRARIES})

add_executable(upros_arm_control
  src/control_center.cpp
)
target_link_libraries(upros_arm_control ${catkin_LIBRARIES})
add_dependencies(upros_arm_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})