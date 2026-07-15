cmake_minimum_required(VERSION 3.0.2)
project(upros_moveit)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  upros_message
  moveit_msgs
  moveit_ros_planning_interface
)

catkin_package(
  CATKIN_DEPENDS upros_message actionlib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(upros_moveit_controller
  src/upros_moveit_controller.cpp
)
add_dependencies(upros_moveit_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(upros_moveit_controller
  ${catkin_LIBRARIES}
)

add_executable(upros_fk_demo src/upros_fk_demo.cpp)
target_link_libraries(upros_fk_demo ${catkin_LIBRARIES})

add_executable(upros_ik_demo src/upros_ik_demo.cpp)
target_link_libraries(upros_ik_demo ${catkin_LIBRARIES})

add_executable(upros_pose_demo src/upros_pose_demo.cpp)
target_link_libraries(upros_pose_demo ${catkin_LIBRARIES})