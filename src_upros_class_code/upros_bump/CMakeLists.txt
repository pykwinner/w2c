cmake_minimum_required(VERSION 3.0.2)
project(upros_bump)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

add_executable(ros_bump_node src/ros_bump.cpp)
target_link_libraries(ros_bump_node
   ${catkin_LIBRARIES}
)

add_executable(bump_avoid_node src/ros_bump_avoid.cpp)
target_link_libraries(bump_avoid_node
   ${catkin_LIBRARIES}
)