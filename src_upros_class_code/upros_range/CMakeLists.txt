cmake_minimum_required(VERSION 3.0.2)
project(upros_range)

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

add_executable(ros_sonic_node src/ros_sonic.cpp)
target_link_libraries(ros_sonic_node
   ${catkin_LIBRARIES}
)

add_executable(sonic_avoid_node src/sonic_avoid.cpp)
target_link_libraries(sonic_avoid_node
   ${catkin_LIBRARIES}
)