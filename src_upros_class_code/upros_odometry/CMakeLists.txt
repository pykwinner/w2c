cmake_minimum_required(VERSION 3.0.2)
project(upros_odometry)

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

add_executable(ros_odom_node src/ros_odom.cpp)
target_link_libraries(ros_odom_node
   ${catkin_LIBRARIES}
)

add_executable(odom_move src/odom_move.cpp)
target_link_libraries(odom_move
   ${catkin_LIBRARIES}
)