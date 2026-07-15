cmake_minimum_required(VERSION 3.0.2)
project(upros_move_linear)

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

add_executable(move_forward src/move_forward.cpp)
target_link_libraries(move_forward
   ${catkin_LIBRARIES}
)

add_executable(turn_right src/turn_right.cpp)
target_link_libraries(turn_right
   ${catkin_LIBRARIES}
)