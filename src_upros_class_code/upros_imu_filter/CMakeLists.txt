cmake_minimum_required(VERSION 3.0.2)
project(upros_imu_filter)

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

add_executable(ros_imu_node src/ros_imu.cpp)
target_link_libraries(ros_imu_node
   ${catkin_LIBRARIES}
)

add_executable(ros_imu_rotate_node src/imu_rotate.cpp)
target_link_libraries(ros_imu_rotate_node
   ${catkin_LIBRARIES}
)

add_executable(ros_imu_calibrate_node src/imu_calibrate.cpp)
target_link_libraries(ros_imu_calibrate_node
   ${catkin_LIBRARIES}
)