cmake_minimum_required(VERSION 2.8.3)
project(handsfree_ros_imu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
)

catkin_package(CATKIN_DEPENDS std_msgs geometry_msgs)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  include include/handsfree_ros_imu/
)

add_executable(imu_calibrate
  src/imu_calibrate.cpp
  src/imu_calibrate_node.cpp
  ${catkin_LIBRARIES}
)
add_dependencies(imu_calibrate ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_calibrate ${catkin_LIBRARIES})
