cmake_minimum_required(VERSION 3.0.2)
project(upros_lidar_filter)

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

add_executable(distance_lidar_filter src/distance_lidar_filter.cpp)
target_link_libraries(distance_lidar_filter
   ${catkin_LIBRARIES}
)

add_executable(angle_lidar_filter src/angle_lidar_filter.cpp)
target_link_libraries(angle_lidar_filter
   ${catkin_LIBRARIES}
)

add_executable(blob_lidar_filter src/blob_lidar_filter.cpp)
target_link_libraries(blob_lidar_filter
   ${catkin_LIBRARIES}
)

add_executable(ros_lidar src/ros_lidar.cpp)
target_link_libraries(ros_lidar
   ${catkin_LIBRARIES}
)

add_executable(move_lidar src/move_lidar.cpp)
target_link_libraries(move_lidar
   ${catkin_LIBRARIES}
)