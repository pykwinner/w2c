cmake_minimum_required(VERSION 3.0.2)
project(upros_depth_vision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  pcl_ros
  tf2
  tf2_ros
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp pcl_ros tf2 tf2_ros
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(upros_image_node
  src/upros_image.cpp
)
target_link_libraries(upros_image_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(upros_red_stop_node
  src/upros_red_stop.cpp
)
target_link_libraries(upros_red_stop_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(upros_point_filter_node 
  src/upros_point_filter.cpp
)
target_link_libraries(upros_point_filter_node
  ${catkin_LIBRARIES}
)
