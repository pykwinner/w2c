cmake_minimum_required(VERSION 3.0.2)
project(upros_slam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslib
  tf
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pub_map_node src/pub_map.cpp)
target_link_libraries(pub_map_node ${catkin_LIBRARIES})

add_executable(map_scan_node src/map_scan.cpp)
target_link_libraries(map_scan_node ${catkin_LIBRARIES})

add_executable(clear_area_node src/clear_area.cpp)
target_link_libraries(clear_area_node ${catkin_LIBRARIES} ${OpenCV_LIBS})