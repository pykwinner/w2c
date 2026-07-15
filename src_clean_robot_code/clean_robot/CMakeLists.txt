cmake_minimum_required(VERSION 3.0.2)
project(clean_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
  apriltag_ros
  upros_message
)

catkin_package(
  CATKIN_DEPENDS 
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(path_generate_node src/path_generate.cpp)
target_link_libraries(path_generate_node ${catkin_LIBRARIES})

add_executable(three_path_generate_node src/three_path_generate.cpp)
target_link_libraries(three_path_generate_node ${catkin_LIBRARIES})

add_executable(move_base_client src/movebase_client.cpp)
target_link_libraries(move_base_client ${catkin_LIBRARIES})

add_executable(core_move_client src/coremove_client.cpp)
target_link_libraries(core_move_client ${catkin_LIBRARIES})

add_executable(apriltag_detect src/apriltag_detect.cpp)
target_link_libraries(apriltag_detect ${catkin_LIBRARIES})

add_executable(clean_robot src/clean_robot.cpp)
target_link_libraries(clean_robot ${catkin_LIBRARIES})

