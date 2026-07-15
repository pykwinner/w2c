cmake_minimum_required(VERSION 3.0.2)
project(lift_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
  tf
  upros_message
  apriltag_ros
)


catkin_package(
  CATKIN_DEPENDS 
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(lift_movebase_client src/movebase_client.cpp)
target_link_libraries(lift_movebase_client ${catkin_LIBRARIES})

add_executable(lift_tf_echo_node src/tf_echo.cpp)
target_link_libraries(lift_tf_echo_node ${catkin_LIBRARIES})

add_executable(lift_apriltag_detect src/apriltag_detect.cpp)
target_link_libraries(lift_apriltag_detect ${catkin_LIBRARIES})

add_executable(lift_robot src/lift_robot.cpp)
target_link_libraries(lift_robot ${catkin_LIBRARIES})

add_executable(lift_up src/lift_up.cpp)
target_link_libraries(lift_up ${catkin_LIBRARIES})
