cmake_minimum_required(VERSION 3.0.2)
project(oil_check_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
  tf
  upros_message
)

catkin_package(
  CATKIN_DEPENDS 
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(oil_movebase_client src/movebase_client.cpp)
target_link_libraries(oil_movebase_client ${catkin_LIBRARIES})

add_executable(oil_tf_echo_node src/tf_echo.cpp)
target_link_libraries(oil_tf_echo_node ${catkin_LIBRARIES})

add_executable(arm_up_node src/arm_up.cpp)
target_link_libraries(arm_up_node ${catkin_LIBRARIES})

add_executable(arm_down_node src/arm_down.cpp)
target_link_libraries(arm_down_node ${catkin_LIBRARIES})

add_executable(oil_check_node src/task_flow.cpp)
target_link_libraries(oil_check_node ${catkin_LIBRARIES})
