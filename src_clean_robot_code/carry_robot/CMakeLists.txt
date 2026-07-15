cmake_minimum_required(VERSION 3.0.2)
project(carry_robot)

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

add_executable(w4a_movebase_client src/movebase_client.cpp)
target_link_libraries(w4a_movebase_client ${catkin_LIBRARIES})

add_executable(w4a_tf_echo_node src/tf_echo.cpp)
target_link_libraries(w4a_tf_echo_node ${catkin_LIBRARIES})

add_executable(w4a_arm_grab_node src/arm_grab.cpp)
target_link_libraries(w4a_arm_grab_node ${catkin_LIBRARIES})

add_executable(w4a_arm_put_node src/arm_put.cpp)
target_link_libraries(w4a_arm_put_node ${catkin_LIBRARIES})

add_executable(w4a_task_flow_node src/task_flow.cpp)
target_link_libraries(w4a_task_flow_node ${catkin_LIBRARIES})

add_executable(w4a_complete_flow_node src/complete_flow.cpp)
target_link_libraries(w4a_complete_flow_node ${catkin_LIBRARIES})