cmake_minimum_required(VERSION 3.0.2)
project(clean_table_robot)


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

add_executable(table_movebase_client src/movebase_client.cpp)
target_link_libraries(table_movebase_client ${catkin_LIBRARIES})

add_executable(table_tf_echo_node src/tf_echo.cpp)
target_link_libraries(table_tf_echo_node ${catkin_LIBRARIES})

add_executable(table_arm_grab_node src/arm_grab.cpp)
target_link_libraries(table_arm_grab_node ${catkin_LIBRARIES})

add_executable(table_arm_put_node src/arm_put.cpp)
target_link_libraries(table_arm_put_node ${catkin_LIBRARIES})

add_executable(table_arm_part_node src/arm_part.cpp)
target_link_libraries(table_arm_part_node ${catkin_LIBRARIES})

add_executable(table_task_node src/task_flow.cpp)
target_link_libraries(table_task_node ${catkin_LIBRARIES})

add_executable(table_complete_node src/complete_flow.cpp)
target_link_libraries(table_complete_node ${catkin_LIBRARIES})