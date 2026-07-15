cmake_minimum_required(VERSION 3.0.2)
project(upros_tutorial)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  upros_message
  dynamic_reconfigure
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ros_node src/ros_node.cpp)
target_link_libraries(ros_node ${catkin_LIBRARIES})

add_executable(ros_publisher src/ros_publisher.cpp)
target_link_libraries(ros_publisher ${catkin_LIBRARIES})

add_executable(ros_subscriber src/ros_subscriber.cpp)
target_link_libraries(ros_subscriber ${catkin_LIBRARIES})

add_executable(ros_server src/ros_server.cpp)
target_link_libraries(ros_server ${catkin_LIBRARIES})

add_executable(ros_client src/ros_client.cpp)
target_link_libraries(ros_client ${catkin_LIBRARIES})

add_executable(ros_param src/ros_param.cpp)
target_link_libraries(ros_param ${catkin_LIBRARIES})

add_executable(ros_action_server src/ros_action_server.cpp)
target_link_libraries(ros_action_server ${catkin_LIBRARIES})

add_executable(ros_action_client src/ros_action_client.cpp)
target_link_libraries(ros_action_client ${catkin_LIBRARIES})

add_executable(ros_log src/ros_log.cpp)
target_link_libraries(ros_log ${catkin_LIBRARIES})

add_executable(ros_dynamic_reconfigure_node src/ros_dynamic_reconfigure.cpp)
add_dependencies(ros_dynamic_reconfigure_node upros_message_gencfg)
target_link_libraries(ros_dynamic_reconfigure_node ${catkin_LIBRARIES} ${upros_message_LIBRARIES})

add_executable(ros_dynamic_speed_node src/ros_dynamic_speed.cpp)
add_dependencies(ros_dynamic_speed_node upros_message_gencfg)
target_link_libraries(ros_dynamic_speed_node ${catkin_LIBRARIES} ${upros_message_LIBRARIES})
