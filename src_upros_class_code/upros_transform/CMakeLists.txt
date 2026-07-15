cmake_minimum_required(VERSION 3.0.2)
project(upros_transform)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(tf_publish_node src/tf_publisher.cpp)
target_link_libraries(tf_publish_node ${catkin_LIBRARIES})

add_executable(tf_listen_node src/tf_listener.cpp)
target_link_libraries(tf_listen_node ${catkin_LIBRARIES})

add_executable(tf_echo_node src/tf_echo.cpp)
target_link_libraries(tf_echo_node ${catkin_LIBRARIES})

add_executable(odom_app_node src/odom_app.cpp)
target_link_libraries(odom_app_node ${catkin_LIBRARIES})
