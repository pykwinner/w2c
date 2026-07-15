cmake_minimum_required(VERSION 3.0.2)
project(upros_navigation)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp actionlib actionlib_msgs move_base_msgs tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ros_pub_goal_node src/ros_pub_goal.cpp)
target_link_libraries(ros_pub_goal_node ${catkin_LIBRARIES})

add_executable(area_path_node src/AreaPathNode.cpp src/AreaPath.cpp)
target_link_libraries(area_path_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(next_goal src/next_goal.cpp)
target_link_libraries(next_goal ${catkin_LIBRARIES})