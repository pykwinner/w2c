cmake_minimum_required(VERSION 3.0.2)
project(clean_desktop_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
  tf
  cv_bridge
  upros_message
  dynamic_reconfigure
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/params_color.cfg
)

catkin_package(
  CATKIN_DEPENDS 
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(movebase_client src/movebase_client.cpp)
target_link_libraries(movebase_client ${catkin_LIBRARIES})

add_executable(tf_echo_node src/tf_echo.cpp)
target_link_libraries(tf_echo_node ${catkin_LIBRARIES})

add_executable(arm_grab_test_node src/arm_test.cpp)
target_link_libraries(arm_grab_test_node ${catkin_LIBRARIES})

add_executable(arm_grab_node src/arm_grab.cpp)
target_link_libraries(arm_grab_node ${catkin_LIBRARIES})

add_executable(arm_put_node src/arm_put.cpp)
target_link_libraries(arm_put_node ${catkin_LIBRARIES})

add_executable(task_flow_node src/task_flow.cpp)
target_link_libraries(task_flow_node ${catkin_LIBRARIES})

add_executable(complete_flow_node src/complete_flow.cpp)
target_link_libraries(complete_flow_node ${catkin_LIBRARIES})

# add_executable(color_detect src/color_detect.cpp)
# target_link_libraries(color_detect ${catkin_LIBRARIES} ${OpenCV_LIBS})