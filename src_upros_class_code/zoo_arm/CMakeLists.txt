cmake_minimum_required(VERSION 3.0.2)
project(zoo_arm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  upros_message
  tf
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  include ${catkin_INCLUDE_DIRS}
  include include/zoo_arm/
)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs upros_message
)

add_executable(zoo_arm_control
  src/serialport.cpp
  src/mini2_arm.cpp
  src/arm_inverse.cpp
  src/control_center.cpp
)
target_link_libraries(zoo_arm_control ${catkin_LIBRARIES})
add_dependencies(zoo_arm_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(arm_grab_node src/grab_put.cpp)
target_link_libraries(arm_grab_node ${catkin_LIBRARIES})

add_executable(tof_grab_node src/tof_grab.cpp)
target_link_libraries(tof_grab_node ${catkin_LIBRARIES})
