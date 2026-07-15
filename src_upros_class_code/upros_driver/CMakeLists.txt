cmake_minimum_required(VERSION 3.0.2)
project(upros_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  upros_message
  serial
  tf
)

catkin_package(
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(upros_driver
  src/main.cpp
  src/base_driver_config.cpp
  src/upros_driver.cpp
  src/dataframe.cpp
  src/check_calc.cpp
)
add_dependencies(upros_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(upros_driver ${catkin_LIBRARIES})
