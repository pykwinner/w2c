cmake_minimum_required(VERSION 2.8.3)
project(zoo_bringup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  dynamic_reconfigure
  upros_message
)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    file(GLOB GLOG_LIBRARIES "/usr/lib/x86_64-linux-gnu/libglog.so")
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    file(GLOB GLOG_LIBRARIES "/usr/lib/aarch64-linux-gnu/libglog.so")
else ()
    message(FATAL_ERROR "Unsupported architecture")
endif ()
set(GLOG_INCLUDE_DIRS /usr/include/glog)
message("glog_include_dirs=${GLOG_INCLUDE_DIRS}")
message("glog_libraries=${GLOG_LIBRARIES}")

catkin_package()

include_directories(
  include ${catkin_INCLUDE_DIRS}
  include ${catkin_INCLUDE_DIRS}
  include include/zoo_bringup/
)


add_executable(zoo_driver
  src/main.cpp
  src/base_driver_config.cpp
  src/base_driver.cpp
  src/simple_dataframe_master.cpp
  src/serial_transport.cpp
)
add_dependencies(zoo_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(zoo_driver
  ${catkin_LIBRARIES}
  ${GLOG_LIBRARIES}
)

add_executable(s2a_arm_driver
  src/s2a_arm_driver.cpp
)
add_dependencies(s2a_arm_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(s2a_arm_driver
  ${catkin_LIBRARIES}
)
