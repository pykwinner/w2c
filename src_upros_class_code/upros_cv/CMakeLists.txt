cmake_minimum_required(VERSION 3.0.2)
project(upros_cv)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/params_color.cfg
)


catkin_package(
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


