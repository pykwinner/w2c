cmake_minimum_required(VERSION 2.8.3)
project(upros_description)


find_package(catkin REQUIRED COMPONENTS
    controller_manager
    roscpp
    rospy
	joint_state_controller
	robot_state_publisher
    std_msgs
    geometry_msgs
    tf
)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(upros_plugin SHARED src/upros_plugin.cpp)
target_link_libraries(upros_plugin ${GAZEBO_LIBRARIES}  ${catkin_LIBRARIES})

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
