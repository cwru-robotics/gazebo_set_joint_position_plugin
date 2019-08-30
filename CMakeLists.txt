cmake_minimum_required(VERSION 3.5)
project(set_joint_position_plugin)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	gazebo_ros#TODO Do we need this?
)
find_package(gazebo REQUIRED)
find_package(sensor_msgs REQUIRED)
#TODO Possibility of boost.
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS} ${gazebo_dev_LIBRARY_DIRS})
#find_package(Boost REQUIRED COMPONENTS thread)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
	DEPENDS roscpp gazebo_ros 
	LIBRARIES set_joint_position_plugin
	CATKIN_DEPENDS roscpp gazebo_ros
)

add_library(set_joint_position_plugin SHARED src/set_joint_position_plugin.cpp)
target_link_libraries(set_joint_position_plugin
	${GAZEBO_LIBRARIES}
	${catkin_LIBRARIES}
	"gazebo"
	#${Boost_LIBRARIES}
)

#add_library(set_joint_velocity_plugin SHARED src/set_joint_velocity_plugin.cpp)
#ament_target_dependencies(set_joint_velocity_plugin
#	${GAZEBO_LIBRARIES}
#	"rclcpp" "gazebo" "sensor_msgs"
	#${Boost_LIBRARIES}
#)
