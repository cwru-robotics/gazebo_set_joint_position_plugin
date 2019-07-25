cmake_minimum_required(VERSION 3.5)
project(set_joint_position_plugin)

find_package(ament_cmake REQUIRED)
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo REQUIRED)
#find_package(Boost REQUIRED COMPONENTS thread)

#TODO What is this for?
SET(CMAKE_BUILD_TYPE RELEASE)

#TODO What about these?
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

#TODO Or this?
include_directories(include)
include_directories(
	${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}
	#${Boost_INCLUDE_DIRS}
) 
link_directories(
	${GAZEBO_LIBRARY_DIRS}
	#${Boost_LIBRARY_DIRS}
)

add_library(set_joint_position_plugin SHARED src/set_joint_position_plugin.cpp)
ament_target_dependencies(set_joint_position_plugin
	${GAZEBO_LIBRARIES} ${catkin_LIBRARIES}
	"rclcpp" "gazebo"
	#${Boost_LIBRARIES}
)

ament_package()
