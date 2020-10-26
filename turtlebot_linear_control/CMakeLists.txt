cmake_minimum_required(VERSION 3.0.2)
project(turtlebot_linear_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
)


## catkin specific configuration
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtlebot_linear_control
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs
#  DEPENDS system_lib
)


## Specify additional locations of header files
include_directories(include
	include/turtlebot_linear_control
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(turtlebot_linear_control
	src/turtlebot_linear_control.cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(turtlebot_linear_control
	${catkin_LIBRARIES}
)

