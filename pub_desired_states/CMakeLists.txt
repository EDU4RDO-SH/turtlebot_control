cmake_minimum_required(VERSION 3.0.2)
project(pub_desired_states)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  std_msgs
  std_srvs
)


## Generate services in the 'srv' folder
add_service_files(DIRECTORY srv FILES
	path.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
	geometry_msgs nav_msgs std_msgs
)


## catkin specific configuration
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pub_desired_states
	CATKIN_DEPENDS geometry_msgs message_runtime nav_msgs roscpp std_msgs std_srvs
#  DEPENDS system_lib
)


## Specify additional locations of header files
include_directories(include
	include/pub_desired_states
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(trajectory_builder
	src/trajectory_builder.cpp
)


## Declare a C++ executable
add_executable(pub_desired_states src/pub_desired_states.cpp)
add_executable(open_loop_controller src/open_loop_controller.cpp)
add_executable(pub_desired_path_client src/pub_desired_path_client.cpp)
add_executable(pub_desired_path_client_square src/pub_desired_path_client_square.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(pub_desired_states trajectory_builder ${catkin_LIBRARIES})
target_link_libraries(open_loop_controller ${catkin_LIBRARIES})
target_link_libraries(pub_desired_path_client ${catkin_LIBRARIES})
target_link_libraries(pub_desired_path_client_square ${catkin_LIBRARIES})


## Add dependencies to generated messages (only for custom messages services or actions)
add_dependencies(pub_desired_states pub_desired_states_generate_messages_cpp)
add_dependencies(pub_desired_path_client pub_desired_states_generate_messages_cpp)
add_dependencies(pub_desired_path_client_square pub_desired_states_generate_messages_cpp)





