cmake_minimum_required(VERSION 2.8.3)
project(data_integrate)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  core_msgs
  image_transport
  cv_bridge
#  message_generation
)

find_package(OpenCV REQUIRED )
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_tutorials_topic
#  CATKIN_DEPENDS sensor_msgs std_msgs core_msgs
#  DEPENDS system_lib
#  message_generation
)



#generate_messgaes(DEPENDENCIES
#	std_msgs
#	sensor_msgs
#	core_msgs
#)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)
add_executable(data_show_node src/data_show.cpp)
add_dependencies(data_show_node core_msgs_generate_messages_cpp)

add_executable(data_integration_node src/data_integration.cpp)
add_dependencies(data_integration_node core_msgs_generate_messages_cpp)

add_executable(data_integration_tB_node src/data_integration_tB.cpp)
add_dependencies(data_integration_tB_node core_msgs_generate_messages_cpp)

add_executable(line_trace_node src/line_tracing.cpp)
add_dependencies(line_trace_node core_msgs_generate_messages_cpp)

target_link_libraries(data_show_node
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)
target_link_libraries(data_integration_node
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)
target_link_libraries(data_integration_tB_node
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)
target_link_libraries(line_trace_node
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)