cmake_minimum_required(VERSION 2.8.3)
project(depth_node)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_conversions
  pcl_ros
  image_transport
)

find_package( OpenCV REQUIRED )
find_package( cv_bridge REQUIRED )

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs image_transport
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  #include
)
add_executable(depth_node src/depth_node.cpp)
#add_dependencies(imu_tutorial_node imu_tutorial_generate_messages_cpp)

target_link_libraries(depth_node
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${cv_bridge_LIBRARIES}
)

#install(TARGETS lidar_tutorial_node
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
