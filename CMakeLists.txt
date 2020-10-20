cmake_minimum_required(VERSION 2.8.3)
project(imu_gps_localization)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/local/eigen3
  imu_gps_localizer/include
  ros_wrapper/include
)

add_subdirectory(imu_gps_localizer)


add_library(ros_wrapper_lib
	ros_wrapper/src/localization_wrapper.cpp
)
target_link_libraries(ros_wrapper_lib
  ${catkin_LIBRARIES}
  imu_gps_localizer_lib
  glog
)

## Localization node.
add_executable(${PROJECT_NAME}_node ros_wrapper/src/localization_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ros_wrapper_lib
)
