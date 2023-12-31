cmake_minimum_required(VERSION 2.8.3)

project(rviz_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED
  roscpp
  geometry_msgs
  visualization_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp geometry_msgs visualization_msgs tf
)

include_directories(
  include
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

##########################
######     MATH   ########
##########################

add_library(${PROJECT_NAME}
  # src/rviz_publisher.cpp
  src/rviz_marker_publisher.cpp
  src/tf_pose_publisher.cpp
  src/path_publisher.cpp
  src/orient_path_publisher.cpp
  src/frame_publisher.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  armadillo
)
