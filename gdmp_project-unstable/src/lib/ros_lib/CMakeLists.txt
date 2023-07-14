cmake_minimum_required(VERSION 2.8.3)

project(ros_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib 
  actionlib_msgs
  sensor_msgs
  geometry_msgs
  message_filters
)

add_action_files(
  DIRECTORY action
  FILES MoveToJointsPos.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(
  include
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

set(LIBRARY_NAME ${PROJECT_NAME})
add_library(${LIBRARY_NAME} 
src/move_to_joints_pos_server.cpp
src/move_to_joints_pos_client.cpp
)

target_link_libraries(
  ${LIBRARY_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(
  ${LIBRARY_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
