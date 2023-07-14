cmake_minimum_required(VERSION 3.0.0)

project(dummy_)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-narrowing")

set(CMAKE_CXX_STANDARD 11) # Tell CMake to compile with C++11

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  image_transport
  cv_bridge
  tf
)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp
  DEPENDS
)

include_directories(
  include
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# ======  dummy camera  ========

link_directories(${PCL_LIBRARY_DIRS})

add_executable(dummy_camera src/dummy_camera.cpp)
target_link_libraries(dummy_camera ${catkin_LIBRARIES} ${PCL_LIBRARIES})