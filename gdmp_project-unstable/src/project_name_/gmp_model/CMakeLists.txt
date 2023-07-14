cmake_minimum_required(VERSION 3.0.0)

project(gmp_model)

 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-narrowing")

set(CMAKE_CXX_STANDARD 11) # Tell CMake to compile with C++11

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  math_lib
  io_lib
  thread_lib
  gmp_lib
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
)

add_library(${PROJECT_NAME}
  src/model.cpp
  src/target_model.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  armadillo
  math_lib
  io_lib
  thread_lib
  gmp_lib
)
