cmake_minimum_required(VERSION 3.0.0)

project(yaml-cpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

file(GLOB SRC_LIST CONFIGURE_DEPENDS
  "src/yaml-cpp_sources/*.h" "src/yaml-cpp_sources/*.cpp" 
  "src/yaml-cpp_sources/contrib/*.h" "src/yaml-cpp_sources/contrib/*.cpp")

add_library(${PROJECT_NAME}
  ${SRC_LIST}
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  armadillo
)
