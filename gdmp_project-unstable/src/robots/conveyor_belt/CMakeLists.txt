cmake_minimum_required(VERSION 3.5)
project(atv12)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# -> SAVVAS uncomment this 
# Default to C++14 
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Require C99 and C++17
# target_compile_features(arl_core2 PUBLIC c_std_99 cxx_std_17) 

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs  
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp roslib std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(run_conveyor src/run_conveyor.cpp src/atv12.cpp)

add_dependencies(run_conveyor
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(run_conveyor 
  ${catkin_LIBRARIES}
  modbus
)


# add_executable(conveyor_tif src/conveyor_belt_node.cpp src/atv12.cpp)
# add_dependencies(conveyor_tif  rclcpp std_msgs)
# target_link_libraries(conveyor_tif modbus)
