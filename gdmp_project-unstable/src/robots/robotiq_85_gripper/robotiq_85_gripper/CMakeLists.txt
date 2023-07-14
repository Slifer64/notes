cmake_minimum_required(VERSION 2.8.3)
project(robotiq_85_gripper)

add_definitions(-DLINUX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(orocos_kdl)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS #roscpp
  DEPENDS orocos_kdl
)

set(SRC_LIST
  src/robotiq_85_gripper.cpp
  src/utils/xml_parser.cpp
)

file(GLOB INCLUDE_LIST "")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# *************************************************
# *************************************************

add_library(${PROJECT_NAME}
  ${SRC_LIST}
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  pthread
  armadillo
  ${catkin_LIBRARIES}
)


# *************************************************
# *************************************************
