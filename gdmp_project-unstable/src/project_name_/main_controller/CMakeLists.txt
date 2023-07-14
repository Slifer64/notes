cmake_minimum_required(VERSION 3.0.0)

project(main_controller)

 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-narrowing")

set(CMAKE_CXX_STANDARD 11) # Tell CMake to compile with C++11
set(CMAKE_AUTOMOC ON) # Tell CMake to run moc when needed.
set(CMAKE_AUTOUIC ON) # Tell CMake to run uic when needed.
set(CMAKE_AUTORCC ON) # Tell CMake to run rcc when needed

# Moc generated files are located in the current dir so we need to tell CMake to look for them.
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find Qt5
find_package(Qt5 COMPONENTS Widgets Core Gui PrintSupport REQUIRED)
# Add Qt5 definitions and includes to build libraries.
# Widgets add Widgets Core and Gui
add_definitions(${Qt5Widgets_DEFINITIONS})
include_directories(${Qt5Widgets_INCLUDES})
SET(Qt_LIBRARIES
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
  Qt5::PrintSupport
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  std_msgs
  thread_lib
  robot_wrapper
  robo_lib
  math_lib
  gui_lib
  ati_sensor
  rviz_lib
  plot_lib
  robotiq_85_gripper
  robot_state_publisher
  apriltag_ros
  yaml-cpp
  io_lib
)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS
)

include_directories(
  include
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

set(SRC_LIST

  src/main_controller.cpp
  src/controller.cpp
  include/${PROJECT_NAME}/main_gui.h
  src/main_gui.cpp
  src/utils.cpp
  src/gripper_control.cpp

  include/${PROJECT_NAME}/robot_joints_recorder/robot_joints_recorder.cpp
  include/${PROJECT_NAME}/robot_joints_recorder/robot_joints_recorder_win.h
  include/${PROJECT_NAME}/robot_joints_recorder/robot_joints_recorder_win.cpp
)

add_library(${PROJECT_NAME}
  ${SRC_LIST}
  resources/resource.qrc
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  armadillo
  thread_lib
  rviz_lib
  robot_wrapper
  robo_lib
  math_lib
  gui_lib
  ${Qt_LIBRARIES}
  pthread
  robotiq_85_gripper
  apriltag_ros_apriltag_listener
  yaml-cpp
  plot_lib
  io_lib
)