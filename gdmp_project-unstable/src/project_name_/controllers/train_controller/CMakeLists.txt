cmake_minimum_required(VERSION 3.0.0)

project(train_controller)

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
  std_msgs
  math_lib
  io_lib
  robo_lib
  plot_lib
  thread_lib
  robot_wrapper
  gui_lib
  gmp_lib
  rviz_lib
  kdl_parser
  yaml-cpp
  main_controller
  gmp_model
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


add_library(${PROJECT_NAME}
  src/utils/demo_data.cpp

  include/${PROJECT_NAME}/utils/trim_data_dialog.h
  src/utils/trim_data_dialog.cpp
  
  include/${PROJECT_NAME}/utils/param_slider.h
  src/utils/param_slider.cpp

  include/${PROJECT_NAME}/utils/remove_stops_dialog.h
  src/utils/remove_stops_dialog.cpp

  src/train_controller.cpp
  include/${PROJECT_NAME}/train_gui.h
  src/train_gui.cpp
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
  robo_lib
  plot_lib
  thread_lib
  gmp_lib
  rviz_lib
  robot_wrapper
  gui_lib
  ${Qt_LIBRARIES}
  pthread
  yaml-cpp
  main_controller
  gmp_model
)
