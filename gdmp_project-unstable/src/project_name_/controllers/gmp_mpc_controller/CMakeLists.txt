cmake_minimum_required(VERSION 3.0.0)

project(gmp_mpc_controller)

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
  math_lib
  io_lib
  robo_lib
  plot_lib
  thread_lib
  robot_wrapper
  gui_lib
  gmp_lib
  rviz_lib
  yaml-cpp
  main_controller
  apriltag_ros
  tf
)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp tf
  DEPENDS
)

include_directories(
  include
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/gmp_mpc_controller.cpp
  include/${PROJECT_NAME}/gmp_mpc_gui.h
  src/gmp_mpc_gui.cpp

  src/experiment/handover_experiment.cpp
  src/experiment/viapoints_experiment.cpp
  src/experiment/obstacles_experiment.cpp
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
  apriltag_ros_apriltag_listener
)

# =======================================================
# =======================================================

add_executable(test_rt_sparsity
  src/test_rt_sparsity.cpp
)

add_dependencies(test_rt_sparsity
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(test_rt_sparsity
  ${catkin_LIBRARIES}
  armadillo
  gmp_lib
  yaml-cpp
  thread_lib
)

# =======================================================
# =======================================================

add_executable(obst_avoid_sim
  src/simulations/obst_avoid_sim.cpp
)

add_dependencies(obst_avoid_sim
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(obst_avoid_sim
  ${catkin_LIBRARIES}
  armadillo
  gmp_lib
  plot_lib
)