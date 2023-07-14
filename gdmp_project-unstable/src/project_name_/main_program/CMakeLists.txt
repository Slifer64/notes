cmake_minimum_required(VERSION 3.0.0)

project(main_program)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-narrowing ") # -Wno-error

set(CMAKE_CXX_STANDARD 11) # Tell CMake to compile with C++11

# set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  main_controller
  gmp_mpc_controller
  train_controller
  admittance_controller
  online_adapt_controller
  im_traj_demo_controller
  grasp_obj_controller
)

catkin_package(
  INCLUDE_DIRS #include
  CATKIN_DEPENDS roscpp
  DEPENDS
)

include_directories(
  #include
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/main.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  main_controller
  gmp_mpc_controller
  train_controller
  admittance_controller
  online_adapt_controller
  im_traj_demo_controller
  grasp_obj_controller
)


## =========== Plot train_data ================
add_executable(plot_train_data
  src/plot_train_data.cpp
)

target_link_libraries(plot_train_data
  ${catkin_LIBRARIES}
  plot_lib
  io_lib
)