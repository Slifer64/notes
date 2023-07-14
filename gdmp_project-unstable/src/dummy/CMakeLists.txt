cmake_minimum_required(VERSION 2.8.3)

project(dummy)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED
  roscpp
  roslib
  yaml-cpp
  kdl_parser
  robot_state_publisher
  tf
  rviz_lib
  io_lib
  plot_lib
  robo_lib
  ros_lib
)
 
catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    roscpp 
    kdl_parser
    robot_state_publisher
    tf
)

include_directories(
  include
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

# =====================================================
# =====================================================

add_executable(cart_space_trajectory src/cart_space_trajectory.cpp)
add_dependencies(cart_space_trajectory
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(cart_space_trajectory
  ${catkin_LIBRARIES}
  armadillo
  plot_lib
  robo_lib
)


# =====================================================
# =====================================================

add_executable(yaml_cpp_example src/yaml_cpp_example.cpp)
add_dependencies(yaml_cpp_example
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(yaml_cpp_example
  ${catkin_LIBRARIES}
  armadillo
  yaml-cpp
)


# =====================================================
# =====================================================

add_executable(rviz_publish_test 
  src/rviz_publish_test.cpp
)
add_dependencies(rviz_publish_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(rviz_publish_test
  ${catkin_LIBRARIES}
  armadillo
  rviz_lib
  io_lib
)

# =====================================================
# =====================================================

add_executable(ode_test 
  src/ode_test.cpp
)
add_dependencies(ode_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(ode_test
  ${catkin_LIBRARIES}
  rviz_lib
  io_lib
  plot_lib
  armadillo
)

# =====================================================
# =====================================================

# add_executable(move_to_joints_pos_action_test 
#   src/move_to_joints_pos_action_test.cpp
# )
# add_dependencies(move_to_joints_pos_action_test
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )

# target_link_libraries(move_to_joints_pos_action_test
#   ${catkin_LIBRARIES}
#   armadillo
#   ros_lib
# )
