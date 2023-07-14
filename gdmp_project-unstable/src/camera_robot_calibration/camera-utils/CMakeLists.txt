cmake_minimum_required(VERSION 3.0.2)
project(camera_utils)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera_registration_node
#  CATKIN_DEPENDS roscpp rospy tf2 tf2_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


find_package(catkin REQUIRED COMPONENTS
  rospy
)

## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS src/rs_camera_node.py  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
