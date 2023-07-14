#!/bin/bash

# ==================================
# define some colors for output
# ==================================
COLOR_RED="\033[1;31m"
COLOR_GREEN="\033[1;32m"
COLOR_YELLOW="\033[1;33m"
COLOR_BLUE="\033[1;34m"
COLOR_CYAN="\033[1;36m"
COLOR_WHITE="\033[1;37m"
COLOR_RESET="\033[0m"

cd $INSTALL_SCRIPTS_DIR/deps

echo -e $COLOR_CYAN"*************************************"$COLOR_RESET
echo -e $COLOR_CYAN"********   ROS DEPENDENCIES  ********"$COLOR_RESET
echo -e $COLOR_CYAN"*************************************"$COLOR_RESET

echo -e $COLOR_BLUE"Installing ROS dependencies..."$COLOR_RESET

sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
sudo apt-get install ros-$ROS_DISTRO-moveit


# if [ $? -eq 0 ]; then
#   echo -e $COLOR_GREEN"Qt successfully installed!"$COLOR_RESET
#   UR_ERROR=0
# else
#   echo -e $COLOR_RED"Qt installation failed!"$COLOR_RESET
#   UR_ERROR=1
# fi
