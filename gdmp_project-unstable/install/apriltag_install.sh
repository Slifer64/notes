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


# ********************************************
# **********   Utility functions   ***********
# ********************************************

# FUNCTION: Prints an info message.
# @arg1: the message
print_info_msg ()
{
  local msg=$1

  echo -e $COLOR_BLUE"$msg"$COLOR_RESET
}

# FUNCTION: Prints a success message.
# @arg1: the message
print_success_msg ()
{
  local msg=$1

  echo -e $COLOR_GREEN"$msg"$COLOR_RESET
}

# FUNCTION: Prints a warning message.
# @arg1: the message
print_warning_msg ()
{
  local msg=$1

  echo -e $COLOR_YELLOW"$msg"$COLOR_RESET
}

# FUNCTION: Prints a failure message.
# @arg1: the message
print_fail_msg ()
{
  local msg=$1

  echo -e $COLOR_RED"$msg"$COLOR_RESET
}

# FUNCTION: checks if the last command command succeded. On failure, terminates the script.
# @arg1: the return value of the last executed command ($?)
# @arg2: a message to print on failure
# @arg3: a message to print on success (optional)
check_success ()
{
  local n_args=$#
  local status=$1
  local fail_msg=$2
  local success_msg=$3

  if [ $status -ne 0 ]; then
    print_fail_msg "$fail_msg"
    exit
  elif [ $n_args -eq 3 ]; then
    print_success_msg "$success_msg"
  fi
}

# FUNCTION: Finds and the returns the path to a folder
# @arg1: the folder whose path is to be found
# @arg2: the 'root' folder from which the search will begin 
# @return: 'echos' the path if found or prints error and terminates on failure.
find_path () 
{
  local folder_name=$1
  local base_path=$2

  qdldl_sources_path=$(find $base_path -name $folder_name -print | head -n 1)

  if [ -z $qdldl_sources_path ]; then
    print_fail_msg "Failed to find ''$folder_name''..."
    exit
  fi
  
  echo $qdldl_sources_path
}

# FUNCTION: Checks if a specific folder exists and removes it.
# @arg1: the name of the folder to be removed.
remove_folder ()
{
  local folder_name=$1

  if [ -d $folder_name ]; then # if directory exists
    rm -rf $folder_name
  fi
}

download_repository ()
{
  local repo_link=$1
  local repo_name=$2

  print_info_msg "Downloading $repo_name git repository..."
  git clone $repo_link $repo_name
  check_success $? "Failed to download $repo_name git repository..."  "Downloaded $repo_name git repository!"
}

# ============================================================
# ===========  Download APRILTAG git repository  =============
# ============================================================

remove_folder "apriltag_lib"

download_repository "https://github.com/AprilRobotics/apriltag.git" "apriltag_lib"

root_path=$(pwd) # store the current root path

# Set the installation folder 
# install_path=${root_path}"/"$(find_path "apriltag" "src/")
install_path=${root_path}"/src/apriltag_ros/apriltag/"
# remove_folder ${install_path}
# mkdir ${install_path}

# ============ Build apriltag ============

print_info_msg "Building apriltag..."

cd apriltag_lib/

mkdir build # folder where the files will be built
mkdir lib # installation folder

cd build/
cmake -DCMAKE_INSTALL_PREFIX:PATH=${install_path} ..
make
check_success $? "Failed to build apriltag..." "Built apriltag successfully!"

# ============ Install apriltag ============
print_info_msg "Installing apriltag to '${install_path}'..."

make install
check_success $? "Failed to install apriltag..." "Installed apriltag successfully!"

# clean up
cd ${root_path}
remove_folder "apriltag_lib/"





