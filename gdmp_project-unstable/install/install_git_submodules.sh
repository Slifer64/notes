#!/bin/bash

# ==================================
# define some colors for output
# ==================================
COLOR_RED="\033[1;31m"
COLOR_GREEN="\033[1;32m"
COLOR_YELLOW="\033[1;33m"
COLOR_BLUE="\033[1;34m"
COLOR_CYAN="\033[1;36m"
COLOR_ORANGE="\033[0;33m"
COLOR_WHITE="\033[1;37m"
COLOR_RESET="\033[0m"


echo -e $COLOR_CYAN"*************************************"$COLOR_RESET
echo -e $COLOR_CYAN"********    GIT SUBMODULES   ********"$COLOR_RESET
echo -e $COLOR_CYAN"*************************************"$COLOR_RESET

git_repo=$COLOR_ORANGE"cpp_plot_lib"$COLOR_RESET

# Create a new empty folder where the git repo will be downloaded
install_path="src/lib/plot_lib"
if [ -d $install_path ]; then
    rm -rf $install_path
fi
# mkdir $install_path

echo -e $COLOR_BLUE"Downloading "$git_repo$COLOR_BLUE"..."$COLOR_RESET
git clone https://github.com/Slifer64/cpp_plot_lib.git

if [ $? -ne 0 ]; then
    echo -e $COLOR_RED"Failed to download "$git_repo$COLOR_RED"...."$COLOR_RESET
else
    mv cpp_plot_lib/src/plot_lib $install_path  && \
    rm -rf cpp_plot_lib  && \
    echo -e $COLOR_GREEN"Downloaded "$git_repo$COLOR_GREEN" successfully"$COLOR_RESET
fi
