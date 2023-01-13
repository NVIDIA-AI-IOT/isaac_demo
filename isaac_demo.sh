#!/bin/bash
# Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

bold=`tput bold`
red=`tput setaf 1`
green=`tput setaf 2`
yellow=`tput setaf 3`
reset=`tput sgr0`

# Requested version to install this set of demo on Jetson
ISAAC_DEMO_ROS_L4T="35.1" # 35.1 = Jetpack 5.0.2
ISAAC_SIM_VERSION="2022.2.0"  # Isaac SIM version

# DO NOT EDIT

ISAAC_ROS_PATH="$(pwd)/isaac_ros-dev/ros_ws"
ISAAC_ROS_SRC_PATH="$ISAAC_ROS_PATH/src"
ISAAC_SIM_PATH="$HOME/.local/share/ov/pkg/isaac_sim-$ISAAC_SIM_VERSION"
ISAAC_SIM_ROS_PATH="$ISAAC_SIM_PATH/ros2_workspace"
ISAAC_SIM_ROS_SRC_PATH="$ISAAC_SIM_ROS_PATH/src"
PROJECT_PATH=$(pwd)

workstation_install()
{
    if [ ! -d $ISAAC_SIM_PATH ] ; then
        echo "${bold}${red}Install Isaac SIM $ISAAC_SIM_VERSION on your workstation${reset}"
        exit 1
    fi

    echo "${green}${bold}Update/Install on desktop${reset}"

    echo " - ${green}Pull or update all Isaac ROS packages${reset}"
    cd $ISAAC_SIM_ROS_PATH
    # Recursive import
    # https://github.com/dirk-thomas/vcstool/issues/93
    vcs import src < $PROJECT_PATH/rosinstall/isaac_demo_workstation.rosinstall --recursive
    vcs pull src

    if [ -d $HOME/.ros/ ] ; then
        if [ ! -f $HOME/.ros/fastdds.xml ] ; then
            echo " - ${green}Copy Fast DDS configuration on .ros folder${reset}"
            cp $PROJECT_PATH/fastdds.xml $HOME/.ros/fastdds.xml
        fi
    else
        echo "${bold}${red}ROS not installed${reset}"
    fi

    echo " - ${green}Start Isaac SIM simulation${reset}"
    # source /opt/ros/foxy/setup.bash
    # https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox/blob/main/docs/tutorial-isaac-sim.md
    # Run Isaac ROS with Carter in a Warehouse
    $ISAAC_SIM_PATH/python.sh $ISAAC_SIM_ROS_SRC_PATH/isaac_ros_nvblox/nvblox_isaac_sim/omniverse_scripts/carter_warehouse.py
}

jetson_l4t_check()
{
    # Read version
    local JETSON_L4T_STRING=$(dpkg-query --showformat='${Version}' --show nvidia-l4t-core)
    # extract version
    local JETSON_L4T_ARRAY=$(echo $JETSON_L4T_STRING | cut -f 1 -d '-')
    # Load release and revision
    local JETSON_L4T_RELEASE=$(echo $JETSON_L4T_ARRAY | cut -f1,2 -d '.')
    local JETSON_L4T_REVISION=${JETSON_L4T_ARRAY#"$JETSON_L4T_RELEASE."}
    echo "$JETSON_L4T_RELEASE"
}


jetson_install()
{
    local JETSON_L4T=$(jetson_l4t_check)
    if [[ $JETSON_L4T != $ISAAC_DEMO_ROS_L4T ]] ; then
        echo "${bold}${red}You cannot install isaac_demo on this Jetpack with L4T $JETSON_L4T need L4T $ISAAC_DEMO_ROS_L4T${reset}"
        exit 1
    fi

    echo "${green}${bold}Install on NVIDIA Jetson L4T $JETSON_L4T${reset}"

    if [ ! -d $ISAAC_ROS_SRC_PATH ] ; then
        echo " - ${green}Make folder $ISAAC_ROS_SRC_PATH${reset}"
        mkdir -p $ISAAC_ROS_SRC_PATH
    fi

    echo " - ${green}Pull or update all Isaac ROS packages${reset}"
    cd $ISAAC_ROS_PATH
    # Recursive import
    # https://github.com/dirk-thomas/vcstool/issues/93
    vcs import src < $PROJECT_PATH/rosinstall/isaac_demo_jetson.rosinstall --recursive
    vcs pull src

    if [ ! -f $ISAAC_ROS_SRC_PATH/isaac_ros_common/scripts/.isaac_ros_common-config  ] ; then
        echo " - ${green}Setup Isaac ROS docker image${reset}"
        cd $ISAAC_ROS_SRC_PATH/isaac_ros_common/scripts
        touch .isaac_ros_common-config 
        echo CONFIG_IMAGE_KEY=humble.nav2.realsense > .isaac_ros_common-config
    fi

    echo " - ${green}Move to Isaac ROS common and run image${reset}"
    cd $ISAAC_ROS_SRC_PATH/isaac_ros_common
    bash scripts/run_dev.sh $ISAAC_ROS_PATH
}


usage()
{
    if [ "$1" != "" ]; then
        echo "${red}$1${reset}" >&2
    fi
    
    local name=$(basename ${0})
    echo "This script install isaac_demo on your desktop or your NVIDIA Jetson" >&2
    echo "$name [options]" >&2
    echo "${bold}options:${reset}" >&2
    echo "   -y                   | Run this script silent" >&2
    echo "   -h|--help            | This help" >&2
}


main()
{
    local PLATFORM="$(uname -m)"
    local SILENT=false

    # Decode all information from startup
    while [ -n "$1" ]; do
        case "$1" in
            -h|--help) # Load help
                usage
                exit 0
                ;;
            -y)
                SILENT=true
                ;;
            *)
                usage "[ERROR] Unknown option: $1" >&2
                exit 1
                ;;
        esac
            shift 1
    done

    # Recap installatation
    echo "------ Configuration ------"
    echo " - ${bold}Install on:${reset} ${green}$PLATFORM${reset}"
    echo " - ${bold}User:${reset} ${green}$USER${reset} - ${bold}Hostname:${reset} ${green}$HOSTNAME${reset}"
    if [ $PLATFORM = "aarch64" ] ; then
        local JETSON_L4T=$(jetson_l4t_check)
        echo " - ${bold}Jetson L4T:${reset} ${green}$JETSON_L4T${reset}"
    fi
    echo "---------------------------"

    """
    while ! $SILENT; do
        read -p "Do you wish to install isaac_demo on this platform? [Y/n] " yn
            case $yn in
                [Yy]* ) break;;
                [Nn]* ) exit;;
            * ) echo "Please answer yes or no.";;
        esac
    done

    # Request sudo password
    sudo -v
    """

    if ! command -v vcs &> /dev/null ; then
        echo " - ${green}Install required packages${reset}"
        sudo apt install -y git-lfs python3-vcstools
        sudo pip3 install -U vcstool
    fi

    # Run a different installation depend of the architecture
    if [[ $PLATFORM != "aarch64" ]]; then
        workstation_install
    else
        jetson_install
    fi

}

main $@
# EOF
