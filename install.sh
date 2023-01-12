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
ISAAC_DEMO_L4T="35.1" # Jetpack 5.0.2
ISAAC_DEMO_PATH="$(pwd)/isaac_ros-dev/ros_ws"
ISAAC_DEMO_SRC_PATH="$ISAAC_DEMO_PATH/src"


workstation_install()
{
    echo "${green}${bold}Install on Desktop${reset}"
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
    echo $JETSON_L4T_RELEASE
}


jetson_install()
{
    local JETSON_L4T=$(jetson_l4t_check)
    if [[ $JETSON_L4T != $ISAAC_DEMO_L4T ]] ; then
        echo "${bold}${red}You cannot install isaac_demo on this Jetpack with L4T $JETSON_L4T need L4T $ISAAC_DEMO_L4T${reset}"
        exit 1
    fi

    echo "${green}${bold}Install on NVIDIA Jetson L4T $JETSON_L4T${reset}"
    echo " - ${green}Install required packages${reset}"
    sudo apt install -y git-lfs python3-vcstools
    if ! command -v vcs &> /dev/null ; then
        sudo pip3 install -U vcstool
    fi

    if [ ! -d $ISAAC_DEMO_SRC_PATH ] ; then
        echo " - ${green}Make folder $ISAAC_DEMO_SRC_PATH${reset}"
        mkdir -p $ISAAC_DEMO_SRC_PATH
    fi

    local project_path=$(pwd)

    echo " - ${green}Pull or update all Isaac ROS packages${reset}"
    cd $ISAAC_DEMO_PATH
    # Recursive import
    # https://github.com/dirk-thomas/vcstool/issues/93
    vcs import src < $project_path/isaac_demo.rosinstall --recursive
    vcs pull src

    echo " - ${green}Move to Isaac ROS common and run image${reset}"
    cd $ISAAC_DEMO_SRC_PATH/isaac_ros_common
    bash scripts/run_dev.sh $ISAAC_DEMO_PATH
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

    
    echo $l4t

    while ! $SILENT; do
        read -p "Do you wish to install isaac_demo on this platform? [Y/n] " yn
            case $yn in
                [Yy]* ) # Break and install jetson_stats 
                        break;;
                [Nn]* ) exit;;
            * ) echo "Please answer yes or no.";;
        esac
    done

    # Request sudo password
    sudo -v

    # Run a different installation depend of the architecture
    if [[ $PLATFORM != "aarch64" ]]; then
        workstation_install
    else
        jetson_install
    fi

}

main $@
# EOF
