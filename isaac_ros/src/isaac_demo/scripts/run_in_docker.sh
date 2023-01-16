#!/bin/bash
# Copyright (c) 2023, NVIDIA CORPORATION. All rights reserved.
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

LOCAL_PATH="/workspaces/isaac_ros-dev"
ISAAC_DEMO_PKG_PATH="$LOCAL_PATH/src/isaac_demo"

main()
{
    local LIBWEBSOCKETPP_PKG=$(dpkg -l 2>/dev/null | grep -m1 "libwebsocketpp")
    if [ -z "$LIBWEBSOCKETPP_PKG" ] ; then
        echo " - ${green}Install dependencies foxglove websocket${reset}"
        sudo apt-get update
        sudo apt-get install -y libwebsocketpp-dev
        sudo rm -rf /var/lib/apt/lists/*
        sudo apt-get clean
    fi

    if [ -d $HOME/.ros/ ] ; then
        if [ ! -f $HOME/.ros/fastdds.xml ] ; then
            echo " - ${green}Copy Fast DDS configuration on .ros folder${reset}"
            cp $ISAAC_DEMO_PKG_PATH/scripts/fastdds.xml $HOME/.ros/fastdds.xml
        fi
    fi

    if [ ! -d $LOCAL_PATH/install ] ; then
        echo " - ${green}Build Isaac ROS${reset}"
        colcon build --symlink-install --merge-install
    fi
    
    echo " - ${green}Run isaac_demo${reset}"
    # source workspace
    source install/setup.bash
    # Run demo
    ros2 launch isaac_demo carter.launch.py
}

main $@
# EOF
