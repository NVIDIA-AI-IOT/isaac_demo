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

run_desktop()
{
    if [ ! -d $LOCAL_PATH/install ] ; then
        echo " - ${green}Build Isaac ROS${reset}"
        colcon build --symlink-install --merge-install --packages-up-to nvblox_rviz_plugin carter_description
    fi
    
    echo " - ${green}Run rviz2${reset}"
    # source workspace
    source install/setup.bash
    # Run demo
    rviz2 -d $ISAAC_DEMO_PKG_PATH/rviz/carter.rviz
}


run_jetson()
{
    local LIBWEBSOCKETPP_PKG=$(dpkg -l 2>/dev/null | grep -m1 "libwebsocketpp")
    if [ -z "$LIBWEBSOCKETPP_PKG" ] ; then
        echo " - ${green}Install dependencies foxglove websocket${reset}"
        sudo apt-get update
        sudo apt-get install -y libwebsocketpp-dev
        sudo rm -rf /var/lib/apt/lists/*
        sudo apt-get clean
    fi
    
    #echo " - ${green}Download model${reset}"
    #cd $LOCAL_PATH/src/isaac_ros_object_detection/isaac_ros_detectnet
    #./scripts/setup_model.sh --height 632 --width 1200 --config-file resources/quickstart_config.pbtxt
    
    if [ ! -d $LOCAL_PATH/bi3d ] ; then
        echo " - ${green}Download models for isaac_ros_proximity_segmentation${reset}"
        mkdir -p bi3d
        cd bi3d
        wget 'https://api.ngc.nvidia.com/v2/models/nvidia/isaac/bi3d_proximity_segmentation/versions/2.0.0/files/featnet.onnx'
        wget 'https://api.ngc.nvidia.com/v2/models/nvidia/isaac/bi3d_proximity_segmentation/versions/2.0.0/files/segnet.onnx'
        
        # Build models
        echo " - ${green}Build models for isaac_ros_proximity_segmentation${reset}"
        /usr/src/tensorrt/bin/trtexec --saveEngine=$LOCAL_PATH/bi3d/bi3dnet_featnet.plan --onnx=$LOCAL_PATH/bi3d/featnet.onnx --int8 --useDLACore=0
        /usr/src/tensorrt/bin/trtexec --saveEngine=$LOCAL_PATH/bi3d/bi3dnet_segnet.plan --onnx=$LOCAL_PATH/bi3d/segnet.onnx --int8 --useDLACore=0
        
        cd $LOCAL_PATH
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


main()
{
    local PLATFORM="$(uname -m)"
    local TESTRESOURCES_PKG=$(dpkg -l 2>/dev/null | grep -m1 "python3-testresources")
    if [ -z "$TESTRESOURCES_PKG" ] ; then
        echo " - ${green}Install dependencies testresources${reset}"
        sudo apt-get update
        sudo apt-get install -y python3-testresources
        sudo python3 -m pip install -U pip "setuptools<66.0.0"
        sudo rm -rf /var/lib/apt/lists/*
        sudo apt-get clean
    fi
    
    if [ -d $HOME/.ros/ ] ; then
        if [ ! -f $HOME/.ros/fastdds.xml ] ; then
            echo " - ${green}Copy Fast DDS configuration on .ros folder${reset}"
            cp $ISAAC_DEMO_PKG_PATH/scripts/fastdds.xml $HOME/.ros/fastdds.xml
        fi
    fi
    
    if [[ $PLATFORM != "aarch64" ]]; then
        run_desktop
    else
        run_jetson
    fi
}


main $@
# EOF
