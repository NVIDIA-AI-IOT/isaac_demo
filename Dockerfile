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

ARG BASE_IMAGE=rbonghi/isaac-ros-base:gems-devel-4.5.0-cuda11.4.1-ubuntu20.04-L4T35.2
FROM ${BASE_IMAGE}


ENV DEMO_ROOT=/opt/ros/demo_ws

RUN mkdir -p ${DEMO_ROOT}/src \
    && cd ${DEMO_ROOT} \
    
    && rm -Rf /var/lib/apt/lists/* \
    && apt-get clean \
    # Build Isaac ROS and clean resources
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -Rf src build log