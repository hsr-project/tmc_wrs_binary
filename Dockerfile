# Copyright (c) 2019 TOYOTA MOTOR CORPORATION
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

#  * Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#  notice, this list of conditions and the following disclaimer in the
#  documentation and/or other materials provided with the distribution.
#  * Neither the name of Toyota Motor Corporation nor the names of its
#  contributors may be used to endorse or promote products derived from
#  this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
FROM devrt/ros-devcontainer-vscode:melodic-desktop AS base

USER root

ENV DEBIAN_FRONTEND noninteractive
ENV CC /usr/bin/gcc
ENV CXX /usr/bin/g++

# install depending packages
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install --no-install-recommends -y \
    mesa-utils htop imagemagick \
    ros-melodic-gazebo-ros ros-melodic-gazebo-plugins ros-melodic-gazebo-ros-control libgazebo9-dev libignition-transport4-dev libpoco-dev python-scipy libgsl-dev \
    ros-melodic-slam-karto \
    ros-melodic-rviz \
    ros-melodic-dwa-local-planner \
    ros-melodic-teleop-twist-joy \
    ros-melodic-eigen-conversions \
    ros-melodic-moveit-commander \
    ros-melodic-joy \
    ros-melodic-robot-state-publisher \
    ros-melodic-moveit-core \
    ros-melodic-moveit-ros-benchmarks \
    ros-melodic-moveit-planners-ompl \
    ros-melodic-moveit-ros-planning \
    ros-melodic-moveit-ros-visualization \
    ros-melodic-moveit-ros-move-group \
    ros-melodic-moveit-simple-controller-manager \
    ros-melodic-moveit-setup-assistant \
    ros-melodic-urdfdom-py \
    ros-melodic-roslint \
    ros-melodic-teleop-twist-keyboard \
    ros-melodic-joint-state-controller \
    ros-melodic-joint-trajectory-controller \
    ros-melodic-gmapping \
    ros-melodic-move-base \
    ros-melodic-map-server \
    ros-melodic-xacro \
    ros-melodic-joint-state-publisher \
    liburdfdom-tools \
    ros-melodic-image-proc \
    ros-melodic-depth-image-proc \
    ros-melodic-amcl \
    ros-melodic-effort-controllers \
    ros-melodic-ros-controllers \
    ros-melodic-hector-mapping \
    ros-melodic-pcl-ros && \
    pip install -U --ignore-installed pyassimp && \
    apt-get autoremove -y && \
    apt-get clean

FROM base AS build

SHELL ["/bin/bash", "-c"]

RUN mkdir /wrs_ws
ADD src /wrs_ws/src
RUN cd /wrs_ws/src && source /opt/ros/$ROS_DISTRO/setup.bash && catkin_init_workspace || true
#RUN cd /wrs_ws && source /opt/ros/$ROS_DISTRO/setup.bash && rosdep update && rosdep install --from-paths src --ignore-src -r -y
RUN cd /wrs_ws && source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make install -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/wrs -DCATKIN_ENABLE_TESTING=0

FROM base

RUN mv /entrypoint.sh /entrypoint-original.sh
ADD entrypoint-wrs.sh /entrypoint.sh

ADD start-simulator.sh /start-simulator.sh
ADD start-simulator-highrtf.sh /start-simulator-highrtf.sh
ADD start-simulator-fast.sh /start-simulator-fast.sh
ADD start-simulator-fast-highrtf.sh /start-simulator-fast-highrtf.sh

COPY --from=build /opt/wrs /opt/wrs
#ADD filterable-rosmaster.py /opt/ros/kinetic/bin/
#RUN rm /opt/ros/kinetic/bin/rosmaster && ln -s /opt/ros/kinetic/bin/filterable-rosmaster.py /opt/ros/kinetic/bin/rosmaster
