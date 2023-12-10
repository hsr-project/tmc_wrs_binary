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
FROM ros:noetic

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && \
    apt-get install -y curl git apt-transport-https python3-pip python-is-python3 && \
    apt-get clean

# OSRF distribution is better for gazebo
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    curl -L http://packages.osrfoundation.org/gazebo.key | apt-key add -

# install depending packages
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install --no-install-recommends -y \
    ros-noetic-gazebo-ros ros-noetic-gazebo-plugins ros-noetic-gazebo-ros-control libgazebo11-dev libignition-transport8-dev libpoco-dev python3-scipy libgsl-dev \
    ros-noetic-dwa-local-planner \
    ros-noetic-eigen-conversions \
    ros-noetic-robot-state-publisher \
    ros-noetic-moveit-core \
    ros-noetic-moveit-plugins \
    ros-noetic-moveit-planners-ompl \
    ros-noetic-moveit-ros-planning \
    ros-noetic-moveit-ros-move-group \
    ros-noetic-moveit-ros-manipulation \
    ros-noetic-moveit-simple-controller-manager \
    ros-noetic-urdfdom-py \
    ros-noetic-roslint \
    ros-noetic-joint-state-controller \
    ros-noetic-joint-trajectory-controller \
    ros-noetic-move-base \
    ros-noetic-map-server \
    ros-noetic-xacro \
    ros-noetic-joint-state-publisher \
    liburdfdom-tools \
    ros-noetic-image-proc \
    ros-noetic-depth-image-proc \
    ros-noetic-effort-controllers \
    ros-noetic-ros-controllers \
    ros-noetic-pcl-ros \
    ros-noetic-tf-conversions \
    ros-noetic-moveit-ros-perception \
    python-configparser && \
    pip install -U --ignore-installed pyassimp supervisor supervisor_twiddler && \
    apt-get autoremove -y && \
    apt-get clean

RUN mkdir /wrs_ws
ADD src /wrs_ws/src
RUN cd /wrs_ws/src && source /opt/ros/$ROS_DISTRO/setup.bash && catkin_init_workspace || true
#RUN cd /wrs_ws && source /opt/ros/$ROS_DISTRO/setup.bash && rosdep update && rosdep install --from-paths src --ignore-src -r -y
RUN cd /wrs_ws && source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make install -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO -DCATKIN_ENABLE_TESTING=0

ADD entrypoint-wrs.sh /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

#ADD filterable-rosmaster.py /opt/ros/noetic/bin/
#RUN rm /opt/ros/$ROS_DISTRO/bin/rosmaster && ln -s /opt/ros/$ROS_DISTRO/bin/filterable-rosmaster.py /opt/ros/$ROS_DISTRO/bin/rosmaster

RUN source /opt/ros/$ROS_DISTRO/setup.bash && rosrun tmc_gazebo_task_evaluators setup_score_widget

ADD supervisord.conf /etc/supervisor/supervisord.conf

VOLUME [ \
    "/opt/ros/noetic/share/hsrb_description", \
    "/opt/ros/noetic/share/hsrb_meshes", \
    "/opt/ros/noetic/share/tmc_wrs_gazebo_worlds", \
    "/opt/ros/noetic/share/gazebo_ros", \
    "/opt/ros/noetic/lib/gazebo_ros", \
    "/opt/ros/noetic/lib/python2.7/dist-packages/gazebo_ros", \
    "/opt/ros/noetic/lib/python3/dist-packages/gazebo_ros", \
    "/opt/ros/noetic/lib/python2.7/dist-packages/gazebo_msgs", \
    "/opt/ros/noetic/lib/python3/dist-packages/gazebo_msgs", \
    "/opt/ros/noetic/share/hsrb_rosnav_config", \
    "/opt/ros/noetic/share/tmc_control_msgs", \
    "/opt/ros/noetic/lib/python2.7/dist-packages/tmc_control_msgs", \
    "/opt/ros/noetic/lib/python3/dist-packages/tmc_control_msgs", \
    "/opt/ros/noetic/include/tmc_control_msgs" \
    ]

CMD ["/usr/local/bin/supervisord", "-n", "-c", "/etc/supervisor/supervisord.conf"]

