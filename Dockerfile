FROM ros:humble-ros-base

# set frontend noninteractive
ENV DEBIAN_FRONTEND noninteractive

# Prevent hash mismatch error for apt-get update, qqq makes the terminal quiet while downloading pkgs
RUN apt-get clean && rm -rf /var/lib/apt/lists/* && apt-get update -yqqq

# create a ROS workspace
ARG ROS2_WS=/root/ros2_ws
RUN mkdir -p ${ROS2_WS}/src

# copy the contents of repo
COPY . ${ROS2_WS}/src

# install dependencies
RUN ${ROS2_WS}/src/install_dependencies.sh

# clone the tcp endpoint repo
RUN cd ${ROS2_WS}/src && git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git -b ROS2v0.7.0

# build the workspace
SHELL ["/bin/bash", "-c"]

RUN cd ${ROS2_WS} && source /opt/ros/${ROS_DISTRO}/setup.sh && colcon build

# copy the ROS paths to the bashrc
RUN echo "source ${ROS2_WS}/install/setup.bash" >> /root/.bashrc