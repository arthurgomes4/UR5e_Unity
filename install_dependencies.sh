#!/bin/bash

apt-get update && apt-get install python3-pip -y

apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-rviz2 \
    ros-humble-ros2-control \
    ros-humble-ur-description \
    ros-humble-ur-msgs \
    ros-humble-ur-moveit-config \
    ros-humble-joint-state-publisher-gui \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-robot-state-publisher \
    ros-humble-urdf \
    ros-humble-xacro \
    ros-humble-rosbridge-suite \
    ros-humble-joint-state-publisher \
    ros-humble-ros2-controllers \
    ros-humble-topic-based-ros2-control

apt-get update -y && pip install rospkg jsonpickle