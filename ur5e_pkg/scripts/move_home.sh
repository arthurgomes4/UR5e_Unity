#!/bin/bash

# Move the UR5e to the home position
ros2 topic pub --once /joint_command std_msgs/msg/Float64MultiArray "{data:[0,0,0,0,0,0]}"