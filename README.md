# UR5e_Unity

This project demonstrates using **Unity Simulation Engine** and **Moveit2** motion planning framework with **ROS2 Humble** for the middleware communication. It features a 6-dof **Universal Robots UR5e**. Later sections also describe the project working and outline a general process for getting any robot Arm working in a Unity Simulation with moveit.

https://github.com/arthurgomes4/UR5e_Unity/assets/55613848/5aa093a9-2fa8-4bdb-91b4-bf910719bcf9

## Quick Start

### Install Prerequisites
This project can be used with either docker or a native ROS2 humble installation. Unity needs to be installed natively in both cases.
* Install the [Unityhub](https://docs.unity3d.com/hub/manual/InstallHub.html#install-hub-linux), once installed, install editor version: `2022.3.1711` or newer. (older versions may also work, but no guarantees given).
* Install git. Most users will have git preinstalled but if not then install from [here](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git).

* **If using docker**: Install [docker engine](https://docs.docker.com/engine/install/ubuntu/) and perform [post-install steps](https://docs.docker.com/engine/install/linux-postinstall/). (recommended way).
* **If using ROS2 natively**: The project requires an installation of ROS2 Humble to work, although it should work on other ROS2 distros just as well. [Install humble](https://docs.ros.org/en/humble/Installation.html) if not installed already.

### Cloning, installing dependencies and building workspace
These instructions are for Linux:

**docker users**: Clone the repository an build the docker image with:
```bash
git https://github.com/arthurgomes4/UR5e_Unity.git # -b <name> if using any branch

cd UR5e_Unity && ./use_docker.sh --build # build the image
```

**Native ROS2 users**: Create a ROS2 workspace, clone the repo and install dependencies with the following commands:
```bash
mkdir -p ros2_ws/src

cd ros2_ws/src

git clone https://github.com/arthurgomes4/UR5e_Unity.git
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git -b main-ros2

sudo ./UR5e_Unity/install_dependencies.sh # root required

cd .. && colcon build
```

### Open Unity Project
Open the Unity Hub and add the [ur5e_project](./ur5e_project/). Open the project and ensure there are no compile errors. Open the [SampleScene.unity](./ur5e_project/Assets/Scenes/SampleScene.unity) and press the play button to start the simulation.

### Run ROS nodes

**docker users**: Run the container and enter the container with the [use_docker.sh](./use_docker.sh).
```bash
./use_docker.sh --run

./use_docker.sh --enter

ros2 launch ur5e_pkg system.launch.py # inside the container
```

**Native ROS2 users**: Source your workspace and run the ROS nodes with the following commands:
```bash
source install/setup.bash

ros2 launch ur5e_pkg system.launch.py
```

## Project Technical Description
![ur5e_unity](https://github.com/arthurgomes4/UR5e_Unity/assets/55613848/b67b212a-307d-48ba-8697-6abc5bfd2fff)

There are a few key areas of interest in this project. This section provides an explanation for the working of the project and the components used. Refer to the above figure when reading these points, they are numbered in sync with the image. 

1. **Unity Simulation**: Similar to the ROS1/2 plugins in gazebo classic/sim, interacting with your simulated robot model requires C# scripts to be written to communicate with the ROS side using a [TCP based API](https://github.com/Unity-Technologies/ROS-TCP-Connector) provided by Unity.

2. **TCP Endpoint**: These nodes form the other side of the Unity-ROS connection. They are provided by the [ros_tcp_endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) package. There exists one main server node and child nodes that are created dynamically during runtime for every incoming/outgoing topic connection.

3. **ROS2 Control**: [ROS2 control](https://control.ros.org/master/index.html) is a convenient method of deploying and managing controllers. It uses hardware interfaces to communicate with the robot either in simulation or in real-life. The exact functioning of ROS1/2 control may be different but the general gist of it is: A hardware interface is a class that is loaded at runtime by the controller manager and contains functions for reading robot state and writing commands. These functions are then used by [any selected controller](https://github.com/ros-controls/ros2_controllers) to provide a topic/action interface to control/read from the robot. In the case of this project, this hardware interface is provided by [ROS2 topic based control](https://github.com/PickNikRobotics/topic_based_ros2_control) and a [Unity script](./ur5e_project/Assets/Scripts/ROSTopicBasedControlPlugin.cs).

4. **Moveit2** (and other nodes):
[Moveit2](https://www.google.com/search?channel=fs&client=ubuntu-sn&q=moveit2) is a set of motion planning libraries, tools and packages that will provide motion planning. The move_group node is the executable that provides the ROS interfaces to receive commands and issue trajectories to the controller being used. The other nodes are:
    * robot state publisher: uses the robots joint states to produce a TF tree of the robot to reflect the changes in the robots kinematic chain.
    * Rviz: provides visualization and graphical control interface for moveit2.

## Guidelines for setting up with new robot

The setup for a new robot is relatively straightforward:

1. Set up your robot model in Unity. This step is a bit lengthy and will not be discussed in this readme, there is material available online to follow. For any doubts reach out to the [author](https://www.linkedin.com/in/arthur-francis-gomes/).

2. Use the ROS2 topic based control scheme mentioned above while taking inspiration from [control.launch.py](./ur5e_pkg/launch/control.launch.py) and [endpoint.launch.py](./ur5e_pkg/launch/endpoint.launch.py).

3. Create up your robots moveit config pkg using moveit setup assistant. Refer to [moveit.launch.py](./ur5e_pkg/launch/moveit.launch.py).  
