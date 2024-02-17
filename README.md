# UR5e_Unity
The UR5e arm in unity simulation interfaced with ROS2 humble

### Aim
The Robot arm in a unity simulation with a moveit2 planning pipeline in ROS2 humble.

### Completed
* RViz visualization and TCP connection

### Underway
* Unity Project - testing underway, no unknowns

### Pending
* Moveit2 integration - will use defaults from UR5e packages.
```bash
xacro /opt/ros/$ROS_DISTRO/share/ur_description/urdf/ur.urdf.xacro name:=ur5e ur_type:=ur5e > ur5e.urdf
```
