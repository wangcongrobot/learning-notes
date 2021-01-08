# Husky Dual UR5 Mobile Manipulation Demo

https://www.clearpathrobotics.com/assets/guides/husky/HuskyDualManip.html

robotiq_fts300: 0.037547

## install

P.S.: The official ROS package is Indigo, we need to update it to Kinetic version.

```bash
mkdir -p ~/dual_ws/src
cd ~/dual_ws/src && catkin_init_workspace
git clone https://github.com/DualUR5Husky/husky
git clone https://github.com/DualUR5Husky/ur_modern_driver
git clone https://github.com/DualUR5Husky/universal_robot
git clone https://github.com/DualUR5Husky/robotiq
git clone https://github.com/DualUR5Husky/husky_simulator
git clone https://github.com/DualUr5Husky/flir_ptu
cd ..

sudo apt-get update
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```

### Bugs

1. MoveIt!

MoveIt has changed a lot from Indigo to Kinetic. We need to change some code to adapt it.

```bash
/home/cong/ros_ws/dual_ws/src/universal_robot/ur_kinematics/include/ur_kinematics/ur_moveit_plugin.h:89:48: fatal error: moveit_msgs/GetKinematicSolverInfo.h: No such file or directory
compilation terminated.
```
Hi, had a similar issue, and fixed it in my IK plugin by this simple action

#include <moveit_msgs/GetKinematicSolverInfo.h>
should be replaced by
#include <moveit_msgs/KinematicSolverInfo.h>

## Upgrade the ROS Version from Indigo to Kinetic

1. UR

Remove the origial package and download latest UR driver and MoveIt package.

```bash
git clone -b kinetic-devel https://github.com/ros-industrial/universal_robot.git
git clone -b kinetic-devel https://github.com/ros-industrial/ur_modern_driver.git

https://github.com/ros-industrial/industrial_core.git
sudo apt-get install ros-kinetic-industrial-core

sudo apt-get install ros-kinetic-universal-robots

roslaunch husky_gazebo husky_empty_world.launch

```

## Husky images

http://packages.clearpathrobotics.com/stable/images/latest/kinetic-husky/amd64/kinetic-husky-amd64-0.3.7.iso



