# How to use a model trained in mujoco to control a ROS-based robot

As we know, ROS is the most popular framework for robotics and MuJoCo Simulator is the most popular physical simulation egine. So, when you train a model in mujoco, how to apply it to the real robot (equal to the communicate with ROS)?

## [mujoco_ros_control](https://github.com/gamleksi/mujoco_ros_control)

The ros_control interface for the MuJoCo simulator. The code parses a given model and register a control interface for each slide or hinge joint. The ros_control effort based controllers could be then loaded as shown in example start_simple_robot.launch. It provides trajectory based interface e.g. for MoveIt.

This package is part of project [Affordance Learning for End-to-End Visuomotor Robot Control](https://github.com/gamleksi/affordance_gym). 








## [mujoco_ros_pkgs](https://github.com/shadow-robot/mujoco_ros_pkgs)

ROS integration of Mujoco simulator