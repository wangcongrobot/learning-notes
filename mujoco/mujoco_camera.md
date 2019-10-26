# Model a Camera in Mujoco



## Preparing simulation models for roboitc tasks

The objective of this project was to create MuJoCo simulation models for a door opening task for the Franka Panda robot.

https://github.com/gamleksi/mujoco_ros_control

The ros_control interface for the MuJoCo simulator. The code parses a given model and register a control interface for each slide or hinge joint. The ros_control effort based controllers could be then loaded as shown in example start_simple_robot.launch. It provides trajectory based interface e.g. for MoveIt.

https://www.youtube.com/playlist?list=PLwO8awctVMheqR0lSKW-R1jILyXx6po5N

Mujoco joint ROS Gazebo to open the door.