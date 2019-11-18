# How to use a model trained in mujoco to control a ROS-based robot

As we know, ROS is the most popular framework for robotics and MuJoCo Simulator is the most popular physical simulation egine. So, when you train a model in mujoco, how to apply it to the real robot (equal to the communicate with ROS)?

## [mujoco_ros_control](https://github.com/gamleksi/mujoco_ros_control)

The ros_control interface for the MuJoCo simulator. The code parses a given model and register a control interface for each slide or hinge joint. The ros_control effort based controllers could be then loaded as shown in example start_simple_robot.launch. It provides trajectory based interface e.g. for MoveIt.

This package is part of project [Affordance Learning for End-to-End Visuomotor Robot Control](https://github.com/gamleksi/affordance_gym). 


ROS Kinetic, MoveIt!, Mujoco(2.0), 

mujoco_control.cpp
void render(mjrRect &viewport) {
    std::cout << "Render" << std::endl;
    // update abstract scene
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    std::cout << "Render1" << std::endl;
    // render scene in offscreen buffer
//    mjr_render(viewport, &scn, &con);
    std::cout << "Render3" << std::endl;
}


## [mujoco_ros_pkgs](https://github.com/shadow-robot/mujoco_ros_pkgs)

ROS integration of Mujoco simulator

```bash
$ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/cong/.mujoco/mjpro150/bin
cong@eclipse:~/ros_ws/mujoco_ros_ws$ roslaunch sr_robot_launch srhand_mujoco.launch
```
rosdep install --from-paths src --ignore-src -r -y


## [gym-sawyer](https://github.com/rlworkgroup/gym-sawyer/blob/master/sawyer/ros/envs/sawyer/README.md)

- catkin_make error:
```bash
$ ImportError: "from catkin_pkg.package import parse_package" failed: No module named 'catkin_pkg'
$ Make sure that you have installed "catkin_pkg", it is up to date and on the PYTHONPATH.
```

[solution](https://stackoverflow.com/questions/43024337/why-this-error-when-i-try-to-create-workspaces-in-ros):

```bash
$ locate catkin_pkg
$ /usr/lib/python2.7/dist-packages/catkin_pkg
$ echo $PYTHONPATH
$ /opt/ros/kinetic/lib/python2.7/dist-packages
$ export PYTHONPATH=$PYTHONPATH:/usr/lib/python2.7/dist-packages
$ echo $PYTHONPATH
$ /opt/ros/kinetic/lib/python2.7/dist-packages:/usr/lib/python2.7/dist-packages/
```

- error: install mujoco that changed LD_LIBARY_PATH
https://answers.ros.org/question/205962/cant-find-libroscppso-fedora-21/

https://answers.ros.org/question/220546/catkin_make-failure-due-to-python-anaconda/

Delete your build folder after turning off anaconda. CMake will cache the Python executable it finds. 

- error: ModuleNotFoundError: No module named 'em'

sollution: 
```bash
pip uninstall em
pip install empy
```


## github

-[sawyer-control](https://github.com/mdalal2020/sawyer_control)

Python3 ROS Interface to Rethink Sawyer Robots with OpenAI Gym Compatibility

- [foresight_rospkg](https://github.com/SudeepDasari/visual_foresight/tree/master/visual_mpc/foresight_rospkg)

- [SawyerImitation](https://github.com/SudeepDasari/SawyerImitation)

imitation_rospkg

- [Working with real robots: baxter, robobo and omnirobot](https://s-rl-toolbox.readthedocs.io/en/latest/guide/real_robots.html)

- [Reinforcement Learning with ROS and Gazebo](https://github.com/vmayoral/basic_reinforcement_learning/blob/master/tutorial7/README.md)

Using [Gazebo-Gym](http://erlerobotics.com/whitepaper/robot_gym.pdf).

- [rl-gazebo-ur5](https://github.com/hjalte33/rl_unscrew)


## mujoco ros related package

- [mujoco_ros_test](https://github.com/savik28/mujoco_test)

ROS - Mujoco bridging and visualisation of simulation inrviz and usage of kdl to compute gravitational force

- [Manipulating Object Collections Using Grounded State Representations](https://github.com/matwilso/object_collections)

It relates the mujoco and ros. In the scripts/ folder: Misc scripts for doing some things, including real world evaluations. Require **ROS/Baxter/MoveIt** knowledge, not likely very useful.