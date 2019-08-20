# Reinforcement Learning Resource with Manipulation

This a collection of some useful resource of reinforcement learning and manipulation. It's hard for beginner in robotics and reinforcement learning. The resource is not 

__*Talk is cheap. Show me the code.*__
			--- Linus Torvalds


# Table of Contents

[TOC]

# Manipulation and Grasping with Deep Reinforcement Learning

## Code

- [Stochastic Latent Actor-Critic](https://github.com/alexlee-gk/slac)

Code will be released soon.

https://alexlee-gk.github.io/slac/

- []()

- []()

- []()

- []()

## Paper




# Useful things for my works

-[Setting  up  a  Reinforcement  Learning  Task  with  a  Real-World  Robot](https://arxiv.org/pdf/1803.07067.pdf)

How to set up a RL task with UR5 robot

- [Benchmarking Reinforcement Learning Algorithmson Real-World Robots](https://arxiv.org/pdf/1809.07731.pdf)

Using UR5 robot in RL. Source code for all tasks available at https://github.com/kindredresearch/SenseAct

- 




# Reinforcement Learning Framework

- [OpenAI gym](https://gym.openai.com/)

A toolkit for developint and comparing reinforcement learning algorithms.


- [garage](https://github.com/rlworkgroup/garage)

A toolkit for reproducible reinforcement learning research.

[Document](https://garage.readthedocs.org/en/latest/)

- [rllab](https://github.com/rll/rllab)

rllab is a framework for developing and evaluating reinforcement learning algorithms, fully compatible with OpenAI Gym. Now maintains it under the name [garage](https://github.com/rlworkgroup/garage)

[Document](https://rllab.readthedocs.org/en/latest/)





- [PyRobot]()

[PyRobot: An Open-source Robotics Framework for Research and Benchmarking](https://arxiv.org/pdf/1906.08236.pdf)


# Reinforcement Learning Simulation

Introduce some useful simulators.

## Mujoco

The most popular simulator in reinforcement learning research, although it's not free.

- [Mujoco](http://www.mujoco.org/)

Offical webset, including the installation, documentation, forum, and some other useful resources.

- [Mujoco_py](https://github.com/openai/mujoco-py)

MuJoCo is a pyhsics engine for detailed, efficient rigid body simulations with contacts. mujoco_py allows using MuJoCo from Python 3.

- [Mujoco_ros_pkgs](https://github.com/shadow-robot/mujoco_ros_pkgs)
  
ROS integration of Mujoco simulator, developed by Shadow Robot.

## Bullet

- [bullet3](http://bulletphysics.org)

Bullet Physics SDK: real-time collision detection and multi-physics simulation for VR, games, visual effects, robotics, machine learning etc.

(https://github.com/bulletphysics/bullet3)

## Gazebo

Using Gazebo to train the RL agent.

#### Gym-Gazebo

[Robot_gym](https://arxiv.org/abs/1808.10369)

[Accelerated robot training through simulation in the cloud with ROS and Gazebo](https://medium.com/@vmayoral/accelerated-robot-training-through-simulation-in-the-cloud-with-ros-and-gazebo-bac6bc493520)

Code:

[gym-gazebo](https://github.com/erlerobot/gym-gazebo/)

[gym-gazebo2](https://github.com/AcutronicRobotics/gym-gazebo2)

## Unity

Unity is another open source reinforcement learning environment, has good render.

## Vrep



# Reinforcement Learning Code

## gym environment

- [gym-kuka-mujoco](https://github.com/HarvardAgileRoboticsLab/gym-kuka-mujoco)

An OpenAI gym environment for the Kuka arm, using Mujoco simulator.

- [gym-drake](https://github.com/HarvardAgileRoboticsLab/gym-drake)

Glue between [Drake](https://drake.mit.edu/) and OpenAI Gym. Including Kuka arm env.

- [pytorch-rl](https://github.com/navneet-nmk/pytorch-rl)

This repository contains model-free deep reinforcement learning algorithms implemented in Pytorch. Using mujoco Fetch and Hand environments.

- [ur5 rl ros gazebo](https://github.com/hjalte33/rl_unscrew)

A reinforcement-Learning Framework for testing learning agents on a UR5 manipulator. The framework consists of 2 ROS packages which are rl_gazebo_sim, rl_moveit_config. Besides the two ROS packages there is the folder rl-gym with contains an OpenAI Gym environment as well as python scripts for runnin reinforcement learning. The rl-gym folder is an implementation example on how to use the framework with OpenAI Gym

The framework is designed and prepared for screwing tasks meanning the simulated UR5 is fitted with a screw tool and the world is fitted with a block with a screw.

- [visual_pushing_grsaping](https://github.com/andyzeng/visual-pushing-grasping)

Train robotic agents to learn to plan pushing and grasping actions for manipulation with deep reinforcement learning. http://vpg.cs.princeton.edu/

	- UR5 robot and two finger robotiq gripper
	- V-REP simulator
	- Intel RealSense D415 Camera
	- Calibrating Camera Extrinsics
	- UR5 python IK library

[ikfastpy](https://github.com/andyzeng/ikfastpy)

Python wrapper over OpenRave's IKFast inverse kinematics solver for a UR5 robot arm.


- [DAPG for Dexterous Hand Manipulation](https://github.com/aravindr93/hand_dapg)

[Project](https://sites.google.com/view/deeprl-dexterous-manipulation)

It's a dexterous hand manipulation control suite, using mujoco simulator and gym RL environment.


- [robotsuite](https://github.com/StanfordVL/robosuite)

Surreal Robitics Suite: standardized and accessible robot manipulation benchmark in physics simulation. http://surreal.stanford.edu

	- standardized tasks: a set of single-arm and bimanual manipulation tasks of large diversity and varying complexity.
	- procedural generation: modularized APIs for programmatically creating new scenes and new tasks as a combinations of robot models, arenas, and parameterized 3D objects;
	- controller modes: a selection of controller types to command the robots, such as joint velocity control, inverse kinematics control, and 3D motion devices for teleoperation;
	- multi-modal sensors: heterogeneous types of sensory signals, including low-level physical states, RGB cameras, depth maps, and proprioception;
	- human demonstrations: utilities for collecting human demonstrations, replaying demonstration datasets, and leveraging demonstration data for learning.


- [surreal](https://github.com/SurrealAI/Surreal)

Open-Source Distributed Reinforcement Learning Framework by Stanford Vision and Learning Lab https://surreal.stanford.edu

- [rllab-curriculum](https://github.com/florensacc/rllab-curriculum)

Curriculum learning

- [DoorGym](https://github.com/PSVL/DoorGym)

Open source domain randomized door opening training environment.

### UR robot

- [python control ur](https://github.com/SintefManufacturing/python-urx)

Python library to control a robot from 'Universal Robots'.

- [ReinforcementLearning4Robot](https://github.com/HsiaoTsan/ReinforcementLearning4Robot)

Reinforcement learning algorithm HER implemented on UR5 robot to execute grasp and place task.

Using two finger robotiq gripper and [python_urx](https://github.com/SintefManufacturing/python-urx) to control the ur robot.

ros control


- []()

# Reinforcement Learning Algorithms


- [OpenAI Baselines](https://github.com/openai/baselines)

OpenAI Baselines: high-quality implementations of reinforcement learning algorithms.

- [TF-Agents](https://github.com/tensorflow/agents)

TF-Agents: A library for reinforcement learning in tensorflow.

- [Stable Baselines]()

- [coach](https://nervanasystems.github.io/coach/)

Coach, developed by Intel AI lab, is a python reinforcement learning framework containing implementation of many state-of-the-art algorithms.



- []()



# Reinforcement Learning Resources


- [OpenAI SpinningUp](https://spinningup.openai.com/)

An educational resource to help anyone learn deep reinforcement learning.

- 



# Sim2Real Problems

How to transfer a model from simulation to the real robot

## ROS Related

For now, most robots can be controlled using ROS. So how to using the reinforcement learning algorithms in ROS is a solution for sim2real.

### Training in ROS



#### Reinforcement Learning in ROS

[reinforcement learning](http://wiki.ros.org/reinforcement_learning)



#### OpenAI ROS

Developed by xxx, the useful tool to use RL in ROS.




