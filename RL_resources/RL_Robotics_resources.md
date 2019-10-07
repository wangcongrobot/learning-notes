# Reinforcement Learning Resource with Manipulation

**Practical Reinforcement Learning and Robotics**

This is a resource collection focusing on reinforcement learning and manipulation. As we all known, it's hard for beginner in robotics and reinforcement learning. My work is manipulation and reinforcement learning, so I try to collect some useful resource about it, mainly for the code in practice.

__*Talk is cheap. Show me the code. --- Linus Torvalds*__

**Contents:**
- [Code](#code)
- [Useful things for my works](#useful-things-for-my-works)
- [Awesome Reinforcement Learning](#awesome-reinforcement-learning)
- [](#)
- []()
- []()
- []()
- []()
- []()
- []()




## Code

- [Stochastic Latent Actor-Critic](https://github.com/alexlee-gk/slac)

Code will be released soon.

Stochastic Latent Actor-Critic: Deep Reinforcement Learning with a Latent Variable Model https://alexlee-gk.github.io/slac/


## Useful things for my works

-[Setting  up  a  Reinforcement  Learning  Task  with  a  Real-World  Robot](https://arxiv.org/pdf/1803.07067.pdf)

How to set up a RL task with UR5 robot

- [Benchmarking Reinforcement Learning Algorithmson Real-World Robots](https://arxiv.org/pdf/1809.07731.pdf)

Using UR5 robot in RL. Source code for all tasks available at https://github.com/kindredresearch/SenseAct

- [Learning how to Grasp Objects with Robotic Gripper using Deep Reinforcement Learning](https://www.doc.ic.ac.uk/~ejohns/Documents/jiaxi_liu_thesis.pdf)

This is a master degree from Imperial College London.

- [CASSL: Curriculum Acceleratd Self-Supervised Learning](https://arxiv.org/pdf/1708.01354.pdf)

A paper using Robotiq 3 finger gripper.


## Reinforcement Learning Code

### gym environment

- [Contimuouscontrol](https://github.com/Jialn/ContinuousControl)

Some continuous control experiemnts based on OpenAI baselines, Gym, and Mujoco. 

Including:
7bot arm physical robot reach problem; 
Ping-pong Game in mujoco; 
Spider/Humanoid robot walking. 

- [cassie-mujoco-sim](https://github.com/osudrl/cassie-mujoco-sim)

A simulation library for Agility Robotics' Cassie robot using MuJoCo

- [cassie-trajectory-editor](https://github.com/osudrl/cassie-trajectory-editor)

MuJoCo trajectory editor for walking robots

- [gym-cassie](https://github.com/p-morais/gym-cassie)

An OpenAI Gym style reinforcement learning interface for Agility Robotics' biped robot Cassie

- [Cassie-Robot-Resources](https://github.com/UBCMOCCA/Cassie_Robot_Resources)


list of papers, videos and codes about the bipedal robot Cassie developed by Agility Robotics

- [marathon-envs](https://github.com/Unity-Technologies/marathon-envs)

A set of high-dimensional continuous control environments for use with Unity ML-Agents Toolkit.

- [gym-kuka-mujoco](https://github.com/HarvardAgileRoboticsLab/gym-kuka-mujoco)

An OpenAI gym environment for the Kuka arm, using Mujoco simulator.

- [Kuka-bullet-for-pick-and-place](https://github.com/cb614611757/Kuka_Pybullet-for-pick-and-place)

Kuka_Pybullet for pick and place This script can be used for the research of robotic reinforcement learning. The script builds a simulation environment for robot manipulation about pick and place based on the pybullet simulation environment. Users can incorporate this environment into the reinforcement learning algorithm for online data collec… 

- [pybullet-robot-envs](https://github.com/robotology-playground/pybullet-robot-envs)
https://github.com/Rhoban/onshape-to-robot
pybullet-robot-envs is a Python package that collects robotic environments based on the **PyBullet** simulator, suitable to develop and test Reinforcement Learning algorithms on simulated **grasping** and **manipulation** applications.

The pybullet-robot-envs inherit from the **OpenAI Gym** interface.

The package provides environments for the **iCub Humanoid robot** and the **Franka Emika Panda manipulator**.

Run the following scripts to train and test the implemented environments with standard **DDPG** algorithm from **Stable Baselines**.

Here is a [introduction](https://github.com/robotology-playground/pybullet-robot-envs/blob/master/pybullet_robot_envs/README.md) of the robot env and task env.

How to train the robot using [baselines](https://github.com/eleramp/robot-agents).

- [gym-drake](https://github.com/HarvardAgileRoboticsLab/gym-drake)

Glue between [Drake](https://drake.mit.edu/) and OpenAI Gym. Including Kuka arm env.

Defines Drake Environments for use with OpenAI RL algorithms.

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

Dexterous Hand Manipulation Reinforcement Learning Control Suite

It's a dexterous hand manipulation control suite, using mujoco simulator and gym RL environment.

[Project](https://sites.google.com/view/deeprl-dexterous-manipulation)

- [Efficient Exploration via State Marginal Matching](https://github.com/RLAgent/state-marginal-matching)

It just uses the default control task in gym mujoco FetchEnv and ManipulationEnv.

Mujoco 1.5


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

Refer to run_walker3d_staged_learning.py for an example on how to setup the training script for the biped walking robot.

Curriculum learning

- [curriculum locomotion](https://github.com/VincentYu68/SymmetryCurriculumLocomotion)

- [DoorGym](https://github.com/PSVL/DoorGym)

Open source domain randomized door opening training environment.

- [raisimGym](https://github.com/leggedrobotics/raisimGym)

raisimGym is an example of a gym environment using raisim, from ETH. It uses stable-baselines (https://github.com/hill-a/stable-baselines) for training and pybind11 (https://github.com/pybind/pybind11) for wrapping raisim in python.

- [SpotmicroAI](https://github.com/FlorianWilk/SpotMicroAI)

The SpotMicroAI project is designed to be a low cost, easily built quadruped robot. The design is roughly based off of Boston Dynamics quadruped robot SpotMini, though with obvious adaptations such as size and sensor suite.

The project is maintained by a community of volunteers and is very much still in its early stages. Any individual is welcome to contribute, and in particular expertise in areas involving simulation, reinforcement learning, and hardware development is greatly appreciated.

All documentation and tutorials can be found at: [spotmicroai.readthedocs.io](https://github.com/FlorianWilk/SpotMicroAI/blob/master/spotmicroai.readthedocs.io)

- [dm_env](https://github.com/deepmind/dm_env)

A python interface for reinforcement learning environments.

- [SenseAct](https://github.com/kindredresearch/SenseAct)

SenseAct: A computational framework for developing real-world robot learning tasks https://www.kindred.ai/SenseAct

This repository provides the implementation of several reinforcement learning tasks with multiple real-world robots. These tasks come with an interface similar to OpenAI-Gym so that learning algorithms can be plugged in easily and in a uniform manner across tasks.  In this computational framework, agent and environment-related computations are ordered and distributed among multiple concurrent processes in a specific way. 

UR robotic arms

- [WalkingSpider_OpenAI_PyBullet_ROS](https://github.com/rubencg195/WalkingSpider_OpenAI_PyBullet)

- [DHER](https://github.com/mengf1/DHER)

here are dynamic goal environments. We modify the robotic manipulation environments created by OpenAI (Brockman et al., 2016) for our experiments.

DHER: Hindsight Experience Replay for Dynamic Goals (ICLR-2019) https://openreview.net/pdf?id=Byf5-30qFX

- [rlg](https://github.com/ashwinreddy/rlg)

[document](https://github.com/ashwinreddy/rlg/wiki)

Robot Learning Gym

### UR robot

- [python control ur](https://github.com/SintefManufacturing/python-urx)

Python library to control a robot from 'Universal Robots'.

- [ReinforcementLearning4Robot](https://github.com/HsiaoTsan/ReinforcementLearning4Robot)

Reinforcement learning algorithm HER implemented on UR5 robot to execute grasp and place task.

Using two finger robotiq gripper and [python_urx](https://github.com/SintefManufacturing/python-urx) to control the ur robot.

ros control

- [as_urrobotiq](https://github.com/ancorasir/as_urobotiq)

as_urrobotiq is inteded to be setup as a general purpose robot platform for autonomous pick-and-place with deep learning.

It use the UR5 arm, Robotiq 3 finger gripper, kinect, Xtion, FT300 sensor, Tensorflow.

- [gym-ignition](https://github.com/robotology/gym-ignition)

Experimental OpenAI Gym environments implemented with Ignition Robotics

- [as_DeepClaw](https://github.com/ancorasir/as_DeepClaw)

It's the same project with [as_urrobotiq](https://github.com/ancorasir/as_urobotiq).

The aim of this project is to explore autonomous and adaptive robotic pick-and-place through deep learning.

- [Robotiq-UR5](https://github.com/cxy1997/Robotiq-UR5)

Simulator of UR5 robotic arm with Robotiq 2 finger gripper, built with MuJoCo.

- [ur-ikfastpy](https://github.com/andyzeng/ikfastpy)

Python wrapper over OpenRave's IKFast inverse kinematics solver for a UR5 robot arm.

- [mujoco-ur5-model](https://github.com/roboticsleeds/mujoco_ur5_model)

Mujoco Model for UR5-Ridgeback-Robotiq Robot 

- [Berkeley Open Arms - Blue](https://github.com/berkeleyopenarms)

- [blue-mujoco-gym](https://github.com/berkeleyopenarms/blue_mujoco_v1)



### ROS Gazebo

- [Robotics AI mobile manipulation](https://github.com/sudhakaranjain/Robotics_AI)

Implementation of various algorithms on domestic robot using ROS. Mobile manipulation, using kinova arm.

- [tiago-gym-gazebo](https://github.com/huiwenzhang/tiago-gym-gazebo)

A TIAGo environment used for manipulation tasks learning based on ros and openai gym

- [ur5 rl](https://github.com/hjalte33/rl_unscrew)

A reinforcement-Learning Framework for testing learning agents on a UR5 manipulator. The framework consists of 2 ROS packages which are rl_gazebo_sim, rl_moveit_config. Besides the two ROS packages there is the folder rl-gym with contains an OpenAI Gym environment as well as python scripts for runnin reinforcement learning. The rl-gym folder is an implementation example on how to use the framework with OpenAI Gym

The framework is designed and prepared for screwing tasks meanning the simulated UR5 is fitted with a screw tool and the world is fitted with a block with a screw.

- [Ur5_DRL](https://github.com/yuecideng/Ur5_DRL)

This is a project about robotic manipulation motion planning using deep reinforcement learning based on ROS and Gazebo simulation

- [AS_6Dof_Arm](https://github.com/yao62995/AS_6Dof_Arm)

robot arm by ROS & Moveit, Train Deep Reinforcement Algorithms

- [gym-sawyer](https://github.com/rlworkgroup/gym-sawyer)

Sawyer environments for reinforcement learning that use the OpenAI Gym interface, as well as Dockerfiles with ROS to communicate with the real robot or a simulated one with Gazebo.

- [navbot](https://github.com/marooncn/navbot)

Using RGB Image as Visual Input for Mapless Robot Navigation

[Project introduction](
https://mp.weixin.qq.com/s?__biz=Mzg2MjExNjY5Mg==&mid=2247483714&idx=1&sn=449c6c1b00272d31b9093e8ae32e5ca5&chksm=ce0d8f79f97a066fcc5929cdbd0fc83ce8412eaf9d97a5c51ed16799d7e8a401027dc3bb6486&mpshare=1&scene=1&srcid=&pass_ticket=9Mwfi8nrJduWesFYZOvfaN1uXqSrd%2B2CuQl%2FzqbUNmBAfv%2Bx%2BxgJyw8xSQfYkcsl#rd)

- [gym introduction](https://zhuanlan.zhihu.com/p/26985029)

- [HorizonRobotics-SocialRobot](https://github.com/HorizonRobotics/SocialRobot)

A python environment for developing interactive learning agent with language communication ability. Using **Gazebo** and **OpenAI gym**.

We provide OpenAI gym interfaces to easily apply different RL algorithms into these different environments. 

Robot: navigation, iCub, PR2. Using [Agent Learning Framework (ALF)](https://github.com/HorizonRobotics/alf) to train the model.

Agent Learning Framework (ALF) is a reinforcement learning framework emphasizing on the flexibility of writing complex model architectures. ALF is built on **Tensorflow 2.0**.

Algorithms: A2C, DDPG, PPO, SAC, ICM, MERLIN

### V-REP

- [RL demo](https://github.com/marooncn/RL)

mobile robot rl demo

- [RLBench](https://github.com/stepjam/RLBench)

RLBench is an ambitious large-scale benchmark and learning environment featuring 100 unique, hand-design tasks, tailored to facilitate research in a number of vision-guided manipulation research areas, including: reinforcement learning, imitation learning, multi-task learning, geometric computer vision, and in particular, few-shot learning.RLBench: The Robot Learning Benchmark & Learning Environment




## Working with real robots (Sim2Real)

https://github.com/iandanforth/mjcf


- [Deep-RL-Sim2Real](https://github.com/harry-uglow/Deep-RL-Sim2Real)

Main development repository for MEng in Computing (Artificial Intelligence) final project titled "Deep Reinforcement Learning in Simulation with Real-world Fine Tuning". Project aims to develop a pipeline for learning robotic control tasks by first training in simulation before transferring to a real robot.
Using **Sawyer** robot and **V-Rep** simulator, **PyTorch**.







## Sim2Real Problems

How to transfer a model from simulation to the real robot

https://github.com/harry-uglow/Deep-RL-Sim2Real

### ROS Related

For now, most robots can be controlled using ROS. So how to using the reinforcement learning algorithms in ROS is a solution for sim2real.

### Training in ROS



#### Reinforcement Learning in ROS

[reinforcement learning](http://wiki.ros.org/reinforcement_learning)



#### OpenAI ROS

Developed by xxx, the useful tool to use RL in ROS.


## RL tools

### Visualization

- [chainerrl-visualizer](https://github.com/chainer/chainerrl-visualizer)

You can analyze ChainerRL agent's behavior in well visualized way, making debugging easier.

## RL debug and fine-tuning

https://zhuanlan.zhihu.com/p/77667356
https://zhuanlan.zhihu.com/p/72586697

understand the gym: https://zhuanlan.zhihu.com/p/28086233

https://blog.csdn.net/u013166171/article/details/89139756

## Working with real robots

https://s-rl-toolbox.readthedocs.io/en/latest/guide/real_robots.html





https://lilianweng.github.io/lil-log/

Andrej Karpathy





## RL in PyTorch

- [DeepRL](https://github.com/ShangtongZhang/DeepRL)



- [rlpyt](https://github.com/astooke/rlpyt)



- [Deep-Reinforcement-Learning-Algorithms-with-PyTorch](https://github.com/p-christ/Deep-Reinforcement-Learning-Algorithms-with-PyTorch)



- [learn2learn](https://github.com/learnables/learn2learn)

PyTorch Meta-Learning Framework for Researchers http://learn2learn.net

learn2learn is a PyTorch library for meta-learning implementations.

The goal of meta-learning is to enable agents to learn how to learn. This is, we would like our agents to become better learners as they solve more and more tasks.

- [pytorch-maml-rl](https://github.com/tristandeleu/pytorch-maml-rl)

Implementation of Model-Agnostic Meta-Learning (MAML) applied on Reinforcement Learning problems in Pytorch. This repository includes environments introduced in (Duan et al., 2016, Finn et al., 2017): multi-armed bandits, tabular MDPs, continuous control with MuJoCo, and 2D navigation task.


https://arxiv.org/abs/1909.12271
RLBench: The Robot Learning Benchmark & Learning Environment

