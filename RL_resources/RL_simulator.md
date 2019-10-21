# RL Simulator


## Reinforcement Learning Simulation

Introduce some useful simulators.

[Tools for dynamics simulation of robots: a survey based on user feedback](https://arxiv.org/pdf/1402.7050.pdf)

### [SimBenchmark](https://leggedrobotics.github.io/SimBenchmark/)

Physics engine benchmark for robotics applications: RaiSim vs. Bullet vs. ODE vs. MuJoCo vs. DartSim

SimBenchmark provides benchmark results of contact simulation on the state-of-the-art physics engines for various robotic tasks.

### Mujoco

The most popular simulator in reinforcement learning research, although it's not free.

- [Mujoco](http://www.mujoco.org/)

Offical webset, including the installation, documentation, forum, and some other useful resources.

- [Mujoco_py](https://github.com/openai/mujoco-py)

MuJoCo is a pyhsics engine for detailed, efficient rigid body simulations with contacts. mujoco_py allows using MuJoCo from Python 3.

- [Mujoco-Unity-Plugin](https://github.com/PSVL/Mujoco-Unity-Plugin)

Open source Mujoco Unity plugin for Doorgym project

- [Mujoco_ros_pkgs](https://github.com/shadow-robot/mujoco_ros_pkgs)
  
ROS integration of Mujoco simulator, developed by Shadow Robot.

- [openai-orrb](https://github.com/openai/orrb)

We present the OpenAI Remote Rendering Backend (ORRB), a system that allows fast  and customizable rendering  of robotics  environments.   It is based  on the Unity3d game engine and interfaces with the MuJoCo physics simulation library.  ORRB was designed with visual domain randomization in mind.  It is optimized for cloud deployment and high throughput operation. 

- [mjcf](https://github.com/iandanforth/mjcf)

Python Library for MuJoCo Format model xml.

generate Mujoco format xml files from python classes

### Bullet

- [bullet3](http://bulletphysics.org)

Bullet Physics SDK: real-time collision detection and multi-physics simulation for VR, games, visual effects, robotics, machine learning etc.

- [pybullet-gym](https://github.com/benelot/pybullet-gym)

PyBullet Gymperium is an open-source implementation of the OpenAI Gym MuJoCo environments for use with the OpenAI Gym Reinforcement Learning Research Platform in support of open research.

(https://github.com/bulletphysics/bullet3)

- [NTP-vat-release](https://github.com/StanfordVL/NTP-vat-release)

gripper grasp

- [Gibson Env: Real-World Perception for Embodied Agents](https://github.com/StanfordVL/GibsonEnv)

- [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot)

Converting OnShape assembly to robot definition (SDF or URDF) through OnShape API, to pybullet simulator

### Gazebo

Using Gazebo to train the RL agent.

#### Gym-Gazebo

[Robot_gym](https://arxiv.org/abs/1808.10369)

[Accelerated robot training through simulation in the cloud with ROS and Gazebo](https://medium.com/@vmayoral/accelerated-robot-training-through-simulation-in-the-cloud-with-ros-and-gazebo-bac6bc493520)

Code:

[gym-gazebo](https://github.com/erlerobot/gym-gazebo/)

[gym-gazebo2](https://github.com/AcutronicRobotics/gym-gazebo2)

### Unity

Unity is another open source reinforcement learning environment, has good render.

- [Unity-al-agent-toolkit](https://github.com/Unity-Technologies/ml-agents)

The Unity Machine Learning Agents Toolkit (ML-Agents) is an open-source Unity plugin that enables games and simulations to serve as environments for training intelligent agents. Agents can be trained using reinforcement learning, imitation learning, neuroevolution, or other machine learning methods through a simple-to-use Python API. We also provide implementations (based on TensorFlow) of state-of-the-art algorithms to enable game developers and hobbyists to easily train intelligent agents for 2D, 3D and VR/AR games. These trained agents can be used for multiple purposes, including controlling NPC behavior (in a variety of settings such as multi-agent and adversarial), automated testing of game builds and evaluating different game design decisions pre-release. The ML-Agents toolkit is mutually beneficial for both game developers and AI researchers as it provides a central platform where advances in AI can be evaluated on Unity’s rich environments and then made accessible to the wider research and game developer communities.

- [marathon-envs-unity-ml](https://github.com/Unity-Technologies/marathon-envs)

A set of high-dimensional continuous control environments for use with Unity ML-Agents Toolkit.

### [V-REP](http://www.coppeliarobotics.com/index.html)

[PyRep](https://github.com/stepjam/PyRep) is a toolkit for robot learning research, build on top of the virtual robotics experimentation platform.

### [Raisim](https://github.com/leggedrobotics/raisimLib)

Raisim is a physics engine for rigid-body dynamics simulation. Although it is a general physics engine, it has been mainly used/tested for robotics and reinforcement learning so far. It features an efficient implementation of recursive algorithms for articulated system dynamics (Recursive Newton-Euler and Composite Rigid Body Algorithm). RaisimLib is an exported cmake package of raisim.

Developed by Robotic Systems Lab, ETH Zurich.

- [SimBenchmark](https://leggedrobotics.github.io/SimBenchmark/): Physics engine benchmark for robotics applications: RaiSim vs. Bullet vs. ODE vs. MuJoCo vs. DartSim
- [raisimOgre](https://github.com/leggedrobotics/raisimOgre): Visualizer for raisim. It is a simple wrapper around Ogre3d(https://www.ogre3d.org/), which is an open-source 3d rendering library.
- [raisimGym](https://github.com/leggedrobotics/raisimGym): a few gym environments using RAISIM
- [raisimPy](https://github.com/robotlearn/raisimpy): a (third-party) python wrapper of RAISIM

### [DeepMind Lab](https://github.com/deepmind/lab)

DeepMind Lab is a 3D learning environment based on id Software's Quake III Arena via ioquake3 and other open source software.

DeepMind Lab provides a suite of challenging 3D navigation and puzzle-solving tasks for learning agents. Its primary purpose is to act as a testbed for research in artificial intelligence, especially deep reinforcement learning.

### [spriteworld](https://github.com/deepmind/spriteworld)

Spriteworld is a python-based RL environment that consists of a 2-dimensional arena with simple shapes that can be moved freely. 
