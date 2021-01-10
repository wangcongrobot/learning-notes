# Sim-to-Real





## code

- [Learning policies for robotic manipulation](https://github.com/rstrudel/rlbc)

- [From simulation to real world using deep generative models](https://github.com/aalitaiga/sim-to-real/)

- [tune-net-release](https://github.com/Kukanani/tune-net-release)

Code for CoRL 2019 paper "TuneNet: One-Shot Residual Tuning for System Identification and Sim-to-Real Robot Task Transfer"

### [robotics-rl-srl](https://github.com/araffin/robotics-rl-srl)

S-RL Toolbox: Reinforcement Learning (RL) and State Representation Learning (SRL) for Robotics https://s-rl-toolbox.readthedocs.io

https://s-rl-toolbox.readthedocs.io/en/latest/guide/real_robots.html

**For real robot, using zmq to communicate with ROS.**
https://github.com/araffin/robotics-rl-srl/tree/master/real_robots

https://github.com/NataliaDiaz/arm_scenario_experiments/tree/rl


- [learning-to-manipulate](https://github.com/rstrudel/rlbc)

in real robot

- [RoboNet](https://github.com/SudeepDasari/RoboNet)

https://github.com/SudeepDasari/RoboNet/wiki/Experiment-Replication

- [visual_foresight](https://github.com/SudeepDasari/visual_foresight)

Code for reproducing experiments in Visual Foresight: Model-Based Deep Reinforcement Learning for Vision-Based Robotic Control

On a high level, Visual Model Predictive Control (visual-MPC) leverages an action-conditioned video prediction model (trained from unsupervised interaction) to enable robots to perform various tasks with only raw-pixel input. This codebase provides an implmentation of: unsupervised data collection, our benchmarking framework, the various planning costs, and - of course - the visual-MPC controller! Additionally, we provide: instructions to reproduce our experiments, Dockerfiles for our simulator environments, and documentation on our Sawyer robot setup.

- [visual_mpc](https://github.com/febert/visual_mpc)

Visual MPC implementation running on Rethink Sawyer Robot

- [vices](https://github.com/StanfordVL/robosuite/tree/vices_iros19)

Surreal Robotics Suite with VICES IROS19 code: standardized and accessible robot manipulation benchmark with physics simulation and integrated analytical controllers https://stanfordvl.github.io/vices/

- [domain-randomizer](https://github.com/montrealrobotics/domain-randomizer)

A standalone library to randomize various OpenAI Gym Environments.

Domain Randomization is a idea that helps with sim2real transfer, but surprisingly has no general open source implementations. This library hopes to fill in that gap by providing a standalone library that you can use in your own work.

- [Active Domain Randomization](https://github.com/montrealrobotics/active-domainrand)

Active Domain Randomization (ADR) is a new method for improved, zero-shot transfer of robotic reinforcement learning policies. Building upon traditional domain randomization, which uniformly samples the randomization space, we show that replacing this with an active search for difficult MDP instances improves generalization and robustness in the resulting policies.

- [Reproducing Domain Randomization for Sim-to-Real](https://github.com/matwilso/domrand)

This repository contains my implementation of domain randomization setup for training an object localization model in simulation to adapt to the real world. I was implementing this as part of a larger research project, and decided to publish this in case others may find it useful.

use the KUKA LBR4 arm 

- [Deep-RL-Sim2Real](https://github.com/harry-uglow/Deep-RL-Sim2Real/tree/master/reality)

Vrep Sawyer sim2real ROS



## PyRobot

https://github.com/facebookresearch/pyrobot

https://github.com/facebookresearch/habitat-api/blob/master/habitat/sims/pyrobot/pyrobot.py

## GibsonEnv

https://github.com/StanfordVL/GibsonEnv

https://github.com/StanfordVL/GibsonEnv/tree/master/examples/ros/gibson-ros


- [sim2real_drone_racing-uzh](https://github.com/uzh-rpg/sim2real_drone_racing)

```
@article{loquercio2019deep,
  title={Deep Drone Racing: From Simulation to Reality with Domain Randomization},
  doi={10.1109/TRO.2019.2942989},
  author={Loquercio Antonio, and Kaufmann Elia, and Ranftl Ren{\'e}, and Dosovitskiy Alexey, and Koltun Vladlen, and Scaramuzza Davide},
  journal={IEEE Transactions on Robotics},
  year={2019}
}
```


