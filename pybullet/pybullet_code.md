# some example of pybullet

## topic in github

https://github.com/topics/pybullet

## GitHub

- [pybullet-gym](https://github.com/benelot/pybullet-gym)

Open-source implementations of OpenAI Gym MuJoCo environments for use with the OpenAI Gym Reinforcement Learning Research Platform. https://pybullet.org/

- [pybullet-cartpole](https://github.com/benelot/cartpoleplusplus)

- [pybullet-robots](https://github.com/bulletphysics/pybullet_robots)

https://github.com/erwincoumans/pybullet_robots

Prototyping robots for PyBullet (F1/10 MIT Racecar, Sawyer, Baxter and Dobot arm, Boston Dynamics Atlas and Botlab environment)

Prototyping robots for PyBullet (F1/10 MIT Racecar, Sawyer, Baxter and Dobot arm, Boston Dynamics Atlas and Botlab environment) http://pybullet.org

- [kuka_rl](https://github.com/mahyaret/kuka_rl/blob/master/kuka_rl.ipynb)

It's a good example to use kuka in pybullet for a picking task.

- [ss-pybullet](https://github.com/caelan/ss-pybullet)

PyBullet Planning

- [robolearn_envs](https://github.com/domingoesteban/robolearn_envs)

A Python package with OpenAI-Gym-like environments for Robot Learning

- [fetch-pybullet](https://github.com/josiahls/bullet3/commit/87defb95e8915e82fb2a6164cb05457766c57aca)

Add fetch robot to pybullet, using gym fetch robot.

### gripper

- [pybullet_grasp_annotator_robotiq_85](https://github.com/matafela/pybullet_grasp_annotator_robotiq_85)



### manipulator

- [real_robots](https://github.com/AIcrowd/real_robots)

- [franka_pybullet](https://github.com/bryandlee/franka_pybullet)

NeurIPS 2019 - Robot open-Ended Autonomous Learning

### manipulation

#### Assembly

- [RLRoboticAssembly](https://github.com/AutodeskRoboticsLab/RLRoboticAssembly)

Simulation to Reality: Reinforcement Learning for Robotic Assembly of Timber Joints.

### locomotion

- [SpotMicroAI](https://github.com/FlorianWilk/SpotMicroAI)

https://spotmicroai.readthedocs.io/en/latest/

The SpotMicroAI project is designed to be a low cost, easily built quadruped robot. The design is roughly based off of Boston Dynamics quadruped robot SpotMini, though with obvious adaptations such as size and sensor suite.

The project is maintained by a community of volunteers and is very much still in its early stages. Any individual is welcome to contribute, and in particular expertise in areas involving simulation, reinforcement learning, and hardware development is greatly appreciated.

## simplest example

This example is from [this](https://github.com/kristery/PyBullet_RL_Example).

```python
import gym
import pybullet_envs
import pybullet as p

import time

# p.GUI for graphical version similar to the MuJoCo one
# it will shut down on my pc though
p.connect(p.DIRECT)

# check the link below for some common environments
# https://github.com/bulletphysics/bullet3/releases
# env = gym.make('AntBulletEnv-v0')

env = p.loadMJCF("/home/cong/ros_ws/openai_ros_ws/src/gym/gym/envs/robotics/assets/dual_ur5_husky/mobile_pick_and_place.xml")

# it is different from how MuJoCo renders environments
# it doesn't differ too much to me w/ and w/o mode='human'
env.render()

# you should call render() before reset()
env.reset()

for _ in range(10000):
	# call sleep() so that it can render at a normal speed
	time.sleep(1./60.)
	action = env.action_space.sample()
	obs, reward, done, _ = env.step(action)
	if done:
		break
```