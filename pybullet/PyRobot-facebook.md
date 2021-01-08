# [Facebook PyRobot](https://github.com/facebookresearch/pyrobot)

It is a good example to learn mobile manipulation. The code is not very complex and it's easy to read and learn.

## Different branches

- [Add PyTorch YoloV3 example](https://github.com/facebookresearch/pyrobot/pull/44/files#diff-0)

- [UR5](https://github.com/facebookresearch/pyrobot/blob/Develop/robots/ur5/README.md)

- [UR5e](https://github.com/facebookresearch/pyrobot/pull/34)

- [UR5]()

- [Husky](https://github.com/facebookresearch/pyrobot/pull/14)

- MoveIt control: https://github.com/facebookresearch/pyrobot/blob/master/src/pyrobot/utils/move_group_interface.py


- calibration: https://github.com/facebookresearch/pyrobot/blob/master/robots/LoCoBot/locobot_calibration/README.md

It use [ar_track_alvar](https://github.com/ros-perception/ar_track_alvar) to calibrate the hand-eye calibration.

## Sim2Real

Use Python2.7 and PyBullet.

https://github.com/facebookresearch/pyrobot/blob/master/examples/sim2real/README.md

### What is Sim2Real?

In this section we will train a simple RL policy which will learn inverse kinematics for the arm using RL(TD3). The **input** to the RL agent is **state (joint angles of arm)** & **goal location (x,y,z)** and the **control action** is the change in each joint angles to achieve the desired goal.
