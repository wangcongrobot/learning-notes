# [SIA 7F Arm MuJoCo model](https://github.com/wangcongrobot/gym/tree/master/gym/envs/robotics/sia_7f_arm)

## mujoco environment

The Mujoco environment is in the folder gym/gym/envs/robotics/sia_7f_arm. 

**It's just a initial version.**

## install

```bash
$ virtualenv env --python=python3.7
$ souurce env/bin/activate
$ pip install tensorflow==1.14.0 tensorboard==1.14.0 tensorflow-probability==0.7.0 ray[rllib]==0.7.5 requests numpy==1.15.0 mujoco-py==2.0.2.2 psutil 
$ git clone https://github.com/wangcongrobot/gym.git
$ cd gym/
$ pip install -e .
```

## train 

in the gym/ folder:
```bash
$ rllib train --run PPO --env SIA7FArmPickAndPlace-v1 --checkpoint-req 20 --config '{"num_workers": 2}'
```
the model will be stored in ~/ray_results/default/

## evaluate

in the gym/ folder:
```bash
$ rllib rollout /path/to/ray_results/default/PPO_SIA7FArmPickAndPlace-v1******/checkpoint_xx/checkpoint_xx --run PPO
```