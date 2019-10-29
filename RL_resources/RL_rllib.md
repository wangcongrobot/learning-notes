# RLLib with Ray

## install

```bash
virtualenv env --python=python3.7
souurce env/bin/activate
pip install tensorflow==1.14.0 tensorboard==1.14.0 tensorflow-probability==0.7.0 ray[rllib] requests numpy==1.15.0 mujoco-py==2.0.2.2 psutil 
pip install your_package

```

## install latest version

https://ray.readthedocs.io/en/latest/installation.html

0.8.0.dev6
```bash
wget https://s3-us-west-2.amazonaws.com/ray-wheels/latest/ray-0.8.0.dev6-cp37-cp37m-manylinux1_x86_64.whl
pip install -U [link to wheel]
pip install tensorflow tensorboard tensorflow-probability requests numpy mujoco-py==2.0.2.2 psutil
pip gym

```

## train

train a agent with rllib:
```bash
rllib train --run PPO --env DualUR5HuskyPickAndPlace-v1 --checkpoint-freq 20 --config '{"num_workers": 20} --restore /path/to/your/model/checkpoint
```

evaluate your model:
```bash
rllib rollout /path/to/your/model/checkpoint --run PPO
```

```bash
rllib train --run PPO --checkpoint-freq 20 --env DualUR5HuskyPickAndPlace-v1 --config '{"num_workers":25, "lambda":0.95, "gamma":0.998, "kl_coeff":1.0, "clip_param":0.2, "observation_filter":"MeanStdFilter", "batch_mode":"complete_episodes", "lr": 0.0005}'
```


