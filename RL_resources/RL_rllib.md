# RLLib with Ray

## install

```bash
virtualenv env --python=python3
souurce env/bin/activate
pip install tensorflow==1.14.0 tensorboard==1.14.0 tensorflow-probability==0.7.0 ray[rllib] requests numpy==1.15.0 mujoco-py==2.0.2.2 psutil 
pip install your_package

```

train a agent with rllib:

rllib train --run PPO --env DualUR5HuskyPickAndPlace-v1 --checkpoint-freq 20 --config '{"num_workers": 20}

evaluate your model:

rllib rollout /path/to/your/model/checkpoint --run PPO

## install from source
