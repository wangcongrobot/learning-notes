# How to use stable-baselines to train a custom model


## example

Here is a quick example of how to train and run PPO2 on a cartpole environment:

```python

import gym

from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines import PPO2

env = gym.make('CartPole-v1')
env = DummyVecEnv([lambda: env])  # The algorithms require a vectorized environment to run

model = PPO2(MlpPolicy, env, verbose=1)
model.learn(total_timesteps=10000)

obs = env.reset()
for i in range(1000):
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    env.render()
```

verbose – (int) the verbosity level: 0 none, 1 training information, 2 tensorflow debug

## Tensorboard Integration

### Basic Usage

```python
# define a tensorboard log location
model = A2C('MlpPolicy', 'CartPole-v1', verbose=1, tensorboard_log="./a2c_cartpole_tensorboard/")

# loading an exiting model
model = A2C("./a2c_cartpole.pkl", env=env, tensorboard_log="./a2c_cartpole_tensorboard/")

# define custom logging name (by default it is the algorithm name)
model.learn(total_timesteps=1000, tb_log_name="first_run")
# Pass reset_num_timesteps=False to continue the training curve in tensorboard
# By default, it will create a new curve
model.learn(total_timesteps=10000, tb_log_name="second_run", reset_num_timesteps=False)
model.learn(total_timesteps=10000, tb_log_name="thrid_run", reset_num_timesteps=False)

```

tensorboard --logdir ./a2c_cartpole_tensorboard/

you can also add past logging folders:

tensorboard --logdir ./a2c_cartpole_tensorboard/;./ppo2_cartpole_tensorboard/

### Logging More Values

Using a callback, you can easily log more values with TensorBoard. Here is a simple example on how to log both additional tensor or arbitrary scalar value:

```python

import tensorflow as tf
import numpy as np

from stable_baselines import SAC

model = SAC("MlpPolicy", "Pendulum-v0", tensorboard_log="/tmp/sac/", verbose=1)
# Define a new property to avoid global varible
model.is_tb_set = False

def callback(locals_, globals_):
    self_ = locals_['self']
    # log additional tensor
    if not self_.is_tb_set():
        with self_.graph.as_default():
            tf.summary.scalar('value_target', tf.reduce_mean(self_.value_target))
            self_.summary = tf.summary.merge_all()
        self_.is_tb_set = True
    # Log scalar value (here a random variable)
    value = np.random.random()
    summary = tf.Summary(value=[tf.Summary.Value(tag='random_value', simple_value=value)])
    locals_['writer'].add_summary(summary, self_.num_timesteps)
    retuen True

model.learn(50000, callback=callback)

```

### Legacy Integration

All the information displayed in the terminal (default logging) can be also logged in tensorboard. For that, you need to define several environment variables.

```bash

# formats are comma-separated, but for tensorboard you only need the last one
# stdout -> terminal
export OPENAI_LOG_FORMAT='stdout,log,csv,tensorboard'
export OPENAI_LOGDIR=/path/to/tensorboard/data

```

and to configure the logger using:

```python

from stable_baselines.logger import configure

configure()

```

Then start tensorboard with:

$ tensorboard --logdir=$OPENAI_LOGDIR


### Visualize the trained agent

```python

env = model.get_env()
# model = SAC.load('SAC-Pendulum-v0')
obs = env.reset()
for i in range(1000):
    action, _states = model.predict(obs, deterministic=True)
    obs, rewards, dones, info = env.step(action)
    env.render()
```

### Multiprocedssing: Vectorized environments

```python

import gym

from stable_baselines.common.vec_env import DummyVecEnv, SubprocVecEnv
from stable_baselines import A2C

def make_env(env_id, rank, seed=0):
    """
    Utility function for multiprocessed env.
    :param env_id: (str) the environment ID
    :parma num_env: (int) the number of environments you wish to have in subprocesses
    :param seed: (int) the initial seed for RNG
    :param rank: (int) index of the subprocess
    """
    def _init():
        env = gym.make(env_id)
        env.seed(seed + rank)
        return env
    return _init

n_cup = 8
env = DummyVecEnv([make_env('CartPole-v1', i) for i in range(n_cpu)])
model = A2C('MlpPolicy', env, verbose=1).learn(int(5e5))

```

### Custom Policy Network

```python

import tensorflow as tf
from stable_baselines import PPO2, SAC, TD3

# Common policies (A2C family: A2C, PPO, TRPO, ACKTR, ACER)
# Custom MLP policy of two layers of size 32 each with tanh
# activation function
policy_kwargs = dict(act_fun=tf.nn.tanh, net_arch=[32, 32])
# Different architecture for actor/critic
# net_arch=[128, dict(vf=[256], pi=[16])]
model = PPO2('MlpPolicy', 'Pendulum-v0', policy_kwargs=policy_kwargs)

# Custom Architecture (DDPG, SAC, TD3)
model = TD3('MlpPolicy', 'MountainCarContinous-v0', policy_kwargs=dict(layers=[400, 300]))
```

### Monitoring Trainging

- Monitor Wrapper

```python

import gym

from stable_baselines import PPO2
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines.bench import Monitor

env = Monitor(gym.make('CartPole-v1'), filename=None, allow_early_resets=True)
env = DummyVecEnv([lambda: env])
model = PPO2('MlpPolicy', env, verbose=1).learn(int(1e5))
```

- Tensorboard

```python

from stable_baselines import SAC

model = SAC('MlpPolicy', 'LunarLanderContinuous-v2', verbose=1, tensorboard_log='/tmp/sac/')
model.learn(int(1e5))
```

- Callback

```python

import numpy as np
from stable_baselines import SAC

def callback(locals_, globals_):
    self_ = locals_['self']
    # Check every 1000 calls
    if self.n_callback_calls % 1000 == 0:
        # Save best model (according to training reward)
        if locals_.get('mean_reward', -np.inf) > self_.best_mean_reward:
            print("Saving best model")
            self_.save('sac_best')
            self_.best_mean_reward = locals_['mean_reward']
        # Stop training when target performance attained
        if self_.best_mean_reward > -800:
            print("Stopping training")
            return False

    self_.n_callback_calls += 1
    return True

model = SAC("MlpPolicy", "Pendulum-v0", verbose=1)
# Define a properties to avoid global variables
model.best_mean_reward = -np.inf
model.n_callback_calls = 0
model.learn(100000, callback=callback)
```











