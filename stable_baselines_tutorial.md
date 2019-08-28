# How to use stable-baselines to train a custom model

rl-baselines-zoo has some scripts to train the model, it's good example to learn how to use stable-baselines.

## How to use multiprocessing 

just add *mpirun -np num_cpu* in the front of nomal python code in bash:

$ mpirun -np 4 python xxxxxx

## How to save and continue the taining process

In stable-baselines, we can save a training model and resume it in the new training process.

Saving as much as detailed parameters in tensorboad is important.

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


https://github.com/openai/baselines/issues/597

Hi @hellandhansen

Personally I changed a little bit the PPO2 code and I use instead tf.train.Saver.

I modified the save and load function in the Model object
```python
def save(save_path):
           """
           Save the model
           """
           saver = tf.train.Saver()
           saver.save(sess, save_path)

def load(load_path):
           """
           Load the model
           """
           saver = tf.train.Saver()
           print('Loading ' + load_path)
           saver.restore(sess, load_path)
```
And the training part of the learn function
```python
`savepath = "./models/" + str(update) + "/model.ckpt"
model.save(savepath)
print('Saving to', savepath)`
```
If you need the complete implementation check here https://github.com/simoninithomas/Deep_reinforcement_learning_Course/tree/master/PPO%20with%20Sonic%20the%20Hedgehog
Hope it helps,


The openai baselines code within it's logger module contains support for tensorboard logging. The stable baselinses code still retains this same logging code. You can activate it via:
```python
    from stable_baselines import logger
    print( 'Configuring stable-baselines logger')
    logger.configure()
```
To control the location where the logs are stores, set the OPENAI_LOGDIR environment variable to a location on your file system. To control the formats of data that are logged (and to enable tensorboard logging), set the OPENAI_LOG_FORMAT environment variable to "stdout,tensorboard".

This form of tensorboard logging does fine across multiple calls to training and yields the same statistics as openai baselines. (Useful for comparing performance across the two forks.)

Here's a comparison of an algorithm running on an environment but with different numbers of timestemps per learning call (1e5, 1e6, 1e9).

More complete snippet that I'm using right now:
```py
basedir = '/some/directory'

try:
    os.makedirs(basedir)
    print("Directory " , basedir ,  " created ")
except FileExistsError:
    pass

os.environ[ 'OPENAI_LOGDIR' ] = basedir
os.environ[ 'OPENAI_LOG_FORMAT' ] = 'stdout,tensorboard'

from stable_baselines import logger
print( 'Configuring stable-baselines logger')
logger.configure()
```
Full code for reference:
https://github.com/jrjbertram/jsbsim_rl/blob/d65d63fe5e3b4e8ac9be580744b0242ab86eafee/compare.py


Hello,

I am mostly concerned about obscure state such as optimizer's internal variables.

that is a good question. I think there are different things that are not currently stored which prevent from a perfect recover from previous training:

the content of replay buffer is not saved for off-policy methods (for disk usage reason)
the state of the optimizer (e.g. Adam) is not saved
the learning rate schedule may be reset when reloading a model / resuming training
However, despite those caveats, I did not experience huge drop in performance (yet) when resuming training.




