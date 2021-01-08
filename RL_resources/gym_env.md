# how to set up a new gym environment

## gym main concept

http://gym.openai.com/docs/

https://github.com/openai/gym/blob/master/docs/creating-environments.md

### Observations

If we ever want to do better than take random actions at each step, it'd probably be good to actually know what our actions are doning to the environment.

The environment's __step__ function returns exactly what we need. In fact, __step__ returns four values. There are:

- **observation** (object): an environment-specific object representing your observation of the environment. For example, pixel data from a camera, joint angles and joint velocities of a robot, or the board state in a board game.
- **reward** (float): amount of reward achieved by the previous action. The scale varies between environments, but the goal is always to increase your total reward.
- **done** (boolean): whether it's time to **reset** the environment again. Most (but not all) tasks are divided up into well-defined episodes, and **done** being **True** indicates the episode has terminated. (For example, perhaps the pole tipped too far, or you lost your last life.)
- **info** (dict): diagnostic information useful for debugging. It can sometimes be useful for learning (for example, it might contain the raw probabilities behind the environment's last state change). However, official evaluations of your agent are not allowed to use this for learning.

This is just an implementation of the classic "agent-environment loop". Each timestep, the agent chooses an **action**, and the environment returns an **observation** and a **reward**.

The process gets started by calling **reset()**, which returns an initial observation. So a more proper way of writing the previous code would be to respect the **done** flag:

```python

import gym
env = gym.make('CartPole-v0')
for i_episode in range(20):
    observation = env.reset()
    for t in range(100):
        env.render()
        print(observation)
        action = env.action_space.sample()
        observation, reward, done, info = env.step(action)
        if done:
            print("Episode finished after {} timesteps".format(t+1))
            break
env.close()

```

### Spaces

In ths examples above, we've been **sampling random actions** from the environment's action space. But what actually are those actions? Every environment comes with an **action_space** and an **observation_space**. These attributes are of type **Space**, and they describe the format of valid actions and observations:

```python

import gym
env = gym.make('CartPole-v0')
print(env.action_space)
#> Discrete(2)
print(env.observation_space)
#> Box(4,)

```

The **Discrete** space allows a fixed range of non-negative numbers, so in this case valid **action** s are either 0 or 1. The **Box** space represents an **n**-dimensional box, so valid **observations** will be an array of 4 numbers. We can also check the **Box**'s bounds:

```python

print(env.observation_space.high)
#> array([2.4, inf, 0.20943951, inf])
print(env.observation_space.low)
#> arrya([-2.4, -inf, -0.20943951, -inf])

```

This introspection can be helpful to write generic code that works for many different envirenments. **Box** and **Discrete** are the most common **Space**s. You can sample from a **Space** or check that something belongs to it:

```python

from gym import spaces
space = spaces.Discrete(8) # Set with 8 elements {0, 1, 2, ..., 7}
x = space.sample()

assert space.contains(x)
assert space.n == 8

```

# Making a custom gym environment
(https://stackoverflow.com/questions/45068568/how-to-create-a-new-gym-environment-in-openai)

file structure:

```shell
 gym-foo/
  README.md
  setup.py
  gym_foo/
    __init__.py
    envs/
      __init__.py
      foo_env.py
``` 

- gym-foo/setup.py

```python
from setuptools import setup

setup(name='gym_foo',
      version='0.0.1',
      install_requires=['gym'] # Add any other dependencies required
)
```

- gym-foo/gym_foo/\__init__.py

```python
from gym.envs.registration import register

register(
    id='foo-v0',
    entry_point='gym_foo.envs:FooEnv',
)
```

- gym-foo/gym_foo/envs/\__init__.py

```python
from gym_foo.envs.foo_env import FooEnv
```

- gym-foo/gym_foo/envs/foo_env.py

```python

import gym
from gym import error, spaces, utils
from gym.utils import seeding

class FooEnv(gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__(self):
    ...
  def step(self, action):
    ...
  def reset(self):
    ...
  def render(self, mode='human', close=False):
    ...
```

- install the package
pip install -e .

- use the package
import gym
import gym_foo
env = gym.make('foo-v0')

- use monitor in gym
env = gym.make('CartPole-v0')
env = gym.wrappers.Momitor(env, "recording")

## setup a new mujoco gym environment

```python

class MyEnv(mujoco_env.MujocoEnv, utils.EzPickle):
    def __init__(self):
        utils.EzPickle.__init__(self)
        FILE_PATH = '' # Absolute path to your .xml MuJoCo scene file.
        # For instance if I had the file in a subfolder of the project where I
        # defined this custom environment I could say 
        # FILE_PATH = os.getcwd() + '/custom_envs/assets/simple_env.xml'
        mujoco_env.MujocoEnv.__init__(self, FILE_PATH, 5)

    def step(self, a):
        # Carry out one step 
        # Don't forget to do self.do_simulation(a, self.frame_skip)

    def viewer_setup(self):
        # Position the camera

    def reset_model(self):
        # Reset model to original state. 
        # This is called in the overall env.reset method
        # do not call this method directly. 

    def _get_obs(self):
      # Observation of environment feed to agent. This should never be called
      # directly but should be returned through reset_model and step
```

```python
__init__.py
from gym.envs.registration import register

register(
    id='MyEnv-v0',
    entry_point='custom_envs.my_env:MyEnv',
)
```

## New gym env

```python

class Car2DEnv(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 2
    }

    def __init__(self):
        self.action_space = None
        self.observation_space = None
        pass

    def step(self, action):
        return self.state, reward, done, {}

    def reset(self):
        return self.state()

    def render(self, mode='human'):
        return None

    def close(self):
        return None

```

必须实现的内容：

- \__init__()：将会初始化动作空间与状态空间，便于强化学习算法在给定的状态空间中搜索合适的动作。gym提供了spaces方法，详细内容可以help查看。；
- step()：用于编写智能体与环境交互的逻辑，它接受action的输入，给出下一时刻的状态、当前动作的回报、是否结束当前episode及调试信息。输入action由__init__()函数中的动作空间给定。我们规定当action为0表示小车不动，当action为1，2，3，4时分别是向上、下、左、右各移动一个单位。据此可以写出小车坐标的更新逻辑；
- reset()：用于在每轮开始之前重置智能体的状态。

## OpenAI Baselines

```python

from baselines import deepq
from Car2D import Car2DEnv

env = Car2DEnv()

model = deepq.models.mlp([32, 16], layer_norm=True)
act = deepq.learn(
    env,
    q_func=model,
    lr=0.01,
    max_timesteps=10000,
    print_freq=1,
    checkpoint_freq=1000
)

print('Finish!')
#act.save("mountaincar_model.pkl")

#act = deepq.load("mountaincar_model.pkl")
while True:
    obs, done = env.reset(), False
    episode_reward = 0
    while not done:
        env.render()
        obs, reward, done, _ = env.step(act(obs[None])[0])
        episode_reward += reward
    print([episode_reward, env.counts])

```

- 训练部分：调用了多层感知机mlp作为我们的简化深度Q网络。deepq.learn()中lr表示学习率，max_timesteps表示本次训练的总时间步steps达到10000步后结束（是的，你没看错！这不是episode而是时间步steps），print_freq表示每隔多少episode打印一次统计信息，checkpoint_freq表示每隔多少steps保存一次模型。最终将选择已保存的模型中平均回报最高的模型，赋给变量act。act可以save为pkl文件，方便下次load。
- 测试部分：act接受当前状态obs后给出action，将其传给环境的step()函数，得到下一时间步的状态、回报、是否结束。我们在Car2DEnv中有一个变量counts记录了每轮从开始到结束的时间步数，表示小车需要的时间。循环打印每轮的总回报和时间步数。如果总回报为正且时间步数较少，则表明我们的算法取得了较好的效果。

作者：Orion Nebula
链接：https://www.zhihu.com/question/58126239/answer/311036968
来源：知乎
著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。




## Tensorforce

Tensorforce通过调用方法OpenAIGym将已注册的gym环境导入。框架设计与Baselines略有不同。以PPO算法为例，直接看代码：

```python

# -*- coding: utf-8 -*-

import numpy as np
from tensorforce.agents import PPOAgent
from tensorforce.execution import Runner
from tensorforce.contrib.openai_gym import OpenAIGym
env = OpenAIGym('Car2D-v0', visualize=False)

# Network as list of layers
network_spec = [
    dict(type='dense', size=32, activation='tanh'),
    dict(type='dense', size=32, activation='tanh')
]

agent = PPOAgent(
    states_spec=env.states,
    actions_spec=env.actions,
    network_spec=network_spec,
    batch_size=4096,
# BatchAgent
    keep_last_timestep=True,
# PPOAgent
    step_optimizer=dict(
        type='adam',
        learning_rate=1e-3
    ),
    optimization_steps=10,
# Model
    scope='ppo',
    discount=0.99,
# DistributionModel
    distributions_spec=None,
    entropy_regularization=0.01,
# PGModel
    baseline_mode=None,
    baseline=None,
    baseline_optimizer=None,
    gae_lambda=None,
# PGLRModel
    likelihood_ratio_clipping=0.2,
    summary_spec=None,
    distributed_spec=None
)

# Create the runner
runner = Runner(agent=agent, environment=env)

# Callback function printing episode statistics
def episode_finished(r):
    print("Finished episode {ep} after {ts} timesteps (reward: {reward})".format(ep=r.episode, ts=r.episode_timestep,
                                                                                 reward=r.episode_rewards[-1]))
    return True

# Start learning
runner.run(episodes=1000, max_episode_timesteps=200, episode_finished=episode_finished)

# Print statistics
print("Learning finished. Total episodes: {ep}. Average reward of last 100 episodes: {ar}.".format(
    ep=runner.episode,
    ar=np.mean(runner.episode_rewards[-100:]))
)

while True:
    agent.reset()
    state, done = env.reset(), False
    episode_reward = 0
    while not done:
        action = agent.act(state, deterministic = True)
        state, done, reward = env.execute(action)
        agent.observe(done, reward)
        episode_reward += reward
    print([episode_reward])

```

为了让代码能够顺利运行，我们需要额外配置一些东西：

1. 注册自定义gym环境首先找到gym环境的文件夹位置。我使用的是anaconda，路径是D:\Anaconda\Lib\site-packages\gym\gym\envs。新建一个文件夹user并进入。将刚才我们编写的Car2D.py放进去。并增加入口文件__init__.py，内容为：

```python
from gym.envs.user.Car2D import Car2DEnv
```

回到D:\Anaconda\Lib\site-packages\gym\gym\envs。修改入口文件__init__.py（不放心的可以备份原文件），在其中增加内容：

```python
# User
# ----------------------------------------

register(
    id='Car2D-v0',
    entry_point='gym.envs.user:Car2DEnv',
    max_episode_steps=100,
    reward_threshold=10.0,
)
```

2. 修改 D:\Anaconda\Lib\site-packages\tensorforce\execution\runner.py，将该文件最后两行注释掉，如下（不放心的同学也可以备份一下）

```python
#self.agent.close()
#self.environment.close()
```

原因是这将导致agent和environment在训练完成后被清空，使得测试部分无法进行。配置好这两项内容后就可以愉快地运行代码啦！Tensorforce中使用runner()进行训练，对相关参数有兴趣的同学可以阅读官方文档相关内容。代码中agent.observe()仅仅是用于更新agent.episode信息，不会更新训练好的模型参数。需要说明的是，由于tensorforce通过导入的形式调用自定义环境，我们自定义的内容如env.counts的正确调用形式是env.gym.env.counts。

作者：Orion Nebula
链接：https://www.zhihu.com/question/58126239/answer/311036968
来源：知乎
著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。


## 强化学习实战 第一讲 gym学习及二次开发

https://zhuanlan.zhihu.com/p/26985029


1. **reset()函数详解**

reset()为重新初始化函数。那么这个函数有什么用呢？

在强化学习算法中，智能体需要一次次地尝试，累积经验，然后从经验中学到好的动作。一次尝试我们称之为一条轨迹或一个episode. 每次尝试都要到达终止状态. 一次尝试结束后，智能体需要从头开始，这就需要智能体具有重新初始化的功能。函数reset()就是这个作用。

reset()的源代码为：

```python
def _reset()

self.state = self.np_random.uniform(low=-0.05, high=0.05, size=(4,))

self.steps_beyond_done = None

return np.array(self.state)
```

第一行代码是利用均匀随机分布初试化环境的状态。

第二行代码是设置当前步数为None

第3行，返回环境的初始化状态。

2. **render()函数详解**

render()函数在这里扮演图像引擎的角色。一个仿真环境必不可少的两部分是物理引擎和图像引擎。物理引擎模拟环境中物体的运动规律；图像引擎用来显示环境中的物体图像。其实，对于强化学习算法，该函数可以没有。但是，为了便于直观显示当前环境中物体的状态，图像引擎还是有必要的。另外，加入图像引擎可以方便我们调试代码。下面具体介绍gym如何利用图像引擎来创建图像。

我们直接看源代码：

```python
def _render(self, mode=’human’, close=False):

if close:

…. #省略，直接看关键代码部分

if self.viewer is None:

from gym.envs.classic_control import rendering

#这一句导入rendering模块，利用rendering模块中的画图函数进行图形的绘制

#如绘制600*400的窗口函数为：

self.viewer = rendering.Viewer(screen_width, screen_height)

其中screen_width=600， screen_height=400

#创建小车的代码为：

l,r,t,b = -cartwidth/2, cartwidth/2, cartheight/2, -cartheight/2

axleoffset =cartheight/4.0

cart = rendering.FilledPolygon([(l,b), (l,t), (r,t), (r,b)])

```

其中rendering.FilledPolygon为填充一个矩形。

创建完cart的形状，接下来给cart添加平移属性和旋转属性。将车的位移设置到cart的平移属性中，cart就会根据系统的状态变化左右运动。具体代码解释，我已上传到github上面了，gxnk/reinforcement-learning-code　。想深入了解的同学可去下载学习。

3. **step()函数详解**

该函数在仿真器中扮演物理引擎的角色。其输入是动作a，输出是：下一步状态，立即回报，是否终止，调试项。

该函数描述了智能体与环境交互的所有信息，是环境文件中最重要的函数。在该函数中，一般利用智能体的运动学模型和动力学模型计算下一步的状态和立即回报，并判断是否达到终止状态。

我们直接看源代码：

```python
def _step(self, action):

assert self.action_space.contains(action), "%r (%s) invalid"%(action, type(action))

state = self.state

x, x_dot, theta, theta_dot = state #系统的当前状态

force = self.force_mag if action==1 else -self.force_mag #输入动作，即作用到车上的力

costheta = math.cos(theta) #余弦函数

sintheta = math.sin(theta) #正弦函数

#底下是车摆的动力学方程式，即加速度与动作之间的关系。

temp = (force + self.polemass_length * theta_dot * theta_dot * sintheta) / self.total_mass

thetaacc = (self.gravity * sintheta - costheta* temp) / (self.length * (4.0/3.0 - self.masspole * costheta * costheta / self.total_mass)) #摆的角加速度

xacc = temp - self.polemass_length * thetaacc * costheta / self.total_mass #小车的平移加速

x = x + self.tau * x_dot

x_dot = x_dot + self.tau * xacc

theta = theta + self.tau * theta_dot

theta_dot = theta_dot + self.tau * thetaacc #积分求下一步的状态

self.state = (x,x_dot,theta,theta_dot)
```

4. 一个简单的demo

下面，我给出一个最简单的demo，让大家体会一下上面三个函数如何使用。

```python
import gym
import time
env = gym.make('CartPole-v0')   #创造环境
observation = env.reset()       #初始化环境，observation为环境状态
count = 0
for t in range(100):
    action = env.action_space.sample()  #随机采样动作
    observation, reward, done, info = env.step(action)  #与环境交互，获得下一步的时刻
    if done:             
        break
    env.render()         #绘制场景
    count+=1
    time.sleep(0.2)      #每次等待0.2s
print(count)             #打印该次尝试的步数
```

## Some gym function and example

1. Monitor


```python

    # You can set the level to logger.DEBUG or logger.WARN if you
    # want to change the amount of output.
    logger.set_level(logger.INFO)

    # You provide the directory to write to (can be an existing
    # directory, including one with existing data -- all monitor files
    # will be namespaced). You can also dump to a tempdir if you'd
    # like: tempfile.mkdtemp().
    outdir = '/tmp/random-agent-results'
    env = wrappers.Monitor(env, directory=outdir, force=True)


```



