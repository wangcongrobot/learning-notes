# how to set up a new gym-mujoco environment

## gym

http://gym.openai.com/docs/


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




## mujoco

https://www.andrewszot.com/blog/machine_learning/reinforcement_learning/gym_with_mujoco



### Set up custom openai gym env with mujoco

http://deepwater.xin/wp-content/uploads/2019/02/set%20up%20custom%20openai%20gym%20env%20with%20mujoco.html

In this post, I will give an introduction on how to set up a custom openai gym env with mujoco. Then train it with openai baselines.

Installation part will be ignored.

All the following topics will be covered:

- Set up a mujoco scene with xml format
- Set up observation space
- Set up an gym env locally
- Train with baselines

My suggestion is to reference the samples in gym/envs/mujoco and find how to set up all the things.

#### Set up a mujoco scene with xml format

Any related problem on mujoco xml elements could be found on [mujoco xml](http://www.mujoco.org/book/XMLreference.html) and [mujoco overview](http://www.mujoco.org/book/index.html).

Here a simplest xml will be given.

```xml

<mujoco model="test">
    <default>
    <sdefault>
    <option timestep='0.001' iterarions="50" tolerance="1e-10" integrator="Euler"/>
    <visual>
        <map fogstart="3" fogend="5" force="0.1" znear="0.5"/>
        <quality shadowsize="2048" offsamples="8"/>
        <global offwidth="800" offheight="800"/>
    </visual>
    <asset>
        <material name='MatPlane" reflectance='0.3' texture="texplane" texrepeat="1 1" texuniform="true"/>
    </asset>
    <worldbody>
        <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
        <geom name='floor' pos='0 0 0' size='1 1 .125' type='plane' material="MatPlane" />
        <body name='pusher' pos='0 0 0.052'>
            <joint type="slide" name="push_joint_x" axis='1 0 0' damping='1' />
            <geom type="box" size="0.05 0.05 0.05" rgba="1.0 0 0 1" />
        </body>
    </worldbody>
    <actuator>
        <motor ctrllimited="true" ctrlrange="-10.0 10.0" joint='push_joint_x' />
    </actuator>
</mujoco>
```

The **\<mujoco> \</mujoco>** should be the top level.

Then set up the simulator params in the **option** tag.

Then goes the **\<worldbody>** tag which contains the rigidbody we attempt to have in the scene.

In the inner of **\<worldbody>**, we have **\<body>** tag.

A **\<body>** tag MUST contains a **\<joint>** part and **\<geom>** tag. The geom part is easy to understand.

Tags like **\<default>** **\<visual>** **\<asset>** are bonus parts.

1. Joint

So give a brief intro on **\<joint>**. ** Joints are almost all about the rigidbody**, its position, rotation, velocities, inertia, damping and so on. Further check is recommended.

In mujoco, a joint of one body is implicitly affected on the body and its parent. For example a hinge joint connects its body and body's parent.

A single body can have multiple joints. In this way composite joints are created automatically, without having to define dummy bodies.

The most important part is its type (free, slide, ball and hinge).

- To have a body moving freely, add a free joint.
- To constrain a body to move along certain direction, use slide joint, the direction is specified in **axis**.
- To constrain a body with another body, use hinge.

Also different types of joint stores different 



```
























