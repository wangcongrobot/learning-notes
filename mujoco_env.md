## mujoco

https://www.andrewszot.com/blog/machine_learning/reinforcement_learning/gym_with_mujoco


### useful function


get the position of a body in the scene simply:

```python
x, y, z = self.get_body_com("name of body here")
```

get the velocity of an object through 

```python
get_body_comvel
```

setting the position of an object in a scene
1) get all the positions and velocities of the objects

```python
qvel = np.array(self.model.data.qvel).flatten()
qpos = np.array(self.model.data.qpos).flatten()
```

2) manually change the postions of bodies through editing this array. Each body has 7 attributes. The first 3 are  the postion and the next 4 are the rotation quaternion.

```pyhton
self.set_state(qpos, qvel)
```

- mujoco_py functions
  
```python
def _get_obs(self):
  return np.concatenate([
    self.sim.data.qpos.flat[:],
    self.sim.data.qvel.flat[:],
    self.get_body_com("goal"),
])
```

how to get all the simulation data, such as positions and rotations from rigidbody and also velocities as well as angular velocities.
pose part(postion and rotation) are in self.sim.data.qpos return a 1D array as the order of the body with joints specified top to down in the xml file.
free joint 7dof qpos data: x,y,z quaternion(1,0,0,0 as default)
velocities part are in self.sim.data.qvel.flat return a 1D array as the order of the body with joints specified top to down in the xml file.
free joint 7dof qvel data: vx, vy, vz, ang_vx, ang_vy, ang_vz

```python
qvel = self.init_qvel
qpos = self.init_qpos
self.set_state(qpos, qvel)
```

With very carefully studied MuJoCo’s documents, we found several parameters could
influence the fraction of the model. We modified them as a whole to realise the fraction, thus list them all below with their functions.
- Contact type setting for environment. ”solref” and ”solimp” parameterise the
function for all frictions in the environment. They will also be influenced by
the ”solver” setting.
- Contact controlling. Several attributes controlling the contact type: ”contype”,
”conaffinity”, ”condim”. The right setting for them all as a good combination
will making the contact between griper and object with right frictional contact,
opposing slip and rotation.
- Friction. It is controlled by attribute ”friction” and it may seems to be the key,
which contains the parameters for sliding friction, torsional friction and rolling
friction. We could set them by ourselves, but usually the default settings are
good enough.
- Object attributes. The ”mass” and ”density” may also influence this problem.
  
After setting them all with caution, all the gripers could then have fraction to successfully grasp objects.




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
    </default>
    <option timestep='0.001' iterations="50" tolerance="1e-10" integrator="Euler"/>
    <visual>
        <map fogstart="3" fogend="5" force="0.1" znear="0.5"/>
        <quality shadowsize="2048" offsamples="8"/>
        <global offwidth="800" offheight="800"/>
    </visual>
    <asset>
        <material name='MatPlane' reflectance='0.3' texture="texplane" texrepeat="1 1" texuniform="true"/>
    </asset>
    <worldbody>
        <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
        <geom name='floor' pos='0 0 0' size='1 1 .125' type='plane' material="MatPlane" />
        <body name='pusher' pos ='0 0 0.052'>
            <joint type="slide" name="push_joint_x"  axis='1 0 0 '  damping='1'/>
            <geom type="box" size="0.05 0.05 0.05" rgba="1.0 0 0 1"/>
        </body>
    </worldbody>
    <actuator>
        <motor ctrllimited="true" ctrlrange="-10.0 10.0"  joint='push_joint_x' />
    </actuator>
</mujoco>

```

The \<mujoco>\</mujoco> should be the top level.

Then set up the simulator params in the option tag

Then goes the \<worldbody> tag which contains the rigidbody we attempt to have in the scene.

In the inner of \<worldbody> , we have \<body> tag.

A \<body> tag MUST contains a \<joint> part and \<geom> tag. The geom part is easy to understand.

Tags like \<default> \<visual> \<asset> are bonus parts.

#### Joint

So give a biref intro on \<joint> . Joints are almost all about the rigidbody, its position,rotation,velocities,inertia,damping and so on. Further check is recommended.

In mujoco, a joint of one body is implicitly affected on the body and its parent. For example a hinge joint connects its body and body's parent.

A single body can have multiple joints. In this way composite joints are created automatically, without having to define dummy bodies.

The most important part is its type (free,slide,ball and hinge).

- To have a body moving freely, add a free joint.
- To constrain a body to move along certain direction, use slide joint, the direction is specified in axis.
- To constrain a body with another body, use hinge.

Also different types of joint stores differnt information in the data of simulation.

| joint type	| DoF |	data |
| :---: | :-: | :--: |
| free | 7	|x,y,z,quaternion(1,0,0,0 as default) |
| ball | 4 |	quaternion(1,0,0,0 as default) |
| hinge |	unknown |	unknown |
| slide |	1 |	x |

This table really takes me a lot of time to figure out and ensure its correctness. It will help.

#### Actuator(set the action space)

We have questioned where to define our actions？ Indeed,that is defined in the actuator part. If no more complicate setup, i suggest use a motor.

```xml
<actuator>
  <motor ctrllimited="true" ctrlrange="-10.0 10.0"  joint='push_joint_x' />
  <motor ctrllimited="true" ctrlrange="-10.0 10.0"   joint='push_joint_y' />
</actuator>
```

- specify the joint you want to control.
- specify the action range.
- clamp the range if you want (ctrllimited="true")

#### To visualize and test your scene:

cd to your folder contains simulate.exe of mujoco. Drag and drop the xml file into the window.

#### Set up a gym env locally

Now that the scene is prepared. We shall have scripts to define the environment's reward setting ,reset ,render and init. I have same urls to share.

- [mujoco_py api reference](https://openai.github.io/mujoco-py/build/html/reference.html#mjsim-basic-simulation)
- [a short introduction](https://www.andrewszot.com/blog/machine_learning/reinforcement_learning/gym_with_mujoco)
- [offical document on how to set up a gym env](https://github.com/openai/gym/tree/master/gym/envs#how-to-create-new-environments-for-gym)

This post is mainly on how to make a mujoco scene into a gym env. So quite a detail part is ignored since we can use the inheritance.

Note that we are setting up locally on your machine but not creating a pip package.

#### find installation of gym

So the first job is to find your installation of openai gym.

Im using anaconda's virtual environment and gym is installed on a virtual env. So it will not be in your anaconda location.

Instead

```bash
$conda info -e
tensorflow               /home/tsq/anaconda2/envs/tensorflow
root                  *  /home/tsq/anaconda2
```

The given path is your installation.

find the gym under envs/env_have_gym/Lib/site-packages/gym

open env folder.

Do as following graph

1. import your environment into the \__init__.py file of the collection. This file will be located at /gym/envs/my_collection/\__init__.py. Add from gym.envs.my_collection.my_awesome_env import MyEnv to this file.
2. Register your env in /gym/envs/\__init__.py:

```python

register(
  id='MyEnv-v0',
  entry_point='gym.envs.my_collection:MyEnv',
)

```

#### put your xml file under gym/envs/mujoco/asset

#### write your env.py

I recommend to read gym/envs/mujoco/mujoco_env.py before you start to write.

```python

import numpy as np
from gym import utils
from gym.envs.mujoco import mujoco_env
import mujoco_py
import random

class PushBallEnv(mujoco_env.MujocoEnv, utils.EzPickle):
  def __init__(self):
    mujoco_env.MujocoEnv.__init__(self, 'push_ball.xml', 4)
    utils.EzPickle.__init__(self)

  def step(self, action):
    pass

  def viewer_setup(self):
    pass

  def reset_model(self):
    pass

  def _get_obs(self):
    return np.contatenate([
      self.sim.data.qpos.flat[:],
      self.sim.data.qvel.flat[:],
      self.get_body_com("goal"),
    ])

```

All have to do is to use the same inhertance. define your step, viewer_setup, reset_model. _get_obs is optional but useful.

#### function and return

|function|return|
|:--:|:--:|
|reset_model()|self._get_obs()|
|step()|ob,reward,done,dict(debug_info)|
|viewer_setup()|not required|

First I want to tell where to get all the simulation data, such as positions and rotations from rigidbody and also velocities as well as angular velocities.

pose part (position and rotation) are in self.sim.data.qpos return 1D array as the order of the body with joints specified stop to down in the xml file.

|joint type|dof|qpos data|
|:--:|:--:|:--:|
|free|7|x,y,z,quaternion(1,0,0,0 as default)
|ball|4|quaternion(1,0,0,0 as default)|
|hinge|unknown|unknown|
|slide|1|x|

#### How to touch and change the scene

the objects in the mujoco scene cannot be change use whatever function. The only way to set their pos and velocity is to first fetch qvel = self.init_qvel and qpos = self.init_qpos, then change and feed back use the function self.set_state(qpos, qvel)

Be careful of the dimension of qvel and qpos, make it unchanged after modification.

To get the position of certain rigidbody, use self.get_body_com(bodyname)

#### Short script to test your env

```python

import gym
env = gym.make('MyPushBall-v0')
env.reset()
for i in range(10000):
  env.render()
  env.step(env.action_space.sample())
```

#### use baselines to train your env

ref to https://github.com/openai/baselines

Watch out this part: Saving, loading and visualizing models


