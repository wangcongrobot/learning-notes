# Making a custom environment in g
(https://stackoverflow.com/questions/45068568/how-to-create-a-new-gym-environment-in-openai)
file structure:
 gym-foo/
  README.md
  setup.py
  gym_foo/
    __init__.py
    envs/
      __init__.py
      foo_env.py

## gym-foo/setup.py

```python
from setuptools import setup

setup(name='gym_foo',
      version='0.0.1',
      install_requires=['gym'] # Add any other dependencies required
)
```

## gym-foo/gym_foo/__init__.py

```python
from gym.envs.registration import register

register(
    id='foo-v0',
    entry_point='gym_foo.envs:FooEnv',
)
```

## gym-foo/gym_foo/envs/__init__.py

```python
from gym_foo.envs.foo_env import FooEnv
```

## gym-foo/gym_foo/envs/foo_env.py

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


## install the package
pip install -e .

## use the package
import gym
import gym_foo
env = gym.make('foo-v0')

## use monitor in gym
env = gym.make('CartPole-v0')
env = gym.wrappers.Momitor(env, "recording")

## setup a new mujoco gym environment

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


__init__.py
from gym.envs.registration import register

register(
    id='MyEnv-v0',
    entry_point='custom_envs.my_env:MyEnv',
)

get the position of a body in the scene simply:
x, y, z = self.get_body_com("name of body here")

get the velocity of an object through get_body_comvel

setting the position of an object in a scene
1) get all the positions and velocities of the objects
qvel = np.array(self.model.data.qvel).flatten()
qpos = np.array(self.model.data.qpos).flatten()
2) manually change the postions of bodies through editing this array. Each body has 7 attributes. The first 3 are  the postion and the next 4 are the rotation quaternion.
self.set_state(qpos, qvel)

## mujoco_py functions
def _get_obs(self):
  return np.concatenate([
    self.sim.data.qpos.flat[:],
    self.sim.data.qvel.flat[:],
    self.get_body_com("goal"),
])

how to get all the simulation data, such as positions and rotations from rigidbody and also velocities as well as angular velocities.
pose part(postion and rotation) are in self.sim.data.qpos return a 1D array as the order of the body with joints specified top to down in the xml file.
free joint 7dof qpos data: x,y,z quaternion(1,0,0,0 as default)
velocities part are in self.sim.data.qvel.flat return a 1D array as the order of the body with joints specified top to down in the xml file.
free joint 7dof qvel data: vx, vy, vz, ang_vx, ang_vy, ang_vz

qvel = self.init_qvel
qpos = self.init_qpos
self.set_state(qpos, qvel)

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

```