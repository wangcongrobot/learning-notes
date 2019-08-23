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
    <sdefault>
    <option timestep='0.001' iterarions="50" tolerance="1e-10" integrator="Euler"/>
    <visual>
        <map fogstart="3" fogend="5" force="0.1" znear="0.5"/>
        <quality shadowsize="2048" offsamples="8"/>
        <global offwidth="800" offheight="800"/>
    </visual>
    <asset>
        <material name='MatPlane" reflectance='0.3' 

```






