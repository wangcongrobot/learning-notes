# convert urdf file to mujoco file

[http://wiki.ros.org/collada_urdf]

install from source 
https://github.com/rdiankov/collada-dom

# get image from mujoco
(https://github.com/openai/gym/pull/1114)
gym/envs/mujoco/mujoco_env.py
            # Extract depth part of the read_pixels() tuple
            data = self._get_viewer(mode).read_pixels(width, height, depth=True)[1]
            # original image is upside-down, so flip it
            return data[::-1, :]
  @pzhokhov
pzhokhov on Sep 15, 2018  Contributor
actually, you lost me now... Why is depth data 2D? Do we still need to flip it upside down?

  @23pointsNorth
23pointsNorth on Sep 17, 2018  Author Contributor
RGB is [h, w, 3] where 3 is for the r, g, b.
Depth I initially assumed is [h, w, 1] , after local testing, it turned out to be [h, w].

  @pzhokhov
pzhokhov on Sep 17, 2018  Contributor
oh got it. Ok.

ok, this is fine with me; one last thing - please include 'depth_array' into the self.metadata['render.modes'].


# render
also, as a temporary patch, you can add camera_id=0 to the render argument here:

gym/gym/envs/mujoco/mujoco_env.py

Line 105 in 4ceff7d

 self._get_viewer(mode).render(width, height) 

(this will switch rendering rgb_mode rendering of mujoco envs to the tracking camera)

# plot in baslines
https://github.com/openai/baselines/blob/master/docs/viz/viz.ipynb

# pybullet kinova
https://github.com/JanMatas/Rainbow_ddpg
https://sites.google.com/view/sim-to-real-deformable

# tensorboard + baselines

add to bashrc, environment variables
export OPENAI_LOG_FORMAT='stdout,log,csv,tensorboard'
export OPENAI_LOGDIR=....your_path... 

tensorboard --logdir=$OPENAI_LOGDIR

# baselines

explain the output of ppo algorithm
https://medium.com/aureliantactics/understanding-ppo-plots-in-tensorboard-cbc3199b9ba2

# Offscreen framebuffer is not complete, error

Use instructions from https://github.com/deepmind/dm_control#additional-instructions-for-linux
export MUJOCO_GL="osmesa" before script
now, you can run the script

from dm_control
Rendering

The MuJoCo Python bindings support three different OpenGL rendering backends: EGL (headless, hardware-accelerated), GLFW (windowed, hardware-accelerated), and OSMesa (purely software-based). At least one of these three backends must be available in order render through dm_control.

    Hardware rendering with a windowing system is supported via GLFW and GLEW. On Linux these can be installed using your distribution's package manager. For example, on Debian and Ubuntu, this can be done by running sudo apt-get install libglfw3 libglew2.0. Please note that:
        dm_control.viewer can only be used with GLFW.
        GLFW will not work on headless machines.

    "Headless" hardware rendering (i.e. without a windowing system such as X11) requires EXT_platform_device support in the EGL driver. Recent Nvidia drivers support this. You will also need GLEW. On Debian and Ubuntu, this can be installed via sudo apt-get install libglew2.0.

    Software rendering requires GLX and OSMesa. On Debian and Ubuntu these can be installed using sudo apt-get install libgl1-mesa-glx libosmesa6.

By default, dm_control will attempt to use GLFW first, then EGL, then OSMesa. You can also specify a particular backend to use by setting the MUJOCO_GL= environment variable to "glfw", "egl", or "osmesa", respectively.

# save tensorflow model from baselines

(https://github.com/openai/baselines/issues/597)
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

`savepath = "./models/" + str(update) + "/model.ckpt"
model.save(savepath)
print('Saving to', savepath)`


# Failed to initialize OpenGL
(https://github.com/openai/mujoco-py/issues/187)
Is there a solution to this (can't use env.render and env.sim.render in the same environment) yet?
I'm experiencing the same issue. And my current work around is to set and unset environment variable "LD_PRELOAD" everytime in python scripts.
Basically there is no way to call env.render() and env.sim.render(w, h) in the same environment because the first one requires export LD_PRELOAD=/usr/lib/...../libGLEW.so but the second one requires this line commented.

 @luizmarao
 
luizmarao commented on Jun 14 • 
After lots of trying and lots of reading, with no success, I tried (for no reason at all) something that worked!
Using the mode as rgb_array in env.sim.render(), I received a warning that mode should be offscreen or window. Offscreen was exactly the mode in that the error was happening... so I changed to window.
At first, the only change was that, immediately after loading the env, a render screen has shown up, without the error from offscreen mode. Then, without closing the render window, I decided to run the command env.sim.render() with offscreen mode on the python shell, and it worked!
I received the image without any error!
So I included in my code a line calling env.sim.render() with mode window before the line with mode offscreen, which gets the camera image.

my code lines (inside my gym environment):
self.sim.render(mode='window', camera_name='first-person', width=16, height=16, depth=False)
img = self.sim.render(mode='offscreen', camera_name='first-person', width=16, height=16, depth=False)

# GlfwError: Failed to initialize GLFW; get image from mujoco
(https://github.com/openai/mujoco-py/issues/172)
Ok sim.render(...) creates an offscreen render_context but returns the image data. So if you want to get images do

```python
model = load_model_from_xml(MODEL_XML)
sim = MjSim(model)
img = sim.render(600, 600)
img will have a [600, 600] np array with image data that you can now use. There will be an offscreen render context inside of sim.render_contexts but this is an array and you shouldn't need this to grab images. Just make additional calls to render(...). So for a basic example, once you have sim and model set up.

imgs = []
for i in range(20):
    sim.step()
    imgs.append(sim.render(600, 600))
Should give you image data for those 20 steps.
```

# get image from mujoco
(https://github.com/openai/mujoco-py/issues/441)

``` python
import numpy as np
import matplotlib.pyplot as plt
import mujoco_py
import gym


env = gym.make("FetchReach-v1")
env.env.viewer == None
## For the confirm that 'env.env.viewer' is 'None'

env.env.viewer = mujoco_py.MjViewer(env.env.sim)
## Without the step above, there will be a "GLEW initialization error". Issue has been submitted.
env.env.sim.render(width=48, height=48, mode='offscreen')
env.env.reset()

env.env._get_viewer().render()

img = env.env.sim.render(width=48, height=48, depth=False)
plt.imshow(img)
plt.show()
## The image showed here is normal
env.env._get_viewer().render()

## Then start to set the state from given mujoco interface
nq = env.env.sim.model.nq
nv = env.env.sim.model.nv

qpos = gym.spaces.Box(high=np.ones(nq), low=-np.ones(nq), dtype=np.float32).sample()
qvel = gym.spaces.Box(high=np.ones(nv), low=-np.ones(nv), dtype=np.float32).sample()

old_state = env.env.sim.get_state()
new_state = mujoco_py.MjSimState(old_state.time, qpos, qvel, old_state.act, old_state.udd_state)
env.env.sim.set_state(new_state)

env.env.sim.forward()
## finish set_state

env.env.sim.render_contexts[0]._set_mujoco_buffers()

img = env.env.sim.render(width=48, height=48, depth=False)
plt.imshow(img)
plt.show()
# Then the image showing has only one color
```



https://github.com/openai/mujoco-py/issues/408

https://github.com/openai/mujoco-py/issues/390
render mode="human" works, but mode="rgb_array" gives GLEW initialization error #390

Describe the bug
When I run env.render(mode="human") everything works as it should. However, if instead I run env.render(mode="rgb_array") I get the error "ERROR: GLEW initalization error: Missing GL version". I am aware of the issue here, and I have added export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so (actually before doing this, I don't think mode="human" worked either), however it doesn't help fix this. It seems that someone else was having this issue, but it doesn't seem to have been reposted here as suggested.

Strangely, if I first call env.render(mode="human") I am able to call env.render(mode="rgb_array") afterwards without getting this error. So, for example, the following code works:

```python
from gym.envs.mujoco.half_cheetah_v3 import HalfCheetahEnv
import numpy as np

env = HalfCheetahEnv()

test = env.reset()
env.render(mode="human")

for i in range(10000):
a,b,c,d = env.step(np.random.randn(6).clip(-1,1))
img=env.render(mode="rgb_array")`
```
But if I miss out the initial env.render(mode="human") I get the error.

Desktop (please complete the following information):

OS: Ubuntu 18.04.2 LTS
Python Version 3.5.2
Mujoco Version 2.0
mujoco-py version 2.0.2.0
Environment

output of: echo $LD_LIBRARY_PATH - /home/henry/.mujoco/mujoco200/bin
output of: echo $HOME /home/henry
output of: echo $USER - henry

# mujoco rgb_array render

A workaround I am using is to call render("human") once, then call render("rgb_array") normally.


Furthermore when I add "None" argument at mysterious position
mujoco_py.MjRenderContextOffscreen(sim, None,device_id=0)
images are correctly rendered, though black winodow appears.

Let the entire program initialize an on-screen render context ('window' mode) and abandon it. Then, make your off-screen render context as normal, which will ensure you get the image you expected. Although this will leave a black window on your screen and system keep reporting 'mujoco-py not responding'.

# git github pull
Remote origin already exists on 'git push' to a new repository

# RuntimeError: Failed to set buffer size when trying to take video record or screenshot 
Please, comment line 153 in mjrendercontext.pyx, because this code is incompatible.

git remote set-url origin git@github.com:username/projectname.git



















