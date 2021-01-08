# MuJoCo Image

How to get the rgb image and depth image in mujoco and put them into the observations

## Camera

http://www.mujoco.org/book/XMLreference.html#camera

https://github.com/openai/mujoco-py/issues/201#issuecomment-491058227

add camera to the scene, add a tag `camera` under `worldbody` in the XML file:

`
<camera name="main1" mode="targetbody" target="box_link" pos="1.3 -1.3 2.0" fovy="42.5"/>
`

Then try the code below:
```python
import mujoco_py
import os
import cv2
import numpy as np

# load scene in MuJoCo
path = os.path.join('your', 'path', 'model.xml')
model = mujoco_py.load_model_from_path(path)
sim = mujoco_py.MjSim(model)

# to speed up computation we need the off screen rendering
viewer = mujoco_py.MjRenderContextOffscreen(sim, 0)
for i in range(3):
    viewer.render(420, 380, 0)
    data = np.asarray(viewer.read_pixels(420, 380, depth=False)[::-1, :, :], dtype=np.uint8)

    # save data
    if data is not None:
        cv2.imwrite("test{0}.png".format(i), data)

    print(i)
    sim.step()
```


## headless gpu render

```markdown
### Install `mujoco-py` with headless gpu rendering

https://github.com/openai/mujoco-py/pull/583

You may perform additional steps to build `mujoco-py` with gpu rendering support on a remote server without any monitor attached. 

First, check whether mujoco_py has already been built with headless gpu rendering
```
$ python3
import mujoco_py
print('gpu' in str(mujoco_py.cymj).split('/')[-1])
```
If the output is `False`, you may perform the steps below to build gpu extension 

```
$ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia-000
$ sudo mkdir -p /usr/lib/nvidia-000
$ python3
import mujoco_py
print('gpu' in str(mujoco_py.cymj).split('/')[-1])
# True
```

```

## Camera control

http://www.mujoco.org/book/haptix.html#uiCamera has a small write up of the camera controls.

in https://github.com/openai/gym/blob/master/gym/envs/mujoco/humanoidstandup.py they have the commands:
```python
self.viewer.cam.trackbodyid = 1
self.viewer.cam.distance = self.model.stat.extent * 1.0
self.viewer.cam.lookat[2] += .8
self.viewer.cam.elevation = -20
```
where self.viewer is an instance of mujoco_py.MjViewer.

So going off of the documentation, you'll probably want to change cam.lookat to help it point straight down. It also looks like you'll have to move the camera itself. That (I'm guessing) might be defined in the model of your simulator, so check there for it's position

You can find some descriptions about the "abstract camera pose specification" here
To make the screen like in the second image, you need to edit the def viewer_setup(self) function in the reacher.py file:
```python
def viewer_setup(self):
    self.viewer.cam.trackbodyid = 0         # id of the body to track ()
    self.viewer.cam.distance = self.model.stat.extent * 1.0         # how much you "zoom in", model.stat.extent is the max limits of the arena
    self.viewer.cam.lookat[0] += 0.5         # x,y,z offset from the object (works if trackbodyid=-1)
    self.viewer.cam.lookat[1] += 0.5
    self.viewer.cam.lookat[2] += 0.5
    self.viewer.cam.elevation = -90           # camera rotation around the axis in the plane going through the frame origin (if 0 you just see a line)
    self.viewer.cam.azimuth = 0              # camera rotation around the camera's vertical axis
```

You can also use the following to use one of the cameras defined in the model as default (here I assume you want to use the first defined camera):
```python
from mujoco_py.generated import const
viewer.cam.type = const.CAMERA_FIXED
viewer.cam.fixedcamid = 0
```
## Camera matrix

https://github.com/openai/mujoco-py/issues/271

https://github.com/deepmind/dm_control/blob/87e046bfeab1d6c1ffb40f9ee2a7459a38778c74/dm_control/mujoco/engine.py#L685

```python
  @property
  def matrix(self):
    """Returns the 3x4 camera matrix.
       For a description of the camera matrix see, e.g.,
       https://en.wikipedia.org/wiki/Camera_matrix.
       For a usage example, see the associated test.
    """
    camera_id = self._render_camera.fixedcamid
    if camera_id == -1:
      # If the camera is a 'free' camera, we get its position and orientation
      # from the scene data structure. It is a stereo camera, so we average over
      # the left and right channels. Note: we call `self.update()` in order to
      # ensure that the contents of `scene.camera` are correct.
      self.update()
      pos = np.mean(self.scene.camera.pos, axis=0)
      z = -np.mean(self.scene.camera.forward, axis=0)
      y = np.mean(self.scene.camera.up, axis=0)
      rot = np.vstack((np.cross(y, z), y, z))
      fov = self._physics.model.vis.global_.fovy
    else:
      pos = self._physics.data.cam_xpos[camera_id]
      rot = self._physics.data.cam_xmat[camera_id].reshape(3, 3).T
      fov = self._physics.model.cam_fovy[camera_id]

    # Translation matrix (4x4).
    translation = np.eye(4)
    translation[0:3, 3] = -pos
    # Rotation matrix (4x4).
    rotation = np.eye(4)
    rotation[0:3, 0:3] = rot
    # Focal transformation matrix (3x4).
    focal_scaling = (1./np.tan(np.deg2rad(fov)/2)) * self.height / 2.0
    focal = np.diag([-focal_scaling, focal_scaling, 1.0, 0])[0:3, :]
    # Image matrix (3x3).
    image = np.eye(3)
    image[0, 2] = (self.width - 1) / 2.0
    image[1, 2] = (self.height - 1) / 2.0
    return image @ focal @ rotation @ translation
```

## depth % segmentation
https://github.com/deepmind/dm_control/blob/87e046bfeab1d6c1ffb40f9ee2a7459a38778c74/dm_control/mujoco/engine.py#L756
```python

  def render(
      self,
      overlays=(),
      depth=False,
      segmentation=False,
      scene_option=None,
      render_flag_overrides=None,
  ):
    """Renders the camera view as a numpy array of pixel values.
    Args:
      overlays: An optional sequence of `TextOverlay` instances to draw. Only
        supported if `depth` and `segmentation` are both False.
      depth: An optional boolean. If True, makes the camera return depth
        measurements. Cannot be enabled if `segmentation` is True.
      segmentation: An optional boolean. If True, make the camera return a
        pixel-wise segmentation of the scene. Cannot be enabled if `depth` is
        True.
      scene_option: A custom `wrapper.MjvOption` instance to use to render
        the scene instead of the default.  If None, will use the default.
      render_flag_overrides: Optional mapping containing rendering flags to
        override. The keys can be either lowercase strings or `mjtRndFlag` enum
        values, and the values are the overridden flag values, e.g.
        `{'wireframe': True}` or `{enums.mjtRndFlag.mjRND_WIREFRAME: True}`. See
        `enums.mjtRndFlag` for the set of valid flags. Must be empty if either
        `depth` or `segmentation` is True.
    Returns:
      The rendered scene.
        * If `depth` and `segmentation` are both False (default), this is a
          (height, width, 3) uint8 numpy array containing RGB values.
        * If `depth` is True, this is a (height, width) float32 numpy array
          containing depth values (in meters).
        * If `segmentation` is True, this is a (height, width, 2) int32 numpy
          array where the first channel contains the integer ID of the object at
          each pixel, and the second channel contains the corresponding object
          type (a value in the `mjtObj` enum). Background pixels are labeled
          (-1, -1).
    Raises:
      ValueError: If either `overlays` or `render_flag_overrides` is requested
        when `depth` or `segmentation` rendering is enabled.
      ValueError: If both depth and segmentation flags are set together.
    """

    if overlays and (depth or segmentation):
      raise ValueError(_OVERLAYS_NOT_SUPPORTED_FOR_DEPTH_OR_SEGMENTATION)

    if render_flag_overrides and (depth or segmentation):
      raise ValueError(
          _RENDER_FLAG_OVERRIDES_NOT_SUPPORTED_FOR_DEPTH_OR_SEGMENTATION)

    if depth and segmentation:
      raise ValueError(_BOTH_SEGMENTATION_AND_DEPTH_ENABLED)

    if render_flag_overrides is None:
      render_flag_overrides = {}

    # Update scene geometry.
    self.update(scene_option=scene_option)

    # Enable flags to compute segmentation labels
    if segmentation:
      render_flag_overrides.update({
          enums.mjtRndFlag.mjRND_SEGMENT: True,
          enums.mjtRndFlag.mjRND_IDCOLOR: True,
      })

    # Render scene and text overlays, read contents of RGB or depth buffer.
    with self.scene.override_flags(render_flag_overrides):
      with self._physics.contexts.gl.make_current() as ctx:
        ctx.call(self._render_on_gl_thread, depth=depth, overlays=overlays)

    if depth:
      # Get the distances to the near and far clipping planes.
      extent = self._physics.model.stat.extent
      near = self._physics.model.vis.map_.znear * extent
      far = self._physics.model.vis.map_.zfar * extent
      # Convert from [0 1] to depth in meters, see links below:
      # http://stackoverflow.com/a/6657284/1461210
      # https://www.khronos.org/opengl/wiki/Depth_Buffer_Precision
      image = near / (1 - self._depth_buffer * (1 - near / far))
    elif segmentation:
      # Convert 3-channel uint8 to 1-channel uint32.
      image3 = self._rgb_buffer.astype(np.uint32)
      segimage = (image3[:, :, 0] +
                  image3[:, :, 1] * (2**8) +
                  image3[:, :, 2] * (2**16))
      # Remap segid to 2-channel (object ID, object type) pair.
      # Seg ID 0 is background -- will be remapped to (-1, -1).
      segid2output = np.full((self._scene.ngeom + 1, 2), fill_value=-1,
                             dtype=np.int32)  # Seg id cannot be > ngeom + 1.
      visible_geoms = self._scene.geoms[self._scene.geoms.segid != -1]
      segid2output[visible_geoms.segid + 1, 0] = visible_geoms.objid
      segid2output[visible_geoms.segid + 1, 1] = visible_geoms.objtype
      image = segid2output[segimage]
    else:
      image = self._rgb_buffer

    # The first row in the buffer is the bottom row of pixels in the image.
    return np.flipud(image)
```

## depth

```python
#@title Depth rendering {vertical-output: true}

# depth is a float array, in meters.
depth = physics.render(depth=True)
# Shift nearest values to the origin.
depth -= depth.min()
# Scale by 2 mean distances of near rays.
depth /= 2*depth[depth <= 1].mean()
# Scale to [0, 255]
pixels = 255*np.clip(depth, 0, 1)
PIL.Image.fromarray(pixels.astype(np.uint8))
```
```python
#@title Segmentation rendering {vertical-output: true}

seg = physics.render(segmentation=True)
# Display the contents of the first channel, which contains object
# IDs. The second channel, seg[:, :, 1], contains object types.
geom_ids = seg[:, :, 0]
# Infinity is mapped to -1
geom_ids = geom_ids.astype(np.float64) + 1
# Scale to [0, 1]
geom_ids = geom_ids / geom_ids.max()
pixels = 255*geom_ids
PIL.Image.fromarray(pixels.astype(np.uint8))
```