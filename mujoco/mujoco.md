

'q' in qpos and qvel refer to the generalize coordinate. The order of q is also dependent on the joint id you specify in the .xml model. For the 'simulation.cpp' example, qpos[0]~qpos[6] corresponds to the 'root' joint cartesian position (qpos[0]~qpos[2]) and orientation (qpos[3]~qpos[6]), and qvel[0]~qvel[5] correspond to the 'root' joint velocity, translational (qvel[0]~qvel[2]) and rotational (qvel[3]~qvel[5]). Note that for orientation you follow the quaternion notation thus need four element but for velocity you use angular velocity which consists of 3 elements. 


Basically sim.data.ctrl is the control signal that is sent to the actuators in the simulation.

## mujoco opengl render error

```python

model = load_model_from_xml(BASIC_MODEL_XML)
sim = MjSim(model)
viewer = MjViewer(sim)
offscreen = MjRenderContextOffscreen(sim, 0)

for i in range(3000):

    if i == 500 or i ==1500 or i==2000:
        offscreen.render(1920, 1080, 0)
        rgb = offscreen.read_pixels(1920, 1080)[0]
        cv2.imwrite("/home/iason/Desktop/fds/obs_" + str(i) +".png", rgb)

    viewer.render()
    sim.step()
```