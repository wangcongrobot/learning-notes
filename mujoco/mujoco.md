

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

## mujoco record

**geom**

**condim** (default: "3"): Dimensionality of the contact force/torque in the contact frame. It can be 1, 3, 4 or 6.
|condim|Description|
|:--:|:--:|
|1|Frictionless contact.|
|**3**|Regular frictional contact, opposing slip in the tangent plane.|
|**4**|Frictional contact, opposing slip in the tangent plane and rotation around the contact normal. This is useful for modeling soft contacts (independent of contact penetration).|
|6||

condim = 4 : 4 for elliptic, 6 for pyramidal
In addition to normal and tangential force, this contact can generate torsional friction torque opposing rotation around the contact normal. **This is useful for modeling soft fingers, and can substantially improve the stability of simulated grasping.** Keep in mind that the torsional (as well as rolling) friction coefficients have different units from the tangential friction coefficients.

Also, it helps to use torsional friction for grasping (by setting condim=4). Soft fingerpads make distributed contacts with objects, and effectively have torsional friction. MuJoCo does not explicitly model the contact area, but allows you to include torsional friction directly which has a similar stabilizing effect.
http://www.mujoco.org/forum/index.php?threads/rope-pincer-grip.3967/#post-5380


quaternion: (cos(a/2), sin(a/2) * (x, y, z))

quat : real(4), "1 0 0 0": 
If the quaternion is known, this is the preferred was to specify the frame orientation because it does not involve conversions. Instead it is normalized to unit length and copied into mjModel during compilation. When a model is saved as MJCF, all frame orientations are expressed as quaternions using this attribute.


## Mujoco Forums

[how to set mujoco gripper parameters]

http://www.mujoco.org/forum/index.php?threads/mujoco-grip-parameters.3746/#post-4844

http://www.mujoco.org/forum/index.php?threads/default-simulation-parameters-do-not-facilitate-simple-grips.2198/

http://www.mujoco.org/forum/index.php?threads/finger-jumping-while-sliding-over-a-table.4034/

http://www.mujoco.org/forum/index.php?threads/robot-hand-grab-objects.4005/

http://www.mujoco.org/forum/index.php?threads/modelling-underactuated-gripper.3527/

http://www.mujoco.org/forum/index.php?resources/universal-robots-ur5-robotiq-s-model-3-finger-gripper.22/

[DAPG for Dexterous Hand Manipulation](https://github.com/aravindr93/hand_dapg)

https://github.com/HarvardAgileRoboticsLab/gym-kuka-mujoco

https://github.com/j96w/MuJoCo_Unity_UR5

https://github.com/shadow-robot/mujoco_ros_pkgs

https://github.com/vikashplus/fetch

https://github.com/roboticsleeds/mujoco_ur5_model

https://github.com/vikashplus/Adroit

https://github.com/saga0619/mujoco_ros_sim

https://github.com/saga0619/dyros_red

https://github.com/psh117/dyros_jet

https://github.com/savik28/mujoco_test

https://github.com/vikashplus/pallet

https://www.andre-gaschler.com/rotationconverter/

https://quaternions.online/


## Slipping problem of grasping in mujoco

This part is from [thesis]().

With very carefully studied MuJoCo's documents, we found several parameters could influence the fraction of the model.

- Contact type setting for environment. **solref** and **solimp** parameterise the function for all frictions in the environment. They will also be influenced by the **solver** setting.
- Contact controlling. Several attributes controlling the contact type: **contype**, **conaffinity**, **condim**. The right setting for them all as a good combination will making the contact between gripper and object with right frictional contact, opposing slip and rotation.
- Friction. It si controlled by attribute **friction** and it may seems to be the key, which contains the parameters for sliding friction, torsional friction and rolling friction. We could set them by ourselves, but usually the default setting good enough.
- Object attributes. The **mass** and **density** may also influence this problem.

After setting them all with caution, all the grippers could then have fraction to successfully grasp objects.