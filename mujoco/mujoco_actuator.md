# Mujoco Actuator

- [How does ctrlrange work with position actuators?](http://www.mujoco.org/forum/index.php?threads/how-does-ctrlrange-work-with-position-actuators.3740/)

`ctrlrange` always specifies the allowed range of ctrl, regardless of the actuator type. It would make sense to define it to be equal to the joint range, but this is up to you, it is not done automatically.

Note also that joint ranges are converted to radians after compilation, while `ctrl` and `ctrlrange` specify raw values and are not converted. So if the model uses degrees and the joint range is (0, 180), the joint range after compilation would be (0, pi) and so the corresponding `ctrlrange` should be defined as (0, pi).

The above assumes that the` gear ratio` is 1. If you change it, then it affects the computation as follows. `The actuator length is the joint angle times the gear ratio`. For a `position actuator`, `ctrl` specifies the reference position for the actuator length (and not directly for the joint angle). So if you prefer position actuators to work in degrees, set gear="57.29577951". Now your ctrl and ctrlrange have the meaning of degrees.
 

For slide joints, the joint range is in meters - both in the definition of the joint, and for ctrlrange for position actuators attached to the joint. The values you entered are quite small, which is why the range is limited.

For most actuators only the first number is used. For actuators such as site, which are jet actuators acting on bodies, the gear vector specifies the 3D force and 3D torque axis along which the actuator acts.
 
http://www.mujoco.org/book/computation.html#geActuation

http://www.mujoco.org/book/modeling.html#CActuator

http://www.mujoco.org/book/XMLreference.html#actuator

Joint angles can be defined in degrees in the XML as you have done, but in the compiled/simulated models they are always in radians. The "ctrlrange" field is generic for all actuators, and the compiler does not recognize it as being in degrees. So you should expect to see angles in radians. Could this explain your results?

`The gear ratio is the mapping between joint angles (mjData.qpos) and servo positions (mjData.actuator_length). So if you specify gear=2 in the XML and set ctrl=0.1 at runtime, the joint angle should converge to qpos=0.2.`
 
When you define an actuator with type `'motor'`, the control signal has the meaning of `torque`. If you want a velocity servo, define an actuator with type 'velocity'. Then the control signal has the meaning of reference velocity. See these sections of the documentation:

http://www.mujoco.org/book/computation.html#geActuation

http://www.mujoco.org/book/modeling.html#CActuator

you can set the qpos in mujoco-py and the robot will move in simulation, but the correct way to control the robot is subtracting the desired qpos and feedback one and then apply a PD controller for example. Then you will have the control signal that can be applied to the motor by `sim.data.ctrl`.

Darwin has collision meshes, which is why they are showing. Sawyer only has visual meshes, which are discarded automatically when `discardvisual="true"`, which is the default when parsing URDFs. So you need to set `discardvisual="false"` in <compiler>. Also, you have a box with size="0 0 0" which is an error. Geom sizes must be positive. See attached model.

Note that collision geoms are placed in geom `group 0`, while visual geoms are placed in geom `group 1`. You can toggle the rendering of each group pressing '0' and '1' respectively.

Save the model as text file, and look at the mjModel field 'nsensordata'. Or you can examine it from your code.

For a motor attached to a slide joint, the meaning of ctrl is force (in Newtons, assuming all the other model parameters you entered are in SI).

If you define a position actuator, then ctrl has the meaning of reference position. If you have an actuator that can generate torque and in addition has a position/velocity servo built into it, you need multiple MuJoCo actuators to model it.