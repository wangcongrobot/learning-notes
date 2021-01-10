# Some useful answers in the forums

Using collision detection to check grasp touching:

http://www.mujoco.org/forum/index.php?threads/how-do-i-calculate-the-distance-from-the-site-to-the-edge-of-the-gripper.3995/

Use V-HACD to deal with complex STL files:

http://www.mujoco.org/forum/index.php?threads/gear-task-stl-files.3983/

http://www.mujoco.org/forum/index.php?threads/joint-compliance-in-non-thumb-fingers.14/#post-44


- equality/connect
http://www.mujoco.org/forum/index.php?threads/assign-point-in-2-mesh-and-connect-them.3473/

- grasping
http://www.mujoco.org/forum/index.php?threads/joint-compliance-in-non-thumb-fingers.14/#post-58


ctrlrange always specifies the allowed range of ctrl, regardless of the actuator type. It would make sense to define it to be equal to the joint range, but this is up to you, it is not done automatically.

Note also that joint ranges are converted to radians after compilation, while ctrl and ctrlrange specify raw values and are not converted. So if the model uses degrees and the joint range is (0, 180), the joint range after compilation would be (0, pi) and so the corresponding ctrlrange should be defined as (0, pi).

The above assumes that the gear ratio is 1. If you change it, then it affects the computation as follows. The actuator length is the joint angle times the gear ratio. For a position actuator, ctrl specifies the reference position for the actuator length (and not directly for the joint angle). So if you prefer position actuators to work in degrees, set gear="57.29577951". Now your ctrl and ctrlrange have the meaning of degrees.
 


- [Robot hand grab objects](http://www.mujoco.org/forum/index.php?threads/robot-hand-grab-objects.4005/)

In general this can happen for two reasons: (1) the simulation is inaccurate; (2) the simulation is accurate and the same thing would have happened in the real world.

(1) could be due to large time step, or poor solver convergence. Make sure you use the Newton solver with sufficient number of iterations (say 100) and reduce the time step to something very small, and see if the problem is still there. Also, what geometry are you using? Box-box contacts suffer from collision detection artifacts in some rare cases. You could try to step through the simulation and see if very large contact forces appear all of a sudden.

(2) could be due to large forces being applied by your robot, combined with a large mass ratio. Have you looked at the contact forces and actuation forces? Note that if there is penetration and the contacts are hard, the contact forces will be very large -- this is supposed to happen. The question is, why did the penetration get large in the first place. Usually this happens due to initialization or due to large time steps.

You could also download the HAPTIX package. It contains the MPL hand model with multiple scenes that have been fine-tuned for object manipulation (under tele-operation). It may not be what you need, but still, it will provide an example environment where grasping is possible without any issues. The HAPTIX documentation chapter has links to videos illustrating this.
 
