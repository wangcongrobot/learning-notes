# Reward Theory

## grasping reward

We consider the outcome of an episode as a success and terminate if, within the time horizon of T = 150 control steps, any object was lifted for h_lift. A natural reward function of this task would be a binary r = r_T in case of success and r = 0 otherwise. Such sparse rewards are difficult to learn from, requiring significant exploration. For this reason, in order to guide training, we also consider an alternative shaped reward formulation, in which the agent additionally receives intermediate reward signals for lifting objects,

r = (graps_detected) * (r~g~ + c * delta_h),

with r~T~ = 10, r~g~ = 1, c = 1000, and delta_h the difference in the robot's height since the last step. The first term in the equation is a binary function that returns 1 if a grap was detected and 0 otherwise. Grasp detection is achieved by checking if the fingers stalled after a closing command was issued. We also include a time penalty of -0.1 and -(r~g~ + c*delta_h_max) for the sparse and shaped reward functions respectively, where delta_h_max is the maximum allowed change in height per step. The latter is chosen such that rewards are shifted to negative values encouraging the agent to complete the task as quickly as possible.

## how to detect the grasping success

- Grasp detection is achieved by checking if the fingers stalled after a closing command was issued.



## curriculum learning
http://bit.ly/reversecurriculum

## energy penalties


## Sim to Real Policy Transfer (Transfer learning)

The final learned policy is directly transferred from simulated environment to real-world UR5 robot without additional training. As show in figure 8, we use different object to define the target position. In our working scenario, RGB-D image can be obtained from depth camera installed above the robot. Another trained deep neural network (VGG-16) output object pixel position (u, v). The target object position p=(x,y,z) under robot coordinate systems can be obtained by the following equation:

where M~in~ is camera inner parameter matrix, z is depth value with respect to the pixel position (u, v), and R and T are the rotation matrix and transformation vector from the camera coordinate system to the robot coordinte system respectively. At time step t, the grippers position p~t~, velocity v~t~, target object position p are contacted into a single vector [p~t~, v~t~, p] fed into the policy network, which is similar to training of fetch arm in simulated environment. The mean of the output of Gaussian policy is send to robot controller, and UR5 robot gripper moced to the next step position p~t+1~. The above procedure is repeated until the ending of the episode.

## Behavior Cloning






