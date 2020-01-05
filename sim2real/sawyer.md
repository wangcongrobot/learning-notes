# Sawyer RL control

- [sawyer_control](https://github.com/mdalal2020/sawyer_control)

Python3 ROS Interface to Rethink Sawyer Robots with OpenAI Gym Compatibility

Sawyer Control is a repository that enables RL algorithms to control Rethink Sawyer robots via an OpenAI Gym Style interface. It is both a ROS package and a set of Sawyer (Gym) Envs combined in one. The ROS portion of the repo handles the actual control of the robot and is executed in Python 2. The environments are all in Python 3, and communicate to the robot via the ROS interface. Currently, this repo is capable of utilizing both the state information of the robot as well as visual input from a Microsoft Kinect sensor.

Features:
```
  - Torque and Position Control Modes
  - End-Effector Reaching Environment
  - Pushing Environment
  - Vision Wrapper
  - Gym-Style Interface
```

- [rl_sawyer](https://srikanth-kilaru.github.io/projects/2018/final-proj-RL)

Master project. 
https://youtu.be/ytn4f9mnRRU

**Observations (Input to policy)**
```
For velocity control mode

[angle_0, angle_1, …., angle_N, x_object, y_object, z_object, x_goal, y_goal, z_goal]

For torque control mode

[angle_0, angle_1, …., angle_N, vel_0, vel_1, …, vel_N, x_object, y_object, z_object, x_goal, y_goal, z_goal]

where N is the number of joints being controlled
```
**Actions (Output of policy)**
```
For velcoity control mode

[vel_0, vel_1, …., vel_N]

For torque control mode

[torque_0, torque_1, …, torque_N]

where N is the number of joints being controlled
```

**ros_env**: https://github.com/wangcongrobot/rl-projects/blob/master/actor-critic/ros_env.py

- [sawyer-setup-berkeley](https://docs.google.com/document/d/1JBKPye4ABGBVQqj6RPL5yw3IKBhpO4FJIZoLzzP0d5Q/edit)

- [Sawyer-RealEnv](https://github.com/harry-uglow/Deep-RL-Sim2Real/blob/master/reality/RealEnv.py)

```python
    def __init__(self):
        """
        'Wobbles' both arms by commanding joint velocities sinusoidally.
        """
        print("Initializing node... ")
        rospy.init_node("rsdk_dish_rack")
        rospy.on_shutdown(self.clean_shutdown)

        self._right_arm = limb.Limb("right")
        self._right_joint_names = self._right_arm.joint_names()

        # control parameters
        self._rate = 20.0  # Hz
        self._missed_cmds = 3
        self.control_rate = rospy.Rate(self._rate)

        print("Getting robot state... ")
        self._rs = RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

        self._init_joint_angles = [self._right_arm.joint_angle(joint_name)
                                   for joint_name in self._right_joint_names]
        rospy.set_param('named_poses/right/poses/neutral', self._init_joint_angles)

        self._right_arm.set_command_timeout((1.0 / self._rate) * self._missed_cmds)
```
```python
    def step(self, action):
        cmd = dict([(joint, action) for joint, action in zip(self._right_joint_names, action)])
        self._right_arm.set_joint_velocities(cmd)

        self.control_rate.sleep()

        self.timestep += 1
        ob = self._get_obs()
        done = (self.timestep == self.ep_len)

        return ob, 0, done, dict()
```

- [sawyer-visual-mpc](https://github.com/febert/visual_mpc/blob/master/python_visual_mpc/sawyer/visual_mpc_rospkg/src/visual_mpc_server.py)

real sawyer robot with visual control

