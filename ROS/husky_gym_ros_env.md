# Husky Gym ROS Env

To apply the model trained in Mujoco to the real robot, we need a ROS Gym environment. First, it should has similar interface with Gym environment. Then, it should based on the Mujoco gym environment, which need observation as input and action as output.

## ROS Gym Env

- [sawyer_control](https://github.com/mdalal2020/sawyer_control)

Python3 ROS Interface to Rethink Sawyer Robots with OpenAI Gym Compatibility

- [rl_projects](https://github.com/srikanth-kilaru/rl-projects)

The `vision` part is implemented to estimate the pose of the object in the gripper and the pose of the goal. These two poses are part of the observations used as input to the RL algorithms (PG and Actor-Critic).

https://srikanth-kilaru.github.io/projects/2018/final-proj-RL

