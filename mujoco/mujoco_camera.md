# Model a Camera in Mujoco



## Preparing simulation models for roboitc tasks

The objective of this project was to create MuJoCo simulation models for a door opening task for the Franka Panda robot.

https://github.com/gamleksi/mujoco_ros_control

The ros_control interface for the MuJoCo simulator. The code parses a given model and register a control interface for each slide or hinge joint. The ros_control effort based controllers could be then loaded as shown in example start_simple_robot.launch. It provides trajectory based interface e.g. for MoveIt.

https://www.youtube.com/playlist?list=PLwO8awctVMheqR0lSKW-R1jILyXx6po5N

Mujoco joint ROS Gazebo to open the door.

## depth image

[How to get camera matrix in Mujoco #271](https://github.com/openai/mujoco-py/issues/271)

I haven't found camera matrix in mujoco, however, I could construct it.
I assumed that the principal point is not shifted and pixels are of square form. You need to know frame size (width, height) and focus distance (in pixels), which can be computed as following:
`
fovy = sim.model.cam_fovy[cam_id]`
`
f = 0.5 * height / math.tan(fovy * math.pi / 360)
`
Then you get camera matrix as 
`np.array(((f, 0, width / 2), (0, f, height / 2), (0, 0, 1)))`
It worked for me.
For more info about camera matrix look here, for more detailed understanding you may have a look on this book

[Depth conversion from OpenGL zbuffer #520](https://github.com/openai/mujoco-py/issues/520#issuecomment-605335819)

solution: You can take a look at the dm_control's [implementation](https://github.com/deepmind/dm_control/blob/master/dm_control/mujoco/engine.py#L734) to convert depth values between [0, 1] to actual values.

[mujoco_2d_projection](https://github.com/yusukeurakami/mujoco_2d_projection)