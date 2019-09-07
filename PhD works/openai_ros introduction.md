# openai_ros

Website: http://wiki.ros.org/openai_ros

Source: https://bitbucket.org/theconstructcore/openai_ros.git

API documentation: https://theconstructcore.bitbucket.io/openai_ros/index.html

## How to use OpenAI Reinforcement Learning ingrastructure to train ROS based robots in Gazebo simulations

OpenAI provides a complete Reinforcement Learning set of libraties that allow to trian software agents on tasks, so the agents can learn by themselves how to best do the task. Main type of agents are software agents, like this example where the OpenAI team trained an agento play Dota 2.

One of the best tools of the OpenAI set of libraries is the Gym. The Gym allows to compare Reinforcement Learning algorithms by providing a common ground called the **Environments**.

Unfortunately, even if the Gym allows to train train robots, does not provide environments to train ROS based robots using Gazebo simulations.

We have created the **openai_ros package** to provide the environments, so we all the ROS roboticists have a common ground where we can compare our best kung-fu when training robots. The package is open source and has **IGPL** license.

## The openai_ros package

The _openai_ros_ package provides a common structure for organizing everything you need to create your robot training from zero, requiring very little implementation. It is basically composed of the following elements.

- It contains the **GazeboEnvironment** class that connects your OpenAI programs to Gazebo.
- It provides a group of already made **RobotEnvironments** for the most popular ROS based robots. The **RobotEnvironments** provide the complete integration between the Gazebo simulation of the robot and the OpenAI algorithm environments, so obtaining sensor information from the robot or sending actions to it are ROS transparent to the OpenAI algorithms and to you, the developer.
- It provides a group of already made **TaskEnvironments** that you can use together with the **RobotEnvironments** and **gazeboEnvironment** to train a robot in the task defined in the **TaskEnvironment**.
- It provides a set of templates to help you create your own **RobotEnvironments** and **TasksEnvironments**, which directly connect to Gazebo (because they inherit from **GazeboEnvironment**)

learning algorithm -> start_training -> TaskEnvironment -> RobotEnvironment -> GazeboEnvironment -> Robot

In general terms, the structure of the figure can be divided in 2 big parts:
- **Training Environments**: The training environments will be the ones in charge of providing to your learning algorithm, all the needed data in order to make the robot learn. They inherit from the OpenAI Gym official environment, so they are completely compatible and use the normal training procedure of the Gym.
  
  There are different types of Training Environments:
  - **Task Environment**. This is the class that that allows to specify the task that the robot has to learn.
  - **Robot Environment**. This is the class that specifies the robot to use on the task.
  - **Gazebo Environment**. This is the class that specifies the connection to the simulator.

Usually, you will not have to touch those Environments, just use the provided ones. Inside the _openai_ros_ package we already provide many environments for robots and tasks, so it is very likely that you will just use the desired ones and concentrate on the learning algorithm.

In case you want to use a non-provided robot or a different task training, you will only have to deal with the first two classes, in order to specify the task and/or the robot. The '**GazeboEnvironment**' is already provided by the '**openai_ros**' and you don't have to modify it.

- **Training Script**: The training script will **define and set up the leanring algorithm** that you are going to use in order to train your robot. This is where usually your main work will be.

### The Training Environments

The training environments are the Python classes provided by the openai_ros package.

- **Task Environment** inherits from the **Robot Environment**.
- **Robot Environment** inherits from the **Gazebo Environment**.
- **Gazebo Environment** inherits from the **Gym Environment**. The Gym Environment (gym.Env) is the most basic Environment structure provided by **OpenAI**.

1. The Gazebo Environment

As I've said before, the Gazebo Environment is mainly used to connect the simulated environment to the Gazebo simulator. For instance, it takes care of the **resets of the simulator after each step**, or the **resets of the controllers** (if needed), it also takes care of all the steps that need to be done on the simulator when doing a training _step_ or a training _reset_ (typical steps in the reinforcement learning loop).

**IMPORTANT**: This class si the one that implements the functions required by the OpenAI Gym infrastructure:

- **step** function
- **seed** function
- **reset** function
- **close** function

However, it calls children classes functions (on the _RobotEnvironment_ and _TaskEnvironment_ classes) to get the observations and apply the actions.

This class also publishes the last episode reward on the topic **/openai/reward**

Anyways, the most important thing you need to know about this environment, is that it will be transparent to you. And what does this mean? Well, it basically means that you don't have to worry about it. **This environment will always be the same, regardless of the robot or the kind of train you want to perform**. So, you won't have to change it or work over it. Good news, right?

**THIS CLASS IS INDEPENDENT OF THE ROBOT AND OF THE TASK**

The code for this class si inside the **robot_gazebo_env.py** of the openai_ros package (in case you want to modify it).

That is the class you would have to modify in case you would like to use this package with a different simulator.

2. The Robot Environment 

The robot environment will then contain all the functions associated to the specific _robot_ that you want to train. This means, it will **contain all the ROS functionalities that your robot will need in order to be controlled**.

This class also **checks that every ROS stuff required is up and running** on the robot (topics, services...).

Within the _openai_ros_ package, we are going to provide the _RobotEnvironment_ for all the ROS robots available, so users do not have to worry about it, just select the robot and use the appropriate _RobotEnvironment_ class.

You will only need to care about this class in case you want to use the package with your own robot that only you know about it.

Currently available Robot Environments:

- Cartpole
- Cube robot
- Hopper robot
- ROSbot by  Husarion
- Wam by Barret
- Parrot drone
- Sawyer by Rethink robotics
- Shadow Robot Grasping Sandbox
- Summit XL by Robotnik
- Turtlebot2
- Turtlebot3 by Robotis
- WAMV water vehicle of the RobotX Challenge

More are on its way.

In the Turtlebot2 example, this is handled by the class **turtlebot2_env.py** of the _openai_ros_ package.

3. The Task Environment

The _task environment class_ provides all the context for the task we want the robot to learn. **It depends on the task and on the robot!!** This means that:

- In case you want to make **the same robot learn another task**, it is here that you have to toucn. Also:
  - You can leave without modifications the _Robot Environment_ and _Gazebo Environment_
  - You will have to do a small mod to the _training script_
- In case you want **another robot to learn the same task**, then it may be necessary to change this class too, if the new robot doesn't have the same interface.

In our _Turtlebot2_ in the _maze_ example, the _task environment class_ is created in the **turtlebot2_maze.py** file.

**Task Environment** specifies the following required for the training:

- How to apply the selected action to the robot: function **_set_action**
- How to get the observations resulting from the action: function **_get_obs**
- How to compute the reward: function **_compute_reward**
- Detect if the training for current episode has finished: function **_is_done**

Additionally, it has a couple of functions for handling the simulation:

- function **_init_env_variables**: used to initialise any var that need to be set back to initial value on every episode
- function **_set_init_pose**: used to initialize the robot position on every episode

**NOTE**: the training step is not defined here. Here is just defines how to do the actions inside the training step

4. The Simulations Needed for each Robot and Task

In the **_version2_** of OpenAI_ROS, each task_environment and robot_environment have a launch file asociated.

This makes the system even simpler because you now don't have to worry about downloading the simulations asociated with each task or robot. It will search for it in your workspace and if not there it will downloading it.

This workspace will be defined by the user in the yaml file loaded when launching the main RL script, through the parameter named **ros_ws_abspath**. Have a look at the tutorials to see examples on how this is used.

So when using a certain Environment, it will be launched automatically without you having to worry. You can only concentrate on the training script.

### The Training Script

The purpose of this training scripts is:

- to set up the learning algorithm that you want to use in order to make your agent learn
- select the task and robot to be used

The **MOST IMPORTANT THING** you need to understand from the training script, is that it is totally independent from the environments. This means that, **you can change the algorithm you use to learn in the training script, without having to worry about modifying your environments structure**.








