# Learn the basic of openai_ros using a Turtlebot2 simulation

http://wiki.ros.org/openai_ros/TurtleBot2%20with%20openai_ros

Description: This tutorial will introduce you to openai_ros by making turtlebot2 simulation learn how to navigate a simple maze.

## TurtleBot 2 with openai_ros

Let's make a Turtlebot 2 robot learn to move without colliding in a simple maze.

### Step 0: Setup the Simulation

You can do it in two ways:

- In the Version 2, you don't need to download any simulation. The needed code will be downloaded in the environment you launch the task environment and robot environement.

In order for the training script to work, you also need to download and compile the openai_ros package, because it is using an **openai_ros** **TaskEnvironment** to do the job.

```bash
$ cd /home/user/simulations_ws/src
$ git clone https://bitbucket.org/theconstructcore/openai_ros.git
$ cd openai_ros; git checkout version2
$ /home/user/simulation_ws; catkin_make; source devel/setup.bash
```

### Step 1: Launch the simulation

The Simulation will be launched automatically through the Task and Robot environment selected. The world asociated to the Task environment that will appear depends on the launch file defined in the Task environment. The same happens with the Robot environment.

### Step 2: Let's create the Environment classes

1. The Training Environments

The training environments are the Python classes provided by the openai_ros package.

2. The Gazebo Environment

As I've said before, the Gazebo Environment is mainly used to connect the simulated environment to the Gazebo simulator. You have to do nothing here. This class is already embedded in the **RobotEnvironment** of the Turtlebot 2.

4. The Robot Environment

The robot environment will then contain all the functions associated to the specific _robot_ that you want to train. This means, it will **contain all the ROS functionalities that your robot will need in order to be controlled**.

In the Turtlebot 2 example, this is handled by the class xxx defined in the **turtlebot2_env.py** file of the _openai_ros_ package.

Again, you have to do nothing here, because the openai_ros package already provids you the **RobotEnvironment** for the Turtlebot2. We are going to use that class in the **TaskEnvironment**.

5. The Task Environment

The _task environment class_ provides all the context for the task we want the robot to learn. **It depends on the task and on the robot!!**.

In our Turtlebot2 in the maze example, the task environment class is created in the **turtlebot2_maze.py** file.

## The Training Script









