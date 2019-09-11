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

The purpose of this training script is:

- to set up the learning algorithm that you want to use in order to make your agent learn
- select the task and robot to be used

The **MOST IMPORTANT THING** you need to undetstand  from the training script, is that it is totally independent from the environments. This means that, **you can chagne the algorithm you use to learn in the training script, without having to worry about modifying your environments structure.

1. Let's do an example

Parameters are divided into two different parts:

- **environment related parameters**: those depend on the *RobotEnvironment* and the *TaskEnvironment* you have selected.
- **RL algorithm parameters**: those depend on the RL algorithm you have selected
- 

https://bitbucket.org/theconstructcore/openai_ros/src/version2/openai_ros/templates/template_my_robot_env.py

```python
from openai_ros import robot_gazebo_env

class MyRobotEnv(robot_gazebo_env.RobotGazeboEnv):
    """
    Superclass for all Robot environments.
    """

    def __init__(self):
        """
        Initializes a new Robot environment.
        """
        # Variables that we give through the constructor.

        # Internal Vars
        self.controllers_list = ['my_robot_controller1', 
                                 'my_robot_contorlller2',
                                 ...,
                                 'my_robot_controllerX']
        
        self.robot_name_space = "my_robot_namespace"

        reset_controls_bool = True or False

        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv

        super(MyRobotEnv, self).__init__(controllers_list=self.controllers_list,
                                        robot_name_space=self.robot_name_space,
                                        reset_controls=reset_controls_bool)
    # Methods needed by the RobotGazeboEnv
    # -------------------------------------

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are operational.
        """
        # TODO
        return True

    # Methods that the TrainingEnvironment will need to define here as virtual
    # because they will be used in RobotGazeboEnv GrandParentClass and defined in the TrainingEnvironment.
    # -------------------------------------------
    def _set_init_pose(self):
        """
        Sets the Robot in its init pose
        """
        raise NotImplementedError()

    def _init_env_variables(self):
        """
        Inits variables needed to be initialized each time we reset at the start of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """
        Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """
        Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        """
        Get the observations.
        """
        raise NotImplementedError()

    def _is_done(self, observations):
        """
        Checks if episode done based on observations given.
        """
        raise NotImplementedError()

    # Methods that the TrainingEnvironment will need.
    # -------------------------------

```

https://bitbucket.org/theconstructcore/openai_ros/src/version2/openai_ros/templates/template_my_training_env.py
```python
from gym import spaces
import my_robot_env
from gym.envs.registration import registor
import rospy

# The path is __init__.py of openai_ros, where we import the MovingCubeOneDiskWalkEnv directly
timestep_limit_per_episode = 1000 # Can be any Value

registor(
    id='MyTrainingEnv-v0'
    entry_point='template_my_training_env:MovingCubeOneDiskWalkEnv',
    timestep_limit=timestep_limit_per_episode,
)

class MyTrainingEnv(cude_single_disk_env.MyRobotEnv):
    def __init__(self):

        # Only variable needed to be set here
        number_actions = rospy.get_param('/my_robot_namespace/n_actions')
        self.action_space = spaces.Discrete(number_actions)

        # This is the most common case of Box observation type
        high = numpy.array([
            obs1_max_value,
            obs2_max_value,
            ...
            obsN_max_value
        ])

        self.observation_space = spaces.Box(-high, high)

        # Variables that we retrieve through the param server, loaded when launch training launch

        # Here we will add any init functions prior to starting the MyRobotEnv
        super(MyTrainingEnv, self).__init__()

    def _set_init_pose(self):
        """
        Sets the Robot in its init pose
        """
        # TODO

    def _init_env_variables(self):
        """
        Inits variables needed to be initialized each time we reset at the start of an episode.
        : return:
        """
        # TODO

    def _set_action(self, action):
        """
        Move the robot based on the action variable given
        """
        # TODO: Move robot

    def _get_obs(self):
        """
        Here we define what sensor data of our robots observations 
        To know which Variables we have access to, we need to read the MyRobotEnv API DOCS
        :return: observations
        """
        # TODO
        return observations

    def _is_done(self, observations):
        """
        Decide if episode is done based on the observations
        """
        # TODO
        return done

    def _compute_reward(self, observations, done):
        """
        Return the reward based on the observations given
        """
        # TODO
        return reward

    # Internal TaskEnv Methods
    
```

robot_gazebo_env.py
```python
import rospy
import gym
from gym.utils import seeding
from .gazebo_connection import GazeboConnection
from .controllers_connection import ControllersConnection
from openai_ros.msg import RLExperimentInfo

class RobotGazeboEnv(gym.Env):

    def __init__(self, robot_name_space, controllers_list, reset_controls, start_init_physics_parameters=True, reset_world_or_sim="SIMULATION"):
        
        # To reset Simulations
        rospy.logbug("START init RobotGazeboEnv")
        self.gazebo = GazeboConnection(start_init_physics_parameters, reset_world_or_sim)
        self.controllers_object = ControllersConnection(namespace=robot_name_space, controllers_list=controllers_list)
        self.reset_controls = reset_controls
        self.seed()

        # Set up ROS related variables
        self.episode_num = 0
        self.cumulated_episode_reward = 0
        self.reward_pub = rospy.Publisher('/openai/reward', RLExperimentInfo, queue_size=1)
        rospy.logdebug("END init RobotGazeboEnv")

    # Env methods
    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        """
        Function executed each time step.
        Here we get the action execute it in a time step and retrieve the observations generated by that action.
        :param action:
        :return: obs, reward, done, info
        """

        """
        Here we should convert the action num to movement action, execute the action int the simulation and get the observations result of performing that action.
        """
        rospy.logdebug("START STEP OpenAIROS")

        self.gazebo.unpauseSim()
        self._set_action(action)
        self.gazebo.pauseSim()
        obs = self._get_obs()
        done = self._is_done(obs)
        info = {}
        reward = self._compute_reward(obs, done)
        self.cumulated_episode_reward += reward

        rospy.logdebug("END STEP OpenAIROS")

        return obs, reward, done, info

    def reset(self):
        rospy.logdebug("Reseting RobotGazeboEnvironment")
        self._reset_sim()
        self._init_env_variables()
        self._update_episode()
        self._update_episode()
        obs = self._get_obs()
        rospy.logdebug("END Reseting RobotGazeboEnvironment")
        return obs

    def close(self):
        """
        Function executed when closing the environment.
        Use it for closing GUIS and other systems that need closing.
        :return:
        """
        rospy.logdebug("Closing RobotGazeboEnvironment")
        rospy.signal_shutdown("Closing RobotGazeboEnvironment")

    def _update_episode(self):
        """
        Publishes the cumulated reward of the episode and increases the episode number by one.
        :return:
        """
        rospy.logward("PUBLISHING REWARD...")
        self._publish_reward_topic(
            self.cumulated_episode_reward, 
            self.episode_num
        )
        rospy.logwarn("PUBLISHING REWARD...DONE="+str(self.cumulated_episode_reward)+",EP="+str(self.episode_num))

        self.episode_num += 1
        self.cumulated_episode_reward = 0

    def _publish_reward_topic(self, reward, episode_number=1):
        """
        This function publishes the given reward in the reward topic for easy access from ROS infrastructure.
        :param reward:
        :param episode_number:
        """
        reward_msg = RLExperimentInfo()
        reward_msg.episode_number = episode_number
        reward_msg.episode_reward = reward
        self.reward_pub.publish(reward_msg)

    # Extension methods
    # ---------------------------

    def _reset_sim(self):
        """
        Resets a simulation
        """
        rospy.logdebug("RESET SIM START")
        if self.reset_controls:
            rospy.logdebug("RESET CONTROLLERS")
            self.gazebo.unpauseSim()
            self.controlllers_object.reset_controllers()
            self._check_all_systems_ready()
            self._set_init_pose()
            self.gazebo.pauseSim()
            self.gazebo.resetSim()
            self.gazebo.unpauseSim()
            self.controllers_object.reset_controllers()
            self._check_all_systems_ready()
            self.gazebo.pauseSim()

        else:
            rospy.logwarn("DONT RESET CONTROLLERS")
            self.gazebo.unpauseSim()
            self._check_all_systems_ready()
            self._set_init_pose()
            self.gazebo.pauseSim()
            self.gazebo.resetSim()
            self.gazebo.unpauseSim()
            self._check_all_systems_ready()
            self.gazebo.pauseSim()

        rospy.logdebug("RESET SIM END")
        return True

    def _set_init_pose(self):

    def _check_all_systems_ready(self):

    def _get_obs(self):

    def init_env_variables(self):

    def set_action(self, action):

    def _is_done(self, observations):

    def _compute_reward(self, observations, done):

    def _env_setup(self, initial_qpos):
        
```


