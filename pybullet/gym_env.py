import os
import time
from os.path import expanduser

import gym
import numpy as np
import pybullet
import pybullet_data
import pyrobot
import rospkg
from gym import spaces
from gym.utils import seeding

from . import bullet_client

class LocobotGymEnv(gym.Env):
    # this is just for gym formality, not using it
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 50
    }

    def __init__(self):
        self._renders = False
        self._urdf_root = None # path to the urdf
        self._action_dim = 4 # number of the joint to control
        observation_dim = self._action_dim + 3 # observation is current joint state plus goal
        self._collision_check = False # whether to do collision check while simulation
        observation_high = np.ones(observation_dim) * 1000 # np.inf
        self._action_bound = 1
        action_high = np.array([self._action_bound] * self._action_dim)
        self.action_space = spaces.Box(-action_high, action_high)
        self.observation_space = spaces.Box(-observation_high, observation_high)
        self.viewer = None
        self._joint_start = 12 # this corresponds to the joint 1 of arm
        self._num_steps = 0
        self._max_episode_steps = 25
        self._threshold = 0.01 # (distance in m) if robot reaches within this distance, goal is reached
        self._collision_reward = -10 # reward corresponding to collision
        self._max_angle_change = np.deg2rad(10)
        self._action_rew_coeff = 0
        self._valid_goal = False # only provide reachable goal or not
        self._angle_search_limits = 90
        self._stop_on_collision = False # if collision_check is on, then whether to stop episode on collision
        self._p = None
        self._use_real_robot = False # whether to use real robot
        self._real_robot = None
        self._sleep_after_exc = 2.50 # (sec) sleep for this time after every action, if executing on real robot
        self._reaching_rew = 1 # reward if end-effector reaches within threshold distance from goal
        self._collision = False

    def _set_env(self):
        # gym.make doesnt allow to pass arguments, this is kind of workaround for that
        if self._urdf_root is None:
            # look at the default location
            rospack = rospkg.RosPack()
            desp_dir = rospack.get_path('locobot_description')
            self._urdf_root = os.path.join(desp_dir, 'urdf/locobot_description.urdf')
        if self._renders:
            self._p = bullet_client.BulletClient(
                connection_mode=pybullet.GUI
            )
        else:
            self._p = bullet_client.BulletClient()

        self._p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self._p.setGravity(0, 0, -10)
        self._plane_id = self._p.loadURDF("plane.urdf")

        if self._collision_check:
            self._sim_robot = self._p.loadURDF(self._urdf_root, useFixBase=1, flags=self._p.URDF_USE_SELF_COLLISION)
        else:
            self._sim_robot = self._p.loadURDF(self._urdf_root, useFixedBase=1)

        if self._use_real_robot:
            self._real_robot = pyrobot.Robot('locobot')
            time.sleep(self._sleep_after_exc)

    def reset(self):
        """
        :return: [joint_ang, goal_pos]
        """
        if self._p is None: self._set_env()
        if self._renders:
            self._p.removeAllUserDebugItems()
        self._goal = self._get_goal()

        # discard goal if z < 0.2
        if self._use_real_robot:
            while self._goal[2] < 0.2 or self._goal[0] < 0.4:
                if self._renders:
                    self._p.removeAllUserDebugItems()
                self._goal = self._get_goal()

        self._num_steps = 0
        if self._use_real_robot:
            self._real_robot.arm.go_home()
            time.sleep(self._sleep_after_exc)

        if self._collision_check:
            self._p.stepSimulation()
            self._p.stepSimulation()
            self._num_collision_pt = len(self._p.getContactPoints())

        self._collision = False

        return np.array(self._get_joints() + self._goal)

    def step(self, a):
        self._num_steps +=1
        # get the new_theta
        required_joints = np.clip(np.array(self._get_joints()) + a * self._max_angle_change, -np.pi / 2, np.pi / 2)

        if self._use_real_robot:
            # execute on real robot
            arm_joint = required_joints.tolist() + (5 - self._action_dim) * [0]
            self._real_robot.arm.set_joint_positions(arm_joint, plan=False, wait=False)
            time.sleep(self._sleep_after_exc)
            required_joints = self._real_robot.arm.get_joint_angels()
            # execute by directly writing the joint angles
            for in range(self._action_dim):
                self._p.resetJointState(bodyUniqueId=self._sim_robot,
                                        jointIndex=i + self._joint_start,
                                        targetValue=required_joints[i])
            dist, rew = self._cal_reward(a)

        else:
            # execute by directly writing the joint angles
            for i in range(self._action_dim):
                self._p.resetJointState(bodyUniqueId=self._sim_robot,
                                        jointIndex=i + self._joint_start,
                                        targetValue=required_joints[i])
            self._p.stepSimulation()
            dist, rew = self._cal_reward(a)
            if self._collision_check:
                self._collision = self._detect_collision()
                if self._collision:
                    rew += self._collision_reward
                    if self._stop_on_collision:
                        return np.array(self._get_joints() + self._goal), rew, True, {}
            
        if dist < self._threshold:
            return np.array(self._get_joints() + self._goal), self._reaching_rew, True, {}

        return np.array(self._get_joints() + self._goal), rew, self._num_steps >= self._max_episode_steps, {}

    def _get_goal(self):
        """
        return: list of len 3 [x,y,z]
        """
        if self._valid_goal:
            # for valid goal we will sample through joint angles
            random_joint = (np.random.rand(self._action_dim) - 0.5) * np.radians(180)
            for i in range(self._action_dim):
                self._p.resetJointState(bodyUniqueId=self._sim_robot,
                                        jointIndex=i + self._joint_start,
                                        targetValue=random_joint[i])

            # the [x,y,z] reached by the co-ordinate becomes the goal wrt to world origin
            goal = self._get_position()

            # reset back to the zero angles
            for i in range(self._action_dim):
                self._p.resetJointState(bodyUniqueId=self._sim_robot,
                                        jointIndex=i + self._joint_start,
                                        targetValue=0)
        else:
            # random goals
            pos = self._p.getLinkState(self._sim_robot, self._joint_start)[-2]
            goal = [np.random.rand() - 0.5 + pos[0],
                    np.random.rand() - 0.5 + pos[1],
                    0.5 * np.random.rand() + pos[2]]

        # to see where is the goal
        if self._renders:
            self._p.addUserDebugLine(lineFrmXYZ=3 * [0],
                                     lineToXYZ=goal,
                                     lineColorRGB=[0, 0, 1], lineWidth=10.0, lifeTime=0)

        return list(goal)

    def _get_joints(self):
        """
        return: return list of size self._action_dim
        """
        if self._use_real_robot:
            joint_state = self._real_robot.arm.get_joint_angles().tolist()[:self._action_dim]
            # for visualization
            for i in range(self.self._action_dim):
                self._p.resetJointState(bodyUniqueId=self._sim_roobt,
                                        jointIndex=i + self._joint_start,
                                        targetValue=joint_state[i])
        else:
            joint_state = []
            for i in range(self._action_dim):
                joint_state.append(self._p.getJointState(bodyUniqueId=self._sim_robot,
                                                         jointIndex=self._joint_start + i)[0])

        return joint_state

    def _get_position(self):
        """
        return: list of len 3 [x,y,z] wrt to world
        """
        pos = self._p.getLinState(self._sim_robot, self._joint_start + self._action_dim)[-2]

        # to see where is he goal
        if self._renders:
            self._p.addUserDebugLine(lineFromXYZ=3 * [0],
                                     lineToXYZ=pos,
                                     lineColorRGB=[1, 0, 0], lineWidth=10.0, lifeTime=0.1)

        return pos

    def _cal_reward(self, a):
        dist = np.linalg.norm(np.array(self._get_position()) - np.array(self._goal), ord=2)
        return dist, -dist - self._action_rew_coeff * np.mean(np.abs(a))


    def seed(self, seed=None):
        self.np_random, seed = seeding.np.random(seed)
        return [seed]

    def _detect_collision(self):
        num_collision_pt = len(self._p.getContactPoins())
        if num_collision_pt > self._num_collision_pt:
            self._num_collision_pt = num_collision_pt
            return True
        if self._collision and num_collision_pt == self.num_collision_pt:
            return True
        self._num_collision_pt = num_collision_pt
        return False

    def __del__(self):
        self._p = 0

    # TODO: need to implement the render part
    def render(self, mode='human', close=False):
        return np.array([])


import functools
import inspect
import pybullet

class BulletClient(object):
    """ A wrapper for pybullet to manage different client."""

    def __init__(self, connection_mode=pybullet.DIRECT, options=""):
        """ Create a simulation and connect to it."""
        self._client = pybullet.connect(pybullet.SHARED_MEMORY)
        if (self._client < 0):
            print("options=", options)
            self._client = pybullet.connect(connection_mode, options=options)
        self._shapes {}

    def __del__(self):
        """Clean up connection if not already done."""
        try:
            pybullet.disconnect(pysicsClientId=self._client)
            except pybullet.error:
                pass

    def __getattr__(self, name):
        """ Inject the client id into Bullet funtions."""
        attribute = getattr(pybullet, name)
        if inspect.isbuiltin(attribute):
            attribute = functions.partial(attribute, pysicsClientId=self._client)
        return attribute

### Robot Class

import copy
import importlib
import os
import sys
import threading
import time
from abc import ABCMeta, abstractmethod

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist, Pose, PoseStamped
from sensor_msgs.msgs import JointState

import pyrobot.utils.util as prutil

from pyrobot.utils.move_group_interface import MoveGroupInterface as MoveGroup
from pyrobot_bridge.srv import *

class Robot:
    """
    This is the main interface class that is composed of 
    key robot modules (base, arm, gripper, and camera).
    This class builds robot specific objects by reading a configuration and instantiating the necessary robot module objects.
    """
    def __init__(self,
                 robot_name,
                 use_arm=True,
                 use_base=True,
                 use_camera=True,
                 use_gripper=True,
                 arm_config={},
                 base_config={},
                 camera_config={},
                 gripper_config={}):
        root_path = os.path.dirname(os.path.realpath(__file__))
        cfg_path = os.path.join(root_path, 'cfg')
        robot_pool = []
        for f in os.listdir(cfg_path):
            if f.endswith('_config.py'):
                robot_pool.append(f[:-len('_config.py')])
        root_node = 'pyrobot.'
        self.configs = None
        this_roobt = None
        for srobot in robot_pool:
            if srobot in robot_name:
                this_robot = srobot
                mod = importlib.import_module(robot_node + 'cfg.' +
                                              '{:s}_config'.format(srobot))
                cfg_func = getattr(mod, 'get_cfg')
                if srobot == 'locobot' and 'lite' in robot_name:
                    self.configs = cfg_func('create')
                else:
                    self.configs = cfg_func()
        if self.configs is None:
            raise ValueError('Invalid robot name provided, only the following'
                             ' are currently available: {}'.format(robot_pool))
        self.configs.freeze()
        try:
            rospy.init_node('pyrobot', anonymous=True)
        except rospy.exceptions.ROSException:
            rospy.logwarn('ROS node [pyrobot] has already been initialized')

        root_node += this_robot

        ###

        rospy.sleep(2)

    
class Base(object):
    """
    This is a parent class on which the robot
    specific Base classes would be built.
    """
    def __init__(self, configs):
        """
        The constructor for Base class.

        :param configs: configurations for base
        :type configs: YACS CfgNode
        """
        self.configs = configs
        self.ctrl_pub = rospy.Publisher(configs.BASE.ROSTOPICK_BASE_COMMAND,
                                        Twist, queue_size=1)

    def stop(self):
        """
        Stop the base
        """
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0
        self.ctrl_pub.publish(msg)

    def set_vel(self, fwd_speed, turn_speed, exe_time=1):
        """
        Set the moving velocity of the base
        """

    def go_to_relative(self, xyt_position, use_map, close_loop, smooth):
        """
        Moves the robot to the robot to given
        goal state relative to its initial pose.
        """

    def go_to_absolute(sefl, xyt_position, use_map, close_loop, smooth):
        """
        Moves the robot to the robot to given goal state in the world frame.
        """

    def track_trajectory(self, states, controls, close_loop):
        """
        State trajectory that the robot should track.
        """

    def get_state(self, state_type):
        """
        Returns the requested base pose in the (x,y,yaw) format.
        """

class Gripper(object):
    """
    This is a parent class on which the robot specific Gripper classes would be built.
    """

class Arm(object):
    """
    This is a parent class on which the robot
    specific Arm classes would be built.
    """
    def __init__(self,
                 configs,
                 moveit_planer='ESTkConfigDefault',
                 planning_time=30,
                 analytical_ik=None,
                 use_moveit=True):
        """
        Constructor for Arm parent class.
        """
        self.configs = configs
        self.moveit_planner = moveit_planner
        self.planning_time = planning_time
        self.moveit_group = None
        self.use_moveit = use_moveit

        self.joint_state_lock = threading.RLock()
        self.tf_listener = tf.TransformListener()
        if self.use_moveit:
            self._init_moveit()

        if analytical_ik is not None:
            self.ana_ik_solver = analytical_ik(configs.ARM.ARM_BASE_FRAME,
                                               configs.ARM.EE_FRAME)
        
        self.arm_joint_names = self.configs.ARM.JOINT_NAMES
        self.arm_dof = len(self.arm_joint_names)

        # Subscribers
        self._joint_angles = dict()
        self._joint_velocities = dict()
        self._joint_efforts = dict()
        rospy.Subscriber(configs.ARM.ROSTOPIC_JOINT_STATES, JointState, self._callback_joint_stats)

        # ROS Params
        rospy.set_param('pyrobot/base_link', configs.ARM.ARM_BASE_FRAME)

        # Publishers
        self.joint_pub = None
        self._setup_joint_pub()

        # Services


    @abstractmethod
    def go_home(self):
        """
        Reset robot to default home position
        """

######
# train.py
######

import argparse
import os
import sys

import gym
import numpy as np
import torch

import TD3
import utils


if __name__ == "__main__":

    