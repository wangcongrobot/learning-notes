## Sawyer Robot working with ROS and RL

export PYTHONPATH="/opt/ros/kinetic/lib/python2.7/dist-packages:$PYTHONPATH"

sudo apt-get install python-catkin-pkg

运行catkin_make时会遇到下面问题：
问题1：ImportError: No module named 'rospkg'
解决：sudo apt-get install python3-rospkg
问题2：ImportError: No module named 'catkin_pkg'
解决：sudo apt-get install python-catkin-pkg
            sudo apt-get install python3-catkin-pkg-modules

运行launch会遇到下面问题：
问题3：ImportError: No module named 'defusedxml'
解决：sudo pip install defusedxml
问题4：ImportError: No module named 'netifaces'
解决：sudo pip install netifaces
————————————————
版权声明：本文为CSDN博主「bm5201314tcdj」的原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/bm5201314tcdj/article/details/98095411

.anaconda_with_ros_wrapper.bash

https://gist.github.com/StefanFabian/17fa715e783cd2be6a32cd5bbb98acd9

which python


## sawyer_control

https://github.com/mdalal2020/sawyer_control

Python3 ROS Interface to Rethink Sawyer Robots with OpenAI Gym Compatibility

Sawyer Control is a repository that enables RL algorithms to control Rethink Sawyer robots via an OpenAI Gym Style interface. It is both a ROS package and a set of Sawyer (Gym) Envs combined in one. The ROS portion of the repo handles the actual control of the robot and is executed in Python 2. The environments are all in Python 3, and communicate to the robot via the ROS interface. Currently, this repo is capable of utilizing both the state information of the robot as well as visual input from a Microsoft Kinect sensor.

## rl with sawyer

https://srikanth-kilaru.github.io/projects/2018/final-proj-RL

https://github.com/srikanth-kilaru/rl-projects/blob/master/actor-critic/README.md

Project Synopsis
My final project in the MS Robotics program at Northwestern University used modern reinforcement learning techniques and training data collected on a real robot to enable a Sawyer robot from Rethink Robotics to learn the task of inserting a solid block into a shape sorting cube. I implemented the Policy Gradient and Actor-Critic, two popular Reinforcement Learning algorithms, for Sawyer to learn the policy required to perform the manipulation task. The policy takes as input observations of the environment (i.e. robot joint angles and joint velocities, position of object in robot’s gripper and location of shape sorting cube), and outputs control actions in continuous domain as either joint torques or velocities.

My implementation uses the same interface between the RL agent and the Sawyer ROS environment as the interface in OpenAI’s Gym simulated environments. Several different combinations of hyper-parameters were searched to find the optimal tradeoff between policy accuracy and training time. Multiple goal locations in Cartesian space were used during training. During testing, the learned policy was able to guide Sawyer to these goals within the accuracy threshold used during the training phase.

Before the training phase, the pose of the object in the end-effector frame and the training goal poses in Sawyer’s base frame used during the training phase are computed using a Computer Vision pipeline. The same pipeline is also used for computing the pose of the test goal in the robot frame. Cartesian coordinates of 3 points on a surface of the object are used to identify the pose of the object.

