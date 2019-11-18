# How to use Python3 in ROS

## blogs

https://www.cnblogs.com/h46incon/p/6207145.html

Add 
```bash
#!/usr/bin/env python3
```
in your python node file


## build ROS source code vith python3

https://answers.ros.org/question/237613/how-to-define-ros-kinetic-to-use-python3-instead-of-python27/

## Bug 

ROS and OpenCV

https://stackoverflow.com/questions/43019951/after-install-ros-kinetic-cannot-import-opencv

```bash
Python 3.5.2 (default, Nov 17 2016, 17:05:23) 
[GCC 5.4.0 20160609] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> import cv2
Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
ImportError: /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so: undefined symbol: PyCObject_Type
```

I tried to remove /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so. Now It is working.

```bash
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove()
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
```

- python import error

https://stackoverflow.com/questions/16981921/relative-imports-in-python-3

- python3-catkin-pkg

https://answers.ros.org/question/245967/importerror-no-module-named-rospkg-python3-solved/

## rllib and ros

```bash
virtualenv py3 --python=python3.7
source py3/bin/actiavte
pip install catkin_pkg rospkg empy 
pip install ray[rllib] requests psutil tensorflow==1.14.0 


catkin_make

rllib train --help
    import cv2
ImportError: /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so: undefined symbol: PyCObject_Type
(py3) cong@eclipse:~/ros_ws/openai_ros_ws$ echo $PYTHONPATH
/home/cong/ros_ws/openai_ros_ws/devel/lib/python3/dist-packages:/home/cong/ros_ws/dual_ws/devel/lib/python2.7/dist-packages:/opt/ros/kinetic/lib/python2.7/dist-packages

export PYTHONPATH=/home/cong/ros_ws/openai_ros_ws/devel/lib/python3/dist-packages:/home/cong/ros_ws/dual_ws/devel/lib/python2.7/dist-packages
```

(py3) cong@eclipse:~/ros_ws/openai_ros_ws/src$ catkin_init_workspace 
Traceback (most recent call last):
  File "/opt/ros/kinetic/bin/catkin_init_workspace", line 11, in <module>
    from catkin.init_workspace import init_workspace
ImportError: No module named catkin.init_workspace

or 

cannot import rospy

source /opt/ros/kinetic/setup.bash

then you can find the rospy

<module 'rospy' from '/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/__init__.py'>

```python
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy
```

>>> import moveit_commander
Failed to import pyassimp, see https://github.com/ros-planning/moveit/issues/86 for more info

pip install pyassimp

