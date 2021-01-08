# How to use ROS with Python3

ROS1 only supports python2.7 officially and ROS2 supports python3. However, sometimes we need to use python3, like the deep learning. Usually, we use the default ROS1 and create a catkin workspace with python3. So to use python3, we also need to build some packages in the python3 environment because they are installed py python2 default.

The python3 version we use is 3.5, in virtualenv, not conda.

## blogs

https://blog.csdn.net/handsome_for_kill/article/details/81947978

https://blog.csdn.net/bluewhalerobot/article/details/80952776

https://www.cnblogs.com/h46incon/p/6207145.html

https://stackoverflow.com/questions/43019951/after-install-ros-kinetic-cannot-import-opencv

https://blog.csdn.net/qq_34544129/article/details/81946494

## tf and tf2 in python3

You can just use default melodic-devel to build.

```bash
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src 
git clone https://github.com/ros/geometry
git clone https://github.com/ros/geometry2
cd ..
virtualenv -p /usr/bin/python3 venv
source venv/bin/activate
pip install catkin_pkg pyyaml empy rospkg numpy
catkin_make
source devel/setup.bash
```
**ERROR**:
```>>> import tf
Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
  File "/data/tf2_ros/catkin_ws/devel/lib/python3/dist-packages/tf/__init__.py", line 35, in <module>
    exec(__fh.read())
  File "<string>", line 30, in <module>
  File "/data/tf2_ros/catkin_ws/devel/lib/python3/dist-packages/tf2_ros/__init__.py", line 35, in <module>
    exec(__fh.read())
  File "<string>", line 38, in <module>
  File "/data/tf2_ros/catkin_ws/devel/lib/python3/dist-packages/tf2_py/__init__.py", line 35, in <module>
    exec(__fh.read())
  File "<string>", line 38, in <module>
ImportError: dynamic module does not define module export function (PyInit__tf2)
```

**Solution**:

https://github.com/ros/geometry2/issues/259#issuecomment-368537754

and you see it is python2.7 -> you need Python3 header files, so execute this line:

sudo apt-get install python3-dev.

## MoveIt with python3

build from source.

https://github.com/ros-planning/moveit

kinetic-version

It doesn't work even after build from source and convert python2 to python3.
```bash
>>> import moveit_commander
Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
  File "/home/cong/ros_ws/openai_ros_ws/devel/lib/python3/dist-packages/moveit_commander/__init__.py", line 35, in <module>
    exec(__fh.read())
  File "<string>", line 2, in <module>
  File "/home/cong/ros_ws/openai_ros_ws/src/moveit/moveit_commander/src/moveit_commander/roscpp_initializer.py", line 35, in <module>
    from moveit_ros_planning_interface import _moveit_roscpp_initializer
ImportError: /home/cong/ros_ws/openai_ros_ws/devel/lib/python3/dist-packages/moveit_ros_planning_interface/_moveit_roscpp_initializer.so: undefined symbol: _ZN5boost6python6detail11init_moduleER11PyModuleDefPFvvE

$ ll /home/cong/ros_ws/openai_ros_ws/devel/lib/python3/dist-packages/moveit_ros_planning_interface/_moveit_roscpp_initializer.so
/home/cong/ros_ws/openai_ros_ws/devel/lib/python3/dist-packages/moveit_ros_planning_interface/_moveit_roscpp_initializer.so -> _moveit_roscpp_initializer.so.0.9.17*

```

## 2to3

/usr/src/Python-3.7.3/Tools/scripts/2to3 --output-dir moveit3 -W -n moveit/

https://blog.csdn.net/angus_monroe/article/details/78765018