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

