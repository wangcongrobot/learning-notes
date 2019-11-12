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

