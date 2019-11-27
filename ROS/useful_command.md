## Some useful command in ROS


Spawn a model in Gazebo
```bash
rosrun gazebo_ros spawn_model -file /path/to/urdf/file -urdf -x 0.1 -y 0.1 -z 0.1 -model my_object
```

$ roslaunch urdf_tutorial display.launch model:=urdf/01-myfirst.urdf

catkin_make --only-pkg-with-deps <target_package>

编译 baxter_simulator遇到 No rule to make target '/opt/ros/kinetic/lib/liborocos-kdl.so.1.3.0'错误

https://blog.csdn.net/u011573853/article/details/103078842

sudo ln -s /opt/ros/kinetic/lib/liborocos-kdl.so.1.3 /opt/ros/kinetic/lib/liborocos-kdl.so.1.3.0


https://answers.ros.org/question/313629/attempt-to-spin-a-callback-queue-from-two-spinners-one-of-them-being-single-threaded/