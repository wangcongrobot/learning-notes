# Eye-Hand Calibration


https://blog.csdn.net/weixin_40799950/article/details/82594537



- [handeye_calib_camodocal](https://github.com/jhu-lcsr/handeye_calib_camodocal)

Easy to use and accurate hand eye calibration which has been working reliably for years (2016-present) with kinect, kinectv2, rgbd cameras, optical trackers, and several robots including the ur5 and kuka iiwa.


- [autoCalibration](https://www.three.ml/2018/08/hand-eye-calibration/)

Automatic Hand-to-Eye Calibration for Dobot Magician and Realsense D415


- [easy_handeye](https://github.com/IFL-CAMP/easy_handeye)

## easy handeye 

- error

Kin chain provided in model doesn't contain standard UR joint 'shoulder_lift_joint'.

https://answers.ros.org/question/238586/ur5kinematicsplugin-fails-to-plan-in-cartesian-space-on-clearpath-husky-equipped-with-ur5/

ur5_arm:
  kinematics_solver: ur_kinematics/UR5KinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
  kinematics_solver_attempts: 3

to

ur5_arm:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  kinematics_solver_timeout: 0.05
  solve_type: Manipulation1



 - error:
```bash
[ERROR] [1574031774.597142069, 746.623000000]: Transform error: "l_finger_1_link_proximal_actuating_hinge" passed to lookupTransform argument source_frame does not exist. 
[ERROR] [1574031774.597246755, 746.623000000]: Transform cache was not updated. Self-filtering may fail.
[ERROR] [1574031778.340112771, 747.652000000]: Transform error: "l_finger_1_link_proximal_actuating_hinge" passed to lookupTransform argument source_frame does not exist. 
[ERROR] [1574031778.340171866, 747.652000000]: Transform cache was not updated. Self-filtering may fail.
[ERROR] [1574031782.171098816, 748.679000000]: Transform error: "l_finger_1_link_proximal_actuating_hinge" passed to lookupTransform argument source_frame does not exist. 
[ERROR] [1574031782.171188049, 748.679000000]: Transform cache was not updated. Self-filtering may fail.
PluginManager._load_plugin() could not load plugin "rqt_easy_handeye/Hand-eye Calibration automatic movement":
Traceback (most recent call last):
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/qt_gui/plugin_handler.py", line 99, in load
    self._load()
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/qt_gui/plugin_handler_direct.py", line 54, in _load
    self._plugin = self._plugin_provider.load(self._instance_id.plugin_id, self._context)
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/qt_gui/composite_plugin_provider.py", line 71, in load
    instance = plugin_provider.load(plugin_id, plugin_context)
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/qt_gui/composite_plugin_provider.py", line 71, in load
    instance = plugin_provider.load(plugin_id, plugin_context)
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rqt_gui_py/ros_py_plugin_provider.py", line 60, in load
    return super(RosPyPluginProvider, self).load(plugin_id, plugin_context)
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/qt_gui/composite_plugin_provider.py", line 71, in load
    instance = plugin_provider.load(plugin_id, plugin_context)
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rqt_gui/ros_plugin_provider.py", line 101, in load
    return class_ref(plugin_context)
  File "/home/cong/ros_ws/dual_ws/src/eye_hand_calibration/easy_handeye/rqt_easy_handeye/src/rqt_easy_handeye/rqt_calibrationmovements.py", line 306, in __init__
    self._widget = CalibrationMovementsGUI()
  File "/home/cong/ros_ws/dual_ws/src/eye_hand_calibration/easy_handeye/rqt_easy_handeye/src/rqt_easy_handeye/rqt_calibrationmovements.py", line 152, in __init__
    self.local_mover = CalibrationMovements(move_group_name, max_velocity_scaling, max_acceleration_scaling)
  File "/home/cong/ros_ws/dual_ws/src/eye_hand_calibration/easy_handeye/rqt_easy_handeye/src/rqt_easy_handeye/rqt_calibrationmovements.py", line 26, in __init__
    self.mgc = MoveGroupCommander(move_group_name)
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/moveit_commander/move_group.py", line 52, in __init__
    self._g = _moveit_move_group_interface.MoveGroupInterface(name, robot_description, ns)
RuntimeError: Unable to connect to move_group action server 'move_group' within allotted time (5s)
```

For husky dual ur5 usage, see [link]().


## Hand eye calibration and coordination tf

https://blog.csdn.net/weixin_40799950?t=1

### Time Setting
```bash
sudo apt-get update
sudo apt-get install chrony ntpdate
sudo ntpdate ntp.ubuntu.com
or
sudo date -s "16:47 1/23/2017"
or sudo ntpdate 192.168.1.105(your PC IP)
```

