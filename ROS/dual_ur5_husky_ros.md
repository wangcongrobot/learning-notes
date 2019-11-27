# Husky Dual UR5 Mobile Manipulation Demo

https://www.clearpathrobotics.com/assets/guides/husky/index.html

## 

[Husky A200 UGV Robot Setup & Configuration](https://youtu.be/wA8UTF0mKBY?list=UUNPP3C-ZK3mwpG2x89VE-2Q)


## change the end effector angle

```bash
<joint name="l_ur5_arm_ee_fixed_joint" type="fixed">
    <parent link="l_ur5_arm_wrist_3_link"/>
    <child link="l_ur5_arm_ee_link"/>
    <origin rpy="-1.0 0.0 1.57079632679" xyz="0.0 0.0823 0.0"/>
</joint>
```

https://answers.ros.org/question/238586/ur5kinematicsplugin-fails-to-plan-in-cartesian-space-on-clearpath-husky-equipped-with-ur5/

I installed trac-ik-kinematics-plugin and changed my kinematics.yaml file from

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

  sudo apt-get install ros-kinetic-trac-ik*


# Husky Dual UR5 Official Document

## Communicating with Husky

To communicate directly with the Husky PC, you can SSH in. It will be necessary to ssh into the robot for tasks such as installing, modifying or removing software and files on the Husky. Note that you will not be able to usse GUI tools such as rviz over an SSH connection.

```bash
$ ssh administrator@192.168.1.11
```
OR
```bash
$ ssh administrator@cpr-a200-0375
```
In order to use rviz and other visualization tools, you must declare the Husky as master. In a new console, type:
```bash
$ export ROS_MASTER_URI=http://cpr-a200-0375:11311
```
You should then be able to view a list of topics published by the Huksy with:
```bash
$ rostopic list
```
In will be necessary to declare the Husky as master in every new terminal window, unless you change the master permanently in your ROS environment variables. **If you are unable to connect with Husky via its hostname**, your computer may not be routing hostname properly. In Ubuntu on your local computer, open your /etc/hosts file:
```bash
$ sudo nano /etc/hosts
```
Add the following line immediately below the line contains 127.0.1.1, substituting in the Husky's current wifi IP address. This address may be obtained by connecting directly to the Husky via Ethernet, and using the "ifconfig" command. You may want to talk to your system administrator about giving the Huksy a permanent wifi address to ensure it always connects with the same IP address. The below example shows the setting if wired directly into the robot lan.
```
192.168.1.11 cpr-a200-0375
```
To ease communications between the robot and your computer, you can also add a similar entry in the Husky's computer, pointing at one or more development computers.

## Software Information

- By default, the sourced workspace on this robot is **~/catkin_ws** 
- This system is pre-configured to start a joystick interface node for teleoperation. At any time, the wireless gamepad may be used to drive the Husky. Hold the **"X"** button for fast mode or **"A"** for slow, and steer with the left analog stick.
- The hardware launch script will run on startup. It can be started in the background with **sudo service husky-core start** and stopped with **sudo service husky-core stop**. It may be launched in the foreground using **sudo husky-core-start**. Your team should never need to start or stop the service--just use roslaunch to launch additional nodes which interface with the persistent ones.

## Network Information

Husky's payloads have been configured with static IP addresses as listed below. To communicate with your Husky, connect your computer to the loose Ethernet cable attached to the 3D LIDAR pedestal. Set a static IP on your computer to 192.168.1.100, or allow the robot to assign an IP to your computer via DHCP. Then, ssh into the Husky computer with:
```
$ ssh administrator@192.168.1.11
```

The Husky contains an onboard wifi router configured as a client. Once connected to the onboard Husky network via ethernet, you may log in to the [Ubiquiti Bullet](https://www.ui.com/airmax/bulletm/) configuration page by entering its IP address in a web browser. Then visit the **Wireless** tab to select which wifi network the robot should connect with. In order to comply with local wireless connecting requirements, you may also want to switch the region to that of your own. Please note that the network you are connecting with should be on a different subnet from the Husky network, or IP conflicts may occor.

When connecting to the Husky computer by wifi, it should be accessible via its hostname:
```
$ ssh administrator@cpr-a200-0375
```

|Parameter|Value|
|---|---|
|Ubiquiti Bullet IP|192.168.1.10|
|Ubiquiti Bullet Login|administrator|
|Husky IP|192.168.1.11|
|Husky Hostname|cpr-a200-0375|
|Husky Login|administrator:clearpath|
|SICK LIDAR IP|192.168.1.14|
|Left UR5 IP|192.168.1.18|
|Left Gripper IP|192.168.1.19|
|Right UR5 IP|192.168.1.28|
|Right Gripper IP|192.168.1.29|
|UR5 Safety Password|1111|
|VLP16 IP (default)|192.168.1.201|

## UR5 and Gripper Information

Your Huksy is highly customized, to permit the safe and effective operation of your dual manipulator system. The Github repository containing most of the custom code is available here:
```
https://github.com/DualUR5Husky/husky/tree/dual_ur5_husky
```
On your workstation, you'll need to clone the following package:
```
mkdir -p ~/dual_ws/src
cd ~/dual_ws/src && catkin_init_workspace
git clone https://github.com/DualUR5Husky/husky
git clone https://github.com/DualUR5Husky/ur_mordern_driver
git clone https://github.com/DualUR5Husky/universal_robot
git clone https://github.com/DualUR5Husky/robotiq
git clone https://github.com/DualUR5Husky/husky_simulator
cd ..
sudo apt-get update
rosdep install --from-paths src --ignore-src --rosdistro=indigo -y
sudo apt-get install ros-indigo-moveit-planners-ompl ros-indigo-moveit
catkin_make
```

Once your workspace has been setup:

1. SSH into the Huksy computer
2. If the Husky was freshly booted, wait for the PTU to compute its its warmup sequence.
3. Turn on the arms, clear the e-stop conditions, and ensure the arms are ready to move. See "SAFETY WARNING" and "QUICK START" sections at the beginning of this document.
4. Launch the arm drivers on the robot using: sudo dual-ur5-start
5. Launch MoveIt! On the roobt using: roslaunch husky_dual_ur5_moveit_config husky_dual_ur5_planning_execution.launch real:=true
6. Ensure you've exported your ROS_MASTER_URI as mentioned in the previous section and export ROS_HOSTNAME=\<hostname of your workstation\> on your workstation
7. Finally, source the workspace on your workstation: source devel/setup.bash (in the dual_ws folder) and then launch the moveit interface on your workstation: roslaunch husky_dual_ur5_moveit_config moveit_rviz.launch config:=true

**NOTE:** When moving the robot arms, always use the plan feature in Moveit first to ensure that the robot will execute the movement without causing any damage or collisions.

The Husky is now ready to receive movement commands. Some examples of movement commands written in python may be found in the dual_ur5_teleop_general package.

## Manual UR5 and Gripper Control

While it is always possible to move the UR5 arms using the UR user interface, it is also possible to move both the arms and the grippers using the Husky gamepad controller. This feature has been provided as a convenience, and should not be considered a precition control scheme. To manually control the arms and grippers using the Logitech gamepad, first ensure that the UR5 arms are enabled and any e-stops have been released. Launch the UR packages as described above. Then, try the ur5 teleop package on the robot:
```
$ rosrun dual_u5_teleop_general dual_ur5_teleop_general.py
```

The control scheme is as follows:

|Logitech Gamepad Button|Action|Notes|
|--|--|--|
|Left Trigger "LT"|Enter Left Arm Control Mode||
|Right Trigger "RT"|Enter Right Arm Control Mode||

NOTES:

- The "X" and "A" buttons persist as the Husky movement "deadman" controls.
- It is not currently possible to change the angle of the grippers using the Logitech controller. The angle must still be changed inside of the MoveIt! Interface, or via custom joint commands to the arm.

## Important Notes

- Be aware that while several joints on the arm can turn 360 degrees in either direction, **the cables cannot**. Plan movements accordingly.
- Limits have been implemented in the UR interface to limit joint movement and help prevent cable damage. These can be removed in the safety screen with the password "1111", though this is not recommended. If the robot is forced into those regions, a manual safety reset is  required.
- Certain movements may cause the arm to move very quickly. Consider breaking movements into steps to limit undesirable movement.
- Gripper configuration is not modelled, as the under actuated fingers depend heavily on the object the gripper is grasping. Without feedback, this is exceedingly difficult to model.
- Planned motions are not guaranteed, and can crash into obstacles not represented in the URDF (what you see in the planner above).
- Collision models are as precise as the sensors and physical model allows, however they are not perfect. This means that passing within millimeters of obstacles or joint interferences is not advised.
- Do not attempt to manually control the arm via the UR interface while a planned movement is executing. Doing so may lead to undefined behaviour.
- The gripper configuration settings may be changed using the Robotiq software (available form the Robotiq website). To connect with the gripper, remove the small oval plate between the COM and PWR connectors. Then, connect a USB A to USB A cable between the gripper and your Windows PC.

## UR Monitor and Mouse
- The flat panel monitor and mouse mounted to the Huksy are connected to the UR control computer, via a KVM switch. They power on automatically with the Husky. If you find that one of the arms does not seem to be displaying an image on the monitor, it may be necessary to restart that arm.
- Certain errors generated by the arm, such as an e-stop event, force limits, joint limits, and power supply errors, can only be cleared using the UR GUI. If these occur, first cancel any operations that may be running via the ROS driver. Then clear the error using the GUI.

## Onboard Sensors

### SICK LIDAR

The LIDAR data produced by the SICK LMS is not in a human-readable format. However, you can check that it is publishing data with the "hz" command. The data it produces is best visualized from within rviz.

```
$ rostopic hz /scan
```

### Garmin GPS

When outdoors, the Husky will publish GPS fix data. You can verify the data in ROS with:
```
$ rostopic echo /gps/fix
```
The raw decoded GPS messages may be viewed with:
```
rostopic echo /gps/nmea_sentence
```

### Microstrain IMU

The raw IMU data may be verified with:
```
$ rostopic echo /imu/data
```

Note that the data in this topic is pre-transform. 

### FLIR PTU
Joint commands for the PTU may be sent to the /ptu/cmd topic.
```
rostopic pub /ptu/cmd sensor_msgs/JointState (press tab to complete)
```

This position array and the velocity array must have two values in each: one for pan, one for tilt. The position angle is in radians, the velocity is in radians/second.

### Bumblebee Camera

You may view the left and right stereo camera feeds from within ROS using the image_view utility:
```
$ rosrun image_view image_view image:=/camera/left/image_color
$ rosrun image_view image_view image:=/camera/right/image_color
```
Upon receipt of your robot, you may want to take the time to calibrate your stereo camera. Please see this tutorial for information on how to do this:
```
https://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration
```

### Battery Charger

Your Husky is equipped with a powerful 40Ah lithium battery pack. A battery charger array has been supplied to charge the pack. To use it, unplug the charge port dongle and plug in the charger array. Then, plug the charger array into an AC outlet. Finally, flip the switch on the battery charger power bar to the On position. "LED2" will turn red on each charger to indicate charging is in progress. A full charge takes about 4 hours.

When all four of the chargers in the array have finished (LED2 turns green), the power bar may be turned off and the charger array may be disconnected. Plug the charger dongle back in to the charge port to enable power to the Husky.

### Shore Power Supply


## Gripper

The commands to use a simple interface with the gripper:

- Double check the gripper drivers are running

```
rostopic list | grep gripper
```
- Launch the simple gripper interface

```
rosrun robotiq_s_model_control SModelSimpleController.py
```
- For more complex control, see https://wiki.ros.org/robotiq

How to launch the planning exectuion node for the arms (while ssh'd inside robot)

- Launch the planning execution:

```
roslaunch husky_dual_ur5_moveit_config husky_dual_ur5_planning_execution.launch real:=true
```
How to launch the arm drivers:
- Launch the drivers for left and right arms (after the arms have powered and booted)

```
roslaunch /etc/ros/indigo/husky-core.d/left_ur5.launch
roslaunch /etc/ros/indigo/husky-core.d/right_ur5.launch
```


REAL ROBOT INSTRUCTIONS:

To use the Clearpath Robotics's Dual UR5 Husky, make sure you have environment variables correctly set up. The correct setup (to be added in your real Husky's /etc/ros/setup.bash file (if you're using sim it will be your ~/.bashrc)) is listed below.

How to launch the RViZ viewer on your computer:

1) Set up your ROS_MASTER_URT and ROS_IP:
```
export ROS_MASTER_URI=http://192.168.1.11:11311
export ROS_IP=your_computers_lan_ip
```
2) Download the Dual UR5 Husky code and install on your computer:
   
```
mkdir -p husky_ws/src
cd husky_ws/src && catkin_init_workspace
git clone http://github.com/husky/husky
cd husky && git checkout dual_ur5_husky
cd ~/husky_ws && catkin_make install
source ~/husky_ws/devel/setup.bash
```
3) Run the moveit_rviz.launch file:
```
roslaunch husky_dual_ur5_moveit_config moveit_rviz.launch config:=true
```

## Vehicles Equations

As a starting point, Clearpath Robotics recommends using the following relationship between wheel velocity and platform velocity:

$$v=\frac{v_r+v_l}{2},w=\frac{v_r-v_l}{w}$$

$v$ represents the instantaneous translational speed of the platform and $w$ the instantaneous rotational speed. $v_r$ and $v_l$ are the right and left wheel velocities, respectively. $w$ is the effective track of the vehicle, $0.555$ m.

## Husky

### Nodes

Standard Husky Nodes

|Node|Description|
|---|---|
|/husky_node|Provides control and communication between the Husky platform and ROS. Accepts velocity commands and provides system feedback on /status|
|/robot_state_publisher|Subscribes to /joint_states and publish the robot's state to tf|
|/bluetooth_teleop|Publishes velocity commands from a joystick to /twist_mux|
|/twist_mux|Takes in multiple sources of velocity commands, and prioritizes what actually gets sent to the controller|
|/ekf_localization|Part of the robot localization package, more information regarding this package can be found at http://wiki.ros.org/robot_localization|

## Topics

Standard Husky Topic

|Topic|Message type|Description|
|---|---|---|
|/bluetooth_teleop/joy|sensor_msgs/Joy|Receives joystick commands, echo this topic to verify your controller is publishing|
|/tf|tf2_msgs/TFMessage|Transforms between coordinate frames, this should always be publishing, and hence a good topic to echo to test your ROS connection|
|/status|husky_msgs/HuskyStatus|Displays system status information|
|/estop|std_msgs/Bool|Displays the estop status|
|/odometry/filtered|nav_msgs/Odometry|The odometry estimate of the robot from /ekf_localization|

Standard Husky Motion Topics

|Motion Topick=s|twist_mux Priority|Description|
|---|---|---|
|husky_velocity_controller/cmd_vel|-|Receives motion commands from twist_mux based off their priority|
|joy_teleop/cmd_vel|10|Joystick teleop input|
|twist_marker_server/cmd_vel|8|Interactive marker teleop input|
|move_base/cmd_vel|2|Autonomous movement input, for the husky navigation packages|
|cmd_vel|1|Miscellaneous external input|

## robot_upstart

http://wiki.ros.org/robot_upstart


This package allows robot administrators to quickly and easily generate one or more background jobs to run roslaunch invocations on system startup. 

http://docs.ros.org/jade/api/robot_upstart/html/

- Upstart for ROS Robots

This package aims to assist with creating simple platform-specific jobs to start your robot's ROS launch files when its PC powers up.

- Usage

The basic or one-off usage is with the `install` script, which can be as simple as:

```
$ rosrun robot_upstart install my_robot_bringup/launch/base.launch
```
This will create a job called `myrobot` on your machine, which launches base.launch. It will start automatically when you next start your machine, or you can bring it up and down manually:
```
$ sudo service myrobot start
$ sudo service myrobot stop
```

If the job is crashing on startup, or you otherwise want to see what is being output to the terminal on startup, check the upstart log:
```
$ sudo tail /var/log/upstart/myrobot.log -n 30
```

## Husky robot_upstart

The expected model for extending Husky is to create a new ROS Workspace in the user's home directory, and add packages there. You can roslaunch additional nodes against the ROS Master already running on Husky without needing to stop or start anything else. See the Overlays page on the ROS wiki for more details:
```
http://wiki.ros.org/catkin/Tutorials/workspace_overlaying
```
When it comes time for some of your own code to be launched on startup of Husky, edit the default workspace setup file, located in `/etc/ros/setup.bash`. Change the final line from the default to instead source your own workspace. When this is done, copy the launch file for your software into the `robot_upstart` folder, which is located at `/etc/ros/hydro/hysky-core.d`. Now start the Husky background service and your nodes should come up with the rest of Husky:
```
sudo service husky-core restart
```
For more details on this process, please see the ROS wiki page about upstart, located at: http://wiki.ros.org/robot_upstart

## For Real Husky

- Connect with Husky via Ethernet

Add a new ethernet setting, named `husky`. In button `IPv4 Settings`, `Method` choose **Manual**, Address: 192.168.1.105, Netmask:24, Routes: Use this connection only for resources on its network (not choose).

- connect with Husky via Wifi

choose wifi `HWU09 Husky`, password: clearpath, `IPv4 Settings`:192.168.1.105 & 24, `Routes`: Use this connection only for resources on its network (choose).

- Multi ROS Master

Add `alias` to bashrc in the local PC (not the Husky PC):
```
alias husky='export ROS_MASTER_URI=http://192.168.1.11:11311; export ROS_IP=192.168.1.105'
```
In your computer, open a new terminal (Ctrl+Alt+T), run `husky` and it will connect with husky with a multi ros master mode. Then you can use `rostopic list` to check the running topic.

- moveit slow mode

```
roslaunch husky_dual_ur5_moveit_config_slow start.launch
```

- bumblebee stereo camera

```
roslaunch bumblebee_bringup bringup.launch
```

- rgbd camera

```
roslaunch astra_husky astra_husky.launch
```

- easy_handeye

```
roslaunch easy_handeye husky_ur5_xtion_calibration.launch
```

The calibration result locates in `~/.ros/easy_handeye/` folder:
```
eye_on_hand: false
robot_base_frame: r_ur5_arm_base_link
tracking_base_frame: camera_link
transformation:
 qw:
 qx:
 qy:
 qz:
 x:
 y:
 z:
```

## UR5 GUI Interface

`Run`:

`Move`: Freedrive: manually move each joint

`Automove`: back to initial position

swich button to choose left or right arm

two blue light button for power

cable red e-stop button

## ros time synchronization