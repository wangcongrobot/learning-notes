# HDT-Global Adroit-M Arm

https://github.com/aurone/rcta_manipulation

https://github.com/sagivald/hdt_rlb_vr

https://github.com/hdtrobotics

The manipulator has 6 revolute joints and a mass of 9 kg.

UWSim

## Arm

The manipulator arm has in total 6 degrees-of-freedom divided into the wrist assembly, elbow assembly and shoulder assembly. the shoulder assembly has two DOFs: the shoulder pitch and yaw joints, the elbow consits of one single DOF and the wrist assambly has roll, yaw and pitch DOFs. At each joint the manipulator has a **position sensor**, a **current sensor**, a **torque sensor** and a **temperature sensor**. The maximum joint speed is 0.7 rad/sec (equivalent of 40 deg/sec) and the maximum joint torque is 60 Nm. The weight in air of the arm is 9 kg wihle in water is has a weight of 3 kg. The manipulator joint position limits are presented in Table 6.1. The arm does not have an end-effector force sensor that can be used to measure the interaction force with the environment. To reduce the weight of the arm in the water, floats can be attached to each link. Nevertheless these restrict the movement of the arm and this reduces the workspace of the arm. Due to this, they are not used in these experiments.

Manipulator arm joint limits
|Joint name| Minimum joint limit (rad)|Maximum joint limit (rad)|
|:-----:|:-----:|:-----:|
|Shoulder roll|-3.14|3.14|
|Shoulder pitch|-1.57|3.14|
|Elbow pitch|-1.57|3.14|
|Wrist roll|-3.14|3.14|
|Wrist yaw|-1.57|1.57|
|Wrist pitch|-1.57|1.57|


**Integration Kit**
The Integration Kit (IK) is used as the mounting base for the arm and to produce the comunication between the arm and the exterior world. The Integration Kit incorporates a micro-controller and consists of the manipulator mounting point and two electrical connectors. A detailed schematic of the Integration Kit is presented in Figure. The communication between the arm and external user is provided by the Integration Kit over a 100 Mbps Ethernet connection. A composite video connection is available to the camera placed at the end-effector. The IK has MCBHRA4M connector for power and receives 48 VDC from an external source and can draw up to 600 W of power. A MCBHRA8M connector is used for communication with external sources. System information and control commands are sent across the network based on a TCP (socket) protocol to the IK. From here this information is transmitted forward to the manipulator drives based on a CAN communication bus. From an external source the manipulator drives can be commanded using the Robot Operating System (ROS) architecture. The commands that can be sent to the drives are joint position, joint velocity and joint currents.

The two controllers described in Section 6.1 are implemented in Python on an external computer. The controllers compute the torque required for the end-effector to move and to obtain the desired contact force. The torque is sent directly to the drives of the manipulator arm, bypassing the existing joint controllers of the arm. The communication with the joint drives is facilitated through ROS at a rate of 10 Hz. This represents the maximum limit at which the arm can communicate. The drives provide in response the current joint position and velocity required in the control structures. The MoveIt software is used to compute the forward and inverse kinematic models of the robot. The same software decides the optimal path the end-effector has to take to reach the desired goal. The path generated represents the optimal path and takes into consideration the joint limits and self-collisiotn information. By using this method to generate the trajectory, it is ensured that the arm is not put into a singular configuration. As a safety feature the maximum current drawn by each motor in limited to 3 Amperes.

## HDT ROS interface documentation

Author: Corina Barbalata
Date: September 2015

Content:
1. System requirements 
2. System components
   - System interface
   - Camera calibration
   - Dynamic simulator
   - System path planning
3. Example ro run simulator


1. **System requirements:**
- ROS indigo (catkin)
- libraries: libreadline-dev, socketcan_interface
- MoveIt-ROS (not mandatory)
- UWSim (not mandatory)

2. **System components:**
   - System interface
     - contains the interface between ROS and HDT arm
     - hdt_mk3_m_description : urdf files of the arm; control over the arm with rviz
     - hdt_mk3_m_drivers: drivers for the arm when communication done with the help of rviz
     - hdt_hoku_drivers: drivers for the arm when communication done with the hocu (manual control) 
     - hdt_mk3_m_moveit_config: kinematics configuration of the arm (moveit)
     - hdt_control:  low-level control and basic manipulation grasping based on visual servoing


   - Camera calibration – results of the HDT- hand camera calibration

   - Dynamic simulator  - a dynamic model for the HDT arm (aimed to be added to the hole HDT simulator)

   - System Path-planning – a path trajectory for the arm together with a OSC controller (under development)

3. Example to run the code

Run arm simulator:
  roslaunch hdt_mk3_m_description hdt_mk_m_rviz_v2.launch
Run real arm:
1) Configure interface and chack connection to arm:
   sudo ifconfig eth0:1 192.168.1.X up
   ping 192.168.1.45
2) Run rviz graphic mode
   roslaunch hdt_mk3_m_description hdt_mk3_m_real_v2.launch

Run separate code with the arm:
1) Configure interface and chack connection to arm:
   sudo ifconfig eth0:1 192.168.1.XX up
   ping 192.163.1.45
2) Run rviz graphic mode
   roslaunch hdt_mk3_m_description hdt_mk3_m_real_v2.launch
3) Kill the joint_state_publisher
   rosnode kill /joint_state_publisher
4) Start the code
   e.g: roslaunch hdt_control hdt_app.launch


## OpenAI gym environment

1. ros control system
   - ros driver
   - ros moveit control
   - ros gazebo
   - ros uwsim environment
  
2. OpenAI gym environment
   - robot environment
   - task environment
   - Gazebo or UWSim simulator

3. Pick & Place task
   - simulation
   - real-world wwater tank