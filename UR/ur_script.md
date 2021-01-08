# The URScript Programming Language

http://www.me.umn.edu/courses/me5286/robotlab/Resources/scriptManual-3.5.4.pdf

https://robotics.stackexchange.com/questions/11137/ur3-tcp-ip-communication-protocol/11140#11140

https://s3-eu-west-1.amazonaws.com/ur-support-site/16496/Client_Interface_V3.12andV5.6.xlsx


## The URScript Programming Language

### Introduction

The Universal Robot can be controlled at two levels:
- The PolyScope or the Graphical User Interface Level
- Script Level

At the **Scipt Level**, the **URScript** is the programming language that controls the robot. The **URScript** includes variables, types, and the flow control statements. There are also built-in variables and functions that monitor and control I/O movements.

## Connecting to URControl

URControl is the low-level robot controller running on the Mini-ITX PC in the Control Box. When the PC boots up, the URControl starts up as a daemon (i.e., a service) and the PolyScope or Graphical User Interface connects as a client using a local TCP/IP connection.

Programming a robot at the *Script Level* is done by writing a client application (running at another PC) and connecting to URControl using a TCP/IP sockect.

- **hostname**: ur-xx (or the IP address found in the **About Dialog-Box** in PolyScope if the robot is not in DNS)
- port: 30002

When a connection has been established URScript programs or commands are sent in clear text on the sockect. Each line is terminated by "\n". Note that text can only consist of extended ASCII characters.

The following conditions must be met to ensure that the URControl correctly recognizes the script:

- The script must start from a function definition or a secondary function definition (either "def" or "sec" keywords) in the first column
- All other script lines must be indented by at least one white space
- The last line of script must be an "end" keyword starting in the first column

### Thread scheduling

Because the primary purpose of the URScript scripting launguage is to control the robot, the scheduling policy is largely based upon the realtime demands of this task.

The robot must be controlled a frequency of 125 Hz, or in other words, it must be told what to do every 0.008 second (each 0.008 second period is called a frame). To achieve this, each thread is given a "physical" (or robot) time slice of 0.008 seconds to use, and all threads in a runnable state is then scheduled in a round robin fashion. (Before the start of each frame the threads are sorted, such that the thread with the largest remaining time slice is to be scheduled first.)

Each time a thread is scheduled, it can use a piece of its time slice (by executing instructions that control the robot), or it can execute instructions that do not control the robot, and therefore do not use any "physical" time. If a thread uses up its entire time slice, it is placed in a non-runable state, and is not allowed to run until the next frame starts. If a thread does not use its time slice within a frame, it is expected to switch to a non-runnable state before the end of the frame. (If this expectation is not met, the program is stopped.) The reason for this state switch can be a join instruction or simply because the thread terminates.

It should be noted that even though the `sleep` instruction does not control the robot, it still uses 'physical' time. The same is true for the `sync` instruction.


## Module motion

### Functions

#### movej(q, a=1.4, v=1.05, t=0, r=0)

Move to position (linear in joint-space)

When using this command, the robot must be at a standstill or come from a `movej` or `movel` with a blend. The speed and acceleration parameters control the trapezoid speed profile of the move. Alternatively, the `t` parameter can be used to set the time for this move. Time setting has priority over speed and acceleration settings.

- Parameters
  - `q`: joint positions (q can also be specified as a pose, then inverse kinematics is used to calculate the corresponding joint positions)
  - `a`: joint acceleration of leading axis (rad/s^2)
  - `v`: joint speed of leading axis (rad/s)
  - `t`: time (S)
  - `r`: blend radius (m)
         If a blend radius is set, the robot arm trajectory will be modified to avoid the robot stopping at the point.

         However, if the blend region of this move overlaps with the blend radius of previous or following waypoints, this move will be skipped and an 'Overlapping Blends' warning message will be generated.

- Example command: 
  movej([0,1.57,-1.57,3.14,-1.57,1.57], a=1.4, v=1.05, t=0, r=0)

  Example Parameters:
  - q = (0,1.57,-1.57,3.14,-1.57,1.57) -> base is at 0 deg rotation, shoulder is at 90 deg rotation, elbow is at -90 deg rotation, wrist 1 is at 180 deg rotation, wrist 2 is at -90 deg rotation, wrist 3 is at 90 deg rotation. Note: joint positions (q can also be specified as a pose, then inverse kinematics is used to calculate the corresponding joint positions)
  - 1=1.4 -> acceleration is 1.4 rad/s/s
  - v=1.05 -> velocity is 1.05 rad/s
  - t=0 -> the time (seconds) to make move is not specified. If it values.
  - r=0 -> the blend radius is zero meters.

#### movel(pose, a=1.2, v=0.25, t=0, r=0)

Move to position (linear in tool-space)

See movej.

- Parameters:
  - pose: target pose (pose can also be specified as joint positions, then forward kinematics is used to calculate the corresponding pose)
  - a: tool acceleration (m/s^2)
  - v: tool speed (m/s)
  - t: time (s)
  - r: blend radius (m)

- Example command: movel(pose, a=1.2,v=0.25,t=0,r=0)
  
  Example Parameters:
  - pose=p(0.2,0.3,0.5,0,0,3.14) -> position in base frame of x=200mm, y=300mm, z=500, rx=0, ry=0, rz=180 deg
  - a=1.2 -> acceleration of 1.2 m/s^2
  - v=0.25 -> velocity of 250 mm/s
  - t=0 -> the time (seconds) to make the move is not specified. If it were specified the command would ignore the a and v values.
  - r=0 -> the blend radius is zero meters.

#### get_actual_joint_positions()

Returns the actual angular positions of all joints

The angular actual positions are expressed in radians and returns as a vector of length 6. Note that the output might differ from the output of get_target_joint_positions(), especially during acceleration and heavy loads.

**Return Value**: The current actual joint angular position vector in rad: (Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3)

#### get_actual_tcp_pose()

Return the current measured tool pose

Return the 6d pose representing the tool position and orientation specified in the base frame. The calculation of this pose is based on the actual robot encoder readings.

Return Value: The current actual TCP vector (X,Y,Z,Rx,Ry,Rz)

#### get_inverse_kin(x,qnear,maxPositionError=1e-10,maxOrientationError=1e-10)

Inverse kinematics

Inverse kinematic transformation (tool space -> joint space). If qnear is defined, the solution closest to qnear is returned. Otherwise, the solution closest to the current joint positions is returned.

Parameters:
- x: tool pose
- qnear: list of joint positions (Optional)
- maxPositionError: the maximum allowed position error (Optional)
- maxOrientationError: the maximum allowed orientation error (Optional)


Return Value: joint positions

Example command: get_inverse_kin(p[.1,.2,.2,0,3.14,0], [0.,3.14,1.57,0.785,0,0])

Example Parameters:
- x=p(.1,.2,.2,0,3.14,0) -> pose with position of x=100mm, y=200mm, z=200mm and rotation vector of rx=0 deg, ry=180 deg, rz=0 deg.
- qnear=(0.,3.14,1.57,0.785,0,0) -> solution should be near to joint angles of j0=0 deg, j1=180 deg, j2=90 deg,j3=45 deg, j4=0 deg, j5=0 deg
- maxPositionError is by default 1e-10m
- maxOrientationError is by default 1e-10 rad
- 