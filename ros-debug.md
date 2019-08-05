

# install mujoco_ros_pkgs

- git clone https://github.com/shadow-robot/mujoco_ros_pkgs.git
- rosdep install --from-paths src --ignore-src -r -y
- change mjpro150 path
- catkin_make
- can only run with mjpro150, not mujoco200
