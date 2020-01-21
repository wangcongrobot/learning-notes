# Real Robot Test

husky_tf

husky_train: node_test -> prepare pose; node

husky-pc: time(ntp)

moveit, bb2_stereo_camera

gym: rollout.py

- dope.launch

source /home/cong/anaconda2/bin/activate
conda activate py27


rostopic echo /transformPose
rostopic echo /dope/pose_so 

python ros_ws/dual_ws/src/ikfastpy/husky_move.py

- connect with husky
- sudo ntpdate -u 192.168.1.28
- roslaunch husky_dual_ur5_moveit_config_slow start.launch
- roslaunch bumblebee_bringup bringup.launch
- roslaunch husky_train husky_ur_motion_planner_node.launch
- python tf_transform.py
- python tf_transform2.py
- roslaunch dope dope.launch
- roslaunch husky_transform husky_ur_tf.launch
- rviz
- python ros_ws/dual_ws/src/ikfastpy/husky_move.py
- python ros_ws/dual_ws/src/ikfastpy/moveit_cartesian.py
- ~/ros_ws/openai_ros_ws/gym$ python gym/envs/robotics/algos/rllib_rollout.py /home/cong/ray_results/default/PPO_HuskyPickAndPlace-v1_0_2020-01-14_19-36-2473kc31gr/checkpoint_300/checkpoint-300 --run PPO --no-render
- 