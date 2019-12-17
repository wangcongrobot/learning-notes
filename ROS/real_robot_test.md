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