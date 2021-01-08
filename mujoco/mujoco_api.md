# Some useful API function and data of mujoco

env.sim.model.joint_names

## gravity compensation

[So if you set qfrc_applied = qfrc_bias in your control law, you will compensate for gravity and all other internal forces.](http://www.mujoco.org/forum/index.php?threads/gravitational-matrix-calculation.3404/#post-3770)

http://www.mujoco.org/forum/index.php?threads/change-the-gravity-for-some-bodies-on-the-model.3710/#post-4742

https://github.com/StanfordVL/robosuite/blob/666588a997b91158ed530629d708b22c25f83e6c/robosuite/environments/panda.py#L217

## mujoco dynamics theory

https://www.dazhuanlan.com/2020/01/04/5e1033bb46910/

https://xueyuechuan.me/2019/08/16/mj-frc-inverse/