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