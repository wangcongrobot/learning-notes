# How to create a mujoco robot model from urdf file

## Official  MJCF Modeling Document
http://mujoco.org/book/modeling.html

## URDF

For the robotics community, ROS is almost the standard for robotics and we can find a robot model of URDF style. So first, we will introduce how to convert a URDF model to a MuJoCo model. It's simple but need some extra works.

https://wiki.aalto.fi/download/attachments/151495783/Final%20Report%20Group%2016%20E8004%202019.pdf?api=v2

## Convert the URDF to mjcf file

### Some reference links

https://github.com/openai/mujoco-py/issues/216

http://mujoco.org/book/modeling.html
http://www.mujoco.org/book/modeling.html#CURDF

http://www.mujoco.org/forum/index.php?threads/generation-mobile-robot-from-urdf-to-mjcf.3973/

https://github.com/iandanforth/mjcf

### convert a urdf file to mjcf file


MuJoCo can load XML model files in its native MJCF format, as well as in the popular but more limited URDF format. This chapter is the MJCF modeling guide. The reference manual is available in the XML Reference chapter. The URDF documentation can be found elsewhere; here we only describe MuJoCo-specific URDF extensions. 

two ways:
- convert urdf to mjmodel
- change the urdf file to xml file based mujoco xml reference


The MjModel is a bin file, converted from urdf.

1. convert xacro to urdf

Add mujoco tag to xacro, ensure that it can find the mesh files.
http://www.mujoco.org/book/XMLreference.html#compiler

For my robot, the **meshdir**, **balanceinertia**, **discardvisual** tags are important. Leave the others default setting.

```xml
  <mujoco>
        <compiler 
        meshdir="../meshes_mujoco/" 
        fusestatic= "true" 
        balanceinertia="true" # some geometry link in urdf could cause error, so set it "true"
        strippath="true" # default is "true", is "false", maybe cannot find the file path
        discardvisual="false" # this tag default is true, discard the visual geometry, if you want your robot look good, set it "false"
        />
  </mujoco>
```

Error: add balanceinertia tag if you has this error
```bash
Error: inertia must satisfy A + B >= C; use 'balanceinertia' to fix
Object name = inertial_link, id = 3
```

Copy all the stl file to the meshes file.

Some xacro files use the dae file to display, but mujoco cannot import the dae file. So we must change it to stl file using [MashLab](http://www.meshlab.net/). See [here](http://www.mujoco.org/forum/index.php?threads/unknown-mesh-file-type-dae.3495/).

[python script](https://github.com/shadow-robot/sr_common/blob/kinetic-devel/sr_description/mujoco_models/meshes/arm_and_hand_meshes/conversion.py) to convert the .dae file to .stl file.

This is a [python script](https://github.com/shadow-robot/sr_common/blob/kinetic-devel/sr_description/mujoco_models/meshes/arm_and_hand_meshes/conversion.py), written for blender for batch conversion of dexterous hand collada (.dae) mesh files to .stl format, for use in the Mujoco simulator.

https://github.com/bulletphysics/bullet3/issues/1507

- [some meshes ignored when converting urdf to mjcf](http://www.mujoco.org/forum/index.php?threads/meshes-ignored-when-converting-urdf-to-mjcf.3433/)

Darwin has collision meshes, which is why they are showing. *Sawyer only has visual meshes, which are discarded automatically when discardvisual="true", which is the default when parsing URDFs. So you need to set **discardvisual="false"** in \<compiler> tag*. Also, you have a box with size="0 0 0" which is an error. Geom sizes must be positive. See attached model.

Note that collision geoms are placed in geom group 0, while visual geoms are placed in geom group 1. You can toggle the rendering of each group **pressing '0' and '1'** respectively (in the GUI, press "0" and "1", you can see the collision rendering(simple geometry) or visual rendering(beautiful)). 

**You should conform that the top xacro/urdf file can find every subfile.**
http://www.mujoco.org/forum/index.php?threads/multiple-mesh-folders.3720/

```bash
roslaunch urdf_tutorial display.launch model:=path/to/file/robot.urdf
```

```bash
rosrun xacro xacro --inorder model.xacro > model.urdf
check_urdf model.urdf
```
2. convert urdf to mjb file

in the path mujoco200/bin
```bash
./compile model.urdf model.mjb
./compile model.urdf model.xml
./compile model.urdf model.txt
```

```bash
cong@eclipse:~/.mujoco/mjpro150/bin$ ./compile 

 Usage:  compile infile outfile
   infile can be in mjcf, urdf, mjb format
   outfile can be in mjcf, mjb, txt format

 Example:  compile model.xml model.mjb
```
Problems:
- You need add some tags manually
- the gripper may need add some tendon tag to control it


1. test mjb file

in mujoco200/bin
./simulate model.mjb
./simulate model.xml

## Add some extra useful elements

tendon: http://www.mujoco.org/book/XMLreference.html#tendon

Grouping element for tendon definitions. The attributes of fixed tendons are a subset of the attributes of spatial tendons, thus we document them only once under spatial tendons. Tendons can be used to impose length limits, simulate spring, damping and dry friction forces, as well as attach actuators to them. When used in equality constraints, tendons can also represent different forms of mechanical coupling.

tendon/fixed

This element creates an abstract tendon whose length is defined as a linear combination of joint positions. Recall that the tendon length and its gradient are the only quantities needed for simulation. Thus we could define any scalar function of joint positions, call it "tendon", and plug it in MuJoCo. Presently the only such function is a fixed linear combination. The attributes of fixed tendons are a subset of the attributes of spatial  tendons and have the same meaning as above.

tendon/fixed/joint

This element adds a joint to the computation of the fixed tendon length. The position or angle of each included joint is multiplied by the corresponding <font color='red'>coef</font> value, and added upo to obtain the tendon length.

joint: string, required
Name of the joint to be added to the fixed tendon. Only scalar joints (slide and hinge) can be referenced here.
coef: real, required
Scalar coefficient multiplying the position or angle of the specified joint.


http://www.mujoco.org/forum/index.php?threads/joint-compliance-in-non-thumb-fingers.14/


## Dual_UR5_Husky MuJoCo model


```bash
cong@eclipse:~/.mujoco/mujoco200/bin$ ./compile /home/cong/ros_ws/husky_ws/src/husky/husky_description/urdf/dual_arm_husky.urdf /home/cong/ros_ws/husky_ws/src/husky/husky_description/urdf/dual_arm_husky.urdf.xml
cong@eclipse:~/.mujoco/mujoco200/bin$ ./simulate /home/cong/ros_ws/husky_ws/src/husky/husky_description/urdf/dual_arm_husky.urdf.xml
```

## damping


C++ tutorial: 
https://github.com/atabakd/MuJoCo-Tutorials

## MuJoCo Robot Model Resource

- [Official resource]()

- [shadow-robot-mujoco-model](https://github.com/shadow-robot/sr_common/tree/kinetic-devel/sr_description/mujoco_models)

This directory contains files necessary for a Mujoco simulation of the dexterous hand (Hand E).


