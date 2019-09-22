# How to create a mujoco robot model from urdf file

For the robotics community, ROS is almost the standard for robotics and we can find a robot model of URDF style. So first, we will introduce how to convert a URDF model to a MuJoCo model. It's simple but need some extra works.

## Convert the URDF to mjcf file

### Some reference links

https://github.com/openai/mujoco-py/issues/216

http://mujoco.org/book/modeling.html
http://www.mujoco.org/book/modeling.html#CURDF

http://www.mujoco.org/forum/index.php?threads/generation-mobile-robot-from-urdf-to-mjcf.3973/

### convert a urdf file to mjcf file


MuJoCo can load XML model files in its native MJCF format, as well as in the popular but more limited URDF format. This chapter is the MJCF modeling guide. The reference manual is available in the XML Reference chapter. The URDF documentation can be found elsewhere; here we only describe MuJoCo-specific URDF extensions. 

two ways:
- convert urdf to mjmodel
- change the urdf file to xml file based mujoco xml reference


The MjModel is a bin file, converted from urdf.

1. convert xacro to urdf

rosrun xacro xacro --inorder model.xacro > model.urdf
check_urdf model.urdf

2. convert urdf to mjb file

in the path mujoco200/bin
./compile model.urdf model.mjb
./compile model.urdf model.xml
./compile model.urdf model.txt

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


