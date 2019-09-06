# How to use urdf in mujoco

https://github.com/openai/mujoco-py/issues/216

http://mujoco.org/book/modeling.html
http://www.mujoco.org/book/modeling.html#CURDF

http://www.mujoco.org/forum/index.php?threads/generation-mobile-robot-from-urdf-to-mjcf.3973/


MuJoCo can load XML model files in its native MJCF format, as well as in the popular but more limited URDF format. This chapter is the MJCF modeling guide. The reference manual is available in the XML Reference chapter. The URDF documentation can be found elsewhere; here we only describe MuJoCo-specific URDF extensions. 

two ways:
- convert urdf to mjmodel
- change the urdf file to xml file based mujoco xml reference


## 1. mjmodel
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

# 2. write the xml file
