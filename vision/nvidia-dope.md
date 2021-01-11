# Nvidia Deep Object Pose Estimation (DOPE) 

https://github.com/NVlabs/Deep_Object_Pose

Model: https://github.com/NVIDIA/Dataset_Utilities

NVIDIA Deep learning Dataset Synthesizer (NDDS): https://github.com/NVIDIA/Dataset_Synthesizer

[UE4 + NDDS (NVIDIA Deep learning Dataset Synthesizer ) Install Chinese](https://zhuanlan.zhihu.com/p/94445159)

## run in husky

in path dope_ws/src/dope,


## Install

Use conda python2.7 env to install pytorch 1.0.

### conda2.7

https://www.anaconda.com/distribution/

- Download the Anaconda2 and install.
- source ~/anaconda2/bin/activate
- conda create -n py27 python=2.7
- conda activate py27
- pip install rospkg catkin_pkg opencv-python opencv-contrib-python pyyaml  enum34 pyrr scipy
- conda install pytorch==1.0.0 torchvision==0.2.1 cuda100 -c pytorch (# CUDA 10.0)
- mkdir -p dope_ws/src
- git clone https://github.com/NVlabs/Deep_Object_Pose dope
- download weights
- rosdep install --from-paths src --ignore-src -r -y
- catkin_make
- git clone https://github.com/ros/geometry
- git clone https://github.com/ros/geometry2
- catkin_make
- change topic name: config/config_pose.yaml
- 

conda remove --name myenv --all


echo $PYTHONPATH
/home/cong/ros_ws/dope_ws/devel/lib/python2.7/dist-packages:/home/cong/ros_ws/ork_ws/devel/lib/python2.7/dist-packages:/opt/ros/kinetic/lib/python2.7/dist-packages

## Training on custom dataset

from [here](https://github.com/NVlabs/Deep_Object_Pose/issues/79#issuecomment-536315835):

It is using [NDDS](https://github.com/NVIDIA/Dataset_Synthesizer). Here are the general stes:
1. 3d model your object, e.g., a box. You can use blender for example.
2. Export your model to UE4 with NDDS. I recomend using FBX format.
3. Create a domain randomization with scene with your object being exported.
4. Generate 20k images (should be enough).
5. Use DOPE training [script](https://github.com/NVlabs/Deep_Object_Pose/blob/master/scripts/train.py), it is native to NDDS exported data, for about 30 epochs.
```bash
python train.py --data /home/atas/Dataset_Synthesizer/Source/NVCapturedData/TestCapturer --object untitled --outf untitled --gpuids 0
```
6. Deploy the trained weights to DOPE ROS adding weights and object dimensions.


https://youtu.be/lRLhNOMCRNA

https://youtu.be/2mU7I_q4OHg

https://www.bilibili.com/video/av76146114

https://www.youtube.com/channel/UC8-a_o2lFL3j5mAkxfLSoOw/videos

## NDDS

You can make use of DR_AnnotatedActor_BP under DomainRandomizationDNN Content for generating random poses. Replace the mesh with your custom object mesh.

Right now the NDDS only work with the UE4 textures. We imported the COCO images into UE4 textures and used it. [Ref](https://docs.unrealengine.com/en-US/Engine/Content/ImportingContent/ImportingTextures/index.html)
In the RandomBackground_actor_BP select the RandomMaterialParam_Texture and set the "Texture Directories" to the directory (note: the path is related to the project content directory, don't use absolute path) where those COCO textures stay.

To anyone who have problem with detection of oval objects;
in NDDS you can modify Blueprint of the object that you want to train. Changing interval of RandomRotation to
(0,0) from (-180,180) in rotational axe(z in my case) solved problem for me.

https://youtu.be/2mU7I_q4OHg:

You can add the OrbitalMovementComponent to your camera. You can mouse over each of the component's properties to read the tooltips, some of the main properties:

- FocalTargetActor - you need to select an actor in the level as the focal target
- PitchRotationRange - this is the attitude range
- YawRotationRange - this is the azimuth range
- TargetDistanceRange - how far the camera stay from the focal target, 60-80cm
NOTE: You need to turn on ShouldChangeDistance flag first
- Other properties allow you to change the speed of the rotation and distance change
Please let me know if it work out for you or not.

- [How to set custom annotation cuboid in NDDS](https://github.com/NVlabs/Deep_Object_Pose/issues/58)
  
Do you have the 3d model of the pen and hand separatedly? If so, you can create another actor as the hand and attach the pen (AnnotatedActor) to it. That way you can randomize the color/texture of the hand, also since the pen is attached to the hand, when you rotate the hand the pen will also rotated.

Step by step of the setup:

Create a blueprint of type AnnotatedActor for the pen - let's name it PenAnnotated_BP. I assume you already did this for your previous dataset.
Create an StaticMeshActor for the hand - let's name it HandActor. I suggest you create a blueprint for this, but for convenience, you can just drag the hand's mesh into the level.
Add ChildActorComponent to the HandActor - let's name this component PenChildComp. In ChildActorComponent, in the field ChildActorClass, select PenAnnotated_BP. Note that the pose of the PenChildComp is the pose of the pen. You can move and rotate PenChildComp to get what you want.
About your test environment, I think you should increase the number of light source and increase the randomizable intensity range of those lights. Also the material of the pen with how accurate its reflection compare to the real life help a lot too.

Please let me know if this setup help you or not.
