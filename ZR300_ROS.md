# Intel ZR300 Depth Camera

## ROS driver

http://wiki.ros.org/RealSense

http://wiki.ros.org/librealsense

http://wiki.ros.org/realsense_camera

First, install librealsense, then install realsense_camera.

## Testing

roslaunch realsense_camera zr300_nodelet_default.launch

rosrun rqt_image_view rqt_image_view

## eye_hand calibration

https://github.com/topics/hand-eye-calibration

## Object Detection

https://github.com/akio/mask_rcnn_ros

https://github.com/justice-suri/detectron_ros

https://www.cnblogs.com/zhencv/p/8384419.html

## Anaconda with ROS

http://wiki.ros.org/IDEs#Anaconda

```bash
source ~/anaconda3/bin/activate
pip install opencv-python opencv-contrib-python
pip install rospkg rospy catkin_pkg
sudo apt-get install python-rospkg
```
export PYTHONPATH=/opt/ros/kinetic/lib/python2.7/dist-packages/

## ROS via python3

https://blog.csdn.net/handsome_for_kill/article/details/81947978

https://blog.csdn.net/bluewhalerobot/article/details/80952776

https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674

https://stackoverflow.com/questions/43019951/after-install-ros-kinetic-cannot-import-opencv

http://wiki.ros.org/cv_bridge


## Facebook Detectron2

We use CUDA 10.0 in Anaconda3

https://github.com/facebookresearch/detectron2/blob/master/INSTALL.md

```bash
# pytorch
source ~/anaconda3/bin/activate
# not pip install
conda install pytorch torchvision cudatoolkit=10.0 -c pytorch
pip install 'git+https://github.com/facebookresearch/fvcore'
pip install cython
pip install 'git+https://github.com/cocodataset/cocoapi.git#subdirectory=PythonAPI'

git clone git@github.com:facebookresearch/detectron2.git
cd detectron2
python setup.py build develop

# Test
python -c 'import torch; from torch.utils.cpp_extension import CUDA_HOME; print(torch.cuda.is_available(), CUDA_HOME)'

pytest

python demo/demo.py --config-file configs/COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml \
  --input input.jpg \
  --opts MODEL.WEIGHTS detectron2://COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x/137849600/model_final_f10217.pkl

  python demo/demo.py --config-file configs/COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml \
  --input-video video.mkv \
  --opts MODEL.WEIGHTS detectron2://COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x/137849600/model_final_f10217.pkl

  python demo/demo.py --config-file configs/COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml \
  --webcam \
  --opts MODEL.WEIGHTS detectron2://COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x/137849600/model_final_f10217.pkl

```

    import cv2
ImportError: /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so: undefined symbol: PyCObject_Type

pip install opencv-python opencv-contrib-python

import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') 
import cv2


