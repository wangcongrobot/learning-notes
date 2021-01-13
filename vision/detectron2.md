## Facebook Detectron2

We use Facebook Detectron2 as the vision network, which has the mask function.

## Install

## training with custom dataset

- [Detectron2训练自己的数据集手把手指导](https://zhuanlan.zhihu.com/p/88139849)

**code**: https://github.com/jinfagang/FruitsNutsSeg

visualize the labelled data: https://www.zhihu.com/question/384519338/answer/1150941204

(***)[How to create custom COCO data set for instance segmentation ](https://www.dlology.com/blog/how-to-create-custom-coco-data-set-for-instance-segmentation/)

(***)[How to train Detectron2 with Custom COCO Datasets](https://www.dlology.com/blog/how-to-train-detectron2-with-custom-coco-datasets/)

[Detectron2_custom_coco_data_segmentation.ipynb](https://colab.research.google.com/github/Tony607/detectron2_instance_segmentation_demo/blob/master/Detectron2_custom_coco_data_segmentation.ipynb)

**code**: https://github.com/Tony607/detectron2_instance_segmentation_demo

[labelme](https://github.com/wkentaro/labelme)


## detectron2_ros

https://github.com/DavidFernandezChaves/Detectron2_ros

```xml
<arg name="input" default="semantic_mapping/RGB" />
<arg name="input" default="camera/color/image_raw" />
```
```python
if '/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2 as cv
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
```
download model file:
```python
self.cfg.MODEL.WEIGHTS = "detectron2://COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x/137849600/model_final_f10217.pkl"
self.cfg.MODEL.WEIGHTS = "/home/wangcong/ros_ws/marker_ws/src/Detectron2_ros/detectron2/configs/COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x/137849600/model_final_f10217.pkl"
```