# Dex-Net

```
virtualenv --python=python3.5 envname
```

Dex-Net as a Service - Task: Compute and Display Robust Robot Grasps for Your Own Objects:
[https://dex-net.app/](https://dex-net.app/)


- [easy-dexnet](https://github.com/LaiQE/easy-dexnet)
- [dexnet-ar](https://github.com/harryzhangOG/dexnet-ar)
- [dexnet_robosuite](https://github.com/iosmichael/dexnet_robosuite)
- [dexnet_rrt_planner_surreal_robosuite](https://github.com/v1viswan/dexnet_rrt_planner_surreal_robosuite)
- [dexnet_updates](https://github.com/schmirob/dexnet_updates)

- [Dex-Net installation procedure on system installed with Anaconda and ROS
](https://aimlrobots.blogspot.com/2017/11/dexnet-installation-anaconda.html)


- [Dex-Net Installation Instructions](https://docs.google.com/document/d/1YImq1cBTy9E1n1On6-00gueDT4hfmYJK4uOcxZIzPoY/edit)


https://blog.csdn.net/yldmkx/article/details/101789379?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-4.channel_param&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-4.channel_param

1. 提示pip需要升级,但执行pip install --upgrade pip后并没有用,并且pip都找不到
解决: 先执行sudo gedit /usr/bin/pip 打开文件,将 from pip import main 改为 from pip._internal import main. 
————————————————
版权声明：本文为CSDN博主「yldmkx」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/yldmkx/article/details/101789379

https://github.com/Toblerity/rtree/issues/64

```
sudo apt install libspatialindex-dev python-rtree
```            

https://www.cnblogs.com/USTBlxq/p/8806395.html

```
pip install  meshrender
conda install -c menpo mayavi=4.5.0
```

## Dex-Net Codebase

The Dex-Net codebase is organized in four main libraries: **database**, **grasping**, **learning**, and **visualization**.

### Database



### Grasping


### Learning


### Visualization


## GQ-CNN

