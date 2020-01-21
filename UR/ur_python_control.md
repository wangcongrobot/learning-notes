# UR python control

- [通过socket通讯控制ur机械臂](https://blog.csdn.net/qq_33859895/article/details/89228388)



### TCP

You need to set the TCP w.r.t arm. In our situation, we set `z=0.15m`. 


```
Unit: radian
SE(3) in numpy.array format
[[ 0.99951965 -0.03083946  0.00306625  0.10580721]
 [ 0.00562179  0.08312386 -0.99652337 -0.43290969]
 [ 0.03047736  0.99606192  0.0832573   0.57717163]
 [ 0.          0.          0.          1.        ]]
SE(3) in ROS Pose format
position: 
  x: 0.10580721319043115
  y: -0.4329096909939745
  z: 0.5771716288235655
orientation: 
  x: 0.6769672233669407
  y: -0.00931273784222284
  z: 0.012387459881407105
  w: 0.7358499862325357
```

### UR communication data type

https://blog.csdn.net/hangl_ciom/article/details/97612246

