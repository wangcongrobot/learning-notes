# RLLib with Ray

## install rllib vitualenv

```bash
virtualenv env --python=python3.7
souurce env/bin/activate
pip install tensorflow==1.14.0 tensorboard==1.14.0 tensorflow-probability==0.7.0 ray[rllib] requests numpy==1.16.0 mujoco-py==2.0.2.2 psutil 
pip install your_package

```
### install ros env
```bash
pip install catkin_pkg rospkg empy
```

## install latest version rllib

https://ray.readthedocs.io/en/latest/installation.html

0.8.0.dev6
```bash
wget https://s3-us-west-2.amazonaws.com/ray-wheels/latest/ray-0.8.0.dev6-cp37-cp37m-manylinux1_x86_64.whl
pip install -U [link to wheel]
pip install tensorflow tensorboard tensorflow-probability requests numpy mujoco-py==2.0.2.2 psutil
pip gym

```

## train

train a agent with rllib:
```bash
rllib train --run PPO --env DualUR5HuskyPickAndPlace-v1 --checkpoint-freq 20 --config '{"num_workers": 20} --restore /path/to/your/model/checkpoint
```

evaluate your model:
```bash
rllib rollout /path/to/your/model/checkpoint --run PPO
```

```bash
rllib train --run PPO --checkpoint-freq 20 --env DualUR5HuskyPickAndPlace-v1 --config '{"num_workers":25, "lambda":0.95, "gamma":0.998, "kl_coeff":1.0, "clip_param":0.2, "observation_filter":"MeanStdFilter", "batch_mode":"complete_episodes", "lr": 0.0005}'
```


```bash
(py35) cong@eclipse:~/ros_ws/openai_ros_ws$ pip list
Package                         Version            Location                                                                                        
------------------------------- ------------------ ------------------------------------------------------------------------------------------------
a2c-ppo-acktr                   0.0.1              /home/cong/ros_ws/openai_ros_ws/src/gym-husky-ur5/gym_husky_ur5/algos/pytorch-a2c-ppo-acktr-gail
absl-py                         0.8.1              
actionlib                       1.11.13            
angles                          1.9.11             
astor                           0.8.0              
atari-py                        0.2.6              
atomicwrites                    1.3.0              
attrs                           19.3.0             
autograd                        1.3                
backcall                        0.1.0              
base-local-planner              1.14.4             
baselines                       0.1.6              /home/cong/ros_ws/openai_ros_ws/py35/lib/python3.5/site-packages/baselines-0.1.6-py3.5.egg      
beautifulsoup4                  4.8.1              
bleach                          3.1.0              
bondpy                          1.8.3              
boto3                           1.10.28            
botocore                        1.13.28            
camera-calibration              1.12.23            
camera-calibration-parsers      1.11.13            
catkin                          0.7.18             
catkin-pkg                      0.4.14             
certifi                         2019.9.11          
cffi                            1.13.2             
chardet                         3.0.4              
Click                           7.0                
cloudpickle                     1.2.2              
cma                             2.7.0              
colorama                        0.4.1              
contextlib2                     0.5.5              
control                         0.8.2              
controller-manager              0.13.3             
controller-manager-msgs         0.13.3             
controller-manager-tests        0.13.3             
cv-bridge                       1.12.8             
cvxopt                          1.2.3              
cvxpy                           1.0.25             
cycler                          0.10.0             
Cython                          0.29.14            
decorator                       4.4.1              
defusedxml                      0.6.0              
diagnostic-analysis             1.9.3              
diagnostic-common-diagnostics   1.9.3              
diagnostic-updater              1.9.3              
dill                            0.3.1.1            
docutils                        0.15.2             
dynamic-reconfigure             1.5.50             
dynamixel-sdk                   3.7.21             
ecos                            2.0.7.post1        
empy                            3.3.4              
entrypoints                     0.3                
filelock                        3.0.12             
Flask                           1.1.1              
Flask-SQLAlchemy                2.4.1              
funcsigs                        1.0.2              
future                          0.18.2             
gast                            0.3.2              
gazebo-plugins                  2.5.19             
gazebo-ros                      2.5.19             
gencpp                          0.6.0              
geneus                          2.2.6              
genlisp                         0.4.16             
genmsg                          0.5.11             
gennodejs                       2.0.1              
genpy                           0.6.7              
git-python                      1.0.3              
gitdb2                          2.0.6              
GitPython                       3.0.5              
glfw                            1.8.4              
google-pasta                    0.1.8              
GPy                             1.9.9              
GPyOpt                          1.2.5              
gpytorch                        0.1.0rc4           
grpcio                          1.25.0             
gTTS                            2.0.4              
gTTS-token                      1.1.3              
gym                             0.15.4             
gym-husky-ur5                   0.0.1              /home/cong/ros_ws/openai_ros_ws/src/gym-husky-ur5                                               
h5py                            2.10.0             
idna                            2.8                
image-geometry                  1.12.8             
imageio                         2.6.1              
importlib-metadata              0.23               
inputs                          0.5                
interactive-markers             1.11.4             
ipykernel                       5.1.3              
ipython                         7.9.0              
ipython-genutils                0.2.0              
ipywidgets                      7.5.1              
itsdangerous                    1.1.0              
jedi                            0.15.1             
Jinja2                          2.10.3             
jmespath                        0.9.4              
joblib                          0.14.0             
jsonschema                      3.2.0              
jupyter                         1.0.0              
jupyter-client                  5.3.4              
jupyter-console                 6.0.0              
jupyter-core                    4.6.1              
kdl-parser-py                   1.12.11            
Keras                           2.3.1              
Keras-Applications              1.0.8              
Keras-Preprocessing             1.1.0              
kiwisolver                      1.1.0              
laser-geometry                  1.6.4              
llvmlite                        0.30.0             
lockfile                        0.12.2             
lxml                            4.4.2              
lz4                             2.2.1              
Markdown                        3.1.1              
MarkupSafe                      1.1.1              
master-discovery-fkie           0.8.12             
master-sync-fkie                0.8.12             
matplotlib                      3.0.3              
message-filters                 1.12.14            
mistune                         0.8.4              
more-itertools                  7.2.0              
moveit-commander                0.9.17             
moveit-python                   0.3.1              
moveit-ros-planning-interface   0.9.17             
moveit-ros-visualization        0.9.17             
mpi4py                          3.0.3              
mpmath                          1.1.0              
mujoco-py                       2.0.2.5            
multiprocess                    0.70.9             
nbconvert                       5.6.1              
nbformat                        4.4.0              
neat-python                     0.92               
netifaces                       0.10.9             
nlopt                           2.4.2.post2        
nmea-navsat-driver              0.5.1              
notebook                        6.0.2              
numba                           0.46.0             
numpy                           1.16.0             
numpy-quaternion                2019.10.3.10.26.21 
opencv-python                   4.1.1.26           
opencv-python-headless          4.1.2.30           
osqp                            0.6.1              
packaging                       19.2               
pandas                          0.24.2             
pandocfilters                   1.4.2              
paramz                          0.9.5              
parso                           0.5.1              
pathlib2                        2.3.5              
pathos                          0.2.5              
pexpect                         4.7.0              
pickleshare                     0.7.5              
Pillow                          6.2.1              
pip                             19.3.1             
pluggy                          0.13.0             
pluginlib                       1.11.3             
pox                             0.2.7              
ppft                            1.6.6.1            
progressbar2                    3.47.0             
prometheus-client               0.7.1              
prompt-toolkit                  2.0.10             
protobuf                        3.10.0             
psutil                          5.6.5              
ptyprocess                      0.6.0              
py                              1.8.0              
py-spy                          0.3.0              
pyassimp                        4.1.4              
pybullet                        2.5.8              
pycparser                       2.19               
pydot                           1.4.1              
pyglet                          1.3.2              
Pygments                        2.5.2              
pyparsing                       2.4.5              
pyrobolearn                     0.1.0              /home/cong/ros_ws/openai_ros_ws/src/pyrobolearn                                                 
pyrsistent                      0.15.6             
pytest                          5.2.4              
python-dateutil                 2.8.0              
python-qt-binding               0.3.4              
python-utils                    2.3.0              
pytz                            2019.3             
PyYAML                          5.1.2              
pyzmq                           18.1.1             
qpsolvers                       1.0.7              
qt-dotgraph                     0.3.11             
qt-gui                          0.3.11             
qt-gui-cpp                      0.3.11             
qt-gui-py-common                0.3.11             
qtconsole                       4.6.0              
quadprog                        0.1.7              
ray                             0.7.6              
redis                           3.3.11             
requests                        2.22.0             
resource-retriever              1.12.4             
robot-upstart                   0.3.0              
rosapi                          0.11.3             
rosbag                          1.12.14            
rosboost-cfg                    1.14.6             
rosbridge-library               0.11.3             
rosbridge-server                0.11.3             
rosclean                        1.14.6             
roscreate                       1.14.6             
rosgraph                        1.12.14            
roslaunch                       1.12.14            
roslib                          1.14.6             
roslint                         0.11.0             
roslz4                          1.12.14            
rosmake                         1.14.6             
rosmaster                       1.12.14            
rosmsg                          1.12.14            
rosnode                         1.12.14            
rosparam                        1.12.14            
rospkg                          1.1.10             
rospy                           1.12.14            
rospy-message-converter         0.5.0              
rosservice                      1.12.14            
rostest                         1.12.14            
rostopic                        1.12.14            
rosunit                         1.14.6             
roswtf                          1.12.14            
rqt-action                      0.4.9              
rqt-bag                         0.4.12             
rqt-bag-plugins                 0.4.12             
rqt-console                     0.4.8              
rqt-controller-manager          0.13.3             
rqt-dep                         0.4.9              
rqt-graph                       0.4.9              
rqt-gui                         0.5.0              
rqt-gui-py                      0.5.0              
rqt-image-view                  0.4.13             
rqt-joint-trajectory-controller 0.13.5             
rqt-launch                      0.4.8              
rqt-logger-level                0.4.8              
rqt-moveit                      0.5.7              
rqt-msg                         0.4.8              
rqt-nav-view                    0.5.7              
rqt-plot                        0.4.8              
rqt-pose-view                   0.5.8              
rqt-publisher                   0.4.8              
rqt-py-common                   0.5.0              
rqt-py-console                  0.4.8              
rqt-reconfigure                 0.5.0              
rqt-robot-dashboard             0.5.7              
rqt-robot-monitor               0.5.8              
rqt-robot-steering              0.5.9              
rqt-runtime-monitor             0.5.7              
rqt-rviz                        0.5.10             
rqt-service-caller              0.4.8              
rqt-shell                       0.4.9              
rqt-srv                         0.4.8              
rqt-tf-tree                     0.6.0              
rqt-top                         0.4.8              
rqt-topic                       0.4.10             
rqt-web                         0.4.8              
rviz                            1.12.17            
s3transfer                      0.2.1              
schema                          0.7.1              
scikit-learn                    0.21.3             
scipy                           1.3.2              
scs                             2.1.1.post2        
Send2Trash                      1.5.0              
sensor-msgs                     1.12.7             
setproctitle                    1.1.10             
setuptools                      41.6.0             
six                             1.13.0             
sklearn                         0.0                
smach                           2.0.1              
smach-ros                       2.0.1              
smclib                          1.8.3              
smmap2                          2.0.5              
soupsieve                       1.9.5              
SQLAlchemy                      1.3.11             
srdfdom                         0.4.2              
stable-baselines                2.8.0              
sympy                           1.4                
tensorboard                     1.14.0             
tensorflow                      1.14.0             
tensorflow-estimator            1.14.0             
termcolor                       1.1.0              
terminado                       0.8.3              
testpath                        0.4.4              
tf                              1.11.9             
tf-conversions                  1.11.9             
tf2-geometry-msgs               0.5.20             
tf2-kdl                         0.5.20             
tf2-py                          0.5.20             
tf2-ros                         0.5.20             
tf2-sensor-msgs                 0.5.20             
topic-tools                     1.12.14            
torch                           1.3.0              
torchvision                     0.4.1              
tornado                         6.0.3              
tqdm                            4.39.0             
trac-ik-python                  1.5.0              
traitlets                       4.3.3              
trimesh                         3.4.4              
ur-driver                       1.2.5              
ur-kinematics                   1.2.5              
urdf-parser-py                  0.0.3              
urdfdom-py                      0.3.3              
urllib3                         1.25.7             
wcwidth                         0.1.7              
webencodings                    0.5.1              
Werkzeug                        0.16.0             
wheel                           0.33.6             
widgetsnbextension              3.5.1              
wiimote                         1.13.0             
wrapt                           1.11.2             
xacro                           1.11.3             
zipp                            0.6.0              
zmq                             0.0.0  
```