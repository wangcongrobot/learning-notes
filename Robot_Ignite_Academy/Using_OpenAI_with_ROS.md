# Using OpenAI with ROS

## How to use OpenAI Baselines with ROS

OpenAI baselines only works for Python 3.5 or higher. This can be a little bit problematic since ROS only supports (completely) Python 2.7. This means that our whole system is based on Python 2.7, too. So we will use a virtual environment. With this virtual environment, we'll be able to use Python 3.5 for baselines, without needing to change anything in our system.

1. Set up the virtual environment

- download code
Create a new ros workspace python3_ws/src and download the baselines code.

```bash
git clone https://github.com/openai/baselines.git
```

- create the virtual environment

Inside the baselines root directory, and create a virtual environment to set up everything we need for using the OpenAI baselines. 
```
virtualenv venv --python=python3
source venv/bin/activate
or
deactiave
```

```bash
pip install tensorflow pyyaml rospkg catkin_pkg exception defusedxml empy numpy
```

## Training a Fetch Robot

