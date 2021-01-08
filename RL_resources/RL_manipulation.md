# RL in Robotic Manipulation tasks Resources

This blog summarizes some reinforcement learning in robotic manipulation tasks.

**Contents:**
- [RL in Robotic Manipulation tasks Resources](#rl-in-robotic-manipulation-tasks-resources)
  - [RL Manipulation Benchmark](#rl-manipulation-benchmark)
    - [FetchRobot](#fetchrobot)
    - [robotsuite](#robotsuite)
    - [RLBench](#rlbench)
  - [Manipulation Tasks](#manipulation-tasks)
    - [Pick and Place Task](#pick-and-place-task)
      - [Task Analysis](#task-analysis)
      - [Reward Analysis](#reward-analysis)
    - [Pin in hole](#pin-in-hole)
    - [rearrangement task](#rearrangement-task)


## RL Manipulation Benchmark

### [FetchRobot]()



### [robotsuite](https://github.com/StanfordVL/robosuite)

Surreal Robitics Suite: standardized and accessible robot manipulation benchmark in physics simulation. http://surreal.stanford.edu

- standardized tasks: a set of single-arm and bimanual manipulation tasks of large diversity and varying complexity.
- procedural generation: modularized APIs for programmatically creating new scenes and new tasks as a combinations of robot models, arenas, and parameterized 3D objects;
- controller modes: a selection of controller types to command the robots, such as joint velocity control, inverse kinematics control, and 3D motion devices for teleoperation;
- multi-modal sensors: heterogeneous types of sensory signals, including low-level physical states, RGB cameras, depth maps, and proprioception;
- human demonstrations: utilities for collecting human demonstrations, replaying demonstration datasets, and leveraging demonstration data for learning.

### [RLBench](https://github.com/stepjam/RLBench)

RLBench is an ambitious large-scale benchmark and learning environment featuring 100 unique, hand-design tasks, tailored to facilitate research in a number of vision-guided manipulation research areas, including: reinforcement learning, imitation learning, multi-task learning, geometric computer vision, and in particular, few-shot learning.RLBench: The Robot Learning Benchmark & Learning Environment

https://arxiv.org/abs/1909.12271 RLBench: The Robot Learning Benchmark & Learning Environment

## Manipulation Tasks

### Pick and Place Task

#### Task Analysis

#### Reward Analysis


### Pin in hole

### rearrangement task

- [rearrangement](https://github.com/huiwenzhang/gym-rearrangement)

rearrangement task based on gym fetch push task.


- [Deep Dynamics Models for Learning Dexterous Manipulation](https://github.com/google-research/pddm)

![1](https://github.com/google-research/pddm/blob/master/pddm/gifs/dclaw_gif.gif)
![2](https://github.com/google-research/pddm/blob/master/pddm/gifs/cube_gif.gif)
![3](https://github.com/google-research/pddm/blob/master/pddm/gifs/handwriting_gif.gif)
![4](https://github.com/google-research/pddm/blob/master/pddm/gifs/baoding_gif.gif)

https://sites.google.com/view/pddm/

Using mujoco to simulate learning dexterous manipulation with shadow hand

```
@INPROCEEDINGS{PDDM, 
     AUTHOR = {Anusha Nagabandi AND Kurt Konoglie AND Sergey Levine AND Vikash Kumar}, 
     TITLE = "{Deep Dynamics Models for Learning  Dexterous Manipulation}", 
     BOOKTITLE = {Conference on Robot Learning (CoRL)}, 
     YEAR = {2019}, }
```

- [google-robotics-research](https://github.com/google-research/tensor2robot)

**Tensor2Robot**

This repository contains distributed machine learning and reinforcement learning infrastructure.

It is used internally at Alphabet, and open-sourced with the intention of making research at Robotics @ Google more reproducible for the broader robotics and computer vision communities.

Projects and Publications Using Tensor2Robot
1. QT-Opt
2. Grasp2Vec
3. Watch, Try, Learn
4. More coming soon.


- [google-robel](https://github.com/google-research/robel)

ROBEL: Robotics Benchmarks for Learning with low-cost robots http://roboticsbenchmarks.org

![robel](https://github.com/google-research/robel/blob/master/media/cover.png)

- [Amazon-racer-rl](https://github.com/awslabs/amazon-sagemaker-examples)



[AWS DeepRacer](https://aws.amazon.com/cn/deepracer/?n=sn&p=sm)