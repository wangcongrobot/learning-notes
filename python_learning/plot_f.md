# Python Plotting

```python
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import matplotlib
matplotlib.use('TkAgg') # Can change to 'Agg' for non-interactive mode
import csv
import os
import time
import json
import glob
from os.path import join
import seaborn as sns
sns.set()

COLORS = ['blue', 'green', 'red', 'cyan', 'magenta', 'yellow', 'black', 'purple', 'pink',
        'brown', 'orange', 'teal', 'coral', 'lightblue', 'lime', 'lavender', 'turquoise',
        'darkgreen', 'tan', 'salmon', 'gold', 'darkred', 'darkblue']

log_dir = '/home/cong/ros_ws/openai_ros_ws/gym/gym/envs/robotics/plot/'
file_path1 = '/home/cong/ros_ws/openai_ros_ws/gym/gym/envs/robotics/plot/run-TD3_ray_tune_episode_reward_max.csv'
file_path2 = '/home/cong/ros_ws/openai_ros_ws/gym/gym/envs/robotics/plot/run-PPO_ray_tune_episode_reward_max.csv'

def smoothed(x, w):
    """ Smoothed x by averaging over sliding windows of w, assuming sufficient length."""
    if len(x) <= w:
        return x
    smooth = []
    for i in range(1, w):
        smooth.append(np.mean(x[0:i]))
    for i in range(w, len(x)+1):
        smooth.append(np.mean(x[i-w:i]))
    assert len(x) == len(smooth), "length: {}, {}".format(lend(x), len(smooth))
    return np.array(smooth)

def get_stuff_from_csv(file):
    scores = []
    steps = []
    times = []
    with open(file, 'r') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for csv_row in csv_reader:
            # First lines don't contain interesting stuff.
            if line_count == 0:
                line_count += 1
                continue
            times.append(float(csv_row[0]))
            steps.append(float(csv_row[1]))
            scores.append(float(csv_row[2]))
            line_count += 1
    print("finished: {}".format(file))
    return scores, steps, times

def my_load_results(dir):
    csv_files = glob.glob(join(dir) + "/*.csv") 
    if not csv_files:
        print("No .csv files")
        return 0
    dfs = []
    for fname in csv_files:
        df = pd.read_csv(fname)
        dfs.append(df)
    df = pd.concat(dfs)
    df.sort_values('Step', inplace=True)
    df.reset_index(inplace=True)
    # df.head()
    # df.tail()
    # df.columns
    # df.index
    # df.sort_values(['Step'])
    return df
    
def load_csv(path):
    # csv_files = sorted([x for x in os.listdir(path) if '.csv' in x])
    csv_files = glob.glob(path + "/*.csv")
    return csv_files

def plot(path, title, smooth_num,):
    nrows, ncols = 1, 2
    fig = plt.figure()
    files = load_csv(path)
    for idx, name in enumerate(files):
        scores, steps, times = get_stuff_from_csv(name)
        scores_smoothed = smoothed(scores, smooth_num)
        env_label = 'env {}'.format(idx)
        plt.plot(steps, scores_smoothed, label=env_label)
    plt.title(title)
    plt.xlabel('steps')
    plt.ylabel('reward')
    plt.legend()
    plt.show()

```