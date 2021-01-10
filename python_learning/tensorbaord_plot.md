# How to plot the data of tensorboard

[读取tensorboard日志数据](https://blog.csdn.net/nima1994/article/details/82844988)

https://github.com/pipatth/robot-rl-cscie89/blob/master/Reward_Analysis_DDPG_HER_Tuning.ipynb

In the jupyter, some package cannot be import, install jypyter in this virtual environment. You may use the jupyter in another virtual environment so that you cannot find the package.

[Seaborn绘图](https://blog.csdn.net/suzyu12345/article/details/69029106)
[python seaborn画图](https://blog.csdn.net/suzyu12345/article/details/69029106)
[读取tensorboard日志数据](https://blog.csdn.net/nima1994/article/details/82844988)

[How do I create a multiline plot using seaborn?](https://stackoverflow.com/questions/52308749/how-do-i-create-a-multiline-plot-using-seaborn) with [pandas.melt](https://pandas.pydata.org/pandas-docs/stable/reference/api/pandas.melt.html)

```python
%matplotlib inline
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import seaborn as sns
from datetime import datetime, timedelta
from tensorboard.backend.event_processing import event_accumulator

# seaborn plotting style
sns.set()

log_dir = '/home/cong/ros_ws/openai_ros_ws/gym/gym/envs/robotics/plot/PPO/'
ppo1 = 'PPO_HuskyPickAndPlace-v1_0_2019-12-11_13-49-56v1cgtsug'
ppo2 = 'PPO_HuskyPickAndPlace-v1_0_2019-12-12_03-37-31n0pvl54k'

reward_max = 'ray/tune/episode_reward_max'
success_rate_max = 'ray/tune/custom_metrics/success_rate_max'
success_rate_mean = 'ray/tune/custom_metrics/success_rate_mean'
reward_mean = 'ray/tune/episode_reward_mean'

def get_item_from_tb(item_name, col_rename, log_dir, smooth):
    ea = event_accumulator.EventAccumulator(log_dir)
    ea.Reload()
    series = ea.scalars.Items(item_name)
    values = [s.value for s in series]
    steps = [s.step for s in series]
    df = pd.DataFrame(values, columns=col_rename)
    df['timesteps'] = steps
    if smooth:
        df[col_rename] = df[col_rename].rolling(window=100, min_periods=2).mean()
    return df

def same_algo_concat(train_list):
    dfs = []
    num = len(train_list)
    if num < 1:
        print('No data input')
        return 0
    elif num == 1:
        return train_list
    elif num >= 2:
        df = train_list[0]
        for i in range(num-1):
            df = pd.concat([df, train_list[i+1]], ignore_index=True)
        return df

def diff_algo_concat():
    pass

def plot(data, xlabel, ylabel, title, label):
    sns.lineplot(x=xlabel, y=ylabel, data=data, label=label)

train_list = []
df1 = get_item_from_tb(reward_max, ['reward_max'], log_dir+ppo1, True)
df2 = get_item_from_tb(reward_max, ['reward_max'], log_dir+ppo2, True)
train_list.append(df1)
train_list.append(df2)

df = same_algo_concat(train_list)

plot(df, 'timesteps', 'reward_max', "reward_max", 'ppo')

```