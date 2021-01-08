# How to plot curve using python



```python
import baselines
from baselines.common import plot_util as pu

log_dir = '/home/cong/ros_ws/openai_ros_ws/src/gym-husky-ur5/gym_husky_ur5/algos/pytorch-a2c-ppo-acktr-gail/plot/husky_ur5/'
# file_path = '/home/cong/ray_results/PPO/PPO_MobileDualUR5HuskyPickAndPlace-v1_1_lr=0.001_2019-12-10_14-10-47r01gnknz/0.monitor.csv'

results = pu.load_results(log_dir)
fig = pu.plot_results(results, average_group=True, split_fn=lambda _: '', shaded_std=False)
```

```python
import pandas as pd
file_path = '/home/cong/ray_results/PPO/PPO_MobileDualUR5HuskyPickAndPlace-v1_1_lr=0.001_2019-12-10_14-10-47r01gnknz/episode_reward_max.csv'
file_path = '/home/cong/ros_ws/openai_ros_ws/src/gym-husky-ur5/gym_husky_ur5/algos/pytorch-a2c-ppo-acktr-gail/plot/husky_ur5/husky_ur5-0/0.monitor.csv'
df = pd.read_csv(file_path)
df.head()
import plotly.express as px
fig = px.line(df, x='t', y='r', title='reward max')
fig.show()
```

```python
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.style.use('seaborn-darkgrid')
import argparse
import csv
import pandas
import os
import sys
import pickle
import numpy as np
from os.path import join

# matplotlib
titlesize = 33
xsize = 30
ysize = 30
ticksize = 25
legendsize = 25
error_region_alpha = 0.25


def smoothed(x, w):
    """Smooth x by averaging over sliding windows of w, assuming sufficient length.
    """
    if len(x) <= w:
        return x
    smooth = []
    for i in range(1, w):
        smooth.append( np.mean(x[0:i]) )
    for i in range(w, len(x)+1):
        smooth.append( np.mean(x[i-w:i]) )
    assert len(x) == len(smooth), "lengths: {}, {}".format(len(x), len(smooth))
    return np.array(smooth)


def _get_stuff_from_monitor(mon):
    """Get stuff from `monitor` log files.

    Monitor files are named `0.envidx.monitor.csv` and have one line for each
    episode that finished in that CPU 'core', with the reward, length (number
    of steps) and the time (in seconds). The lengths are not cumulative, but
    time is cumulative.
    """
    scores = []
    steps  = []
    times  = []
    with open(mon, 'r') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for csv_row in csv_reader:
            # First two lines don't contain interesting stuff.
            if line_count == 0 or line_count == 1:
                line_count += 1
                continue
            scores.append(float(csv_row[0]))
            steps.append(int(csv_row[1]))
            times.append(float(csv_row[2]))
            line_count += 1
    print("finished: {}".format(mon))
    return scores, steps, times


def plot(args):
    """Load monitor curves and the progress csv file. And plot from those.
    """
    nrows, ncols = 1, 2
    fig, ax = plt.subplots(nrows, ncols, squeeze=False, sharey=True, figsize=(11*ncols,7*nrows))
    title = args.title

    # Global statistics across all monitors
    scores_all = []
    steps_all = []
    times_all = []
    total_train_steps = 0
    train_hours = 0

    monitors = sorted(
        [x for x in os.listdir(args.path) if '.csv' in x and '.swp' not in x]
    )
    progfile = join(args.path,'progress.csv')

    # First row, info from all the monitors, i.e., number of CPUs.
    for env_idx,mon in enumerate(monitors):
        monitor_path = join(args.path, mon)
        scores, steps, times = _get_stuff_from_monitor(monitor_path)

        # Now process to see as a function of episodes and training steps, etc.
        num_episodes = len(scores)
        tr_episodes = np.arange(num_episodes)
        tr_steps = np.cumsum(steps)
        tr_times = np.array(times) / 60.0 # get it in minutes

        # Plot for individual monitors.
        envlabel = 'env {}'.format(env_idx)
        sm_10 = smoothed(scores, w=10)
        ax[0,0].plot(tr_steps, sm_10, label=envlabel+'; avg {:.1f} last {:.1f}'.format(
                np.mean(sm_10), sm_10[-1]))
        sm_100 = smoothed(scores, w=100)
        ax[0,1].plot(tr_times, sm_100, label=envlabel+'; avg {:.1f} last {:.1f}'.format(
                np.mean(sm_100), sm_100[-1]))

        # Handle global stuff.
        total_train_steps += tr_steps[-1]
        train_hours = max(train_hours, tr_times[-1] / 60.0)

    # Bells and whistles
    for row in range(nrows):
        for col in range(ncols):
            ax[row,col].set_ylabel("Scores", fontsize=30)
            ax[row,col].tick_params(axis='x', labelsize=25)
            ax[row,col].tick_params(axis='y', labelsize=25)
            leg = ax[row,col].legend(loc="best", ncol=1, prop={'size':25})
            for legobj in leg.legendHandles:
                legobj.set_linewidth(5.0)
    ax[0,0].set_title(title+', Smoothed (w=10)', fontsize=titlesize)
    ax[0,0].set_xlabel("Train Steps (total {})".format(total_train_steps), fontsize=xsize)
    ax[0,1].set_title(title+', Smoothed (w=100)', fontsize=titlesize)
    ax[0,1].set_xlabel("Train Time (in Hours {:.2f})".format(train_hours), fontsize=xsize)
    plt.tight_layout()
    figname = '{}.png'.format(title)
    plt.savefig(figname)
    print("\nJust saved: {}".format(figname))


if __name__ == "__main__":
    pp = argparse.ArgumentParser()
    pp.add_argument('--path', type=str)
    pp.add_argument('--title', type=str)
    args = pp.parse_args()
    plot(args)

```

## OpenAI Baselines

```python
def load_results(root_dir_or_dirs, enable_progress=True, enable_monitor=True, verbose=False):
    '''
    load summaries of runs from a list of directories (including subdirectories)
    Arguments:
    enable_progress: bool - if True, will attempt to load data from progress.csv files (data saved by logger). Default: True
    enable_monitor: bool - if True, will attempt to load data from monitor.csv files (data saved by Monitor environment wrapper). Default: True
    verbose: bool - if True, will print out list of directories from which the data is loaded. Default: False
    Returns:
    List of Result objects with the following fields:
         - dirname - path to the directory data was loaded from
         - metadata - run metadata (such as command-line arguments and anything else in metadata.json file
         - monitor - if enable_monitor is True, this field contains pandas dataframe with loaded monitor.csv file (or aggregate of all *.monitor.csv files in the directory)
         - progress - if enable_progress is True, this field contains pandas dataframe with loaded progress.csv file
    '''
    import re
    if isinstance(root_dir_or_dirs, str):
        rootdirs = [osp.expanduser(root_dir_or_dirs)]
    else:
        rootdirs = [osp.expanduser(d) for d in root_dir_or_dirs]
    allresults = []
    for rootdir in rootdirs:
        assert osp.exists(rootdir), "%s doesn't exist"%rootdir
        for dirname, dirs, files in os.walk(rootdir):
            if '-proc' in dirname:
                files[:] = []
                continue
            monitor_re = re.compile(r'(\d+\.)?(\d+\.)?monitor\.csv')
            if set(['metadata.json', 'monitor.json', 'progress.json', 'progress.csv']).intersection(files) or \
               any([f for f in files if monitor_re.match(f)]):  # also match monitor files like 0.1.monitor.csv
                # used to be uncommented, which means do not go deeper than current directory if any of the data files
                # are found
                # dirs[:] = []
                result = {'dirname' : dirname}
                if "metadata.json" in files:
                    with open(osp.join(dirname, "metadata.json"), "r") as fh:
                        result['metadata'] = json.load(fh)
                progjson = osp.join(dirname, "progress.json")
                progcsv = osp.join(dirname, "progress.csv")
                if enable_progress:
                    if osp.exists(progjson):
                        result['progress'] = pandas.DataFrame(read_json(progjson))
                    elif osp.exists(progcsv):
                        try:
                            result['progress'] = read_csv(progcsv)
                        except pandas.errors.EmptyDataError:
                            print('skipping progress file in ', dirname, 'empty data')
                    else:
                        if verbose: print('skipping %s: no progress file'%dirname)

                if enable_monitor:
                    try:
                        result['monitor'] = pandas.DataFrame(monitor.load_results(dirname))
                    except monitor.LoadMonitorResultsError:
                        print('skipping %s: no monitor files'%dirname)
                    except Exception as e:
                        print('exception loading monitor file in %s: %s'%(dirname, e))

                if result.get('monitor') is not None or result.get('progress') is not None:
                    allresults.append(Result(**result))
                    if verbose:
                        print('successfully loaded %s'%dirname)

    if verbose: print('loaded %i results'%len(allresults))
    return allresults

COLORS = ['blue', 'green', 'red', 'cyan', 'magenta', 'yellow', 'black', 'purple', 'pink',
        'brown', 'orange', 'teal',  'lightblue', 'lime', 'lavender', 'turquoise',
        'darkgreen', 'tan', 'salmon', 'gold',  'darkred', 'darkblue']

def smooth(y, radius, mode='two_sided', valid_only=False):
    '''
    Smooth signal y, where radius is determines the size of the window
    mode='twosided':
        average over the window [max(index - radius, 0), min(index + radius, len(y)-1)]
    mode='causal':
        average over the window [max(index - radius, 0), index]
    valid_only: put nan in entries where the full-sized window is not available
    '''
    assert mode in ('two_sided', 'causal')
    if len(y) < 2*radius+1:
        return np.ones_like(y) * y.mean()
    elif mode == 'two_sided':
        convkernel = np.ones(2 * radius+1)
        out = np.convolve(y, convkernel,mode='same') / np.convolve(np.ones_like(y), convkernel, mode='same')
        if valid_only:
            out[:radius] = out[-radius:] = np.nan
    elif mode == 'causal':
        convkernel = np.ones(radius)
        out = np.convolve(y, convkernel,mode='full') / np.convolve(np.ones_like(y), convkernel, mode='full')
        out = out[:-radius+1]
        if valid_only:
            out[:radius] = np.nan
    return out

def default_xy_fn(r):
    x = np.cumsum(r.monitor.l)
    y = smooth(r.monitor.r, radius=10)
    return x,y

def default_split_fn(r):
    import re
    # match name between slash and -<digits> at the end of the string
    # (slash in the beginning or -<digits> in the end or either may be missing)
    match = re.search(r'[^/-]+(?=(-\d+)?\Z)', r.dirname)
    if match:
        return match.group(0)

def plot_results(
    allresults, *,
    xy_fn=default_xy_fn,
    split_fn=default_split_fn,
    group_fn=default_split_fn,
    average_group=False,
    shaded_std=True,
    shaded_err=True,
    figsize=None,
    legend_outside=False,
    resample=0,
    smooth_step=1.0,
    tiling='vertical',
    xlabel=None,
    ylabel=None
):
    '''
    Plot multiple Results objects
    xy_fn: function Result -> x,y           - function that converts results objects into tuple of x and y values.
                                              By default, x is cumsum of episode lengths, and y is episode rewards
    split_fn: function Result -> hashable   - function that converts results objects into keys to split curves into sub-panels by.
                                              That is, the results r for which split_fn(r) is different will be put on different sub-panels.
                                              By default, the portion of r.dirname between last / and -<digits> is returned. The sub-panels are
                                              stacked vertically in the figure.
    group_fn: function Result -> hashable   - function that converts results objects into keys to group curves by.
                                              That is, the results r for which group_fn(r) is the same will be put into the same group.
                                              Curves in the same group have the same color (if average_group is False), or averaged over
                                              (if average_group is True). The default value is the same as default value for split_fn
    average_group: bool                     - if True, will average the curves in the same group and plot the mean. Enables resampling
                                              (if resample = 0, will use 512 steps)
    shaded_std: bool                        - if True (default), the shaded region corresponding to standard deviation of the group of curves will be
                                              shown (only applicable if average_group = True)
    shaded_err: bool                        - if True (default), the shaded region corresponding to error in mean estimate of the group of curves
                                              (that is, standard deviation divided by square root of number of curves) will be
                                              shown (only applicable if average_group = True)
    figsize: tuple or None                  - size of the resulting figure (including sub-panels). By default, width is 6 and height is 6 times number of
                                              sub-panels.
    legend_outside: bool                    - if True, will place the legend outside of the sub-panels.
    resample: int                           - if not zero, size of the uniform grid in x direction to resample onto. Resampling is performed via symmetric
                                              EMA smoothing (see the docstring for symmetric_ema).
                                              Default is zero (no resampling). Note that if average_group is True, resampling is necessary; in that case, default
                                              value is 512.
    smooth_step: float                      - when resampling (i.e. when resample > 0 or average_group is True), use this EMA decay parameter (in units of the new grid step).
                                              See docstrings for decay_steps in symmetric_ema or one_sided_ema functions.
    '''

    if split_fn is None: split_fn = lambda _ : ''
    if group_fn is None: group_fn = lambda _ : ''
    sk2r = defaultdict(list) # splitkey2results
    for result in allresults:
        splitkey = split_fn(result)
        sk2r[splitkey].append(result)
    assert len(sk2r) > 0
    assert isinstance(resample, int), "0: don't resample. <integer>: that many samples"
    if tiling == 'vertical' or tiling is None:
        nrows = len(sk2r)
        ncols = 1
    elif tiling == 'horizontal':
        ncols = len(sk2r)
        nrows = 1
    elif tiling == 'symmetric':
        import math
        N = len(sk2r)
        largest_divisor = 1
        for i in range(1, int(math.sqrt(N))+1):
            if N % i == 0:
                largest_divisor = i
        ncols = largest_divisor
        nrows = N // ncols
    figsize = figsize or (6 * ncols, 6 * nrows)

    f, axarr = plt.subplots(nrows, ncols, sharex=False, squeeze=False, figsize=figsize)

    groups = list(set(group_fn(result) for result in allresults))

    default_samples = 512
    if average_group:
        resample = resample or default_samples

    for (isplit, sk) in enumerate(sorted(sk2r.keys())):
        g2l = {}
        g2c = defaultdict(int)
        sresults = sk2r[sk]
        gresults = defaultdict(list)
        idx_row = isplit // ncols
        idx_col = isplit % ncols
        ax = axarr[idx_row][idx_col]
        for result in sresults:
            group = group_fn(result)
            g2c[group] += 1
            x, y = xy_fn(result)
            if x is None: x = np.arange(len(y))
            x, y = map(np.asarray, (x, y))
            if average_group:
                gresults[group].append((x,y))
            else:
                if resample:
                    x, y, counts = symmetric_ema(x, y, x[0], x[-1], resample, decay_steps=smooth_step)
                l, = ax.plot(x, y, color=COLORS[groups.index(group) % len(COLORS)])
                g2l[group] = l
        if average_group:
            for group in sorted(groups):
                xys = gresults[group]
                if not any(xys):
                    continue
                color = COLORS[groups.index(group) % len(COLORS)]
                origxs = [xy[0] for xy in xys]
                minxlen = min(map(len, origxs))
                def allequal(qs):
                    return all((q==qs[0]).all() for q in qs[1:])
                if resample:
                    low  = max(x[0] for x in origxs)
                    high = min(x[-1] for x in origxs)
                    usex = np.linspace(low, high, resample)
                    ys = []
                    for (x, y) in xys:
                        ys.append(symmetric_ema(x, y, low, high, resample, decay_steps=smooth_step)[1])
                else:
                    assert allequal([x[:minxlen] for x in origxs]),\
                        'If you want to average unevenly sampled data, set resample=<number of samples you want>'
                    usex = origxs[0]
                    ys = [xy[1][:minxlen] for xy in xys]
                ymean = np.mean(ys, axis=0)
                ystd = np.std(ys, axis=0)
                ystderr = ystd / np.sqrt(len(ys))
                l, = axarr[idx_row][idx_col].plot(usex, ymean, color=color)
                g2l[group] = l
                if shaded_err:
                    ax.fill_between(usex, ymean - ystderr, ymean + ystderr, color=color, alpha=.4)
                if shaded_std:
                    ax.fill_between(usex, ymean - ystd,    ymean + ystd,    color=color, alpha=.2)


        # https://matplotlib.org/users/legend_guide.html
        plt.tight_layout()
        if any(g2l.keys()):
            ax.legend(
                g2l.values(),
                ['%s (%i)'%(g, g2c[g]) for g in g2l] if average_group else g2l.keys(),
                loc=2 if legend_outside else None,
                bbox_to_anchor=(1,1) if legend_outside else None)
        ax.set_title(sk)
        # add xlabels, but only to the bottom row
        if xlabel is not None:
            for ax in axarr[-1]:
                plt.sca(ax)
                plt.xlabel(xlabel)
        # add ylabels, but only to left column
        if ylabel is not None:
            for ax in axarr[:,0]:
                plt.sca(ax)
                plt.ylabel(ylabel)

    return f, axarr
```