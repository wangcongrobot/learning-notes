# Distrubued Reinforcement Learning

Distributed Deep RL

## catalyst-rl

- [rl_gym](https://github.com/catalyst-team/catalyst/tree/master/examples/rl_gym)

- [catalyst-rl-framework](https://github.com/catalyst-team/catalyst-rl-framework)

Catalyst.RL: A Distributed Framework for Reproducible RL Research https://arxiv.org/abs/1903.00027

- []()


## Berkeley Ray

- [ray](https://github.com/ray-project/ray)

- [rllib]()

https://arxiv.org/pdf/1712.09381.pdf

- [sonic-on-ray](https://github.com/openai/sonic-on-ray)

- []()


### How to use the ray with gym environment


## GPU-Accelerated Robotic Simulation for Distributed Reinforcement Learning

https://sites.google.com/view/accelerated-gpu-simulation/home

## RLlib: Scalable Reinforcement Learning

RLlib is an open-source library for reinforcement learning that offers both high scalability and a unified API for a variety of applications. RLlib natively supports TensorFlow, TensorFlow Eager, and PyTorch, but most of its internals are framework agnostic.

![RLlib framework](https://ray.readthedocs.io/en/latest/_images/rllib-stack.svg)



### bug

install tf-nightly-gpu


https://github.com/flow-project/flow/blob/master/tutorials/tutorial03_rllib.ipynb

Run RL experiments in Ray
1. Import

First, we must import modules required to run experiments in Ray. Ray-relatd imports are required: the PPO algorithm agent, ray.tune's experiment runner, and environment helper methods register_env and make_create_env.
```python
import ray
try:
    from ray.rllib.agents.agent import get_agent_class
except Import Error:
    from ray.rllib.agents.registry import get_agent_class
from ray.tune import run_experiments
from ray.tune.registry import register_env
```
2. Initializing Ray
Here, we initialize Ray and experiment-based constant variables specifying specifying parallelism in the experiment as well as experiment batch size in term s of number of rollouts. redirect_output sends stdout and stderr for non-worker processes to files if True.
```python
# number of parallel worker
N_CPUS = 2
# number of rollouts per training iteration
N_ROLLOUTS = 1

ray.init(redirect_output=True, num_cpus=N_CPUS)
```
3. Configuration and Setup
Here, we copy and modify the default configuration for the PPO algorithm. The agent has the number of parallel workers specified, a batch size corresponding to N_ROLLOUTS rollouts (each of which has length HORIZON steps), a discount rate $\gamma$ of 0.999, two hidden layers of size 16, uses Generalized Advantage Estimation, $\lambda$ of 0.97, and other parameters as set below.

4. Running Experiments


5. Visualizing the results


6. Restart from a checkpoint / Transfer learning

If you wish to do transfer learning, or to resume a previous training, you will need to start the simulation from a previous checkpoint. To do that, you can add a restore parameter in the run_experiments argument, as follows:
```python
trials = run_experiments({
    flow_params["exp_tag"]: {
        "run": alg_run,
        "env": gym_name,
        "config": {
            **config
        },
        "restore": "/ray_results/experiment/dir/checkpoint_50/checkpoint-50"
        "checkpoint_freq": 1,
        "checkpoint_at_end": True,
        "max_failures": 999,
        "stop": {
            "training_iteration": 1,
        }
    }
})
```
The "restore" path should be such that the [restore]/.tune_metadata file exists.
There is also a "resume" parameter that you can set to True if you just wish to continue the training from a previously saved checkpoint, in case you are still training on the same experiment.

