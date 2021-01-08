# Combine MPC with RL

Model-based RL gets better performance than model-free RL.

- [Simulator Predictive Control: Using Learned Task Representations and MPC for Zero-Shot Generalization and Sequencing](http://www.cs.columbia.edu/~zhanpeng/publications/mpc_embeddings_nips.pdf)

We present a method to efficiently performing new robotic tasks directly on a real robot, based on model-predictive control (MPC) and learned task representations. This work is published in Conference on Neural Information Processing Systems 2018 Deep RL Workshop.

**Code: ** https://github.com/ryanjulian/embed2learn

- [Zero-Shot Skill Composition and Simulation-to-Real Transfer by Learning Task Representations](https://arxiv.org/pdf/1810.02422.pdf)

- [Scaling simulation-to-real transfer by learning composable robot skills](https://arxiv.org/pdf/1809.10253.pdf)

https://github.com/ryanjulian/embed2learn/commit/ce6b01946567723f7d4687760099ae1bb985d6d2

```python
""" Model Predictive Controller"""
import numpy as np

class MPCPolicy:
    def __init__(self, embedding, n_learned_skills, inner_env, inner_policy, gamma=1., n_sampled_action=50, rollout_length=3):
        self._embedding = embedding

    def get_action(self, observation, state):
        actions = self._sample_actions()

    
```


- [Neural Network Dynamics for Model-Based Deep Reinforcement Learning with Model-Free Fine-Tuning](https://github.com/nagaban2/nn_dynamics)

In this work, we demonstrate that medium-sized neural network models can in fact be combined with model predictive control (MPC) to achieve excellent sample complexity in a model-based reinforcement learning algorithm, producing stable and plausible gaits to accomplish various complex locomotion tasks. We also propose using deep neural network dynamics models to initialize a model-free learner, in order to combine the sample efficiency of model-based approaches with the high task-specific performance of model-free methods.

- [Model Predictive Path Integral Controller (MPPI)](https://github.com/AutoRally/autorally/wiki/Model-Predictive-Path-Integral-Controller-(MPPI))

The model predictive path integral (MPPI) controller is a novel approabh for autonomous vehicle control based on stochastic sampling of trajectories. The method is derivative free, and can handle complex non-linear dynamics and cost functions, which makes it ideal for performing aggressive maneuvers with the AutoRally vehicle.

- [Model Predictive Control and HalfCheetah](https://hollygrimm.com/rl_modelbased)

The dynamics model can be implemented using a Gaussian Process, a Neural Network, or other methods.

One advantage of model-based RL is that they require fewer samples to train compared to model-free. 

**Code:** https://github.com/hollygrimm/cs294-homework/tree/master/hw4

The author implemented the CS294 Homework to understand the interaction between the Neural Network, Model Predictive Control, and data aggregation. The environment was a MuJoCo HalfCheetah simulation in OpenAI Gym.

**Collect Base Data:** The first step is to initialize a dataset of trajectories by running a random policy. Here is a video of the HalfCheetah while the data is being collected.

**Neural Network Dynamics Model:** The paper [1] uses a neural network for the dynamics model with two fully-connected layers of 500 units each and a ReLU activation. When fitting the dynamics model, normalized state and action pairs are input, and the state differences (or deltas) between the input state and next state are output. By predicting a change in state, rather than just the next state, the dynamics model can predict over several timesteps instead of just one timestep.

The mean squared error between the predicted and expected state deltas is minimized during training with the Adam optimizer.

**Reward Function:** The reward function was provided in the homework code, and was a combination of the location of the HalfCheetah's leg, shin, and foot position.

**Model Predictive Controller:** The Model Predictive Controller (MPC) selects an action for a particular state by first generating ten simulated paths each with a horizon of five actions into the future. The next state is predicted using the dynamics model. The candidate paths are evaluated using the reward function and the best performing trajectory is selected. The first action of that trajectory is then performed.

When ten samples are completed with the MPC, the new data is then aggregated into the dataset. This completes the first iteration. For the next iteration, the dynamics model is refitted to the new data, and new samples are again generated using the MPC.

**Homework:** http://rail.eecs.berkeley.edu/deeprlcourse-fa17/f17docs/hw4.pdf

