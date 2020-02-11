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

