# RL & Model-Predictive Control(MPC)

- [Zero-Shot Skill Composition and Simulation-to-Real Transfer by Learning Task Representations](https://arxiv.org/pdf/1810.02422.pdf)

Our proposed approach is based on four key components: reinforcement learing with policy gradients (RL), variational inference, model-predictive control (MPC), and physics simulation. We use variational inference to learn a low-dimensional latent space of skills which are useful for tasks, and RL to simultaneously learn single policy which is conditioned on these latent skills. The precise long-horizon behavior of the policy for a given latent skill is difficult to predict, so we use MPC and an online simulation to evaluate latent skill plans in the in simulation before executing them on the real robot.

In this work, we combine task representation learning, simulation-to-real training, and model-predictive control to efficiently adapt to unseen tasks with no additional on-robot training or exploration. Our experiments show that applying model predictive control to these learned skill representations can be a efficient method for online adaptation. 

https://github.com/ryanjulian/embed2learn/commit/ce6b01946567723f7d4687760099ae1bb985d6d2

- [Neural Network Dynamics for Model-Based Deep Reinforcement Learning with Model-Free Fine-Tuning](https://arxiv.org/pdf/1708.02596.pdf)

In model-based reinforcement learning, a model of the dynamics is used to make predictions, which is used for action selection. Let $\hat{f}_\theta(\mathbf{s}_t, \mathbf{a}_t)$ denote a learned discrete-time dynamics function, parameterized by $\theta$, that takes the current state $\mathbf{s}_t$ and action $\mathbf{a}_t$ and outputs an estimate of the next state at time $t+\Delta t$. We can then choose actions by solving the following optimization problem:
$$
\left(\mathbf{a}_{t}, \ldots, \mathbf{a}_{t+H-1}\right)=\arg \max _{\mathbf{a}_{t}, \ldots, \mathbf{a}_{t+H-1}} \sum_{t^{\prime}=t}^{t+H-1} \gamma^{t^{\prime}-t} r\left(\mathbf{s}_{t^{\prime}}, \mathbf{a}_{t^{\prime}}\right)
$$

In practice, it is often desirable to solve this optimization at each time step, execute only the first action $\mathbf{a}_t$ from the sequence, and then replan at the next time step with updated state information. Such a control scheme is often referred to as model predictive control (MPC), and is known to compensate well for errors in the model.

