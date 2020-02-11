# Model-Based RL


## resource

https://github.com/topics/model-based-rl

https://github.com/topics/model-based-reinforcement-learning

https://github.com/hejia-zhang/awesome-model-based-reinforcement-learning

https://github.com/cap-ntu/baconian-project

http://www.mpc.berkeley.edu/mpc-course-material

https://www.cambridge.org/us/academic/subjects/engineering/control-systems-and-optimization/predictive-control-linear-and-hybrid-systems?format=AR#6JTF4QUKg6HWeQE9.97

- [DeepMPC](http://deepmpc.cs.cornell.edu/)

More formally, DeepMPC is an approach to model learning for predictive control control designed to handle both variations in robot's environment and variations that might occur while the robot acts. The two main components of this algorithm are a Model Predictive Controller (MPC) and Deep Learning (DL).

![mpc](http://deepmpc.cs.cornell.edu/img/blockdia-web.png)

**Model Predictive Control:** The key idea behind MPC is essentially allowing a robot to ask "what if?" If we have an accurate model of how the world responds to the robot's actions, it can use this to try new ones out and pick the one it predicts will do best. The problem is that we need an accurate model so these predictions match the real world. Hand-coding models for complex, varying tasks like cutting food is almost impossible, so instead we want the robot to be able to learn from experience.

**Deep Learning:** To learn these models, we apply deep learning algorithms. Deep learning is an exciting new class of machine learning algorithms which learn simulated neural networks to solve problems. These algorithms are especially powerful because they can learn good representations (abstractions) directly from data, allowing us to deal with real-world variety without having to model it ourselves.

![mpc](http://deepmpc.cs.cornell.edu/img/deepModel.png)

In DeepMPC,  we develop a new deep architecture and learning algorithm for modeling complex physical dynamics like we see when cutting food. This architecture allows us to model the effects of variations between and within materials without having to manually define a set of properties to do so. Instead, our algorithm automatically learns a set of features which let the model recognize these properties and predict their effects.

- [Byron Boots](https://homes.cs.washington.edu/~bboots/)

- [Visual Foresight: Model-Based Deep Reinforcement Learning for Vision-Based Robotic Control](https://sites.google.com/view/visualforesight)

**Visual Foresight:** On a high level, Visual Model Predictive Control (visual-MPC) leverages an action-conditioned model (trained from unsupervised interaction) to enable robots to perform various tasks with only raw-pixel input. This codebase provides an implementation of: unsupervised data collection, our benchmarking framework, the various planning costs, and - of course - the visual-MPC controller! Additionally, we provide: instructions to reproduce our experiments, Dockerfiles for our simulator environments, and documentation on our Sawyer robot setup.

- [visual-mpc](https://github.com/febert/visual_mpc/tree/dev)

Visual MPC implementation running on Rethink Sawyer Robot

https://arxiv.org/pdf/1812.00568.pdf

## ppt

- [Model-Based RL](http://mlg.eng.cam.ac.uk/mlss09/mlss_slides/Littman_1.pdf)

https://bair.berkeley.edu/blog/2019/12/12/mbpo/

- [Benchmarking Model-Based Reinforcement Learning](https://arxiv.org/abs/1907.02057)

http://www.cs.toronto.edu/~tingwuwang/mbrl.html

Model-based reinforcement learning (MBRL) is widely seen as having the potential to be significantly more sample efficient than model-free RL. However, research in model-based RL has not been very standardized. It is fairly common for authors to experiment with self-designed environments, and there are several separate lines of research, which are sometimes closed-sourced or not reproducible. Accordingly, it is an open question how these various existing MBRL algorithms perform relative to each other. To facilitate research in MBRL, in this paper we gather a wide collection of MBRL algorithms perform relative to each other. To facilitate research in MBRL, in this paper we gather a wide collection of MBRL algorithms and propose over 18 benchmarking environment specially designed for MBRL. We benchmark these MBRL algorithms with unified problem settings, including noisy environments. Beyond cataloguing performance, we explore and unify the underlying algorithmic difference across MBRL algorithms. We characterize three key research challenges for future MBRL research: the dynamics coupling effect, the planning horizon dilemma, and the early-termination dilemma.

**MBPO (high-level)**
1. Collect environment trajectories; add to $\mathcal{D}_{\mathrm{env}}$
2. Train model ensemble on environment data $\mathcal{D}_{\mathrm{env}}$
3. Perform k-step model rollouts branched from $\mathcal{D}_{\mathrm{env}}$; add to $\mathcal{D}_\mathrm{model}$
4. Update policy parameters on model data $\mathcal{D}_{\mathrm{model}}$


- [Learning to Adapt Dynamic, Real-World Environment through Meta-Reinforcement Learning](https://github.com/iclavera/learning_to_adapt)

paper: https://arxiv.org/pdf/1803.11347.pdf

video: https://sites.google.com/berkeley.edu/metaadaptivecontrol

Although reinforcement learning methods can achieve impressive results in simulation, the real world presents two major challenges: generating samples is exceedingly expensive, and unexpected perturbations or unseen situations cause proficient but specialized policies to fail at test time. Given that it is impractical to train separate policies to accommodate all situations the agent may see in the real world, this work proposes to learn how to quickly and effectively adapt online to new tasks. To enable sample-efficient learning, we consider learning online adaptation in the context of model-based reinforcement learning. Our approach uses meta-learning to train a dynamics model prior such that, when combined with recent data, this prior can be rapidly adapted to the local context. Our experiments demonstrate online adaptation for continuous control tasks on both simulated and real-world agents. We first show simulated agents adapting their behavior online to novel terrains, crippled body parts, and highly-dynamic environments. We also illustrate the importance of incorporating online adaptation into autonomous agents that operate in the real world by applying our method to a real dynamic legged millirobot. We demonstrate the agent's learned ability to quickly adapt online to a missing leg, adjust to novel terrains and slopes, account for miscalibration or errors in pose estimation, and compensate for pulling payloads.


