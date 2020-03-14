# Manipulation Tasks

- [Learning manipulation tasks](https://www.di.ens.fr/willow/research/hrlbc/)

UR5 Robotiq 3 finger gripper manipulation tasks.

- [Relay Policy Learning:
Solving Long-Horizon Tasks via Imitation and Reinforcement Learning](https://relay-policy-learning.github.io/)

- [Setting up a Reinforcement Learning Task with a Real-World Robot](https://arxiv.org/pdf/1803.07067.pdf)

**Kindred Research**

The low-level robot controller of UR5, called ***URControl***, can be programmed by communicating over a **TCP/IP** connection. The robot can be controlled at the script-level using a programming language called ***URScript***. After establishing a connection, we can send URScript programs from a computer to URControl as strings over the socket. URScript programs run on URControl in real-time, which streams status packets every 8ms. Each packet from the URControl contains the sensorimotor information of the robot including angular positions, velocities, target accelerations, and currents for all joints. The robot can be controlled with URScript by sending low-level actuation commands on the same 8ms clock. The URScript ***servoj*** command offers a position control interface, and ***speedj*** offers a velocity control interface. Unlike Gym Reacher, there is no torque control interface.

**UR5 Reacher 6D:** The `observation` vector includes the `joint angles`, the `joint velocities`, and the vector `difference` between the target and the fingertip coordinates. Unlike Gym Reacher, we do not include the sines or cosines of joint angles or the target-position coordinates to simplify and reduce the observation space without losing essential information. We include the `previous action` as part of the observation vector, which can be helpful for learning in systems with `delays`. In Gym Reacher, the reward function is defined as: $R_t=-d_t-p_{t-1}$, where $d_t$ is the `Euclidean distance` between the target and the fingertip positions, and $p_t$ is the L2-norm of $A_t$ to `penalize` large torques. We use the same reward function but simplify it by dropping the penalty term. UR5 Reacher consists of episodes of interactions, where each episode is 4 seconds long to allow adequate exploration. The fingertip of UR5 is confined within a 2-dimensional $0.7m \times 0.5m$ boundary in UR5 Reacher and within a 3-dimensional $0.7m \times 0.5m \times 0.4m$ boundary in UR5 Reacher 6D. At each episode, the target position is chosen randomly within the boundary, and the arm starts from the middle of the boundary. In addition to constraining the fingertip within a boundary, the robot is also constrained within a joint-angular boundary to avoid self-collision.

There are several other crucial aspects of a real-world task that are rarely studied in simulations, such as the action cycle time, the medium of connection, the choice of actuation type, and concurrency and delays in computation. These aspects are the main focus of the current work.

In real-world tasks, time marches on during each agent and environment-related computations. Therefore, the agent always operates on delayed sensorimotor information. The overall latency can be further amplified by misplaced synchronization and ordering of computations, which may result in a more difficult learning problem and reduced potential for responsive control. Therefore, a design objective in setting up a learning task is to manage and minimize delays. Different approaches are proposed to alleviate this issue, such as augmenting the state space with actions or predicting the future state of action execution. These approaches do not minimize the delay but compensate for it from the perspective of learning agents. A Seldom discussed aspect of this issue is that different orderings or concurrencies of task computations may have different overall latencies.

In UR5 Reacher, we implemented the computational steps in Python and distributed them into two asynchronous processes: the *robot communication process* and the *reinforcement learning (RL) process*. They exchange sensorimotor information and actuation commands. 