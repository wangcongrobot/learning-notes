# Meta Learning 

from Hung-yi Lee

## Meta learning = learn to learn

life-long learning: one model for all the tasks
Meta learning: How to learn a new model
few-shot learning: widely considered in few-show learning

Learning Algorithm: F: D -> T

Step 1: define a set of learning algorithm (Function F)

Step 2: Define the goodness fo a function F

Loss function:

$$L(F)=\sum_{n=1}^N {l^n}$$

find the best function

Step 3: Find the best function $F^*$

## Omniglot dateset

Few-shot Classification

- N-ways K-shot classification: there are N classes, each has K examples


Techniques
- MAML: Model-Agnostic Meta-Learning
- Reptile: 

## MAML and Reptile method

MAML: **Only focus on initialization parameter.** How to find the best initialization parameter.

$\phi\leftarrow\phi-\eta$

Model Pre-training

Widely used in transfer learning

- Fast...Fast...Fast
- Good to truly train a model with one step

Toy Example

Given a target sine function $y=asin(x+b)$

blog: paper repro deep meta-learning using maml and reptile



Math



MAML - Real Implementation



## Meta Learning (Part 2): Gradient Descent as LSTM

RNN: Recurrent Neural Network

c change slowly
h change faster

$lc^t\theta\sigma \dot{x}\subset{x}tanh$

LSTM

LSTM for Gradient Descent



Real Implementation

The LSTM used only has one cell. Share across all parameters

- Reasonable model size
- In typical gradient descent, all the parameters use the same update rule
- Training and testing model architecture


Meta Learning Metric-based






