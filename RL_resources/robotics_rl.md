# Some examples to explain how to train the robot with rl

## articles

[Training a Robotic Arm to do Human-Like Tasks using RL](https://medium.com/datadriveninvestor/training-a-robotic-arm-to-do-human-like-tasks-using-rl-8d3106c87aaf)

### [吐槽openai gym mujoco](https://zhuanlan.zhihu.com/p/78652777)

_Author: Ming,CS Phd student_

最近在尝试解决openai gym里的mujoco一系列任务，期间遇到数坑，感觉用这个baseline太不科学了，在此吐槽一下。

1. Walker2d的两只脚的摩擦系数不同：具体见以下github链接：https://github.com/openai/gym/blob/master/gym/envs/mujoco/assets/walker2d.xml。 一只脚是0.9一只脚是1.9，一般不是写完一只脚后直接复制粘贴成为另一只脚吗，怎么做到不同的的。。。
2. 个人理解，一般mujoco task的action space都是限定在[-1, 1]之间，所以一般actor都会直接在最后加个tanh限制输出。然而，humanoid是个特例：https://github.com/openai/gym/blob/master/gym/envs/mujoco/assets/humanoid.xml。里面写着ctrlrange='-0.4 04'. 之前一直吐槽一些算法为何不敢尝试humanoid，现在终于理解。SAC（https://arxiv.org/pdf/1801.01290.pdf）竟然能够在humanoid上效果这么好，只能膜拜了。之后我测试都是将里面motor gear乘0.4，然后将ctrlrange改成“-1，1”了。
3. humanoid的reward设置的极其不科学：https://github.com/openai/gym/blob/master/gym/envs/mujoco/humanoid.py。其中alive_bonus设到了极其高的5， 观察sac里的humanoid learning curve可以发现，最高reward也就到了6000，如果最后结果是能完整跑1000步的话，也就是说其他reward平均每步只有1。一般alive_bonus主要是为了让agent在得到负数reward的时候不会想去自杀，像hopper和walker2d都只设成了1，其他reward在收敛后平均每步都能有3的样子，因此我很不理解为什么他们决定将humanoid的弄的那么高。我将alive_bonus去掉之后，用ppo能训练出跑完整1000步得到reward3000多，也就是加上alive_bonus的话会有8000多。附上某次训练出的蜜汁走位：

4. humanoid的observation极其不科学。基本上能用到的东西都加到state里面了，包括每个link的位置和速度和受到的外力。最后observation竟然有300多维，知道真相的我是奔溃的。一堆observation其实永远是0 （除了脚其他部位基本没机会受到外力），link的位置和速度从joint angle／velocity完全可以用forward kinematic算出来，基本上是一大堆重复信息。

后来自我反思，也许openai是故意想将baseline变难考验我们的算法呢，也是用心良苦了。