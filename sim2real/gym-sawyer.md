# gym-sawyer

https://github.com/rlworkgroup/gym-sawyer

## Analysis

### reward

#### reward_type = 'dense'


#### reward_type = 'sparse'


### control mothod

#### task_space_control

Only use the mocap when using task space control, otherwise it will mess up the position control.

### observation space



### action space

```python
def action_space(self):
    if self._control_method == 'torque_control':
        return super(SawyerEnv, self).action_space
    elif self._control_method == 'task_space_control':
        return Box(
            np.array([-0.15, -0.15, -0.15, -1.]),
            np.array([0.15, 0.15, 0.15, 1.]),
            dype=np.float32)
    elif self._control_method == 'position_control':
        return Box(
            low=np.full(9, -0.02),
            high=np.full(9, 0.02),
            dtype=np.float32)
    else:
        raise NotImplementedError
```

```python
def step(self, action):
    assert action.shape == self.action_space.shape

    # Note: you MUST copy the action if you modify it
    a = action.copy()

    # Clip to action space
    a *= self._action_scale
    a = np.clip(a, self.action_space.low, self.action_space.high)
    if self._control_method == "torque_control":
        self.forward_dynamics(a)
        self.sim.forward()
    elif self._control_method == "task_space_control":
        reset_mocap2body_xpos(self.sim)
        self.sim.data.mocap_pos[0, :3] = self.sim.data.mocap_pos[0, :3] + a[:3]
        self.sim.data.mocap_quat[:] = np.array([0, 1, 0, 0])
        self.set_gripper_state(a[3])
        for _ in range(5):
            self.sim.step()
        self.sim.forward()
    elif self._control_method == "position_control":
        curr_pos = self.joint_positions

        next_pos = np.clip(
            a + curr_pos[:],
            self.joint_position_space.low[:],
            self.joint_position_space.high[:]
        )
        self.sim.data.ctrl[:] = next_pos[:]
        self.sim.forward()
        for _ in range(5):
            self.sim.step()
    else:
         raise NotImplementedError
    self._step += 1

    obs = self.get_obs()

    # collision checking is expensive so cache the value
    in_collision = self.in_collision

    info = {
        "l": self._step,
        "grasped": obs["has_object"],
        "gripper_state": obs["gripper_state"],
        "gripper_position": obs["gripper_pos"],
        "object_position": obs["object_pos"],
        "is_success": self._is_success,
        "in_collision": in_collision,
    }

    r = self.compute_reward(
        achieved_goal=self._achieved_goal,
        desired_goal=self._desired_goal,
        info=info)

    self._is_success = self._success_fn(self, self._achieved_goal,
                                        self._desired_goal, info)
    done = self._is_success and not self._never_done

    # collision detection
    if in_collision:
        r -= self._collision_penalty
        if self._terminate_on_collision:
            done = True
    
    if self._is_success:
        r = self._completion_bonus

    info["r"] = r
    info["d"] = done

    return obs, r, done, info
```

**action_noise:** Noise added to the controls, which will be proportional to the action bounds.
```python
def inject_action_noise(self, action):
    # generate action noise
    noise = self.action_noise * \
            np.random.normal(size=action.shape)
    # rescale the noise to make it proportional to the action bounds
    lb, ub = self.action_bounds
    noise = 0.5 * (ub - lb) * noise
    return action + noise

def forward_dynamics(self, action):
    self.sim.data.ctrl[:] = self.inject_action_noise(action)
    for _ in range(self.frame_skip):
        self.sim.step()
    self.sim.forward()
    new_com = self.sim.data.subtree_com[0] # Center of Mass
    self.dcom = new_com - self.current_com
    self.current_com = new_com
```

**control_freq:** how many control signals to receive in every simulated second. This sets the amount of simulation time that passes between every action input.
```python
def initialize_time(self, control_freq):
    """
    Initializes the time constants used for simulation.
    """
    self.cur_time = 0
    self.model_timestep = self.sim.model.opt.timestep
    if self.model_timestep <= 0:
        raise XMLError("xml model defined non-positive time step")
    self.control_freq = control_freq
    if control_freq <= 0:
        raise SimulationError("control frequency {} is invalid".format(control_freq))
    self.control_timestep = 1. / control_freq

def step(self, action):
    """
    Takes a step in simulation with control command @action.
    """
    if self.done:
        raise ValueError("executing action in terminated episode")

    self.timestep += 1
    self._pre_action(action)
    end_time = self.cur_time + self.control_timestep
    while self.cur_time < end_time:
        self.cur_time += self.model_timestep
    reward, done, info = self._post_action(action)
    return self._get_observation(), reward, done, info
```

```python
def pose_in_base_from_name(self, name):
    """
    A helper function that takes in a named data field and returns the pose
    of that object in the base frame.
    """

    pos_in_world = self.sim.data.get_body_xpos(name)
    rot_in_world = self.sim.data.get_body_xmat(name).reshape((3,3))
    pose_in_world = T.make_pose(pos_in_world, rot_in_world)

    base_pos_in_world = self.sim.data.get_body_xpos("base")
    base_rot_in_world = self.sim.data.get_body_xmat("base").reshape((3,3))
    bose_pose_in_world = T.make_pose(base_pos_in_world, base_rot_in_world)
    world_pose_in_base = T.pose_inv(base_pose_in_world)

    pose_in_base = T.pose_in_A_to_pose_in_B(pose_in_world, world_pose_in_base)
    return pose_in_base

def make_pose(translation, rotation):
    """
    Makes a homogenous pose matrix from a translation vector and a rotation matrix.
    
    Args:
        translation: a 3-dim iterable
        rotation: a 3x3 matrix
    Returns:
        pose: a 4x4 homogenous matrix
    """
    pose = np.zeros((4,4))
    pose[:3, :3] = rotation
    pose[:3, 3] = tranlation
    pose[3, 3] = 1.0
    return pose
```




