# reward function example

## some blogs
[reward shaping](https://zhuanlan.zhihu.com/p/56425081)

https://www.researchgate.net/post/What_is_the_best_Reward_function_in_Reinforcement_Learning

https://medium.com/@BonsaiAI/deep-reinforcement-learning-models-tips-tricks-for-writing-reward-functions-a84fe525e8e0

https://stats.stackexchange.com/questions/189067/how-to-make-a-reward-function-in-reinforcement-learning

https://hal.archives-ouvertes.fr/hal-00331752v2/document

https://www.quora.com/How-does-one-learn-a-reward-function-in-Reinforcement-Learning-RL

https://deepblue.lib.umich.edu/bitstream/handle/2027.42/136931/guoxiao_1.pdf?sequence=1&isAllowed=y

https://www.analyticsindiamag.com/understanding-the-role-of-reward-functions-in-reinforcement-learning/

https://www.quora.com/How-do-I-design-the-reward-function-in-reinforcement-learning-Can-I-use-the-states-and-control-signals-of-multiple-moments-Would-it-effect-the-Markov-property-or-Bellmans-Principle-of-Optimality

https://ai-alignment.com/the-reward-engineering-problem-30285c779450



## alive_bonus
as a constant number, every step plus a number.
the longer alive time, the more reward

## gym mujoco pusher env

```python
def step(self, a):
    vec_1 = self.get_body_com("object") - self.get_body_com("tips_arm")
    vec_2 = self.get_body_com("object") - self.get_body_com("goal")

    reward_near = - np.linalg.norm(vec_1)
    reward_dist = - np.linalg.norm(vec_2)
    reward_ctrl = - np.square(a).sum()
    reward = reward_dist + 0.1 * reward_ctrl + 0.5 * reward_near

    self.do_simulation(a, self.frame_skip)
    ob = self._get_obs()
    done = False
    return ob, reward, done, dict(reward_dist=reward_dist,
            reward_ctrl=reward_ctrl
```

## open the door with hand

```python
def _step(self, a):
    a = np.clip(a, -1.0, 1.0)
    try:
        a = self.act_mid + a*self.act_rng # mean center and scale
    except:
        a = a                             # only for the initialization phase
    self.do_simulation(a, self.frame_skip)
    ob = self._get_obs()
    handle_pos = self.data.site_xpos[self.handle_sid].ravel()
    palm_pos = self.data.site_xpos[self.grasp_sid].ravel()
    door_pos = self.data.qpos[self.door_hinge_did]

    # get to handle
    reward = -0.1*np.linalg.norm(palm_pos-handle_pos)
    # open door
    reward += -0.1*(door_pos - 1.57)*(door_pos - 1.57)
    # velocity cost
    reward += -1e-5*np.sum(self.data.qvel**2)

    # Bonus
    if door_pos > 0.2:
        reward += 2
    if door_pos > 1.0:
        reward += 8
    if door_pos > 1.35:
        reward += 10

    return ob, reward, False, {}

```

hammer:
```python
def _step(self, a):
    a = np.clip(a, -1.0, 1.0)
    try:
        a = self.act_mid + a * self.act_rng  # mean center and scale
    except:
        a = a  # only for the initialization phase
    self.do_simulation(a, self.frame_skip)
    ob = self._get_obs()
    obj_pos = self.data.body_xpos[self.obj_bid].ravel()
    palm_pos = self.data.site_xpos[self.S_grasp_sid].ravel()
    tool_pos = self.data.site_xpos[self.tool_sid].ravel()
    target_pos = self.data.site_xpos[self.target_obj_sid].ravel()
    goal_pos = self.data.site_xpos[self.goal_sid].ravel()
    
    # get to hammer
    reward = - 0.1 * np.linalg.norm(palm_pos - obj_pos)
    # take hammer head to nail
    reward -= np.linalg.norm((tool_pos - target_pos))
    # make nail go inside
    reward -= 10 * np.linalg.norm(target_pos - goal_pos)
    # velocity penalty
    reward -= 1e-2 * np.linalg.norm(self.data.qvel.ravel())

    # bonus for lifting up the hammer
    if obj_pos[2] > 0.04 and tool_pos[2] > 0.04:
        reward += 2

    # bonus for hammering the nail
    if (np.linalg.norm(target_pos - goal_pos) < 0.020):
        reward += 25
    if (np.linalg.norm(target_pos - goal_pos) < 0.010):
        reward += 75

    return ob, reward, False, {}
```

pan:
```python
def _step(self, a):
    a = np.clip(a, -1.0, 1.0)
    try:
        a = self.act_mid + a*self.act_rng # mean center and scale
    except:
        a = a                             # only for the initialization phase
    self.do_simulation(a, self.frame_skip)

    obj_pos  = self.data.body_xpos[self.obj_bid].ravel()
    desired_loc = self.data.site_xpos[self.eps_ball_sid].ravel()
    obj_orien = (self.data.site_xpos[self.obj_t_sid] - self.data.site_xpos[self.obj_b_sid])/self.pen_length
    desired_orien = (self.data.site_xpos[self.tar_t_sid] - self.data.site_xpos[self.tar_b_sid])/self.tar_length

    # pos cost
    dist = np.linalg.norm(obj_pos-desired_loc)
    reward = -dist
    # orien cost
    orien_similarity = np.dot(obj_orien, desired_orien)
    reward += orien_similarity

    # bonus for being close to desired orientation
    if dist < 0.075 and orien_similarity > 0.9:
        reward += 10
    if dist < 0.075 and orien_similarity > 0.95:
        reward += 50

    # penalty for dropping the pen
    if obj_pos[2] < 0.15:
        reward -= 5

    return self._get_obs(), reward, False, {}
```

relocate
```python
def _step(self, a):
    a = np.clip(a, -1.0, 1.0)
    try:
        a = self.act_mid + a*self.act_rng # mean center and scale
    except:
        a = a                             # only for the initialization phase
    self.do_simulation(a, self.frame_skip)
    ob = self._get_obs()
    obj_pos  = self.data.body_xpos[self.obj_bid].ravel()
    palm_pos = self.data.site_xpos[self.S_grasp_sid].ravel()
    target_pos = self.data.site_xpos[self.target_obj_sid].ravel()

    reward = -0.1*np.linalg.norm(palm_pos-obj_pos)              # take hand to object
    if obj_pos[2] > 0.04:                                       # if object off the table
        reward += 1.0                                           # bonus for lifting the object
        reward += -0.5*np.linalg.norm(palm_pos-target_pos)      # make hand go to target
        reward += -0.5*np.linalg.norm(obj_pos-target_pos)       # make object go to target
    if np.linalg.norm(obj_pos-target_pos) < 0.1:
        reward += 10.0                                          # bonus for object close to target
    if np.linalg.norm(obj_pos-target_pos) < 0.05:
        reward += 20.0                                          # bonus for object "very" close to target

    return ob, reward, False, {}
```

fetch catch env:
```python
def compute_reward(self, achieved_goal, goal, info):
    reward_ctrl = - 0.05 * np.square(self.action).sum()

    dist_to_end_location = np.linalg.norm(self.sim.data.get_site_xpos('robot0:grip') -
                                            self.end_location)
    reward_dist = tolerance(dist_to_end_location, margin=0.8, bounds=(0., 0.02),
                            sigmoid='linear',
                            value_at_margin=0.)

    reward = 0.25 * reward_dist

    if self.sim.data.get_site_xpos('tar')[2] < 0.1: # if z < 0.1, then drop out and restart
        self._restart_target()

    sparse_reward = 0.
    dist = np.linalg.norm(self.sim.data.get_site_xpos('robot0:grip') -
                            self.sim.data.get_site_xpos('tar'))
    if dist < 0.05: # catch
        reward += 4.
        sparse_reward += 2.
        self._restart_target()

    reward += reward_ctrl

    info = dict(scoring_reward=sparse_reward)

    return reward, False, info
```

Fetch keep up env:
```python
def compute_reward(self, achieved_goal, goal, info):
    if not self.high_motion_penalty:
        reward_ctrl = - 0.05 * np.square(self.action).sum()
    else:
        reward_ctrl = - 0.075 * np.square(self.action).sum()


    dist = np.linalg.norm(self.sim.data.get_site_xpos('robot0:grip')[:2] -
                            self.sim.data.get_site_xpos('tar')[:2])
    reward_dist = tolerance(dist, margin=0.5, bounds=(0., 0.02),
                            sigmoid='linear',
                            value_at_margin=0.)

    reward = 0.2 * reward_dist + reward_ctrl

    done = False
    if self.sim.data.get_site_xpos('tar')[2] < 0.4:
        done = True
        reward = -1.

    sparse_reward = 0.
    if self.give_reflection_reward:
        sparse_reward = 1.
        self.give_reflection_reward = False

    reward += 0.2 * sparse_reward

    info = dict(scoring_reward=sparse_reward)

    return reward, done, info
```

## Pick and Place

```python

def reward(self, action=None):
    # compute sparse rewards
    self._check_success()
    reward = np.sum(self.objects_in_bins)

    # add in shaped rewards
    if self.reward_shaping:
        staged_rewards = self.staged_rewards()
        reward += max(staged_rewards)
    return reward

def staged_rewards(self):
    """
    Returns staged rewards based on current physical states.
    Stages consist of reaching, grasping, lifting, and hovering.
    """

    reach_mult = 0.1
    grasp_mult = 0.35
    lift_mult = 0.5
    hover_mult = 0.7

    # filter out objects that are already in the correct bins
    objs_to_reach = []
    geoms_to_grasp = []
    target_bin_placements = []
    for i in range(len(self.ob_inits)):
        if self.objects_in_bins[i]:
            continue
        obj_str = str(self.item_names[i]) + "0"
        objs_to_reach.append(self.obj_body_id[obj_str])
        geoms_to_grasp.append(self.obj_geom_id[obj_str])
        target_bin_placements.append(self.target_bin_placements[i])
    target_bin_placements = np.array(target_bin_placements)

    ### reaching reward governed by distance to closest object ###
    r_reach = 0.
    if len(objs_to_reach):
        # get reaching reward via minimum distance to a target object
        target_object_pos = self.sim.data.body_xpos[objs_to_reach]
        gripper_site_pos = self.sim.data.site_xpos[self.eef_site_id]
        dists = np.linalg.norm(
            target_object_pos - gripper_site_pos.reshape(1, -1), axis=1
        )
        r_reach = (1 - np.tanh(10.0 * min(dists))) * reach_mult

    ### grasping reward for touching any objects of interest ###
    touch_left_finger = False
    touch_right_finger = False
    for i in range(self.sim.data.ncon):
        c = self.sim.data.contact[i]
        if c.geom1 in geoms_to_grasp:
            bin_id = geoms_to_grasp.index(c.geom1)
            if c.geom2 in self.l_finger_geom_ids:
                touch_left_finger = True
            if c.geom2 in self.r_finger_geom_ids:
                touch_right_finger = True
        elif c.geom2 in geoms_to_grasp:
            bin_id = geoms_to_grasp.index(c.geom2)
            if c.geom1 in self.l_finger_geom_ids:
                touch_left_finger = True
            if c.geom1 in self.r_finger_geom_ids:
                touch_right_finger = True
    has_grasp = touch_left_finger and touch_right_finger
    r_grasp = int(has_grasp) * grasp_mult

    ### lifting reward for picking up an object ###
    r_lift = 0.
    if len(objs_to_reach) and r_grasp > 0.:
        z_target = self.bin_pos[2] + 0.25
        object_z_locs = self.sim.data.body_xpos[objs_to_reach][:, 2]
        z_dists = np.maximum(z_target - object_z_locs, 0.)
        r_lift = grasp_mult + (1 - np.tanh(15.0 * min(z_dists))) * (
            lift_mult - grasp_mult
        )

    ### hover reward for getting object above bin ###
    r_hover = 0.
    if len(objs_to_reach):
        # segment objects into left of the bins and above the bins
        object_xy_locs = self.sim.data.body_xpos[objs_to_reach][:, :2]
        y_check = (
            np.abs(object_xy_locs[:, 1] - target_bin_placements[:, 1])
            < self.bin_size[1] / 4.
        )
        x_check = (
            np.abs(object_xy_locs[:, 0] - target_bin_placements[:, 0])
            < self.bin_size[0] / 4.
        )
        objects_above_bins = np.logical_and(x_check, y_check)
        objects_not_above_bins = np.logical_not(objects_above_bins)
        dists = np.linalg.norm(
            target_bin_placements[:, :2] - object_xy_locs, axis=1
        )
        # objects to the left get r_lift added to hover reward, those on the right get max(r_lift) added (to encourage dropping)
        r_hover_all = np.zeros(len(objs_to_reach))
        r_hover_all[objects_above_bins] = lift_mult + (
            1 - np.tanh(10.0 * dists[objects_above_bins])
        ) * (hover_mult - lift_mult)
        r_hover_all[objects_not_above_bins] = r_lift + (
            1 - np.tanh(10.0 * dists[objects_not_above_bins])
        ) * (hover_mult - lift_mult)
        r_hover = np.max(r_hover_all)

    return r_reach, r_grasp, r_lift, r_hover
```

why: reward += 0.5 * (1 - np.tanh(dist))
reason: the longer distance, the less reward; the value of dist will be limited to (-1, 1); distance > 0, so np.tanh(dist) will be (0, 1)

## Check Contact

```python
def find_contacts(self, geoms_1, geoms_2):
    """
    Finds contact between two geom groups.

    Args:
        geoms_1: a list of geom names (string)
        geoms_2: another list of geom names (string)

    Returns:
        iterator of all contacts between @geoms_1 and @geoms_2
    """
    for contact in self.sim.data.contact[0 : self.sim.data.ncon]:
        # check contact geom in geoms
        c1_in_g1 = self.sim.model.geom_id2name(contact.geom1) in geoms_1
        c2_in_g2 = self.sim.model.geom_id2name(contact.geom2) in geoms_2
        # check contact geom in geoms (flipped)
        c2_in_g1 = self.sim.model.geom_id2name(contact.geom2) in geoms_1
        c1_in_g2 = self.sim.model.geom_id2name(contact.geom1) in geoms_2
        if (c1_in_g1 and c2_in_g2) or (c1_in_g2 and c2_in_g1):
            yield contact

def _check_contact(self):
    """
    Returns True if gripper is in contact with an object.
    """
    collision = False
    contact_geoms = (
        self.gripper_right.contact_geoms() + self.gripper_left.contact_geoms()
    )
    for contact in self.sim.data.contact[: self.sim.data.ncon]:
        if (
            self.sim.model.geom_id2name(contact.geom1) in contact_geoms
            or self.sim.model.geom_id2name(contact.geom2) in contact_geoms
        ):
            collision = True
            break
    return collision
```

