# Mujoco Contact

# mujoco_py contact define

        int ncon                        # number of detected contacts

    // computed by mj_fwdPosition/mj_collision
    mjContact* contact;             // list of all detected contacts            (nconmax x 1)

    #------------------------------ mjContact ----------------------------------------------

    ctypedef struct mjContact:                   # result of collision detection functions
        # contact parameters set by geom-specific collision detector
        mjtNum dist                     # distance between nearest points; neg: penetration
        mjtNum pos[3]                   # position of contact point: midpoint between geoms
        mjtNum frame[9]                 # normal is in [0-2]

        # contact parameters set by mj_collideGeoms
        mjtNum includemargin            # include if dist<includemargin=margin-gap
        mjtNum friction[5]              # tangent1, 2, spin, roll1, 2
        mjtNum solref[mjNREF]           # constraint solver reference
        mjtNum solimp[mjNIMP]           # constraint solver impedance

        # storage used internally by constraint solver
        mjtNum mu                       # friction of regularized cone
        mjtNum H[36]                    # cone Hessian, set by mj_updateConstraint

        # contact descriptors set by mj_collideGeoms
        int dim                         # contact space dimensionality: 1, 3, 4 or 6
        int geom1                       # id of geom 1
        int geom2                       # id of geom 2

        # flag set by mj_fuseContact or mj_instantianteEquality
        int exclude                     # 0: include, 1: in gap, 2: fused, 3: equality

        # address computed by mj_instantiateContact
        int efc_address                 # address in efc; -1: not included, -2-i: distance constraint i ???

## mujoco forum

The contacts are returned in the mjContact structure defined here:

http://www.mujoco.org/book/source/mjdata.h

Each contact involves two geoms, whose geom ids are given in mjContact. The think you are calling a "link" is a body in MuJoCo. One body can have multiple geoms attached to id. So if you have the geomid, you can find the corresponding body in mjModel.geom_bodyid[geomid]. This is C, but you should be able to figure out the Python version.

Contype and conaffinity determine which pairs of geoms are checked for collisions in the first place. Once the collision check passes, an mjContact is added to the list of contacts.

There is a number ncon which indicate the number of contacts. I can read the first ncon from the contacts list to get current contacts!

## code

```python
#!/usr/bin/env python

import os
import mujoco_py
import numpy as np

PATH_TO_HUMANOID_XML = os.path.expanduser('~/.mujoco/mjpro150/model/humanoid.xml')

# Load the model and make a simulator
model = mujoco_py.load_model_from_path(PATH_TO_HUMANOID_XML)
sim = mujoco_py.MjSim(model)

# Simulate 1000 steps so humanoid has fallen on the ground
for _ in range(10000):
    sim.step()

print('number of contacts', sim.data.ncon)
for i in range(sim.data.ncon):
    # Note that the contact array has more than `ncon` entries,
    # so be careful to only read the valid entries.
    contact = sim.data.contact[i]
    print('contact', i)
    print('dist', contact.dist)
    print('geom1', contact.geom1, sim.model.geom_id2name(contact.geom1))
    print('geom2', contact.geom2, sim.model.geom_id2name(contact.geom2))
    # There's more stuff in the data structure
    # See the mujoco documentation for more info!
    geom2_body = sim.model.geom_bodyid[sim.data.contact[i].geom2]
    print(' Contact force on geom2 body', sim.data.cfrc_ext[geom2_body])
    print('norm', np.sqrt(np.sum(np.square(sim.data.cfrc_ext[geom2_body]))))
    # Use internal functions to read out mj_contactForce
    c_array = np.zeros(6, dtype=np.float64)
    print('c_array', c_array)
    mujoco_py.functions.mj_contactForce(sim.model, sim.data, i, c_array)
    print('c_array', c_array)

print('done')
```

output:

```bash
(dher-env) cong@eclipse:~/workspace/DHER/gym-catcher$ python gym_catcher/test/contact_test.py 
number of contacts 13
contact 0
dist 0.0007609956711303323
geom1 0 floor
geom2 2 head
 Contact force on geom2 body [-1.89635674e-08  4.11075990e+01 -5.21771186e-08  2.33198806e+00
  3.96728500e-07  1.09637473e+02]
norm 117.11382677148879
c_array [0. 0. 0. 0. 0. 0.]
c_array [ 4.70748160e+01 -1.20388405e-07 -7.77331815e-01  0.00000000e+00
  0.00000000e+00  0.00000000e+00]
contact 1
dist 0.000841307432185634
geom1 0 floor
geom2 3 uwaist
 Contact force on geom2 body [-1.89635674e-08  4.11075990e+01 -5.21771186e-08  2.33198806e+00
  3.96728500e-07  1.09637473e+02]
norm 117.11382677148879
c_array [0. 0. 0. 0. 0. 0.]
c_array [ 3.12813296e+01 -2.58748382e-07  7.77328196e-01  0.00000000e+00
  0.00000000e+00  0.00000000e+00]
contact 2
dist 0.0008413074416442901
geom1 0 floor
geom2 3 uwaist
 Contact force on geom2 body [-1.89635674e-08  4.11075990e+01 -5.21771186e-08  2.33198806e+00
  3.96728500e-07  1.09637473e+02]
norm 117.11382677148879
c_array [0. 0. 0. 0. 0. 0.]
c_array [ 3.12813277e+01 -2.58748382e-07  7.77328049e-01  0.00000000e+00
  0.00000000e+00  0.00000000e+00]
contact 3
dist 0.0001904186006554641
geom1 0 floor
geom2 8 right_foot_cap1
 Contact force on geom2 body [ -9.01652943 -24.82506081  -1.55435095  -1.83574472  -9.3855429
 106.86256552]
norm 110.50368201217177
c_array [0. 0. 0. 0. 0. 0.]
c_array [21.75935005 -0.6522487   4.11828118  0.          0.          0.        ]
contact 4
dist 0.0005627517921205534
geom1 0 floor
geom2 8 right_foot_cap1
 Contact force on geom2 body [ -9.01652943 -24.82506081  -1.55435095  -1.83574472  -9.3855429
 106.86256552]
norm 110.50368201217177
c_array [0. 0. 0. 0. 0. 0.]
c_array [11.7790304  -0.65222272  0.55718872  0.          0.          0.        ]
contact 5
dist -0.00018000492663448192
geom1 0 floor
geom2 9 right_foot_cap2
 Contact force on geom2 body [ -9.01652943 -24.82506081  -1.55435095  -1.83574472  -9.3855429
 106.86256552]
norm 110.50368201217177
c_array [0. 0. 0. 0. 0. 0.]
c_array [31.7058012   0.47275693  4.135484    0.          0.          0.        ]
contact 6
dist -0.0005485187897493055
geom1 0 floor
geom2 9 right_foot_cap2
 Contact force on geom2 body [ -9.01652943 -24.82506081  -1.55435095  -1.83574472  -9.3855429
 106.86256552]
norm 110.50368201217177
c_array [0. 0. 0. 0. 0. 0.]
c_array [41.61838386  0.47275212  0.57461213  0.          0.          0.        ]
contact 7
dist 0.00019041860409436256
geom1 0 floor
geom2 12 left_foot_cap1
 Contact force on geom2 body [  9.01652943 -24.82506078   1.55435096  -1.83574477   9.38554253
 106.86256563]
norm 110.50368207658713
c_array [0. 0. 0. 0. 0. 0.]
c_array [21.75934996 -0.65224861 -4.11828088  0.          0.          0.        ]
contact 8
dist 0.0005627518225853959
geom1 0 floor
geom2 12 left_foot_cap1
 Contact force on geom2 body [  9.01652943 -24.82506078   1.55435096  -1.83574477   9.38554253
 106.86256563]
norm 110.50368207658713
c_array [0. 0. 0. 0. 0. 0.]
c_array [11.77902957 -0.65222263 -0.55718886  0.          0.          0.        ]
contact 9
dist -0.00018000494107106232
geom1 0 floor
geom2 13 left_foot_cap2
 Contact force on geom2 body [  9.01652943 -24.82506078   1.55435096  -1.83574477   9.38554253
 106.86256563]
norm 110.50368207658713
c_array [0. 0. 0. 0. 0. 0.]
c_array [31.7058016   0.47275692 -4.13548369  0.          0.          0.        ]
contact 10
dist -0.0005485188129107678
geom1 0 floor
geom2 13 left_foot_cap2
 Contact force on geom2 body [  9.01652943 -24.82506078   1.55435096  -1.83574477   9.38554253
 106.86256563]
norm 110.50368207658713
c_array [0. 0. 0. 0. 0. 0.]
c_array [41.6183845   0.47275211 -0.57461227  0.          0.          0.        ]
contact 11
dist 0.0006454472303879591
geom1 0 floor
geom2 14 right_uarm1
 Contact force on geom2 body [-6.75586847e+00  4.27163228e+00  2.17160654e-03  6.69983383e-01
  1.04633581e+00  2.61363163e+01]
norm 27.359446855446734
c_array [0. 0. 0. 0. 0. 0.]
c_array [26.1363163  -0.27029671 -1.21269784  0.          0.          0.        ]
contact 12
dist 0.0006454472299376249
geom1 0 floor
geom2 17 left_uarm1
 Contact force on geom2 body [ 6.75586850e+00  4.27163230e+00 -2.17157292e-03  6.69983275e-01
 -1.04633584e+00  2.61363163e+01]
norm 27.359446896045025
c_array [0. 0. 0. 0. 0. 0.]
c_array [26.13631634 -0.2702966   1.21269783  0.          0.          0.        ]
done

```

example:

``` python

import gym

def str_mj_arr(arr):
    return ' '.join(['%0.3f' % arr[i] for i in range(arr._length_)])

def print_contact_info(env):
    d = env.unwrapped.data
    for coni in range(d.ncon):
        print('  Contact %d:' % (coni,))
        con = d.obj.contact[coni]
        print('    dist     = %0.3f' % (con.dist,))
        print('    pos      = %s' % (str_mj_arr(con.pos),))
        print('    frame    = %s' % (str_mj_arr(con.frame),))
        print('    friction = %s' % (str_mj_arr(con.friction),))
        print('    dim      = %d' % (con.dim,))
        print('    geom1    = %d' % (con.geom1,))
        print('    geom2    = %d' % (con.geom2,))

def run_env(env, step_cb):
    env.reset()
    stepi = 0
    while True:
        print('Step %d:' % (stepi,))
        step_cb(env)
        obs, rew, done, info = env.step(env.action_space.sample())
        stepi += 1
        if done: break

def main():
    run_env(gym.make('HalfCheetah-v1'), print_contact_info)

if __name__ == '__main__': main()
```
