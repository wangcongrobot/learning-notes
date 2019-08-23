
# render mujoco_py

There is an altenetial option to render, with user' implement.

https://blog.csdn.net/jianghao_ava/article/details/80874254

https://github.com/openai/mujoco-py/issues/44


```python
#!/usr/bin/env python3
"""
Shows how to use render callback.
"""
from mujoco_py import load_model_from_path, MjSim, MjViewer
from mujoco_py.modder import TextureModder
import os

modder = None
def render_callback(sim, viewer):
    global modder
    if modder is None:
        modder = TextureModder(sim)
    for name in sim.model.geom_names:
        modder.rand_all(name)

model = load_model_from_path("xmls/fetch/main.xml")
sim = MjSim(model, render_callback=render_callback)

viewer = MjViewer(sim)

t = 0

while True:
    viewer.render()
    t += 1
    if t > 100 and os.getenv('TESTING') is not None:
        break
```

# mujoco

```python

#!/usr/bin/env python3
"""
Example for how to modifying the MuJoCo qpos during execution.
"""

import os
from mujoco_py import load_model_from_xml, MjSim, MjViewer

model = load_model_from_xml(MODEL_XML)
sim = MjSim(model)
viewer = MjViewer(sim)

states = [{'box:x': +0.8, 'box:y': +0.8},
          {'box:x': -0.8, 'box:y': +0.8},
          {'box:x': -0.8, 'box:y': -0.8},
          {'box:x': +0.8, 'box:y': -0.8},
          {'box:x': +0.0, 'box:y': +0.0}]

# MjModel.joint_name2id returns the index of a joint in
# MjData.qpos.
x_joint_i = sim.model.get_joint_qpos_addr("box:x")
y_joint_i = sim.model.get_joint_qpos_addr("box:y")

print_box_xpos(sim)

while True:
    for state in states:
        sim_state = sim.get_state()
        sim_state.qpos[x_joint_i] = state["box:x"]
        sim_state.qpos[y_joint_i] = state["box:y"]
        sim.set_state(sim_state)
        sim.forward()
        print("updated state to", state)
        print_box_xpos(sim)
        viewer.render()

    if os.getenv('TESTING') is not None:
        break
```