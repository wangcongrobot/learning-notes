# MuJoCo Dynamics Randomization

https://github.com/ARISE-Initiative/robosuite/blob/a3be094e419bbac25121d252a79c84286222f229/robosuite/utils/mjmod.py#L1400
```python
class PhysicalParameterModder(BaseModder):
    """
    Modder for various physical parameters of the mujoco model
    can use to modify parameters stored in MjModel (ie friction, damping, etc.) as
    well as optimizer parameters like global friction multipliers (eg solimp, solref, etc)
    To modify a parameteter, use the parameter to be changed as a keyword argument to
    self.mod and the new value as the value for that argument. Supports arbitray many
    modifications in a single step.
    :NOTE: It is necesary to perform sim.forward after performing the modification.
    :NOTE: Some parameters might not be able to be changed. users are to verify that
          after the call to forward that the parameter is indeed changed.
    Args:
        sim (MjSim): Mujoco sim instance
        random_state (RandomState): instance of np.random.RandomState, specific
            seed used to randomize these modifications without impacting other
            numpy seeds / randomizations
    """
    def __init__(self, sim, random_state=None):
        super().__init__(sim=sim, random_state=random_state)

    @property
    def opt(self):
        """
        Returns:
             ?: MjModel sim options
        """
        return self.sim.model.opt

    def __getattr__(self, name):
        try:
            opt_attr = getattr(self.opt, name)
        except AttributeError:
            opt_attr = None

        try:
            model_attr = getattr(self.model, name)
        except AttributeError:
            model_attr = None

        ret = opt_attr if opt_attr is not None else model_attr
        if callable(ret):
            def r(*args):
                return ret(*args)
            return r

        return ret

    def mod(self, **kwargs):
        """
        Method to actually mod. Assumes passing in keyword arguments with key being the parameter to
        modify and the value being the value to set
        Feel free to add more as we see fit.
        Args:
            **kwargs (dict): Physical parameters to actually modify mid-sim
        """
        for to_mod in kwargs:
            val = kwargs[to_mod]

            param = to_mod
            ind = None
            if 'geom_friction' in param:
                joint = param.replace('_geom_friction', '')
                ind = self.geom_name2id(joint)
                param = 'geom_friction'
            elif 'dof_damping' in param:
                joint = param.replace('_dof_damping', '')
                param = 'dof_damping'
                joint = self.joint_name2id(joint)
                ind = np.zeros(self.nv)

                for i in range(self.model.nv):
                    if self.dof_jntid[i] == joint:
                        ind[i] = 1

            if ind is None:
                setattr(self, param, val)
            else:
                self.__getattr__(param)[ind] = val
```

https://github.com/ARISE-Initiative/robosuite/pull/90

- Basic Domain Randomization functionality.
    - This includes texture, lighting, material, and camera randomization.
- Add "name" property when constructing generated objects.
- Add textures to robots and grippers for texture randomization.
- Relevant files to look at:
    - scripts/demo_domain_randomization.py
    - wrappers/domain_randomization_wrapper.py
    - utils/mjmod.py
- TODOs
    - Add dynamics randomization wrapper (we might want a separate PR for this)
    - Support for sensor delay / repeat / dropout (separate PR / wrapper?)
    - Support action noise (separate PR)