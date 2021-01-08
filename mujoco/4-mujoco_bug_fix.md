# mujoco bugs

```bash
WARNING: Nan, Inf or huge value in QACC at DOF 4. The simulation is unstable. Time = 0.0060.
```
you can add this:
```xml
    <contact>
        <exclude body1="base" body2="link1"></exclude>
        <exclude body1="link1" body2="link2"></exclude>
        <exclude body1="link2" body2="link3"></exclude>
        <exclude body1="link3" body2="link4"></exclude>
        <exclude body1="link4" body2="link5"></exclude>
        <exclude body1="link5" body2="link6"></exclude>
        <exclude body1="link6" body2="right_hand"></exclude>
    </contact>
```
