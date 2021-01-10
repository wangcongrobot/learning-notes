# Record some useful Python code snip

- for the begining of a python code

```python

# add parent dir to find package. Only needed for source code build, pip install doesn't need it.

import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)
```

- python multilprocessing

```python
from mpi4py import MPI

rank = MPI.COMM_WORLD.Get_rank()
if rank == 0:
    logger.configure()
else:
    logger.configure(format_strs=[])
workerseed = seed + 10000 * MPI.COMM_WORLD.Get_rank()
set_global_seeds(workerseed)

```



