# Software install

## Nvidia

- [Why `torch.cuda.is_available()` returns False even after installing pytorch with cuda?](https://stackoverflow.com/questions/60987997/why-torch-cuda-is-available-returns-false-even-after-installing-pytorch-with)

The easiest way to check if PyTorch supports your compute capability is to install the desired version of PyTorch with CUDA support and run the following from a python interpreter

```python
>>> import torch
>>> torch.zeros(1).cuda()
```

If you get an error message which reads

```
Found GPU0 XXXXX which is of cuda capability #.#.
PyTorch no longer supports this GPU because it is too old.
```

then that means PyTorch was not compiled with support for your compute capability. If this runs without issue then you should be good to go.
