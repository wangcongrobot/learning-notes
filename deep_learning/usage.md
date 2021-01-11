# general

- GPUs

The code can only be run on a single GPU. You can specify which GPU to use with the CUDA_VISIBLE_DEVICES environment variable:
```bash
CUDA_VISIBLE_DEVICES=2 python train.py --model_name mono_model
```