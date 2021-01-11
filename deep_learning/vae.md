# VAE

## data generate

Here are the prcedures  for preparing training images:
1. cut the video into shorter pieces:
```bash
ffmpeg -ss 0:0:37 -i input.mp4 -t 0:3:20  -c copy out.mp4 
```

1. extract 1000 images from the cut video:
```bash
ffmpeg -i out.mp4 -vf fps=1000/3.33*60 %d.png
```