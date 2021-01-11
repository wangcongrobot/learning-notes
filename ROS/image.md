# ros image

How to save image sequences from a ros topic:
`http://wiki.ros.org/image_view`
```bash
rosrun image_view image_saver image:=image_raw _save_all_image:=false _filename_format:=foo.jpg __name:=image_saver
```