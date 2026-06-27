# lidar_camera_calibration

## How to use

### Camera intrinsic parameters calibration

```
rosrun camera_calibration cameracalibrator.py --size 10x7 --square 0.1 image:=/cam4/camera1/image_raw
```

where,

- `--size 10x7` Indicates that the checkerboard is 10 columns and 7 rows according to the corner points (where the two black blocks meet).
- `--square 0.1` Indicates that the side length of each black and white block is 10cm.
- `image:=/cam4/camera1/image_raw`  setup the rgb image topic



### lidar_camera_sync

```
roslaunch lidar_camera_sync launch_lidar_cam_info_sync.launch
```

### img_transport

```
roslaunch img_convert launch_compress2image.launch
```

### pcl_time_change 

```
roslaunch pcl_time_change launch_time_change.launch
```

### Combined process

```
roslaunch lidar_camera_post_ft launch_post_process.launch
```

### calibration_publisher

lidar_camera_calibrator/calibration_publisher

```
roslaunch calibration_publisher cross_check_mod.launch
```

### lidar_camera_calibration_gui

```
rosrun lidar_camera_calibration_gui gui2pryxyz_all.py 
```

### change camera info topic

```
roslaunch camera_info_publisher launch_cam_info_repub.launch
```

### Lidar_camera_calibrator

```
roslaunch autoware_camera_lidar_calibrator camera_lidar_calibration_mod.launch intrinsics_file:=/home/zhaoliang/zzl/zhz03_github/smart_intersection/code/ros_code/lidar_cam_ws/src/lidar_camera_calibrator/calibration_publisher/example_yaml_file/ost.yaml
```

