# lidar_camera_sync

This repo is for synchronize the Lidar data and camera data online in ROS.

## How to use

If your camera data comes with `camera_info` topic, then use:

```
roslaunch lidar_camera_sync launch_lidar_cam_info_sync.launch
```

If camera data comes without `camera_info`, then use:

```
roslaunch lidar_camera_sync launch_lidar_cam_sync.launch
```

