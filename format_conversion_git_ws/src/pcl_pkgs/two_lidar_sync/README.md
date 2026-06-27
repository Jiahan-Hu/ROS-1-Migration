# How to sync

Play two rosbag reparately:

Play the lidar_sync. Path: `/home/zhaoliang/zzl/zhz03_github/smart_intersection/code/ros_code/pcl_ws`

```
source devel/setup.bash
roslaunch two_lidar_sync launch_lidar_sync.launch
```

## How to run

This package is to do synchronization and transformation on two pcl.

(1) launch_lidar_sync

This is to synchronize two LiDAR topics.

```
roslaunch two_lidar_sync launch_lidar_sync.launch
```

(2) launch_lidar_sync_comb

This is to synchronize  two transformed LiDAR topics and combined them into one LiDAR topics. 

```
roslaunch two_lidar_sync launch_lidar_sync_comb.launch
```

(3) launch_lidar_sync_tf_pub_comb

This is to first synchronize two LiDAR topics, and then transform two LiDAR topics into a relative correct position and then combine these transformed topics into one final LiDAR topic: 

```
roslaunch two_lidar_sync launch_lidar_sync_tf_pub_comb.launch
```

