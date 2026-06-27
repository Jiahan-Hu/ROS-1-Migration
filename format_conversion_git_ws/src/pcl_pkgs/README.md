# pcl_pkgs

This is a ros pointcloud package comb. It contains all kinds of different pcl pkgs that I and other developers developed.

## 0 Content

- [x] line_convert(rs_ray_convert)
- [x] pcl_header_pub
- [x] Bag2PCD_ROStool
- [x] ros_transform_utils
- [x] two_lidar_sync
- [x] pcl_time_change
- [ ] gpsimu_lidar_tdiff

## 1 How to run 

### 1.1 line_convert 

This package is to convert 128 robosense LiDAR to velodyne VLC-32c.

```
roslaunch line_convert launch_convert_pc.launch
```

### 1.2 pcl_header_pub

This package is to publish pcl header as a new topic. 

```
roslaunch pcl_header_pub launch_pcl_header_pub.launc
```

### 1.3 Bag2PCD_ROStool

```
rosrun Bag2PCD_ROStool bag_to_pcd <cloud_topic> <output_directory>
```

Where: 

***cloud_topic*** is the point cloud topic to save.

***output_directory*** is the directory on disk in which to create PCD files from the point cloud messages.

### 1.4 transformation_pub

This package is to do some transformation on pcl.

**(1) tf2pry**

This is to transform rotation matrix into pitch, roll, yaw angle in degree 

```
launch transformation_pub launch_tf2pry.launch
```

**(2) launch_tf_pub**

This is to transform two Lidar topic into two transformed Lidar topics in relative right position

```
launch transformation_pub launch_tf_pub.launch
```

**(3) launch_bbx_tf_pub**

This is to transform two bbx into  into two transformed bbx topics in relative right position

```
launch transformation_pub launch_bbx_tf_pub.launch
```

### 1.5 two_lidar_sync

This package is to do synchronization and transformation on two pcl.

**(1) launch_lidar_sync**

This is to synchronize two LiDAR topics.

```
roslaunch two_lidar_sync launch_lidar_sync.launch
```

**(2) launch_lidar_sync_comb**

This is to synchronize  two transformed LiDAR topics and combined them into one LiDAR topics. 

```
roslaunch two_lidar_sync launch_lidar_sync_comb.launch
```

**(3) launch_lidar_sync_tf_pub_comb**

This is to first synchronize two LiDAR topics, and then transform two LiDAR topics into a relative correct position and then combine these transformed topics into one final LiDAR topic: 

```
roslaunch two_lidar_sync launch_lidar_sync_tf_pub_comb.launch
```

### 1.6 pcl_time_change 

This package is to change the header time of a point cloud topic. 

```
roslaunch pcl_time_change launch_time_change.launch
```

### 