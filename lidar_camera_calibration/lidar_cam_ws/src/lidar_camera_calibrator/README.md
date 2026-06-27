# Lidar_camera_calibrator

# Compile the packages
```
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

```
catkin_make
```



# Run the package
```
roslaunch autoware_camera_lidar_calibrator camera_lidar_calibration.launch intrinsics_file:=/home/zhaoliang/zzl/zhz03_github/smart_intersection/code/ros_code/lidar_cam_ws/src/lidar_camera_calibrator/calibration_publisher/example_yaml_file/ost.yaml
```



# Cross check the calibration results
```
roslaunch calibration_publisher cross_check.launch
```

In the `cross_check.launch`:

```xml
<?xml version="1.0"?>
<!--
roslaunch autoware_camera_lidar_calibrator camera_lidar_calibration.launch intrinsics_file:=/home/ne0/Desktop/calib_heat_camera1_rear_center_fisheye.yaml compressed_stream:=True camera_id:=camera1
-->
<launch>

    <arg name="intrinsics_file" default="/media/jiaqi/T7 Shield1/calib_all/calibration-fine-tune/yaml/zzl/right_solvePnPRansac_cam.yaml" />
    <arg name="target_frame" default="veh2_rslidar"/>
    <arg name="camera_frame" default="zed2i_v2_left_camera_optical_frame"/>
    <arg name="camera_id" default="/" />

    <node pkg="calibration_publisher" type="calibration_publisher" name="calibration_publisher" ns="$(arg camera_id)" output="screen">
        <param name="register_lidar2camera_tf" type="bool" value="true"/>
        <param name="publish_extrinsic_mat" type="bool" value="true"/>
        <param name="publish_camera_info" type="bool" value="true"/>
        <param name="image_topic_src" value="/zed2i_v2/zed_node1/left_update_sync/image_rect_color"/>   
        <param name="calibration_file" value="$(arg intrinsics_file)"/>
        <param name="target_frame" type="str" value="$(arg target_frame)"/>
        <param name="camera_frame" type="str" value="$(arg camera_frame)"/>
    </node>

    <node pkg="pcd_republisher" type="pcd_republisher" name="pcd_republisher" output="screen">
        <param name="image_raw_topic_str" type="str" value="/zed2i_v2/zed_node1/left_update_sync/image_rect_color"/>
        <param name="point_cloud_topic_name_str" type="str" value="/veh_2/sync/rslidar_points"/>
        <param name="point_cloud_output_topic_name_str" type="str" value="/newrslidar_repub"/>
        <param name="lidar_new_frame_id_str" type="str" value="newrslidar_repub"/>
        <param name="point_cloud_transformed_topic_name_str" type="str" value="/new/rslidar_points_transformed"/>
        <param name="which_camera" type="int" value="1"/>
        <param name="cross_check" type="int" value="1"/>
    </node>

    <node pkg="rviz" type="rviz" name="cross_check_rviz" args="-d  $(find calibration_publisher)/rviz/cross_check.rviz">

    </node>

</launch>
```

