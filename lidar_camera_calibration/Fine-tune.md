# Fine-tune process

You need to launch the following code one by one:
## 1 lidar_camera_calibration_gui

You need to do some preparation:

- Install the conda virtual environment: `conda env create -f environment.yml `
- Activate your virtual environment first. 

To launch the code:

```shell
rosrun lidar_camera_calibration_gui gui2pryxyz_all.py 
```

Before you launch the code:

- Navigate to ` [gui2pryxyz_all.py](lidar_cam_ws/src/lidar_camera_calibration_gui/script/gui2pryxyz_all.py) `

- Change the initial `pitch,roll,yaw,x,y,z` based on the previous rough calibration:

  ```python
          # infra 214_final
          self.pitch=4.499569999999989
          self.roll=-95.55200000000002
          self.yaw=-101.88409999999966
          self.x=-0.9807144999999998
          self.y=-0.5750050000000002
          self.z=5.112746999999997
  ```

## 2 Image conversion

You need to convert the image from compress to image raw:

```shell
roslaunch img_convert launch_compress2image_nw.launch
```

Before you launch, you need to check the launch file

- Location: ` [launch_compress2image_nw.launch](lidar_cam_ws/src/img_convert/launch/launch_compress2image_nw.launch) `

- You need to change the following code based on the real situations:

  ```xml
    <arg name="compressed_topic" default="/axis214/image_raw/compressed"/>
    <arg name="image_topic" default="/axis214_new/image_raw"/>
  ```

## 3 Change camera info topic

In some cases, the camera information is not correct, therefore, we need to change/republish the camera information: 

```shell
roslaunch camera_info_publisher launch_cam_info_repub.launch
```

Before you launch the code, you need to check the change the launch file:

- Location: 

- Change the following topics based on the real situations

  ```xml
      <param name="~cam_info_ori_topic" value="/axis214/camera_info" />
      <param name="~cam_info_pub_topic" value="/axis214_new/camera_info" />
  ```

## 4 Cross check result

You can use the cross_check package to cross check the results:

```shell
roslaunch calibration_publisher cross_check_mod.launch
```

Before you launch, you need to check and modify the launch file:

- Location: ` [cross_check_mod.launch](lidar_cam_ws/src/lidar_camera_calibrator/calibration_publisher/launch/cross_check_mod.launch) `

- You need to check and modify the following code:

  ```xml
      <arg name="intrinsics_file" default="/media/zhaoliang/Elements/calibration_related/infra_214_ext/214_solvePnPRansac_autoware_lidar_camera_calibration.yaml" />
      <arg name="target_frame" default="os_sensor"/>
      <arg name="camera_frame" default="axis_camera"/>
      <arg name="camera_id" default="/" />
      <arg name="image_topic_src_ori" default = "/axis214_new/image_raw"/>
      <arg name="pc_topic" default="/ouster_nw/points" />
  ```

  
