# README

Author: Zhaoliang

## How to use

- launch line_convert

  ```
  roslaunch line_convert launch_convert_pc.launch
  ```

- launch ndt_matching

  ```
  roslaunch lidar_localizer ndt_matching_dataset.launch 
  ```

- launch rosbag play at 0.1 rate:

  - ```
    rosbag play --pause -r 0.1 2023-04-07-14-30-37_0.bag
    ```

  - You can also play all rosbag together

- Specify initial point on the map

## Modify ndt_matching

change `line 2031`: 

```
{
  //std::cout << "start" << std::endl;
  if (SAVEDATA)
  {
    outdata_.open("/home/letiangao18/Bags/results and analysis/ForMatlabNDTsavedData.txt", std::ios::app | std::ios::out);
  }
  if (SAVEDATA_datasetprocess)
  {
    outdata_datasetprocessing_.open("/home/jiaqi/catkin_ws_glt/202411.txt", std::ios::app | std::ios::out);
  }
  std::cout << "At first line previous_pose_x:"<<previous_pose.x<<";previous_pose_yaw:"<<previous_pose.yaw<<std::endl;
```

- change `"/home/jiaqi/catkin_ws_glt/202411.txt"` to whatever path you want to change

change `max_iter` to modify its accuracy:

```
// Default values
static int max_iter = 30;        // Maximum iterations
```

## Output file

Here is your output file in txt:

```
1680907312.102469000,0,0,0,0,0,0,0,0,0
1680907312.202471000,335.88,-399.44,0.88093,0.0072169,-1.7867,-0.012795,224.05,32,1376
1680907312.302472000,429.41,-448.28,-2.943,0.03729,1.3975,-0.001609,16.865,32,354.06
1680907312.402472000,428.16,-448.16,-8.9381,0.11657,1.4051,-0.007803,5.6928,32,436.56
1680907312.502472000,427.4,-447.73,-12.98,0.06709,1.425,-0.021652,0.97716,32,673.67
1680907312.602473000,427.34,-447.62,-14.021,0.03409,1.4509,-0.034227,1.2721,32,532.12
1680907312.702467000,429.39,-448.3,-12.97,0.061157,1.4922,-0.022156,0.18166,32,717.3
1680907312.802469000,429.44,-448.19,-12.951,0.057366,1.5124,-0.018852,0.25173,32,727.27
1680907312.902462000,429.55,-447.49,-12.928,0.057065,1.4976,-0.02186,0.13224,20,548.9
1680907313.002471000,429.63,-447.04,-12.912,0.055487,1.4974,-0.023476,0.13662,7,214.49
1680907313.102471000,429.7,-446.6,-12.9,0.0561,1.4985,-0.025003,0.13971,2,91.385
1680907313.202468000,429.76,-446.16,-12.899,0.05708,1.4999,-0.026126,0.14087,2,89.725
```

- To verify the output is correct, use the last-third value as the indicator: 
  - For example, second line: `224.05`, this is the similarity value of localization matching 
  - when this value is small enough, for example, line 5: `0.97716`

## NOTE

- Even your play rate is at 0.1, there could also have some rate that is missing. So you will need to check all these by yourself.
- After you specify your initial points, the txt output will start. 
