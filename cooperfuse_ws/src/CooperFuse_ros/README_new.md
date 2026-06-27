# CooperFuse

## Prerequisite 

Install some prerequisites to use Python3 with ROS:

```
sudo apt update
sudo apt install python3-catkin-pkg-modules python3-rospkg-modules python3-empy
```

Prepare catkin workspace:

```
mkdir -p ~/cooperfuse_ws/src
cd ~/cooperfuse_ws
catkin_make
source devel/setup.bash
wstool init
wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v 0.6.5
wstool up
rosdep install --from-paths src --ignore-src -y -r
```

Finally compile for Python 3:

```
catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
```

Do not forget to always source your workspace!

Note:

- If your platform is not x86_64, don't forget to change to: 

  `-DPYTHON_LIBRARY=/usr/lib/YOUR_ARCH_NAME-linux-gnu/libpython3.6m.so`

## Installation

```
conda install -c anaconda scipy
pip install matplotlib
pip install numpy
pip install cvxopt
pip install rospkg
```

## Issues

### Fusion code

Fusion code should satisfy the following:

- Frequency of fusion topic should be same as the ego bbx topic.
- Every time only publish one same time stamp.
- Need to delete the previous marker. 
- When the second topic is gone, the fusion bbx should disappear, the ego bbx should still there.

## Bug

```
ImportError: /usr/lib/x86_64-linux-gnu/libstdc++.so.6: version `GLIBCXX_3.4.26' not found (required by /home/jiaqi/anaconda3/envs/cooperfuse/lib/python3.8/site-packages/scipy/linalg/_matfuncs_sqrtm_triu.cpython-38-x86_64-linux-gnu.so)
[get_config_node-1] process has died [pid 2010, exit code 1, cmd /home/jiaqi/ros_ws/CooperFuse_Framework/CooperFuse_ws/src/CooperFuse_ros/scripts/subscribe_topics_fusion.py __name:=get_config_node __log:=/home/jiaqi/.ros/log/8836b13c-4b63-11ee-a122-a4bb6ddc4c3c/get_config_node-1.log].
log file: /home/jiaqi/.ros/log/8836b13c-4b63-11ee-a122-a4bb6ddc4c3c/get_config_node-1*.log

```

How to solve: 

```
sudo apt-get install --only-upgrade libstdc++6
```

## Input

```
markers: 
  - 
    header: 
      seq: 0
      stamp: 
        secs: 1671230177
        nsecs: 599894762
      frame_id: "rslidar"
    ns: "objects/rslidar"
    id: 0
    type: 1
    action: 0
    pose: 
      position: 
        x: 45.4708709717
        y: 12.1758594513
        z: -1.60077482462
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.996115103104
        w: 0.0880607822357
    scale: 
      x: 5.94324159622
      y: 2.72593331337
      z: 1.15656268597
    color: 
      r: 0.0
      g: 0.0
      b: 1.0
      a: 0.5
    lifetime: 
      secs: 0
      nsecs:         0
    frame_locked: False
    points: []
    colors: []
    text: "4.23 m/s, <ID= 4>, <ds= 0.924100>, <ss= 0.0,0.0,0.0>, <kds= 0.0>"
    mesh_resource: ''
    mesh_use_embedded_materials: False
```

## Rerefence:

[1] https://answers.ros.org/question/326226/importerror-dynamic-module-does-not-define-module-export-function-pyinit__tf2/
