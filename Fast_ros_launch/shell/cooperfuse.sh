#!/bin/bash

source /home/mobility/anaconda3/etc/profile.d/conda.sh

conda activate cooperfuse

cd /home/mobility/ros_ws/cooperfuse_ws

source devel/setup.bash

roslaunch cooperfuse_ros test_tag2_subscribe_topics.launch

