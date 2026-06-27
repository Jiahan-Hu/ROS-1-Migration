#!/bin/bash

source /home/mobility/anaconda3/etc/profile.d/conda.sh

conda activate cooperfuse

sudo chmod 777 /dev/ttyUSB0
cd /home/mobility/ros_ws/smart_intersection_vehicle/python_code

python test_pyserial_get_gprmc_send_gprmc.py

