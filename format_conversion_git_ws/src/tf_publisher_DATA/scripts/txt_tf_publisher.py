#! /usr/bin/env python
# coding: utf-8

import rospy
import tf

rospy.init_node('tf_publisher_node')

# Get the data path from ROS parameter server
data_path = rospy.get_param('~data_path', 'data.txt')  # Default: 'data.txt'

# Get the parent and child frames from ROS parameter server
parent_frame = rospy.get_param('~parent_frame', 'map')  # Default: 'map'
child_frame = rospy.get_param('~child_frame', 'veh1_rslidar')  # Default: 'veh1_rslidar'

# Create a TF broadcaster
broadcaster = tf.TransformBroadcaster()

# rate = rospy.Rate(10)  # Set the publishing rate to 10 Hz

with open(data_path, 'r') as file:
    for line in file:
        # Parse the line into individual values
        data = line.strip().split(',')
        time = float(data[0])
        x, y, z = float(data[1]), float(data[2]), float(data[3])
        roll, yaw, pitch, = float(data[4]), float(data[5]), float(data[6])

        # Set the translation and rotation
        translation = (x, y, z)
        rotation = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        # Broadcast the transformation
        broadcaster.sendTransform(
            translation,
            rotation,
            rospy.Time.from_sec(time),
            child_frame,
            parent_frame
        )
        print("---")
        print(time)

        # rate.sleep()  # Sleep to maintain the desired publishing rate
        rospy.sleep(0.1)  # Pause for 0.1 seconds (100 milliseconds)
