#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

"""
Description: Subscribes to the lidar topic "/hardware_interface/velodyne_1/velodyne_points" 
and republishes it as a new topic "/hardware_interface/velodyne_1_front/velodyne_points" 
with the frame ID "velodyne_1_front"

Author: Zhaoliang
"""

class LidarRepublisherNode:

    def __init__(self):
        
        # Initialize ROS node
        rospy.init_node('lidar_republisher')

        # Get parameters
        lidar_topic = rospy.get_param('~lidar_topic', '/hardware_interface/velodyne_1/velodyne_points')
        republish_topic = rospy.get_param('~republish_topic', '/hardware_interface/velodyne_1_front/velodyne_points')
        self.frame_id = rospy.get_param('~new_frame_id','velodyne_1_side')

        # Set up subscribers and publishers
        self.lidar_sub = rospy.Subscriber(lidar_topic, PointCloud2, self.lidar_callback)
        self.republish_pub = rospy.Publisher(republish_topic, PointCloud2, queue_size=10)

        rospy.loginfo('Lidar republisher node initialized with lidar topic "{}" and republish topic "{}"'.format(lidar_topic, republish_topic))

    def lidar_callback(self, msg):
        # Modify the header frame ID
        msg.header.frame_id = self.frame_id

        # Republish the modified message
        self.republish_pub.publish(msg)

if __name__ == '__main__':
    try:
        node = LidarRepublisherNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

