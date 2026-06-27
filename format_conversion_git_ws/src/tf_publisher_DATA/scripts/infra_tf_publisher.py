#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import PointCloud2

def pointcloud_callback(msg):
    child_frame = msg.header.frame_id
    
    # Create a TF broadcaster
    broadcaster = tf.TransformBroadcaster()
    
    # Set the translation and rotation
    translation = (x, y, z)
    rotation = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    
    # Broadcast the transformation
    broadcaster.sendTransform(
        translation,
        rotation,
        msg.header.stamp,
        child_frame,
        parent_frame
    )

if __name__ == "__main__":
    rospy.init_node('tf_publisher_node')
    
    rospy.loginfo("infra TF Publisher Node Started")

    parent_frame = 'map'

    # Get the fixed values from ROS parameter server
    x = rospy.get_param('~x', 29.252044126767487)
    y = rospy.get_param('~y', -157.99108715588525)
    z = rospy.get_param('~z', 2.504110691351501)
    roll = rospy.get_param('~roll', -0.02814657310165272)
    yaw = rospy.get_param('~yaw', -0.04571448047729036)
    pitch = rospy.get_param('~pitch', -0.05262715935993412)

    # Get the subscribed topic from ROS parameter server
    subscribed_topic = rospy.get_param('~subscribed_topic', '/ouster_nw/points')

    # Subscribe to the PointCloud2 topic
    rospy.Subscriber(subscribed_topic, PointCloud2, pointcloud_callback)

    # Spin until the node is shut down
    rospy.spin()
