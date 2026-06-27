#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sys

class PointCloudProcessor:
    def __init__(self):
        # Initialize the node
        rospy.init_node('point_cloud_processor', anonymous=True)

        # Set up the subscriber and publisher
        self.subscriber = rospy.Subscriber('/input_point_cloud', PointCloud2, self.callback)
        self.publisher = rospy.Publisher('/output_point_cloud', PointCloud2, queue_size=10)

    def callback(self, data):
        # Create a new PointCloud2 message
        new_point_cloud = PointCloud2()

        # Subtract 100 ms from the header timestamp
        new_time = data.header.stamp - rospy.Duration(0.1)

        # Update the new PointCloud2 message with the modified header and original data
        new_point_cloud.header = Header(data.header.seq, new_time, data.header.frame_id)
        new_point_cloud.height = data.height
        new_point_cloud.width = data.width
        new_point_cloud.fields = data.fields
        new_point_cloud.is_bigendian = data.is_bigendian
        new_point_cloud.point_step = data.point_step
        new_point_cloud.row_step = data.row_step
        new_point_cloud.data = data.data
        new_point_cloud.is_dense = data.is_dense

        # Publish the modified point cloud
        self.publisher.publish(new_point_cloud)

def main(args):
    try:
        pc_processor = PointCloudProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if _name_ == '_main_':
    main(sys.argv)