#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo

class CameraInfoPublisher:
    def __init__(self):
        self.which_camera_topic = rospy.get_param('~which_camera_topic', '/camera/which_camera')
        if self.which_camera_topic == 'image_raw':
            self.image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
            rospy.loginfo("Using raw image topic: %s", self.image_topic)
        else: # compressed
            self.image_topic = rospy.get_param('~image_compressed_topic', '/camera/compressed')
            rospy.loginfo("Using compressed image topic: %s", self.image_topic)

        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/camera_info')

        self.camera_info_pub = rospy.Publisher(self.camera_info_topic, CameraInfo, queue_size=1)

        self.camera_info = CameraInfo()
        self._initialize_camera_info()

        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)

    def _initialize_camera_info(self):
        self.camera_info.height = 1080
        self.camera_info.width = 1920
        self.camera_info.distortion_model = "plumb_bob"
        self.camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info.K = [1054.3284912109375, 0.0, 996.54345703125, 0.0, 1054.3284912109375, 539.5215454101562, 0.0, 0.0, 1.0]
        self.camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.camera_info.P = [1054.3284912109375, 0.0, 996.54345703125, 0.0, 0.0, 1054.3284912109375, 539.5215454101562, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info.binning_x = 0
        self.camera_info.binning_y = 0
        self.camera_info.roi.x_offset = 0
        self.camera_info.roi.y_offset = 0
        self.camera_info.roi.height = 0
        self.camera_info.roi.width = 0
        self.camera_info.roi.do_rectify = False

    def image_callback(self, msg):
        self.camera_info.header = msg.header
        self.camera_info_pub.publish(self.camera_info)

if __name__ == "__main__":
    rospy.init_node('camera_info_node')
    camera_info_publisher = CameraInfoPublisher()
    rospy.spin()
