#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo

class CameraInfoRepublisher:
    def __init__(self):
        self.camera_info_ori = rospy.get_param('~cam_info_ori_topic', '/camera/image_raw')

        self.camera_info_topic = rospy.get_param('~cam_info_pub_topic', '/camera/image_raw')
        self.camera_info_pub = rospy.Publisher(self.camera_info_topic, CameraInfo, queue_size=1)

        self.camera_info = CameraInfo()
        self.camera_info_sub = rospy.Subscriber(self.camera_info_ori,CameraInfo, self.camera_info_callback5)

    def camera_info_callback(self, msg):
        rospy.loginfo('camera_info_callback')
        self.camera_info = msg
        self.camera_info.height = 772
        self.camera_info.width = 1024
        self.camera_info.roi.do_rectify = False
        self.camera_info_pub.publish(self.camera_info)
    
    def camera_info_callback2(self, msg):
        rospy.loginfo('camera_info_callback2')
        self.camera_info = msg
        self.camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info_pub.publish(self.camera_info)

    def camera_info_callback3(self, msg):
        rospy.loginfo('camera_info_callback3')
        self.camera_info = msg
        # self.camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info_pub.publish(self.camera_info)

    def camera_info_callback4(self,msg):
        rospy.loginfo('camera_info_callback4')
        self.camera_info = msg
        self.camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info.height = 772
        self.camera_info.width = 1024
        self.camera_info.roi.do_rectify = False
        self.camera_info_pub.publish(self.camera_info)  

    def camera_info_callback5(self,msg):
        rospy.loginfo('camera_info_callback5')
        self.camera_info = msg
        self.camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info.K = [1221.6199, 0.0, 961.75906, 0.0, 1223.04251, 554.60495, 0.0, 0.0, 1.0]
        self.camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.camera_info.P = [1221.7135, 0.0, 959.56667, 0.0, 0.0, 1221.35864, 559.53674, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info.height = 1080
        self.camera_info.width = 1920
        self.camera_info.roi.do_rectify = False
        self.camera_info_pub.publish(self.camera_info)                

if __name__ == "__main__":
    rospy.init_node('camera_info_repub_node')
    camera_info_republisher = CameraInfoRepublisher()
    rospy.spin()

        