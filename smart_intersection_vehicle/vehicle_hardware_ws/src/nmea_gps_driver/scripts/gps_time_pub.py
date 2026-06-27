#!/usr/bin/env python
"""
This code is to subscribe the gps_time topic from originap gps time (1hz)
and publish the gps time in other hz
"""
import serial
import datetime
import string
import time
import calendar
import rospy
from sensor_msgs.msg import TimeReference 
from std_msgs.msg import Header

class gps_time_pub:
    def __init__(self):
        rospy.init_node('gps_time_hz_pub')
        # get serial port from parameter server
        self.gps_ori_topic = rospy.get_param('~ori_topic', '/gps')
        self.gps_time_rate = rospy.get_param('~gps_time_hz', 10)
        self.publish_topic = rospy.get_param('~publish_topic', '/gps_time_hz')

        rospy.Subscriber(self.gps_ori_topic, TimeReference, self.gps_time_callback)
        self.ori_gps_time = TimeReference()
        self.previous_gps_time_hz = TimeReference()
        self.previous_header = Header()
        self.previous_second = 0
        self.previous_nsec = 0

        self.gpstimeHzPub = rospy.Publisher(self.publish_topic, TimeReference, queue_size=10)

    def gps_time_callback(self, msg):
        self.ori_gps_time = msg

    def publish_gps_time_hz(self):
        """
        This function is used to publish gps time in hz based on ori_gps_time
        """

        rate = rospy.Rate(self.gps_time_rate)
        while not rospy.is_shutdown():
            gps_time_hz = TimeReference()
            gps_time_hz.header.stamp = rospy.Time.now()
            gps_time_hz.header.frame_id = "gps_time_hz"
            rospy.loginfo("-------------------------gps_time_pub_node-------------------------")
            time_diff = gps_time_hz.header.stamp - self.previous_header.stamp
            print("time_diff:",time_diff.to_nsec())
            self.previous_header = gps_time_hz.header

            ros_time_secs = self.ori_gps_time.time_ref.secs
            ros_time_nsecs = self.ori_gps_time.time_ref.nsecs
            # print("ros_time_secs:",ros_time_secs)
            # print("ros_time_nsecs:",ros_time_nsecs)

            if ros_time_secs != self.previous_second or ros_time_nsecs != self.previous_nsec:
                self.previous_second = ros_time_secs
                self.previous_nsec = ros_time_nsecs
            else:
                ros_time_nsecs = self.previous_gps_time_hz.time_ref.nsecs + time_diff.to_nsec()

            gps_time_hz.time_ref = rospy.Time(ros_time_secs,ros_time_nsecs)
            print("------")
            print("new_ros_time_second:",ros_time_secs)
            print("new_nsec:",ros_time_nsecs)
            self.previous_gps_time_hz = gps_time_hz
            self.gpstimeHzPub.publish(gps_time_hz)
            rate.sleep() 

if __name__ == '__main__':
    my_gps_time = gps_time_pub()
    rospy.loginfo("gps_time_pub_node: starting")
    my_gps_time.publish_gps_time_hz()








