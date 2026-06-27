#!/usr/bin/env python

import rospy
from std_msgs.msg import Header

def callback1(header1):
    global header1_time
    header1_time = header1
def callback2(header2):
    global header1_time
    time_diff = header2.stamp - header1_time.stamp
    rospy.loginfo("Time difference is: %f seconds" % time_diff.to_sec())

if __name__ == '__main__':
    global header1_time
    rospy.init_node('header_time_diff_node')
    
    header1_sub = rospy.Subscriber('/rslidar_header', Header, callback1)
    header2_sub = rospy.Subscriber('/os_header', Header, callback2)

    rospy.spin()
