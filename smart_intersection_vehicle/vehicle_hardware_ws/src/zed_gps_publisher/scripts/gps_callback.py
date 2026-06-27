#!/usr/bin/env python
import rospy
from sensor_msgs.msg import TimeReference

def time_callback(msg):
    rospy.loginfo("Received time_ref: %f", msg.time_ref.to_sec())

if __name__ == '__main__':
    rospy.init_node('gps_time_subscriber')
    sub = rospy.Subscriber('/veh_2/gps_can_time_hz10', TimeReference, time_callback)
    rospy.spin()
