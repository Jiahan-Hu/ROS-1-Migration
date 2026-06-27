#!/usr/bin/env python
import rospy
import pytz
from datetime import datetime
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

def callback(data):
    utc_time = data.header.stamp.to_sec()
    #convert the UTC time to a datetime object
    utc_time = datetime.utcfromtimestamp(utc_time)
    #set the timezone to UTC
    utc_time = pytz.utc.localize(utc_time)
    #convert the time to PST
    pst_time = utc_time.astimezone(pytz.timezone('America/Los_Angeles'))
    #publish the PST time as a local time topic
    pub.publish(pst_time.strftime('%H:%M:%S %Z'))

if __name__ == '__main__':
    rospy.init_node('local_time_publisher')
    rospy.loginfo("Starting local_time_publisher")
    # get parameters
    GPStime_topic = rospy.get_param('~gps_time_topic','/fix/gprmc')
    pub_time_topic = rospy.get_param('~local_time_topic','local_time')
    rospy.Subscriber(GPStime_topic, NavSatFix, callback)
    pub = rospy.Publisher(pub_time_topic, String, queue_size=10)
    rospy.spin()
