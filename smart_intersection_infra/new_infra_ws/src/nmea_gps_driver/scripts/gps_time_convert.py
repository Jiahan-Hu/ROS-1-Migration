#!/usr/bin/env python
#coding:utf-8
"""
This code is to subscribe to the novatel_data/inspvax topic 
and convert the gps time to ros time stamp
"""
import rospy
from datetime import datetime, timedelta
from novatel_oem7_msgs.msg import INSPVAX
from novatel_oem7_msgs.msg import Oem7Header
from sensor_msgs.msg import TimeReference
from std_msgs.msg import Header

class time_convert:
    def __init__(self):
        rospy.init_node('time_convert')

        self.inspvax_topic = rospy.get_param('~inspvax_topic','/novatel/oem7/inspva_new')
        self.gps_can_time_topic = rospy.get_param('~gps_can_time_topic','/gps_can_time')
        self.gps_time_hz = rospy.get_param('~gps_time_hz',1)
        
        rospy.Subscriber(self.inspvax_topic, INSPVAX, self.callback)
        self.gpsCanTimePub = rospy.Publisher(self.gps_can_time_topic, TimeReference, queue_size=1)
    
        self.inspvax_data = INSPVAX()
        self.ins_nov_header = Oem7Header()
        self.previous_second = 0
        self.previous_nsec = 0
        self.previous_header = Header()
        self.previous_gps_can_time = TimeReference()
        self.LEAP_SECONDS = 18

    def callback(self, data):
        self.inspvax_data = data
        self.ins_nov_header = self.inspvax_data.nov_header
    
    def gps_week_seconds_to_utc(self,gpsweek, gpsseconds):
        leapseconds=self.LEAP_SECONDS
        datetimeformat = "%Y-%m-%d %H:%M:%S.%f"
        epoch = datetime.strptime("1980-01-06 00:00:00.000", datetimeformat)
        # timedelta function will deal with seconds when it's negative 
        elapsed = timedelta(days=(gpsweek*7), seconds=(gpsseconds-leapseconds))
        return datetime.strftime(epoch+elapsed, datetimeformat)
    
    def utc_to_ros_timestamp(self,utc_time):
        # Parse the UTC time string into a datetime object
        utc_datetime = datetime.strptime(utc_time, "%Y-%m-%d %H:%M:%S.%f")

        # Calculate the difference between the UTC time and the ROS epoch
        ros_timestamp = (utc_datetime - datetime(1970,1,1)).total_seconds()

        #return int(datetime.strptime(utc_time, "%Y-%m-%d %H:%M:%S.%f").timestamp())

        # Split the ROS timestamp into seconds and fractional seconds
        seconds, fractional_seconds = divmod(ros_timestamp, 1)

        # Convert the fractional seconds to nanoseconds
        nsec = int(fractional_seconds * 1e9)

        # Combine the seconds and nanoseconds into a ROS timestamp
        ros_timestamp = int(seconds), nsec

        return ros_timestamp
    
    def gps_to_ros_time(self):
        print("self.ins_nov_header.gps_week_number:",self.ins_nov_header.gps_week_number)
        print("self.ins_nov_header.gps_week_milliseconds:",self.ins_nov_header.gps_week_milliseconds)
        # gps_week_milliseconds format: 438331200
        gps_week_milliseconds = self.ins_nov_header.gps_week_milliseconds
        number = float(gps_week_milliseconds) / 1000
        # formatted_timestamp format: 438331.200
        formatted_string = "{:.3f}".format(number)

        utc_time = self.gps_week_seconds_to_utc(self.ins_nov_header.gps_week_number,float(formatted_string))
        ros_time_second,nsec = self.utc_to_ros_timestamp(utc_time)
        print("ros_time_second:",ros_time_second)
        print("nsec:",nsec)
        return ros_time_second,nsec

    def publish_gps_can_time1(self):
        # if ins_nov_header has data, then publish gps_can_time
        if self.ins_nov_header.gps_week_number != 0:
            gps_can_time = TimeReference()
            gps_can_time.header.stamp = rospy.Time.now()
            ros_time_second,nsec = self.gps_to_ros_time()
            gps_can_time.time_ref = rospy.Time(ros_time_second,nsec)
            self.gpsCanTimePub.publish(gps_can_time)

    def publish_gps_can_time(self):
        # if ins_nov_header has data, then publish gps_can_time
        
        if self.ins_nov_header.gps_week_number != 0:
            rospy.loginfo("-------------------------------------------------")
            gps_can_time = TimeReference()
            gps_can_time.header.stamp = rospy.Time.now()
            # calculate time difference between gps_can_time.header and self.previous_header
            time_diff = gps_can_time.header.stamp - self.previous_header.stamp
            print("time_diff:",time_diff)
            self.previous_header = gps_can_time.header

            ros_time_second,nsec = self.gps_to_ros_time()

            if ros_time_second != self.previous_second or nsec != self.previous_nsec:
                self.previous_second = ros_time_second
                self.previous_nsec = nsec
            else:
                # ros_time_second = self.previous_gps_can_time.time_ref.secs + time_diff.to_sec()
                nsec = self.previous_gps_can_time.time_ref.nsecs + time_diff.to_nsec()

            gps_can_time.time_ref = rospy.Time(ros_time_second,nsec)
            print("------")
            print("new_ros_time_second:",ros_time_second)
            print("new_nsec:",nsec)
            self.previous_gps_can_time = gps_can_time
            self.gpsCanTimePub.publish(gps_can_time)

if __name__ == '__main__':
    time_converter = time_convert()
    rate = rospy.Rate(time_converter.gps_time_hz)
    rospy.loginfo("time_convert node is running")
    print("time_converter.gps_time_hz:",time_converter.gps_time_hz)
    while not rospy.is_shutdown():
        time_converter.publish_gps_can_time()
        rate.sleep()


        
