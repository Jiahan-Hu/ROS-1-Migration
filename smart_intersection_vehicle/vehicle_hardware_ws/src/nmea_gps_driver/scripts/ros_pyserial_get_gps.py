#!/usr/bin/env python

import serial
import datetime
import string
import time
import calendar
import rospy
from sensor_msgs.msg import TimeReference 
from std_msgs.msg import Header

class pyse_gps:
    def __init__(self):
        rospy.init_node('ucla_gps_driver')
        # get serial port from parameter server
        self.myport = rospy.get_param('~port', '/dev/ttyUSB0')
        self.mybaud = rospy.get_param('~baud', 9600)
        self.mytopic = rospy.get_param('~publish_topic', 'gps')

        self.ser = serial.Serial(self.myport, baudrate=self.mybaud)
        self.ser.flushInput()
        self.ser.flushOutput()
        self.idx = 0

        self.nmea_data = b""
        self.gpstime = TimeReference()

        # skip first line, since it could be incomplete
        self.ser.readline()

        self.gpstimePub = rospy.Publisher(self.mytopic, TimeReference, queue_size=10)

    def convertNMEATimeToROS(self,nmea_utc,nmea_utc_date):
        #Get current time in UTC for date information
        utc_struct = time.gmtime() #immutable, so cannot modify this one
        utc_list = list(utc_struct)
        hours = int(nmea_utc[0:2])
        minutes = int(nmea_utc[2:4])
        seconds = int(nmea_utc[4:6])
        day = int(nmea_utc_date[0:2])
        utc_list[2] = day
        utc_list[3] = hours
        utc_list[4] = minutes
        utc_list[5] = seconds

        print(utc_list)
        unix_time = calendar.timegm(tuple(utc_list))
        return rospy.Time.from_sec(unix_time),unix_time
    
    def publish_gps_time_ori(self):
        """
        This function is used to publish the original GPS time (1 hz)
        """
        fix_gps_rate = 1
        rate = rospy.Rate(fix_gps_rate)
        while not rospy.is_shutdown():
            self.idx += 1
            timeNow = rospy.get_rostime()
            self.nmea_sentence = self.ser.readline()
            if "GPRMC" in (self.nmea_sentence) or "GNRMC" in (self.nmea_sentence):
                rospy.loginfo("====================ucla_gps_driver_node: get gps time====================")
                gprmc_data_plain = self.nmea_sentence
                print(gprmc_data_plain)

                fields = gprmc_data_plain.split(',')
                gprmc_hms = fields[1]
                gprmc_date = fields[9]

                ros_unix_time,unix_time = self.convertNMEATimeToROS(gprmc_hms,gprmc_date)
                your_date = datetime.datetime.fromtimestamp(unix_time)
                # print("gprmc_hms:",gprmc_hms)
                # print("gprmc_date:",gprmc_date)
                print(your_date)
                # print("idx:=",self.idx)
                
                self.gpstime.header.stamp = timeNow
                self.gpstime.header.frame_id = "gps_ori"
                self.gpstime.time_ref = ros_unix_time
                self.gpstimePub.publish(self.gpstime)
                rate.sleep()
                
            
if __name__ == '__main__':
    my_pyse_gps = pyse_gps()
    rospy.loginfo("ucla_gps_driver_node: starting")
    my_pyse_gps.publish_gps_time_ori()







