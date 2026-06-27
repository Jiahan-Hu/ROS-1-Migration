#!/usr/bin/env python

import serial
import datetime
import string
import time
import calendar
import rospy
from sensor_msgs.msg import TimeReference 

def convertNMEATimeToROS(nmea_utc,nmea_utc_date):
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

def convertNMEATime(nmeatime_utc):
    hours = int(nmeatime_utc[0:2])
    minutes = int(nmeatime_utc[2:4])
    seconds = int(nmeatime_utc[4:6])
    return [hours,minutes,seconds]

def convertNMEADate(nmeadate_utc):
    day = int(nmeadate_utc[0:2])
    month = int(nmeadate_utc[2:4])
    year = int(nmeadate_utc[4:6])

def utc_to_local(utc_data,utc_time):
    tz_utc = pytz.timezone('UTC')
    tz_la = pytz.timezone('America/Los_Angeles')
    return tz_utc.localize(datetime.datetime.strptime(str(utc_data)+str(utc_time),'%Y-%m-%d%H:%M:%S')).astimezone(tz_la).strftime("%Y-%m-%d %H:%M:%S")

if __name__ == '__main__':
    rospy.init_node('ucla_gps_driver')

    # get serial port from parameter server
    myport = rospy.get_param('~port', '/dev/ttyUSB0')
    mybaud = rospy.get_param('~baud', 9600)
    myhz = rospy.get_param('~hz', 1)
    mytopic = rospy.get_param('~topic', 'gps')

    ser = serial.Serial(myport, baudrate=mybaud)
    ser.flushInput()
    ser.flushOutput()
    idx = 0

    nmea_data = b""
    gpstime = TimeReference()


    # skip first line, since it could be incomplete
    ser.readline()

    gpstimePub = rospy.Publisher(mytopic, TimeReference, queue_size=1)
    rate = rospy.Rate(myhz)
    while not rospy.is_shutdown():
        idx += 1
        timeNow = rospy.get_rostime()
        nmea_sentence = ser.readline()

        # print("dix",idx)
        # print(nmea_sentence)
        # if its is in python2
        # nmea_sentence = str(nmea_sentence).split('\'')
        # nmea_sentence = nmea_sentence[1].split('\\')
        # nmea_sentence = nmea_sentence[0]
        # print(nmea_sentence)
        if "GPRMC" in (nmea_sentence) or "GNRMC" in (nmea_sentence):
            gprmc_data_plain = nmea_sentence
            print(gprmc_data_plain)
            # gprmc_data = pynmea2.parse(nmea_sentence)
            # print("gprmc_hms:",gprmc_data.datestamp)
            # print("gprmc_date:",gprmc_data.timestamp)
            # datetimestamp = utc_to_local(gprmc_data.datestamp,gprmc_data.timestamp)
            # print(datetimestamp)
            # nmea_data += nmea_sentence

            fields = gprmc_data_plain.split(',')
            gprmc_hms = fields[1]
            gprmc_date = fields[9]

            ros_unix_time,unix_time = convertNMEATimeToROS(gprmc_hms,gprmc_date)
            your_date = datetime.datetime.fromtimestamp(unix_time)
            print("gprmc_hms:",gprmc_hms)
            print("gprmc_date:",gprmc_date)
            print(your_date)
            print("--------------------------")
            gpstime.header.stamp = timeNow
            gpstime.time_ref = ros_unix_time
            gpstimePub.publish(gpstime)
            rate.sleep()






