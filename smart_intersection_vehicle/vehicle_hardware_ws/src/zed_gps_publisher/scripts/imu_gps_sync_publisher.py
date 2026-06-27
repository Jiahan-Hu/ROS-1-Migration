#!/usr/bin/env python

import rospy
from sensor_msgs.msg import TimeReference, Imu

# Initialize the global variable for time_ref
time_ref_global = rospy.Time()

# Define the callback function for the "/veh_2/gps_can_time_hz10" topic
def time_ref_callback(msg):
    global time_ref_global
    time_ref_global = msg.time_ref

# Define the callback function for the "/veh_2/gps/imu" topic
def imu_callback(msg):
    global time_ref_global

    # Create a new Imu message and set its header.stamp to the current time_ref
    # imu_msg = Imu()
    # imu_msg.header.stamp = time_ref_global

    msg.header.stamp = time_ref_global

    # Copy the remaining fields from the input message to the output message
    # imu_msg.header.frame_id = msg.header.frame_id
    # imu_msg.orientation = msg.orientation
    # imu_msg.orientation_covariance = msg.orientation_covariance
    # imu_msg.angular_velocity = msg.angular_velocity
    # imu_msg.angular_velocity_covariance = msg.angular_velocity_covariance
    # imu_msg.linear_acceleration = msg.linear_acceleration
    # imu_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance

    # Publish the new Imu message to the "/veh_2/gps_sync/imu" topic
    # imu_pub.publish(imu_msg)
    imu_pub.publish(msg)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('gps_sync_node')

    # get the parameters "/veh_2/gps_can_time_hz100" and "/veh_2/gps/imu" from the launch file
    time_ref_topic = rospy.get_param('~time_ref_topic', '/gps_can_time_hz100')
    gps_topic = rospy.get_param('~gps_imu_topic', '/gps/imu')
    output_imu_topic = rospy.get_param('~output_imu_topic', '/gps_sync/imu')

    # Subscribe to the "/veh_2/gps_can_time_hz100" topic and set the callback function
    rospy.Subscriber(time_ref_topic, TimeReference, time_ref_callback)

    # Subscribe to the "/veh_2/gps/imu" topic and set the callback function
    rospy.Subscriber(gps_topic, Imu, imu_callback)

    # Create a publisher for the "/veh_2/gps_sync/imu" topic
    imu_pub = rospy.Publisher(output_imu_topic, Imu, queue_size=10)

    # Enter the main loop
    rate = rospy.Rate(100) # 100 Hz
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()

