#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Code description.
"""
# Author: Zhaoliang Zheng <zhz03@g.ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import rospy
import subprocess
from ros_network_analyzer.msg import NetworkDelay  # Replace with your actual package name

def calculate_network_latency():
    # Initialize the ROS node
    rospy.init_node('network_latency_node', anonymous=True)

    # Get the target host address and topic name from the ROS parameter server
    target_host = rospy.get_param('~target_host', '127.0.0.1')
    topic_name = rospy.get_param('~network_delay_topic', 'network_delay')  # Default topic name

    # Create a publisher to publish network delay data
    delay_publisher = rospy.Publisher(topic_name, NetworkDelay, queue_size=10)

    rate = rospy.Rate(1)  # Publishing rate (1 time per second)

    while not rospy.is_shutdown():
        network_delay_msg = NetworkDelay()
        network_delay_msg.ip = target_host  # Set the IP address

        try:
            # Measure latency using the ping command
            ping_result = subprocess.check_output(['ping', '-c', '1', target_host])
            # print(ping_result)
            # Decode the binary data to a string
            ping_result_str = ping_result.decode('utf-8')
            # Parse the ping result string and extract the latency time
            lines = ping_result_str.split('\n')
            if len(lines) >= 2:
                latency_line = lines[1].split('=')[-1].strip().split(' ')[0]
                network_delay_msg.delay = float(latency_line)
                network_delay_msg.alive = True
                # Assign the current ROS time to the header timestamp
                network_delay_msg.header.stamp = rospy.Time.now()
                # Publish the network delay data to the ROS topic
                delay_publisher.publish(network_delay_msg)
                rospy.loginfo(f'Network delay: {network_delay_msg.delay} ms')
            else:
                rospy.logwarn('Failed to retrieve delay data')
                network_delay_msg.alive = False

        except subprocess.CalledProcessError as e:
            rospy.logerr(f'Error executing the ping command: {e}')
            network_delay_msg.alive = False

        rate.sleep()

if __name__ == '__main__':
    try:
        calculate_network_latency()
    except rospy.ROSInterruptException:
        pass

