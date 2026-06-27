# -*- coding: utf-8 -*-
"""
Code description.
"""
# Author: Zhaoliang Zheng <zhz03@g.ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

#!/usr/bin/env python
import rospy
import subprocess
from std_msgs.msg import Float32

def calculate_network_latency():
    # Initialize the ROS node
    rospy.init_node('network_latency_node', anonymous=True)

    # Get the target host address from the ROS parameter server, default to '127.0.0.1' if not set
    target_host = rospy.get_param('~target_host', '127.0.0.1')

    # Create a publisher to publish network latency data
    latency_publisher = rospy.Publisher('network_latency', Float32, queue_size=10)

    rate = rospy.Rate(1)  # Publishing rate (1 time per second)

    while not rospy.is_shutdown():
        try:
            # Measure latency using the ping command
            ping_result = subprocess.check_output(['ping', '-c', '1', target_host])
            # Parse the ping result and extract the latency time
            lines = ping_result.split('\n')
            if len(lines) >= 2:
                latency_line = lines[1].split('=')[-1].strip().split(' ')[0]
                latency_ms = float(latency_line)
                # Publish the latency data to the ROS topic
                latency_publisher.publish(latency_ms)
                rospy.loginfo(f'Network latency: {latency_ms} ms')
            else:
                rospy.logwarn('Failed to retrieve latency data')
        except subprocess.CalledProcessError as e:
            rospy.logerr(f'Error executing the ping command: {e}')

        rate.sleep()

if __name__ == '__main__':
    try:
        calculate_network_latency()
    except rospy.ROSInterruptException:
        pass

