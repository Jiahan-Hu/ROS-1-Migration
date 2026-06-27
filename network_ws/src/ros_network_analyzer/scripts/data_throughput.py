#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ROS Node to monitor and calculate data transfer rates on specified ROS topics.
"""
# Author: Zhaoliang Zheng <zhz03@g.ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import rospy
import sys
from std_msgs.msg import Header
from ros_network_analyzer.msg import NetworkDataThroughput, Throughput  # Replace with your actual package name

class DataTransferMonitor:
    def __init__(self):
        rospy.init_node('data_transfer_monitor', anonymous=True)

        # Get configuration from ROS parameter server
        self.target_host = rospy.get_param('~target_host', '127.0.0.1')
        monitored_topics = rospy.get_param('~monitored_topics', [])

        self.topic_data_sizes = {topic: 0 for topic in monitored_topics}
        self.topic_subscribers = {}

        # Create a subscriber for each topic
        for topic in monitored_topics:
            self.topic_subscribers[topic] = rospy.Subscriber(topic, rospy.AnyMsg, self.topic_callback, callback_args=topic)

        # Create a publisher to publish data transfer rates
        self.rate_publisher = rospy.Publisher('data_transfer_rates', NetworkDataThroughput, queue_size=10)

        self.rate = rospy.Rate(1)  # 1 Hz

    def topic_callback(self, data, topic):
        # Increment the data size counter for this topic
        self.topic_data_sizes[topic] += sys.getsizeof(data)

    def publish_transfer_rates(self):
        while not rospy.is_shutdown():
            header = Header()
            header.stamp = rospy.Time.now()

            throughput_list = []
            for topic, size in self.topic_data_sizes.items():
                throughput_msg = Throughput()
                throughput_msg.topic = topic
                throughput_msg.throughput = size
                throughput_list.append(throughput_msg)
                self.topic_data_sizes[topic] = 0  # Reset for the next interval

            network_data_throughput_msg = NetworkDataThroughput()
            network_data_throughput_msg.header = header
            network_data_throughput_msg.ip = self.target_host
            network_data_throughput_msg.throughput = throughput_list

            self.rate_publisher.publish(network_data_throughput_msg)

            rospy.loginfo(f"Published data throughput for {len(throughput_list)} topics.")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        monitor = DataTransferMonitor()
        monitor.publish_transfer_rates()
    except rospy.ROSInterruptException:
        pass

