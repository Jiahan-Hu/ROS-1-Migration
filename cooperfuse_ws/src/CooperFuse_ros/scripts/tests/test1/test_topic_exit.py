#! /usr/bin/env python
# coding: utf-8

import rospy
from visualization_msgs.msg import Marker,MarkerArray

def callback1(msg):
    print("Received data from topic.")

def subscribe_to_topic(topic_name):
    # Check if there are any connections to the topic
    published_topics = rospy.get_published_topics()
    if any(topic == topic_name for topic, _ in published_topics):
        print(f"Subscribed to topic {topic_name}.")
        return True
    else:
        print(f"Topic {topic_name} does not exist.")
        return False


def main():
    # Initialize the ROS node
    rospy.init_node('test_topic_exit', anonymous=True)

    # Get the tracking parameter dictionary
    fusion_params = rospy.get_param('~v2xfusion')

    # Extract individual parameters
    sub_ego_bbx_topic = fusion_params['sub_ego_bbx']
    sub_agent1_bbx_topic = fusion_params['sub_agent1_bbx']
    pub_fusion_bbx_topic = fusion_params['pub_fusion_bbx']

    # Print the extracted parameters
    rospy.loginfo("config parameters are as follows:")
    rospy.loginfo(f"sub_infra1_bbx_topic: {sub_ego_bbx_topic}")
    rospy.loginfo(f"sub_veh1_bbx_topic: {sub_agent1_bbx_topic}")
    rospy.loginfo(f"pub_fusion_bbx_topic: {pub_fusion_bbx_topic}")
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        
        if subscribe_to_topic(sub_ego_bbx_topic):
            sub = rospy.Subscriber(sub_ego_bbx_topic, MarkerArray, callback1)
        rate.sleep()  

if __name__ == '__main__':
    main()

    