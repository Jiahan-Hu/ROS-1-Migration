#! /usr/bin/env python
# coding: utf-8

import rospy
from visualization_msgs.msg import Marker,MarkerArray
from empty_function import clear_markerarray

def callback0(msg):
    global ego_markers
    global first_ego_marker
    ego_markers = msg
    first_ego_marker = ego_markers.markers[0]
    # print("Received data from callback0.")

def callback1(msg):
    global agent1_markers
    global first_agent1_marker
    agent1_markers = msg
    first_agent1_marker = agent1_markers.markers[0]
    # print("Received data from callback1.")

def subscribe_to_topic(topic_name):
    # Check if there are any connections to the topic
    published_topics = rospy.get_published_topics()
    if any(topic == topic_name for topic, _ in published_topics):
        print(f"Subscribed to topic {topic_name}.")
        return True
    else:
        print(f"Topic {topic_name} does not exist.")
        return False

def clear_markerarray_1(ego_maker):
    global fusion_pub
    empty_markers = MarkerArray()
    # create a clear marker
    clear_marker = Marker()
    # use the first marker in the topic1_markers to create a clear marker
    clear_marker.header = ego_maker.header
    clear_marker.ns = "/fusion/objects"
    clear_marker.id = 0
    clear_marker.action = clear_marker.DELETEALL
    clear_marker.lifetime = rospy.Duration(0)
    # clear_marker.type = clear_marker.CUBE
    # put the clear marker into the empty marker array
    empty_markers.markers.append(clear_marker)
    # publish the clear marker
    fusion_pub.publish(empty_markers)    

def fusion_add(ego_topic,agent1_topic):
    global fusion_pub
    global ego_markers
    global agent1_markers
    global first_ego_marker
    global first_agent1_marker

    # Define a new MarkerArray object
    fusion_markers = MarkerArray()

    # Clear existing markers
    if first_ego_marker.action == 3:
        print("this is empty! ")
        clear_markerarray(first_ego_marker,fusion_pub)
    # clear_markerarray(first_ego_marker,fusion_pub)
    
    # clear_markerarray_1(first_ego_marker,fusion_pub)

    rospy.loginfo("-----------------------------------------")
    rospy.loginfo("ego_markers length: %d", len(ego_markers.markers))
    rospy.loginfo("agent1_markers length: %d", len(agent1_markers.markers))

    if len(ego_markers.markers)==1:
        rospy.logerr("ego_marker empty!")
        print(ego_markers.markers[0].action)
    """
    if len(ego_markers.markers)==0:
        ego_markers = MarkerArray()
        fusion_markers.markers.extend(ego_markers.markers)
    
    if len(agent1_markers.markers)==0:
        agent1_markers = MarkerArray()
        fusion_markers.markers.extend(agent1_markers.markers)
    
    if not subscribe_to_topic(ego_topic):
        ego_markers = MarkerArray()
        fusion_markers.markers.extend(ego_markers.markers)

    if not subscribe_to_topic(agent1_topic):
        agent1_markers = MarkerArray()
        fusion_markers.markers.extend(agent1_markers.markers)        
    """
    # topic_markers = ego_markers.markers + agent1_markers.markers

    # Append ego_markers to fusion_markers
    if first_ego_marker.action != 3 and len(ego_markers.markers) > 0:
        fusion_markers.markers.extend(ego_markers.markers)

    # Append agent1_markers to fusion_markers
    if first_agent1_marker.action != 3 and len(agent1_markers.markers) > 0:
        fusion_markers.markers.extend(agent1_markers.markers)

    # Modify color of markers to yellow
    # add if fusion_makers are not empty 
    if len(fusion_markers.markers) > 0:
        for marker in fusion_markers.markers:
            marker.color.r = 0.9
            marker.color.g = 0.9
            marker.color.b = 0.0
            marker.color.a = 0.5

        # Publish the fused and modified MarkerArray
        if first_ego_marker.action != 3:
            fusion_pub.publish(fusion_markers)

def main():
    pass

if __name__ == '__main__':
    """
    The goal of this code is to test when agent1 bbx show up later in the network, 
        what will be the output of the fused bbx.

        [] run rosbag to check.
        [] check sub-module clear_markerarray.
    """
    global fusion_pub
    global ego_markers
    global agent1_markers
    global first_ego_marker
    global first_agent1_marker

    ego_markers = MarkerArray()
    agent1_markers = MarkerArray()
    first_ego_marker = Marker() 
    first_agent1_marker = Marker() 

    # Initialize the ROS node
    rospy.init_node('subscribe_test', anonymous=True)

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
    
    # frequency of the loop 
    rate = rospy.Rate(10)

    # publisher 
    fusion_pub = rospy.Publisher(pub_fusion_bbx_topic, MarkerArray, queue_size=10)

    while not rospy.is_shutdown():
        """

        if subscribe_to_topic(sub_ego_bbx_topic):
            sub = rospy.Subscriber(sub_ego_bbx_topic, MarkerArray, callback0)
        
        if subscribe_to_topic(sub_ego_bbx_topic):
            sub1 = rospy.Subscriber(sub_agent1_bbx_topic, MarkerArray, callback1)
        """
        

        sub = rospy.Subscriber(sub_ego_bbx_topic, MarkerArray, callback0)
        sub1 = rospy.Subscriber(sub_agent1_bbx_topic, MarkerArray, callback1)
        fusion_add(sub_ego_bbx_topic,sub_agent1_bbx_topic)

        # rospy.spin()
        rate.sleep() 

    