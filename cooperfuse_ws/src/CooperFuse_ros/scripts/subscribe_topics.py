#! /usr/bin/env python
# coding: utf-8

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from empty_function import clear_markerarray

class FusionNode:
    def __init__(self):
        self.ego_markers = MarkerArray()
        self.agent1_markers = MarkerArray()
        self.first_ego_marker = Marker()
        self.first_agent1_marker = Marker()
        self.last_published_timestamp = None

        rospy.init_node('subscribe_test', anonymous=True)

        self.fusion_params = rospy.get_param('~v2xfusion')
        self.sub_ego_bbx_topic = self.fusion_params['sub_ego_bbx']
        self.sub_agent1_bbx_topic = self.fusion_params['sub_agent1_bbx']
        self.pub_fusion_bbx_topic = self.fusion_params['pub_fusion_bbx']

        rospy.loginfo("Config parameters are as follows:")
        rospy.loginfo(f"sub_infra1_bbx_topic: {self.sub_ego_bbx_topic}")
        rospy.loginfo(f"sub_veh1_bbx_topic: {self.sub_agent1_bbx_topic}")
        rospy.loginfo(f"pub_fusion_bbx_topic: {self.pub_fusion_bbx_topic}")

        self.fusion_pub = rospy.Publisher(self.pub_fusion_bbx_topic, MarkerArray, queue_size=100)
        self.sub = rospy.Subscriber(self.sub_ego_bbx_topic, MarkerArray, self.callback0)
        self.sub1 = rospy.Subscriber(self.sub_agent1_bbx_topic, MarkerArray, self.callback1)

    def callback0(self, msg):
        self.ego_markers = msg
        self.first_ego_marker = self.ego_markers.markers[0]
        if self.first_ego_marker.action == 3:
            self.clear_markerarray(self.first_ego_marker, self.fusion_pub)
        # print("data received!")
        self.fusion_add()

    def callback1(self, msg):
        self.agent1_markers = msg
        self.first_agent1_marker = self.agent1_markers.markers[0]

    def subscribe_to_topic(self, topic_name):
        published_topics = rospy.get_published_topics()
        if any(topic == topic_name for topic, _ in published_topics):
            print(f"Subscribed to topic {topic_name}.")
            return True
        else:
            print(f"Topic {topic_name} does not exist.")
            return False

    def clear_markerarray_1(self, ego_maker):
        empty_markers = MarkerArray()
        clear_marker = Marker()
        clear_marker.header = ego_maker.header
        clear_marker.ns = "objects"
        clear_marker.id = 0
        clear_marker.action = clear_marker.DELETEALL
        clear_marker.lifetime = rospy.Duration(0)
        empty_markers.markers.append(clear_marker)
        self.fusion_pub.publish(empty_markers)

    def clear_markerarray(self,ego_maker,ma_pub):
        global fusion_pub
        empty_markers = MarkerArray()
        # create a clear marker
        clear_marker = Marker()
        # use the first marker in the topic1_markers to create a clear marker
        clear_marker.header = ego_maker.header
        clear_marker.ns = "objects"
        clear_marker.id = 0
        clear_marker.action = clear_marker.DELETEALL
        clear_marker.lifetime = rospy.Duration(0)
        # clear_marker.type = clear_marker.CUBE
        # put the clear marker into the empty marker array
        empty_markers.markers.append(clear_marker)
        # publish the clear marker
        ma_pub.publish(empty_markers)  
        print(clear_marker.DELETEALL)

    def fusion_add(self):
        fusion_markers = MarkerArray()

        #if self.first_ego_marker.action == 3:
        #    clear_markerarray(self.first_ego_marker, self.fusion_pub)
        """
        if self.subscribe_to_topic(self.sub_ego_bbx_topic) == False:
            self.ego_markers = MarkerArray()
            fusion_markers.markers.extend(self.ego_markers.markers)
        """
        
        if self.subscribe_to_topic(self.sub_agent1_bbx_topic) == False:
            self.agent1_markers = MarkerArray()
            fusion_markers.markers.extend(self.agent1_markers.markers)

        current_timestamp = self.first_ego_marker.header.stamp
        
        # Check if we have already published for this timestamp
        if self.last_published_timestamp == current_timestamp:
            return

        rospy.loginfo("-----------------------------------------")
        rospy.loginfo("ego_markers length: %d", len(self.ego_markers.markers))
        rospy.loginfo("agent1_markers length: %d", len(self.agent1_markers.markers))
        
        """
        if len(self.ego_markers.markers) == 1:
            rospy.logerr("ego_marker empty!")
            print(self.ego_markers.markers[0].action)
        """

        if self.first_ego_marker.action != 3 and len(self.ego_markers.markers) > 0:
            fusion_markers.markers.extend(self.ego_markers.markers)

        if self.first_agent1_marker.action != 3 and len(self.agent1_markers.markers) > 0:
            fusion_markers.markers.extend(self.agent1_markers.markers)

        if len(fusion_markers.markers) > 0:
            for marker in fusion_markers.markers:
                marker.color.r = 0.9
                marker.color.g = 0.9
                marker.color.b = 0.0
                marker.color.a = 0.5

            if self.first_ego_marker.action != 3:
                self.fusion_pub.publish(fusion_markers)
                self.last_published_timestamp = current_timestamp

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # self.fusion_add()
            rate.sleep()

    def run1(self):
        while not rospy.is_shutdown():
            # self.fusion_add()
            rospy.spin()

if __name__ == '__main__':
    fusion_node = FusionNode()
    fusion_node.run()
