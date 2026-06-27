#! /usr/bin/env python
# coding: utf-8

import rospy
from visualization_msgs.msg import Marker, MarkerArray

class FusionNode:
    def __init__(self):
        rospy.init_node('subscribe_test', anonymous=True)

        self.fusion_params = rospy.get_param('~v2xfusion')
        self.sub_ego_bbx_topic = self.fusion_params['sub_ego_bbx']
        self.sub_agent1_bbx_topic = self.fusion_params['sub_agent1_bbx']
        self.pub_fusion_bbx_topic = self.fusion_params['pub_fusion_bbx']

        rospy.loginfo("Config parameters are as follows:")
        rospy.loginfo(f"sub_infra1_bbx_topic: {self.sub_ego_bbx_topic}")
        rospy.loginfo(f"sub_veh1_bbx_topic: {self.sub_agent1_bbx_topic}")
        rospy.loginfo(f"pub_fusion_bbx_topic: {self.pub_fusion_bbx_topic}")

        self.fusion_pub_ = rospy.Publisher(self.pub_fusion_bbx_topic, MarkerArray, queue_size=100)
        self.ego_sub_ = rospy.Subscriber(self.sub_ego_bbx_topic, MarkerArray, self.callback_ego)
        self.agent1_sub_ = rospy.Subscriber(self.sub_agent1_bbx_topic, MarkerArray, self.callback_agent1)

        self.ego_markers = MarkerArray()
        self.agent1_markers = MarkerArray()

    def callback_ego(self,msg):
        self.ego_markers = msg
        self.double_repub()

    def callback_agent1(self,msg1):
        self.agent1_markers = msg1

    def single_repub(self):
        fusion_markers = MarkerArray()

        for marker in self.ego_markers.markers:
            if marker.action != 3:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.5
                # add marker to fusion_markers
                fusion_markers.markers.append(marker)
            else:

                self.publishClearMarkerArray(marker.header)
        
        # if fusion_markers is not empty, then publish fusion_markers
        if fusion_markers.markers:
            self.fusion_pub_.publish(fusion_markers)
        
    def double_repub(self):
        fusion_markers = MarkerArray()

        for marker in self.ego_markers.markers:
            if marker.action != 3:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.5
                # add marker to fusion_markers
                fusion_markers.markers.append(marker)
            else:

                self.publishClearMarkerArray(marker.header)
        
        for marker in self.agent1_markers.markers:
            if marker.action != 3:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.5
                # add marker to fusion_markers
                fusion_markers.markers.append(marker)
        
        # if fusion_markers is not empty, then publish fusion_markers
        if fusion_markers.markers:
            self.fusion_pub_.publish(fusion_markers)        

    def publishClearMarkerArray(self,header):
        # Create empty MarkerArray
        empty_markers = MarkerArray()
        
        # Create clear marker
        clear_marker = Marker()
        clear_marker.header = header
        clear_marker.ns = "objects"
        clear_marker.id = 0
        clear_marker.action = Marker.DELETEALL
        clear_marker.lifetime = rospy.Duration()  # Set lifetime to zero
        
        # Add clear_marker to empty_markers
        empty_markers.markers.append(clear_marker)
        self.fusion_pub_.publish(empty_markers)

if __name__ == '__main__':
    try:
        fusion_node = FusionNode()
        rospy.spin()  # Keep the program running
    except rospy.ROSInterruptException:
        pass    
    