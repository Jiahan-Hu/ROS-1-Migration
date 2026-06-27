#! /usr/bin/env python
# coding: utf-8

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from empty_function import clear_markerarray
from copy import deepcopy
from size_consistency_3d import size_consistency_score
from kd_consistency_3d import kd_consistent_score
from bbx_feature_based_fusion import calculate_iou_matrix,fuse_multi_markerarray
from CooperFuse.fusion.fusion_bbx import find_clusters

class FusionNode:
    def __init__(self):
        self.ego_markers = MarkerArray()
        self.ego_markers_pre = MarkerArray()
        self.agent1_markers = MarkerArray()
        self.agent1_markers_pre = MarkerArray()
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
        self.sub0 = rospy.Subscriber(self.sub_ego_bbx_topic, MarkerArray, self.callback0)
        self.sub1 = rospy.Subscriber(self.sub_agent1_bbx_topic, MarkerArray, self.callback1)

    def callback0(self, msg):
        self.ego_markers = msg
        self.first_ego_marker = self.ego_markers.markers[0]
        if self.first_ego_marker.action == 3:
            self.clear_markerarray(self.first_ego_marker, self.fusion_pub)
        # print("data received!")
        # self.fusion_add() # tested
        # self.test_single_bbx() # tested
        self.feature_based_fusion()

    def callback1(self, msg):
        self.agent1_markers = msg
        self.first_agent1_marker = self.agent1_markers.markers[0]
        print("callback1:")
        print(self.first_agent1_marker.text)

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
        """
        Clears the specified marker array by publishing a clear marker.

        Parameters
        ----------
        ego_maker : Marker from visualization_msgs.msg
            An ego marker used to obtain header information for the clear marker.
        ma_pub : Publisher from rospy.Publisher
            A publisher used to publish the empty marker array containing the clear marker.

        Returns
        -------
        None

        Notes
        -----
        This function creates a clear marker with the DELETEALL action and publishes it
        using the provided publisher. The header information of the provided ego marker
        is used for the clear marker's header. The clear marker is appended to an empty
        marker array before publishing.

        Example
        -------
        ego_maker: Marker
            An ego marker containing header information.
        ma_pub: Publisher
            A publisher for marker arrays.
        clear_markerarray(ego_maker, ma_pub)
            Clears the specified marker array by publishing a clear marker.
        """
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
        """
        Combines and publishes fused markers based on specific conditions.

        This function combines markers from different sources (ego and agent1)
        into a single MarkerArray called fusion_markers. It checks for conditions
        like subscribing to agent1 markers, timestamp matching, and availability
        of markers before deciding whether to publish fused markers.

        Parameters
        ----------
        self : instance of a class
            The class instance containing the necessary data and methods.

        Returns
        -------
        None

        Example
        -------
        fusion_add()
            Combines markers from ego and agent1, performs fusion, and publishes the fused markers.
        """

        fusion_markers = MarkerArray()
        
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
            else:
                return 
        else:
            return 

    def feature_based_fusion(self):
        comb_markers = MarkerArray()
        fusion_markers_array = MarkerArray()
        flag = 0
        # ego agent first 
        current_timestamp = self.first_ego_marker.header.stamp
        # Check if we have already published for this timestamp
        if self.last_published_timestamp == current_timestamp:
            return
        

        if not self.ego_markers_pre.markers:  # Check if the markers list is empty
            print("no history info_ego")
            if self.first_ego_marker.action != 3 and len(self.ego_markers.markers) > 0:
                comb_markers.markers.extend(self.ego_markers.markers)
                print(self.ego_markers.markers[0].text)
        else:
            if self.first_ego_marker.action != 3:
                print("hello_ego:")
                print(self.ego_markers.markers[0].text)
                self.ego_markers = size_consistency_score(self.ego_markers_pre,self.ego_markers)
                self.ego_markers = kd_consistent_score(self.ego_markers_pre,self.ego_markers)
                print('---')
                print(self.ego_markers.markers[0].text)
                if self.first_ego_marker.action != 3 and len(self.ego_markers.markers) > 0:
                    comb_markers.markers.extend(self.ego_markers.markers)

        
        # Then other agents
        # if no sub_agent1_bbx_topic in the network, then agent1_markers is assigned to be empty.
        if self.subscribe_to_topic(self.sub_agent1_bbx_topic) == False:
            self.agent1_markers = MarkerArray()
            comb_markers.markers.extend(self.agent1_markers.markers) #Need to check 
            flag = 1

        if not self.agent1_markers_pre.markers: # Check if the markers list is empty
            print("no history info_agent1")
            if self.first_agent1_marker.action != 3 and len(self.agent1_markers.markers) > 0:
                comb_markers.markers.extend(self.agent1_markers.markers)
        else:
            if self.first_agent1_marker.action != 3:
                print("hello there:")
                print(self.agent1_markers_pre.markers[0].text)
                print(self.agent1_markers.markers[0].text)
                self.agent1_markers = size_consistency_score(self.agent1_markers_pre,self.agent1_markers)
                self.agent1_markers = kd_consistent_score(self.agent1_markers_pre,self.agent1_markers)
                print(self.agent1_markers.markers[0].text)
                if self.first_agent1_marker.action != 3 and len(self.agent1_markers.markers) > 0:
                    comb_markers.markers.extend(self.agent1_markers.markers)    

        # print("test2:")
        # print(self.agent1_markers.markers[0].text)
        # fuse the comb_markers from multi-agents
        # if flag == 1:
        #     fusion_markers_array = comb_markers
        # else:
        comb_markers_copy = deepcopy(comb_markers)
        iou_matrix = calculate_iou_matrix(comb_markers_copy)
        together_pairs = find_clusters(iou_matrix,iou_threshold=0.01)
        print("together_pairs:",together_pairs)
        fusion_markers_array = fuse_multi_markerarray(comb_markers_copy,together_pairs)   

        # print("test4:")
        # print(fusion_markers_array.markers[0].text)

        if len(fusion_markers_array.markers) > 0:   
            if self.first_ego_marker.action != 3:
                self.fusion_pub.publish(fusion_markers_array)
                self.last_published_timestamp = current_timestamp
                self.ego_markers_pre = self.ego_markers
            if self.first_agent1_marker.action != 3:
                # print("test3:")
                # print(self.agent1_markers.markers[0].text)
                self.agent1_markers_pre = self.agent1_markers
                # print("test:")
                # print(self.agent1_markers_pre.markers[0].text)

    def test_single_bbx(self):
        comb_markers = MarkerArray()
        # ego agent first 
        current_timestamp = self.first_ego_marker.header.stamp
        # Check if we have already published for this timestamp
        if self.last_published_timestamp == current_timestamp:
            return
        
        if not self.ego_markers_pre.markers:  # Check if the markers list is empty
            print("no history info_ego")
            if self.first_ego_marker.action != 3 and len(self.ego_markers.markers) > 0:
                comb_markers.markers.extend(self.ego_markers.markers)
        else:
            if self.first_ego_marker.action != 3:
                self.ego_markers = size_consistency_score(self.ego_markers_pre,self.ego_markers)
                self.ego_markers = kd_consistent_score(self.ego_markers_pre,self.ego_markers)
                if self.first_ego_marker.action != 3 and len(self.ego_markers.markers) > 0:
                    comb_markers.markers.extend(self.ego_markers.markers)

        # Then other agents
        # if no sub_agent1_bbx_topic in the network, then agent1_markers is assigned to be empty.
        if self.subscribe_to_topic(self.sub_agent1_bbx_topic) == False:
            self.agent1_markers = MarkerArray()
            comb_markers.markers.extend(self.agent1_markers.markers) #Need to check 

        if len(comb_markers.markers) > 0:
            for marker in comb_markers.markers:
                marker.lifetime = rospy.Duration(2)  # set lifetime as 0.1 second
                marker.color.r = 0.9
                marker.color.g = 0.9
                marker.color.b = 0.0
                marker.color.a = 0.5

        if len(comb_markers.markers) > 0:   
            if self.first_ego_marker.action != 3:
                self.fusion_pub.publish(comb_markers)
                self.last_published_timestamp = current_timestamp
                self.ego_markers_pre = self.ego_markers
                # self.agent1_markers_pre = self.agent1_markers

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
    fusion_node.run1()
