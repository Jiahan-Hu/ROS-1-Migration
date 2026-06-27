#! /usr/bin/env python
# coding: utf-8

import rospy
from copy import deepcopy
from visualization_msgs.msg import Marker, MarkerArray
from myTimer import Timer
from size_consistency_3d import size_consistency_score
from kd_consistency_3d import kd_consistent_score
from bbx_feature_based_fusion import calculate_iou_matrix_opt,calculate_iou_matrix,fuse_multi_markerarray,bbx_iou_new
from CooperFuse.fusion.fusion_bbx import find_clusters

class FusionNode:
    def __init__(self):
        rospy.init_node('subscribe_test', anonymous=True)

        self.fusion_params = rospy.get_param('~v2xfusion')
        self.sub_ego_bbx_topic = self.fusion_params['sub_ego_bbx']
        self.sub_agent1_bbx_topic = self.fusion_params['sub_agent1_bbx']
        self.pub_fusion_bbx_topic = self.fusion_params['pub_fusion_bbx']
        self.pub_same_pose_bbx_topic = self.fusion_params['pub_same_pose']

        rospy.loginfo("Config parameters are as follows:")
        rospy.loginfo(f"sub_infra1_bbx_topic: {self.sub_ego_bbx_topic}")
        rospy.loginfo(f"sub_veh1_bbx_topic: {self.sub_agent1_bbx_topic}")
        rospy.loginfo(f"pub_fusion_bbx_topic: {self.pub_fusion_bbx_topic}")
        rospy.loginfo(f"pub_same_pose_bbx_topic: {self.pub_same_pose_bbx_topic}")

        self.fusion_pub_ = rospy.Publisher(self.pub_fusion_bbx_topic, MarkerArray, queue_size=100)
        self.fusion_sp_bbx_pub_ = rospy.Publisher(self.pub_same_pose_bbx_topic, MarkerArray, queue_size=100)

        self.ego_sub_ = rospy.Subscriber(self.sub_ego_bbx_topic, MarkerArray, self.callback_ego)
        self.agent1_sub_ = rospy.Subscriber(self.sub_agent1_bbx_topic, MarkerArray, self.callback_agent1)

        self.ego_markers = MarkerArray()
        self.agent1_markers = MarkerArray()
        self.fusion_markers = MarkerArray()
        self.last_published_timestamp = None 
        self.current_timestamp = None 
        self.pub_all_flag = -1
        self.receive_flag = -1 
        self.clear_header = None
        self.ego_markers_pre = MarkerArray()
        self.agent1_markers_pre = MarkerArray()
        self.ego_no_pre_flag = -1
        self.agent1_no_pre_flag = -1 

        # ==========module testing flag ==========
        self.test_feature = False

    def callback_ego(self,msg):
        self.ego_markers = msg
        self.current_timestamp = self.ego_markers.markers[0].header.stamp
        # self.double_repub_debug2()
        self.double_repub_tag1_debug()

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

    def double_repub_tag1(self):
        print("============== debug begin==================")
        timer = Timer()  # Create a timer
        ego_flag = 0

        for marker in self.ego_markers.markers:
            if marker.action != 3:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.5
                # marker.lifetime = rospy.Duration(0.1)
                # add marker to fusion_markers
                self.fusion_markers.markers.append(marker)
                ego_flag = 1 
            else:
                print("----")
                self.clear_header = marker.header
                # self.publishClearMarkerArray(marker.header)
                self.fusion_markers = MarkerArray()
                self.pub_all_flag = 0
                print("flag1:",self.pub_all_flag)

        if ego_flag == 1:
            self.pub_all_flag = 3
            print("flag3:",self.pub_all_flag)

        # if self.agent1_markers.markers:
        if self.pub_all_flag == 0 and self.agent1_markers.markers:
            self.pub_all_flag = 2 
            self.receive_flag = 1
            print("----agent1")
            print("flag2:",self.pub_all_flag)
            for marker in self.agent1_markers.markers:
                if marker.action != 3:
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 0.5
                    # marker.lifetime = rospy.Duration(0.1)
                    # add marker to fusion_markers
                    self.fusion_markers.markers.append(marker) 
        
        if self.pub_all_flag == 3 and self.receive_flag == 1:
            timer0 = Timer()  # Create a timer
            # deepcopy takes about 10 ms
            comb_markers_copy = deepcopy(self.fusion_markers)
            # comb_markers_copy = self.fusion_markers
            elapsed_time0 = timer0.takeRealTime()
            print(f"Elapsed Time0 (ms): {elapsed_time0}")
            
            timer1 = Timer()  # Create a timer
            # around 50ms - 120ms
            # iou_matrix = calculate_iou_matrix(comb_markers_copy)
            # around 40 - 70ms
            iou_matrix = calculate_iou_matrix_opt(comb_markers_copy)
            elapsed_time1 = timer1.takeRealTime()
            print(f"Elapsed Time1 (ms): {elapsed_time1}")
            # print("iou_matrix:")
            # print(iou_matrix)
            
            timer2 = Timer()  # Create a timer
            together_pairs = find_clusters(iou_matrix,iou_threshold=0.01)
            elapsed_time2 = timer2.takeRealTime()
            print(f"Elapsed Time2 (ms): {elapsed_time2}")
            print("together_pairs:",together_pairs)
            self.get_together_index(together_pairs,comb_markers_copy)

        if self.pub_all_flag == 3 and self.fusion_markers.markers:
            print("----pub_clear")
            self.publishClearMarkerArray(self.clear_header)
            print("------------------")
            # print timestamp of fusionmarkers
            time_stamp_sec = self.fusion_markers.markers[0].header.stamp.secs
            time_stamp_nsec = self.fusion_markers.markers[0].header.stamp.nsecs
            print(f"time stamp sec: {time_stamp_sec}")
            print(f"time stamp nsec: {time_stamp_nsec}")
            # print the size of fusion_markers
            size_of_fusion_markers = len(self.fusion_markers.markers)  # Get the size of fusion_markers
            print(f"Size of fusion_markers: {size_of_fusion_markers}")
            self.fusion_pub_.publish(self.fusion_markers) 
            # reset ego and agent markers so that if one topic disapear, 
            # it will be ereased from the previous frame. 
            self.ego_markers = MarkerArray()
            self.agent1_markers = MarkerArray()
            self.receive_flag = 0

        elapsed_time = timer.takeRealTime()
        print(f"Elapsed Time Final (ms): {elapsed_time}")
        print("============== END ==================")

    def bbx_feature_cal_ego(self):
        if not self.ego_markers_pre.markers:
            print("no history info_ego")
            self.ego_no_pre_flag = 1
        else:
            self.ego_no_pre_flag = 0
            if self.ego_markers.markers and self.ego_markers.markers[0].action != 3:
                timer_size = Timer()
                self.ego_markers = size_consistency_score(self.ego_markers_pre,self.ego_markers)
                elapsed_time_size = timer_size.takeRealTime()
                print(f"Elapsed Time_size (ms): {elapsed_time_size}")
                timer_kd = Timer()
                self.ego_markers = kd_consistent_score(self.ego_markers_pre,self.ego_markers)
                elapsed_time_kd = timer_kd.takeRealTime()
                print(f"Elapsed Time_kd (ms): {elapsed_time_kd}")

    def bbx_feature_cal_agent1(self):
        if not self.agent1_markers_pre.markers:
            print("no history info_agent1")
            self.agent1_no_pre_flag = 1
        else:
            self.agent1_no_pre_flag = 0
            if self.agent1_markers.markers and self.agent1_markers.markers[0].action != 3:
                print("current:")
                print(self.agent1_markers.markers[0].text)
                print("previous:")
                print(self.agent1_markers_pre.markers[0].text)
                self.agent1_markers = size_consistency_score(self.agent1_markers_pre, self.agent1_markers)
                self.agent1_markers = kd_consistent_score(self.agent1_markers_pre, self.agent1_markers)
        
    def double_repub_tag1_debug(self):
        print("============== debug begin==================")
        timer = Timer()  # Create a timer
        ego_flag = 0

        # ========== Here is to calculate the feature of bbx =============
        if self.test_feature:
            timer_bbx = Timer()
            self.bbx_feature_cal_ego()
            elapsed_time_ego = timer_bbx.takeRealTime()
            print(f"Elapsed Time_ego (ms): {elapsed_time_ego}")

            timer_bbx_agent = Timer()
            # around 40ms
            if self.receive_flag == 1:
                self.bbx_feature_cal_agent1()
                if self.agent1_no_pre_flag == 0:
                    print(self.agent1_markers.markers[0].text)
            elapsed_time_agent = timer_bbx_agent.takeRealTime()
            print(f"Elapsed Time_agent (ms): {elapsed_time_agent}")

            if self.ego_no_pre_flag == 0:
                print(self.ego_markers.markers[0].text)
        # ========== Feature calculation ends =============

        for marker in self.ego_markers.markers:
            if marker.action != 3:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.5
                # marker.lifetime = rospy.Duration(0.1)
                # add marker to fusion_markers
                self.fusion_markers.markers.append(marker)
                ego_flag = 1 
            else:
                print("----")
                self.clear_header = marker.header
                # self.publishClearMarkerArray(marker.header)
                self.fusion_markers = MarkerArray()
                self.pub_all_flag = 0
                print("flag1:",self.pub_all_flag)

        if ego_flag == 1:
            self.pub_all_flag = 3
            print("flag3:",self.pub_all_flag)
            ego_num = len(self.ego_markers.markers)
            print("ego_num:",ego_num)

        # if self.agent1_markers.markers:
        if self.pub_all_flag == 0 and self.agent1_markers.markers:
            self.pub_all_flag = 2 
            print("----agent1")
            print("flag2:",self.pub_all_flag)
            for marker in self.agent1_markers.markers:
                if marker.action != 3:
                    self.receive_flag = 1
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 0.5
                    # marker.lifetime = rospy.Duration(0.1)
                    # add marker to fusion_markers
                    self.fusion_markers.markers.append(marker) 
                else:
                    rospy.logwarn("---agent1 clear marker---")

        # some times, self.receive_flag = 0, 
        #   meaning that data is missing at the point
        #   this is why fusion_sp_bbx_pub_ will flash
        # print("self.receive_flag:", self.receive_flag)
        if self.receive_flag == 0:
            rospy.logwarn("No valid agent1 marker received!")
        else:
            received_agent_num = len(self.agent1_markers.markers)
            print("received_agent_num:",received_agent_num)
        
        # ==========================Fusion starts=============================
        if self.pub_all_flag == 3 and self.receive_flag == 1:
            timer0 = Timer()  # Create a timer
            # deepcopy takes about 10 ms
            # comb_markers_copy = deepcopy(self.fusion_markers)
            comb_markers_copy = self.fusion_markers
            elapsed_time0 = timer0.takeRealTime()
            print(f"Elapsed Time0 (ms): {elapsed_time0}")
            
            timer1 = Timer()  # Create a timer
            # around 50ms - 120ms
            # iou_matrix = calculate_iou_matrix(comb_markers_copy)
            # around 40 - 70ms
            iou_matrix = calculate_iou_matrix_opt(comb_markers_copy)
            elapsed_time1 = timer1.takeRealTime()
            print(f"Elapsed Time1 (ms): {elapsed_time1}")
            # print("iou_matrix:")
            # print(iou_matrix)
            
            timer2 = Timer()  # Create a timer
            together_pairs = find_clusters(iou_matrix,iou_threshold=0.1)
            elapsed_time2 = timer2.takeRealTime()
            print(f"Elapsed Time2 (ms): {elapsed_time2}")
            print("together_pairs:",together_pairs)
            # show bbx that needs to fuse
            # self.get_together_index(together_pairs,comb_markers_copy)
            self.fusion_markers = self.keep_ego_data(comb_markers_copy,together_pairs)
        # ========================Fusion ends=================================

        if self.pub_all_flag == 3 and self.fusion_markers.markers:
            print("----pub_clear")
            self.publishClearMarkerArray(self.clear_header)
            print("------------------")
            # print timestamp of fusionmarkers
            time_stamp_sec = self.fusion_markers.markers[0].header.stamp.secs
            time_stamp_nsec = self.fusion_markers.markers[0].header.stamp.nsecs
            print(f"time stamp sec: {time_stamp_sec}")
            print(f"time stamp nsec: {time_stamp_nsec}")
            # print the size of fusion_markers
            size_of_fusion_markers = len(self.fusion_markers.markers)  # Get the size of fusion_markers
            print(f"Size of fusion_markers: {size_of_fusion_markers}")
            self.fusion_pub_.publish(self.fusion_markers) 
            
            # ========================== assign to previous markers ==========================================
            self.ego_markers_pre = self.ego_markers
            # print("self.agent1_markers:",self.agent1_markers)
            if self.agent1_markers.markers and self.receive_flag != 0 and self.agent1_markers.markers[0].action != 3:
                self.agent1_markers_pre = self.agent1_markers
            print("ego_pre_num:",len(self.ego_markers_pre.markers))
            print("agent1_pre_num:",len(self.agent1_markers_pre.markers))

            # reset ego and agent markers so that if one topic disapear, 
            # it will be ereased from the previous frame. 
            self.ego_markers = MarkerArray()
            self.agent1_markers = MarkerArray()
            self.receive_flag = 0

        elapsed_time = timer.takeRealTime()
        print(f"Elapsed Time Final (ms): {elapsed_time}")
        print("============== END ==================")

    def get_together_index(self,together_pairs,comb_markers_copy):
        together_markers = MarkerArray()
        for item in together_pairs:
            if len(item) > 1:
                print("pair:",item)
                iou = bbx_iou_new(comb_markers_copy.markers[item[0]],comb_markers_copy.markers[item[1]])
                print("iou:",iou)
                for pair in item:
                    marker = comb_markers_copy.markers[pair]
                    marker.color.r = 0.9
                    marker.color.g = 0.9
                    marker.color.b = 0.0
                    marker.color.a = 0.5
                    together_markers.markers.append(marker)
        self.publishClearMarkerArray_interdemia(self.clear_header)
        self.fusion_sp_bbx_pub_.publish(together_markers)
    
    def keep_ego_data(self,fusion_markers,together_pairs):
        """
        Parameters
        ----------
        fusion_markers: Visualization_msgs/MarrayArray 
            - Example: 
        together_paris: List
            - Example: [[0], [1], [2, 17], [3]]
        Returns
        -------
        fusion_markers 
        """
        for pair in together_pairs:
            if len(pair) > 1:
                last_item = pair[-1]
                for item in pair:
                    if item != last_item:
                        fusion_markers.markers[item] = Marker()
        return fusion_markers
                    # keep the last of item of pair and remove the other from fusion_markers


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

    def publishClearMarkerArray_interdemia(self,header):
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
        self.fusion_sp_bbx_pub_.publish(empty_markers)

    def run(self):
        try:
            rospy.spin()  # Keep the program running
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    fusion_node = FusionNode()
    fusion_node.run()
    