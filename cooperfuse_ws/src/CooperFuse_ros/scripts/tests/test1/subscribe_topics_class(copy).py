#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

class MarkerFusionNode:
    def __init__(self):
        self.fusion_pub = rospy.Publisher('/fusion/bbx', MarkerArray, queue_size=10)
        self.ego_markers = MarkerArray()
        self.agent1_markers = MarkerArray()
        self.first_ego_marker = Marker() 
        self.first_agent1_marker = Marker() 

        # Subscribe to the transformed objects topics and specify their respective callback functions
        rospy.Subscriber('/tracking/transformed_objects_box_score', MarkerArray, self.transformed_objects_callback_1)
        rospy.Subscriber('/tracking2/transformed_objects_box_score', MarkerArray, self.transformed_objects_callback_2)

        self.rate = rospy.Rate(10)

    def transformed_objects_callback_1(self, msg):
        self.ego_markers = msg
        self.first_ego_marker = self.ego_markers.markers[0]
        # print("Received data from callback0.")

    def transformed_objects_callback_2(self, msg):
        self.agent1_markers = msg
        self.first_agent1_marker = self.agent1_markers.markers[0]
        # print("Received data from callback1.")

    def clear_markerarray_1(self, ego_maker):
        empty_markers = MarkerArray()
        # create a clear marker
        clear_marker = Marker()
        # use the first marker to create a clear marker
        clear_marker.header = ego_maker.header
        clear_marker.ns = "/fusion/objects"
        clear_marker.id = 0
        clear_marker.action = clear_marker.DELETEALL
        clear_marker.lifetime = rospy.Duration(0)
        # put the clear marker into the empty marker array
        empty_markers.markers.append(clear_marker)
        # publish the clear marker
        self.fusion_pub.publish(empty_markers)    

    def fusion_add(self):
        yellow_color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.5)

        # Define a new MarkerArray object
        fusion_markers = MarkerArray()

        # Clear existing markers
        if self.first_ego_marker.action == 3:
            self.clear_markerarray_1(self.first_ego_marker)

        rospy.loginfo("-----------------------------------------")
        rospy.loginfo("ego_markers length: %d", len(self.ego_markers.markers))
        rospy.loginfo("agent1_markers length: %d", len(self.agent1_markers.markers))

        if len(self.ego_markers.markers) == 1:
            rospy.logerr("ego_marker empty!")
            print(self.ego_markers.markers[0].action)

        # Append ego_markers to fusion_markers
        if self.first_ego_marker.action != 3 and len(self.ego_markers.markers) > 0:
            fusion_markers.markers.extend(self.ego_markers.markers)

        # Append agent1_markers to fusion_markers
        if self.first_agent1_marker.action != 3 and len(self.agent1_markers.markers) > 0:
            fusion_markers.markers.extend(self.agent1_markers.markers)

        # Modify color of markers to yellow
        if len(fusion_markers.markers) > 0:
            for marker in fusion_markers.markers:
                marker.color = yellow_color

            # Publish the fused and modified MarkerArray
            if self.first_ego_marker.action != 3:
                self.fusion_pub.publish(fusion_markers)

    def run(self):
        while not rospy.is_shutdown():
            self.fusion_add()
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('marker_fusion_node')
    fusion_node = MarkerFusionNode()
    fusion_node.run()
