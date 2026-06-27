#! /usr/bin/env python
# coding: utf-8

import rospy
from visualization_msgs.msg import Marker,MarkerArray

def clear_markerarray(ego_maker,ma_pub):
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