#! /usr/bin/env python
# coding: utf-8

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from marker_decoder import decode_marker_all
from marker_encoder import marker_encode_ss
from testing_marker import create_a_markerarray_1,create_a_markerarray_2

def size_consistency_score(markers_array_pre,markers_array_cur):
    """
    Param:
        markers_array_pre: 
            - Type: MarkerArray from visualization_msgs.msg
            - This is the marker Array in the previous frame 
            - Example: markers_pre.marker[x].text: "6.14 m/s , <ID= 4>, <ds=0.958259>, <ss=0.0,0.0,0.0>, <kds=0.0>"
        markers_array_cur:
            - Type: MarkerArray from visualization_msgs.msg
            - This is the marker Array in the current frame
            - Example: markers_cur.marker[x].text: "6.14 m/s , <ID= 4>, <ds=0.958259>, <ss=0.0,0.0,0.0>, <kds=0.0>"
    return:
        ss_markers:
            - Type: MarkerArray from visualization_msgs.msg
            - After finish the calculation of <ss=0.0,0.0,0.0>.
            - Example: ss_markers.marker[x].text: "6.14 m/s , <ID= 4>, <ds=0.958259>, <ss=0.91,0.98,0.89>, <kds=0.0>"
    """
    ss_markers = MarkerArray()

    for marker_cur in markers_array_cur.markers:
        cur_x, cur_y, cur_z, cur_yaw, cur_l, cur_w, cur_h, cur_velo, cur_ID, cur_ds, cur_ss, cur_kds = decode_marker_all(marker_cur)
        ss = [0.0,0.0,0.0]
        for marker_pre in markers_array_pre.markers:
            pre_x, pre_y, pre_z, pre_yaw, pre_l, pre_w, pre_h, pre_velo, pre_ID, pre_ds, pre_ss, pre_kds = decode_marker_all(marker_pre)
            if cur_ID == pre_ID:
                ss_l = min(cur_l,pre_l)/max(cur_l,pre_l)
                ss_w = min(cur_w,pre_w)/max(cur_w,pre_w)
                ss_h = min(cur_h,pre_h)/max(cur_h,pre_h)
                ss = [ss_l,ss_w,ss_h]
                break 
            else:
                ss = [cur_ds,cur_ds,cur_ds]
        marker_cur_new = marker_encode_ss(marker_cur,ss) 
        # append marker_cur_new to ss_markers
        # ss_markers.markers.extend(marker_cur_new)
        ss_markers.markers.append(marker_cur_new)

    return ss_markers

def test1():
    markers_array_pre = create_a_markerarray_1()
    markers_array_cur = create_a_markerarray_2()
    markers_array_cur = size_consistency_score(markers_array_pre,markers_array_cur)
    for marker in markers_array_cur.markers:
        print(marker)

if __name__ == '__main__':
    test1()