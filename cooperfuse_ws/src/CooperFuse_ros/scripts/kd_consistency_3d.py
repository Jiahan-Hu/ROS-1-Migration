#! /usr/bin/env python
# coding: utf-8

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from marker_decoder import decode_marker_all
from marker_encoder import marker_encode_ss,marker_encode_kds
from comb_planning_minE_3d import minE_calculation_3d

from testing_marker import create_a_markerarray_1,create_a_markerarray_2

def kd_consistent_score(markers_array_pre,markers_array_cur):
    """
    This code is to get the kds based on the previous and current marker_array

    Parameters
    ----------
    markers_array_pre : MarkerArray from visualization_msgs.msg
        This is the marker Array in the previous frame 
        Example: 
            markers_array_pre.markers[x].text= "6.14 m/s , <ID= 4>, <ds= 0.958259>, <ss= 0.0,0.0,0.0>, <kds= 0.0>"
    markers_array_cur : MarkerArray from visualization_msgs.msg
        This is the marker Array in the current frame
        Example: 
            markers_array_cur.markers[x].text= "6.14 m/s , <ID= 4>, <ds= 0.958259>, <ss= 0.99,0.88,0.77>, <kds= 0.0>"

    Returns
    -------
    kd_markers : MarkerArray from visualization_msgs.msg
        After finish the calculation of <ss=0.0,0.0,0.0>.
        Example:
            kd_markers.markers[x].text = "6.14 m/s , <ID= 4>, <ds= 0.958259>, <ss= 0.99,0.88,0.77>, <kds= E14.8>"
    """
    kd_markers = MarkerArray()
    
    for marker_cur in markers_array_cur.markers:
        cur_x, cur_y, cur_z, cur_yaw, cur_l, cur_w, cur_h, cur_velo, cur_ID, cur_ds, cur_ss, cur_kds = decode_marker_all(marker_cur)
        kds = -1
        for marker_pre in markers_array_pre.markers:
            pre_x, pre_y, pre_z, pre_yaw, pre_l, pre_w, pre_h, pre_velo, pre_ID, pre_ds, pre_ss, pre_kds = decode_marker_all(marker_pre)
            if cur_ID == pre_ID:
                minE = minE_calculation_3d(marker_pre,marker_cur)
                kds = minE
                break 
            else:
                kds = -1 
        marker_cur_new = marker_encode_kds(marker_cur,kds) 
        # append marker_cur_new to ss_markers
        # ss_markers.markers.extend(marker_cur_new)
        kd_markers.markers.append(marker_cur_new)  

    return kd_markers

def test_kd_consistent_score():
    # Create some example marker arrays for testing
    markers_array_pre = create_a_markerarray_1()  # Replace with your function to create a marker array
    markers_array_cur = create_a_markerarray_2()  # Replace with your function to create a marker array
    
    # Call the function you want to test
    kd_markers = kd_consistent_score(markers_array_pre, markers_array_cur)
    
    # Print or assert the results
    for marker in kd_markers.markers:
        print(marker)
        print("--------------")
        # print("New KDS:", marker.text)

if __name__ == "__main__":
    test_kd_consistent_score()