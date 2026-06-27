#! /usr/bin/env python
# coding: utf-8

import rospy
from copy import deepcopy
from visualization_msgs.msg import Marker, MarkerArray    
from testing_marker import create_two_markers_debug3

def filter_class(full_class_markers, filter_class="car"):
    """
    Filter markers based on a specified class filter.

    This function takes a MarkerArray containing markers and filters them based on
    a specified class filter. It checks if the 'ns' (namespace) field of each marker
    contains the provided filter_class string and adds matching markers to a new
    MarkerArray.

    Args:
        full_class_markers (MarkerArray): The input MarkerArray containing markers to filter.
        filter_class (str, optional): The class filter to apply (default is "car").

    Returns:
        MarkerArray: A new MarkerArray containing markers that match the filter_class.
    """
    filtered_class_markeres = MarkerArray()

    # Iterate through markers in the input MarkerArray
    for marker in full_class_markers.markers:
        # Check if the 'ns' field of the current marker contains the filter_class
        if filter_class in marker.ns:
            # Add the marker to the filtered_class_markeres if it matches the filter
            filtered_class_markeres.markers.append(marker)
            # remove this marker from the full_class_markers 
            full_class_markers.markers.remove(marker)

    # Return the filtered MarkerArray
    return filtered_class_markeres,full_class_markers

if __name__ == '__main__':
    input_markers = MarkerArray()
    marker1,marker2 = create_two_markers_debug3()
    input_markers.markers.append(marker1)
    input_markers.markers.append(marker2)
    output_markers,input_markers = filter_class(input_markers)
    # print output_markers 
    for marker in output_markers.markers:
        print("-----")
        print(marker)
    # make sure output operation won't change the input markers
    print("=====")
    # print input markers 
    for marker in input_markers.markers:
        print("-----")
        print(marker)