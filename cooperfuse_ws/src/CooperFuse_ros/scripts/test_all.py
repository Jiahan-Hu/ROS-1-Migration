#! /usr/bin/env python
# coding: utf-8

import rospy
from visualization_msgs.msg import Marker, MarkerArray

from size_consistency_3d import size_consistency_score
from kd_consistency_3d import kd_consistent_score
from bbx_feature_based_fusion import calculate_iou_matrix,fuse_multi_markerarray
from CooperFuse.fusion.fusion_bbx import find_clusters

from testing_marker import create_a_markerarray_1,create_a_markerarray_2,create_a_markerarray_3,create_a_markerarray_4

if __name__ == '__main__':
    # create_a_markerarray_1()
    ego_markers_pre = MarkerArray()
    if not ego_markers_pre.markers:  # Check if the markers list is empty
        print("no history info")

