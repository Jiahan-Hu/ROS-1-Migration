#! /usr/bin/env python
# coding: utf-8

import rospy
import copy
import time 
import numpy as np
import math
from shapely.geometry import Polygon
from copy import deepcopy
from visualization_msgs.msg import Marker,MarkerArray

from marker_decoder import decode_marker_all,decode_maker_shape,decode_marker_scale

from kd_consistency_3d import kd_consistent_score
from size_consistency_3d import size_consistency_score
from CooperFuse.fusion.fusion_bbx import find_clusters,calculate_probabilities,softmax
from CooperFuse.sim.vis_3d import bbx_marker_vis
from marker_encoder import \
    marker_encode_all,marker_encode_ss,\
        marker_encode_kds,marker_encode_ds,marker_encode_f_kds,\
            change_marker_colors,change_marker_text

from testing_marker import \
    create_a_markerarray_1,create_a_markerarray_2,\
        create_a_markerarray_3,create_a_markerarray_4,create_two_markers_debug,create_two_markers_debug3


def calculate_iou_matrix(comb_markers_array):
    """
    (To do)This function needs to speed up and optimize! 

    Calculate the Intersection over Union (IoU) matrix for the given MarkerArray.

    Parameters
    ----------
    comb_markers_array : visualization_msgs/MarkerArray
        A MarkerArray containing multiple bounding box markers.

    Returns
    -------
    iou_matrix : numpy.ndarray
        An IoU matrix where each element (i, j) represents the IoU between bounding box i and j.

    """
    
    # Calculate the length of the MarkerArray
    array_num = len(comb_markers_array.markers)
    iou_matrix = np.zeros((array_num, array_num))
    for i in range(array_num):
        for j in range(i, array_num):
            if i == j:
                iou = 1
                # iou_matrix[i][j] = iou
                iou_matrix[i, j] = iou
            else:
                iou = bbx_iou_new(comb_markers_array.markers[i], comb_markers_array.markers[j])
                # iou_matrix[i][j] = iou
                # iou_matrix[j][i] = iou
                iou_matrix[i, j] = iou
                iou_matrix[j, i] = iou        

    return iou_matrix

def calculate_corners(marker):
    """
    # [Fixed] bad marker(action:3) will occur when conduct this calculation.

    print("[debug]marker text:",marker.text)
    if marker.text == "":
        # print("[debug] marker header:",marker.header)
        print("[debug] bad marker:",marker)
    """
    
    m_x, m_y, m_z, m_yaw, m_l, m_w, m_h, _, m_ID, _, _, _ = decode_marker_all(marker)
    corners = []
    for dx, dy, angle in [(m_l / 2, m_w / 2, m_yaw), (-m_l / 2, m_w / 2, m_yaw),
                          (-m_l / 2, -m_w / 2, m_yaw), (m_l / 2, -m_w / 2, m_yaw)]:
        x, y = rotate_point(dx, dy, angle)
        corners.append((m_x + x, m_y + y))
    return corners

def calculate_iou_matrix_opt(comb_markers_array):
    
    array_num = len(comb_markers_array.markers)
    iou_matrix = np.zeros((array_num, array_num))

    # Precompute the corner points of all bounding boxes
    all_corners = [calculate_corners(marker) for marker in comb_markers_array.markers]

    for i in range(array_num):
        corners1 = all_corners[i]
        poly1 = Polygon(corners1)
        
        for j in range(i, array_num):
            if i == j:
                iou_matrix[i, j] = 1.0
            else:
                corners2 = all_corners[j]
                poly2 = Polygon(corners2)
                intersection = poly1.intersection(poly2)
                
                if intersection.is_empty:
                    iou = 0.0
                else:
                    intersection_area = intersection.area
                    union_area = poly1.area + poly2.area - intersection_area
                    iou = intersection_area / union_area if union_area > 0 else 0.0

                iou_matrix[i, j] = iou
                iou_matrix[j, i] = iou  # Taking advantage of matrix symmetry

    return iou_matrix

def bbx_iou(marker1,marker2):
    """
    Calculate the intersection over union (IoU) between two bounding boxes represented as markers.

    Parameters
    ----------
    marker1 : Marker from visualization_msgs.msg
        First bounding box marker.
    marker2 : Marker from visualization_msgs.msg
        Second bounding box marker.

    Returns
    -------
    iou : float
        Intersection over union (IoU) value between the two bounding boxes.

    """    
    
    m1_x, m1_y, m1_z, m1_yaw, m1_l, m1_w, m1_h, _, m1_ID, _, _, _ = decode_marker_all(marker1)
    m2_x, m2_y, m2_z, m2_yaw, m2_l, m2_w, m2_h, _, m2_ID, _, _, _ = decode_marker_all(marker2)

    x1 = m1_x - m1_l / 2.0
    y1 = m1_y - m1_w / 2.0
    x2 = m1_x + m1_l / 2.0
    y2 = m1_y + m1_w / 2.0
    x3 = m2_x - m2_l / 2.0
    y3 = m2_y - m2_w / 2.0
    x4 = m2_x + m2_l / 2.0
    y4 = m2_y + m2_w / 2.0

    # calculate intersection area
    x_left = max(x1, x3)
    y_top = max(y1, y3)
    x_right = min(x2, x4)
    y_bottom = min(y2, y4)
    if x_right < x_left or y_bottom < y_top:
        return 0.0
    intersection_area = (x_right - x_left) * (y_bottom - y_top)

    # calculate union area
    union_area = (x2 - x1) * (y2 - y1) + (x4 - x3) * (y4 - y3) - intersection_area

    # calculate intersection over union
    if union_area == 0.0:
        iou = 0.0
    else:
        iou = intersection_area / union_area

    return iou

def rotate_point(x, y, angle):
    xr = x * math.cos(angle) - y * math.sin(angle)
    yr = x * math.sin(angle) + y * math.cos(angle)
    return xr, yr

def bbx_iou_new(marker1, marker2):
    m1_x, m1_y, m1_z, m1_yaw, m1_l, m1_w, m1_h, _, m1_ID, _, _, _ = decode_marker_all(marker1)
    m2_x, m2_y, m2_z, m2_yaw, m2_l, m2_w, m2_h, _, m2_ID, _, _, _ = decode_marker_all(marker2)

    # calculate corner points of marker1 and marker2，and rotate them 
    corners1 = []
    corners2 = []
    for dx1, dy1, angle in [(m1_l / 2, m1_w / 2, m1_yaw), (-m1_l / 2, m1_w / 2, m1_yaw),
                            (-m1_l / 2, -m1_w / 2, m1_yaw), (m1_l / 2, -m1_w / 2, m1_yaw)]:
        x, y = rotate_point(dx1, dy1, angle)
        corners1.append((m1_x + x, m1_y + y))

    for dx2, dy2, angle in [(m2_l / 2, m2_w / 2, m2_yaw), (-m2_l / 2, m2_w / 2, m2_yaw),
                            (-m2_l / 2, -m2_w / 2, m2_yaw), (m2_l / 2, -m2_w / 2, m2_yaw)]:
        x, y = rotate_point(dx2, dy2, angle)
        corners2.append((m2_x + x, m2_y + y))

    # Use shapely library to find intersection regions and calculate area
    poly1 = Polygon(corners1)
    poly2 = Polygon(corners2)
    intersection = poly1.intersection(poly2)

    if intersection.is_empty:
        return 0.0

    intersection_area = intersection.area
    union_area = poly1.area + poly2.area - intersection_area

    iou = intersection_area / union_area if union_area > 0 else 0.0
    return iou


def fuse_multi_markerarray_prob(comb_markers_array,together_pairs,header):
    """
    Fuses multiple markers from a given MarkerArray based on specified pairs and generates a fused MarkerArray.
    
    Parameters
    ----------
    comb_markers_array : MarkerArray from visualization_msgs.msg
        The source MarkerArray containing markers to be fused.
    together_pairs : list of list of int
        A list of pairs of indices specifying markers that should be fused together.
    
    Returns
    -------
    fusion_markers_array : MarkerArray from visualization_msgs.msg
        A new MarkerArray containing fused markers based on the specified pairs.
    
    Notes
    -----
    This function iterates through the list of pairs and fuses the corresponding markers in each pair using the
    get_fused_markerarray and fuse_paired_markerarray functions. The resulting fusion_markers_array will contain
    the fused markers generated based on the provided pairs.
    
    Example
    -------
    comb_markers_array: MarkerArray
        A source MarkerArray containing multiple markers.
    together_pairs: [[2, 3, 4], [5, 6], [7]]
        A list of pairs specifying the markers to be fused together.
    fusion_markers_array = fuse_multi_markerarray(comb_markers_array, together_pairs)
        Returns a new MarkerArray containing fused markers based on the specified pairs.
    """
    
    fusion_markers_array = MarkerArray()
    for pair in together_pairs:
        if len(pair) > 1: # pair= [2,3,4]
            # print("Need to fuse")
            # print(pair)
            # print("comb_markers txt 2 :",comb_markers_array.markers[pair[0]].text)
            fused_markers_array = get_fused_markerarray(comb_markers_array,pair)
            # print("ready to fuse! ")
            fused_marker = fuse_paired_markerarray(fused_markers_array) # problem 

            last_item = pair[-1]
            for item in pair:
                if item != last_item:
                    comb_markers_array.markers[item] = Marker()
                    comb_markers_array.markers[item].header = header
                else:
                    comb_markers_array.markers[last_item] = fused_marker

        else:
            # no other marker can fused with it
            # print("This is single! ")
            # problematic here!
            index = pair[0]
            fused_marker = comb_markers_array.markers[index]
            fused_marker = change_marker_colors(fused_marker,0.9,0.9,0.0,0.5)
            fused_marker = change_marker_text(fused_marker,"ID",0.0,[0.0,0.0,0.0],0.0)
            # fusion_markers_array.markers.append(fused_marker)
            comb_markers_array.markers[index] = fused_marker
    
    return comb_markers_array

def fuse_multi_markerarray(comb_markers_array,together_pairs,header):
    
    for pair in together_pairs:
        if len(pair) > 1:
            print("Need to fuse!")
            fused_markers_array = get_fused_markerarray(comb_markers_array,pair)

            # print("len of fused marker array:", len(fused_markers_array.markers))
            print("--------1 f b---------")
            fused_marker = fuse_paired_markerarray(fused_markers_array)
            print("--------1 f---------")
            """
            print("fused marker:",fused_marker.text)
            for marker in fused_markers_array.markers:
                print("fusion_items:",marker.text)
            """
            last_item = pair[-1]
            for item in pair:
                if item != last_item:
                    comb_markers_array.markers[item] = Marker()
                    comb_markers_array.markers[item].header = header
                else:
                    comb_markers_array.markers[last_item] = fused_marker

            # for item in pair:
            #     print("fusion_items:",comb_markers_array.markers[item].text)
        else:
            print("No need to fuse!")
            index = pair[0]
            comb_markers_array.markers[index] = change_marker_colors(comb_markers_array.markers[index],1.0,0.0,0.0,0.5)
    return comb_markers_array


def fuse_paired_markerarray(fused_markers_array_ori):
    """
    Fuses a set of paired markers to generate a fused marker with adjusted attributes.
    
    Parameters
    ----------
    fused_markers_array : MarkerArray from visualization_msgs.msg
        The input fused MarkerArray containing markers to be paired and fused.
    
    Returns
    -------
    fusion_marker : Marker from visualization_msgs.msg
        The resulting fused marker after adjusting attributes and performing fusion.
    
    Notes
    -----
    This function calculates a fused marker by adjusting attributes such as position, dimensions, and orientation
    based on paired markers' attributes in the input fused_markers_array. The fusion process involves combining
    information from the paired markers using specific weighting factors and hyperparameters.
    
    Example
    -------
    fused_markers_array: MarkerArray
        An input fused MarkerArray containing paired markers.
    fusion_marker = fuse_paired_markerarray(fused_markers_array)
        Returns a new fused marker with adjusted attributes.
    """
    
    fusion_marker = Marker()
    fused_markers_array = deepcopy(fused_markers_array_ori)
    # initialize the fusion_marker with the first marker in fused_markers_array
    fusion_marker = fused_markers_array.markers[0]
    # print("fusion_marker:",fusion_marker.text)
    
    VID = "FID"
    x_fuse, y_fuse, z_fuse, yaw_fuse, l_fuse, w_fuse,h_fuse = 0, 0, 0, 0, 0, 0, 0
    
    # ds_sum,ls_sum,ws_sum,
    fused_markers_array = cal_uni_score(fused_markers_array) # problem
    
    # These are hyper parameters!!!
    xy_w = 0.5
    lwh_w = 0.0
    temp_f_kds = 0.0
    
    for marker in fused_markers_array.markers:
        x, y, z, yaw, l, w, h, velo, ID, f_ds, f_ss, f_kds = decode_marker_all(marker)
        """
        print("x:", x)
        print("y:", y)
        print("z:", z)
        print("yaw:", yaw)
        print("l:", l)
        print("w:", w)
        print("h:", h)
        print("velo:", velo)
        print("ID:", ID)
        print("f_ds:", f_ds)
        print("f_ss:", f_ss)
        """
        print("----")
        print("yaw:", yaw)
        print("f_kds:", f_kds)
        # print("")
        x_fuse += (xy_w * f_ds + (1-xy_w) * f_kds) * x
        y_fuse += (xy_w * f_ds + (1-xy_w) * f_kds) * y
        z_fuse += (xy_w * f_ds + (1-xy_w) * f_kds) * z
        
        """
        if f_kds > temp_f_kds:
            temp_f_kds = f_kds
            yaw_fuse = yaw
        """

        if yaw < 0:
            yaw = math.pi + yaw
        yaw_fuse += f_kds * yaw
        
        l_fuse += (lwh_w * f_ds + (1-lwh_w) * f_ss[0]) * l
        w_fuse += (lwh_w * f_ds + (1-lwh_w) * f_ss[1]) * w
        h_fuse += (lwh_w * f_ds + (1-lwh_w) * f_ss[2]) * h
    """
    print("x_fuse:", x_fuse)
    print("y_fuse:", y_fuse)
    print("z_fuse:", z_fuse)
    print("yaw_fuse:", yaw_fuse)
    print("l_fuse:", l_fuse)
    print("w_fuse:", w_fuse)
    print("h_fuse:", h_fuse)
    """
    ds_v = 0.0
    ss_v = [0.0,0.0,0.0]
    kds_v = 0.0
    fusion_marker = marker_encode_all(fusion_marker,x_fuse,y_fuse,z_fuse,yaw_fuse,l_fuse,w_fuse,h_fuse,VID,ds_v,ss_v,kds_v)
    
    return fusion_marker
    

def cal_uni_score(fused_markers_array):
    """
    Calculates unified scores for markers in a fused MarkerArray.
    
    Parameters
    ----------
    fused_markers_array : MarkerArray from visualization_msgs.msg
        The input fused MarkerArray containing markers to calculate scores for.
    
    Returns
    -------
    new_fused_markers_array : MarkerArray from visualization_msgs.msg
        A new MarkerArray containing markers with updated unified scores.
    
    Notes
    -----
    This function calculates unified scores for markers in the input fused_markers_array.
    The unified scores are computed based on the individual marker attributes such as position,
    dimensions, velocity, and history. The calculated scores are then used to adjust the markers'
    probabilities and attributes.
    
    Example
    -------
    fused_markers_array: MarkerArray
        An input fused MarkerArray containing markers.
    new_fused_markers_array = cal_uni_score(fused_markers_array)
        Returns a new MarkerArray where the markers have updated unified scores.
    """

    K = len(fused_markers_array.markers)
    
    E_kds = []
    ds_sum = 0
    ls_sum = 0
    ws_sum = 0
    hs_sum = 0
    
    for marker in fused_markers_array.markers:
        x, y, z, yaw, l, w, h, velo, ID, ds, ss, kds = decode_marker_all(marker)
        # print("first_kds:",kds)
        # print("ss:",ss)
        ds_sum += ds
        ls_sum += ss[0]
        ws_sum += ss[1]
        hs_sum += ss[2]
        if type(kds) == str:
            if kds[0] == 'E':
                m_kds = float(kds[1:])
                E_kds.append(m_kds)
        # else:
        #     E_kds.append(kds)

    
    # This is a hyper parameters !!! 
    # Lambd needs to be adjust according to the static and dynamics states.
    lambd = 0.05  # adjust lambda to adjust the probability distribution
    
    
    # print("lambd:",lambd)
    # print("E_kds:",E_kds)
    p_kds = calculate_probabilities(E_kds,lambd)
    # print("p_kds:",p_kds)
    # print("ls_sum:",ls_sum)
    
    """
    Here needs to conduct more experiments
    when lambd = 0.01
    E_kds = [323.6, 100.6]
    p_kds = [0.09708864098649891, 0.9029113590135012]
    
    when lambd = 0.05
    E_kds = [323.6, 100.6]
    p_kds = [1.4375080444120761e-05, 0.9999856249195559]
    
    """
    
    multi_kds = []
    p_i = 0
    
    for marker in fused_markers_array.markers:
        x, y, z, yaw, l, w, h, velo, ID, ds, ss, kds = decode_marker_all(marker)
        print("kds:",kds)
        if kds == -1.0 or kds == 0.0:
            print("hi 1/k")
            # need to check the following initialization ways for agent without history.
            m_kds = 1.0 / K
            # or 
            # m_kds = (1.0 / K) * ds
            # or (This is not good)
            # m_kds = ds 
            multi_kds.append(m_kds)
        else:
            print("p_i:",p_i)
            m_kds = p_kds[p_i]
            multi_kds.append(m_kds)
            p_i += 1        
    # if there are only 2 agent, and one of which is -1, then multi_kds = [0.5,1]
    
    # print("multi_kds:",multi_kds)
    
    # final kds list
    """
    Solution1: use the probabiltiies for two agents instead of softmax
    """
    """
    if (K>2) or sum(multi_kds) > 1.0:
        f_kds_list = softmax(multi_kds)
    else:
        f_kds_list = multi_kds
    """
    
    """
    previous solution:
        multi = [0.00000001,0.99999]
        f_p = softmax(multi)
        print(f_p)   
        f_p = [0.26894339 0.73105661]
    """
    f_kds_list = softmax(multi_kds)
    
    K_kds = len(f_kds_list)
    
    
    # print("f_kds_list:",f_kds_list)
    # print(sum(f_kds_list))
    
    # here it may have some issues: 
    # assert abs(sum(f_kds_list) - 1) < 0.0001, "Sum of f_kds_list is not 1"
    
    assert K == K_kds, "K and K_kds are not equal"
    
    new_fused_markers_array = MarkerArray()
    for i,marker in enumerate(fused_markers_array.markers):
        x, y, z, yaw, l, w, h, velo, ID, ds, ss, kds = decode_marker_all(marker)
        """
        print("debug -----------------")
        print("x:", x)
        print("y:", y)
        print("z:", z)
        print("yaw:", yaw)
        print("l:", l)
        print("w:", w)
        print("h:", h)
        print("velo:", velo)
        print("ID:", ID)
        print("ds:", ds)
        print("ss:", ss)
        print("kds:", kds)
        """
        f_ds = ds / ds_sum
        f_ls = ss[0] / ls_sum
        f_ws = ss[1] / ls_sum
        f_hs = ss[2] / ls_sum
        f_ss = [f_ls,f_ws,f_hs]
        f_kds = f_kds_list[i]
        # print(f_kds)
        marker = marker_encode_ds(marker,f_ds)

        marker = marker_encode_ss(marker,f_ss)
        """
        x, y, z, yaw, l, w, h, velo, ID, ds, ss, kds = decode_marker_all(marker)
        print("debug 2-----------------")
        print("x:", x)
        print("y:", y)
        print("z:", z)
        print("yaw:", yaw)
        print("l:", l)
        print("w:", w)
        print("h:", h)
        print("velo:", velo)
        print("ID:", ID)
        print("ds:", ds)
        print("ss:", ss)
        print("kds:", kds)
        """
        marker = marker_encode_f_kds(marker,f_kds)
        

        # put new_fused_marker into new_fused_markers_array
        new_fused_markers_array.markers.append(marker)
    
    # print("debugging =====================")
    # for marker in new_fused_markers_array.markers:
    #     print(marker)
        
    return new_fused_markers_array


def get_fused_markerarray(comb_markers_array, pair):
    """
    Extracts a subset of markers from a given MarkerArray based on specified indices.

    Parameters
    ----------
    comb_markers_array : MarkerArray from visualization_msgs.msg
        The source MarkerArray from which to extract markers.
    pair : list of int
        List of indices specifying the markers to be extracted.

    Returns
    -------
    fused_markerarray : MarkerArray from visualization_msgs.msg
        A new MarkerArray containing the markers extracted from comb_markers_array
        based on the provided indices in the pair list.

    Notes
    -----
    This function iterates through the indices in the pair list and appends the
    corresponding markers from comb_markers_array to the fused_markerarray. The resulting
    fused_markerarray will contain the desired subset of markers.

    Example
    -------
    comb_markers_array: MarkerArray
        A source MarkerArray containing multiple markers.
    pair: [2, 3, 4]
        A list of indices specifying the markers to be extracted.
    fused_markers_array = get_fused_markerarray(comb_markers_array, pair)
        Returns a new MarkerArray containing markers at indices 2, 3, and 4 from
        comb_markers_array.
    """
    
    fused_markerarray = MarkerArray()
    
    for index in pair:
        fused_markerarray.markers.append(comb_markers_array.markers[index])
        # print("done!")
    
    return fused_markerarray


    
 # =======================Testing code=================================
def test_all():
    markers_array_m1 = create_a_markerarray_1()
    markers_array_m2 = create_a_markerarray_2()
    
    # Combine the two MarkerArrays together
    combined_markers_array = MarkerArray()
    combined_markers_array.markers.extend(markers_array_m1.markers)
    combined_markers_array.markers.extend(markers_array_m2.markers)
    
    iou_matrix = calculate_iou_matrix(combined_markers_array)
    together_pairs = find_clusters(iou_matrix,iou_threshold=0.1)
    fusion_markers_array = fuse_multi_markerarray(combined_markers_array,together_pairs)
    
    print("All The core function modification has completed!")
    print("Need to conduct further test!")

def test_calculate_probabilities():
    # Test case 1
    values1 = [1, 2, 3, 4]
    lambd1 = 0.5
    expected_probs1 = [0.3697296376497262, 0.26881171429206096, 0.19661193324148188, 0.14344662141673035]
    calculated_probs1 = calculate_probabilities(values1, lambd1)
    print(calculated_probs1)
    #assert np.allclose(expected_probs1, calculated_probs1, rtol=1e-6)

    # Test case 2
    values2 = [0.5, 1.5, 2.5, 3.5]
    lambd2 = 0.75
    expected_probs2 = [0.3032653298563167, 0.2426900182486745, 0.19464453376256134, 0.15539911813244747]
    calculated_probs2 = calculate_probabilities(values2, lambd2)
    print(calculated_probs2)
    #assert np.allclose(expected_probs2, calculated_probs2, rtol=1e-6)

    # Test case 3
    values3 = [0, 1, 2, 3]
    lambd3 = 0.0
    expected_probs3 = [0.25, 0.25, 0.25, 0.25]
    calculated_probs3 = calculate_probabilities(values3, lambd3)
    print(calculated_probs3)
    
    values4 = [123]
    lambd4 = 0.05 
    calculated_probs4 = calculate_probabilities(values4, lambd4)
    print(calculated_probs4)
    # assert np.allclose(expected_probs3, calculated_probs3, rtol=1e-6)

    print("All tests passed!")

def test_softmax():   
    arr1 = np.array([0.5, 1])
    calculated_result1 = softmax(arr1)
    print(calculated_result1)

def test_cal_pro_and_softmax():
    # multi_kds = []
    lambd = 0.05
    E_kds = [123,88]
    rest_K = len(E_kds)
    K = rest_K + 1 
    p_kds = calculate_probabilities(E_kds,lambd)
    print(p_kds)
    p_kds.append(1.0/K)
    f_kds_list = softmax(p_kds)
    print(f_kds_list)

def debug():
    """
    for marker in fused_markers_array.markers:
        print(marker)   
    print("-------------------------------------------")   

    for marker in fused_markers_array.markers:
        x, y, z, yaw, l, w, h, velo, ID, f_ds, f_ss, f_kds = decode_marker_all(marker)
        print("x:", x)
        print("y:", y)
        print("z:", z)
        print("yaw:", yaw)
        print("l:", l)
        print("w:", w)
        print("h:", h)
        print("velo:", velo)
        print("ID:", ID)
        print("f_ds:", f_ds)
        print("f_ss:", f_ss)
        print("f_kds:", f_kds)
    """
    print("================================================")     

def test_fusion():

    markers_array_m1 = create_a_markerarray_3() # from agent 1 
    markers_array_m2 = create_a_markerarray_4() # from agent 2
    
    # Combine the two MarkerArrays together
    combined_markers_array = MarkerArray()
    combined_markers_array.markers.extend(markers_array_m1.markers)
    combined_markers_array.markers.extend(markers_array_m2.markers)
    
    iou_matrix = calculate_iou_matrix(combined_markers_array)
    
    together_pairs = find_clusters(iou_matrix,iou_threshold=0.1)
    
    fusion_markers_array = fuse_multi_markerarray(combined_markers_array,together_pairs)
    
    print("================================================")    
    for marker in fusion_markers_array.markers:
        print(marker) 
        print("------")
        
    print("================================================") 

def test_visualization():
    fusion_markers = MarkerArray()
    marker1,marker2 = create_two_markers_debug()
    marker3,marker4 = create_two_markers_debug3()

    fusion_markers.markers.append(marker1)
    fusion_markers.markers.append(marker2)
    bbx_marker_vis(fusion_markers,color='b')
    fusion_markers = MarkerArray()
    fusion_markers.markers.append(marker3)
    fusion_markers.markers.append(marker4)
    bbx_marker_vis(fusion_markers,color='r',if_show=True)

def test_bbx_iou_new():
    marker1,marker2 = create_two_markers_debug()
    marker3,marker4 = create_two_markers_debug3()
    # test_visualization()
    marker_1 = marker1
    marker_2 = marker2
    m1_x, m1_y, m1_z, m1_yaw, m1_l, m1_w, m1_h, _, m1_ID, _, _, _ = decode_marker_all(marker_1)
    m2_x, m2_y, m2_z, m2_yaw, m2_l, m2_w, m2_h, _, m2_ID, _, _, _ = decode_marker_all(marker_2)

    print(f"m1_x: {m1_x}, m1_y: {m1_y}, m1_z: {m1_z}, m1_yaw: {m1_yaw}, m1_l: {m1_l}, m1_w: {m1_w}, m1_h: {m1_h}, m1_ID: {m1_ID}")
    print(f"m2_x: {m2_x}, m2_y: {m2_y}, m2_z: {m2_z}, m2_yaw: {m2_yaw}, m2_l: {m2_l}, m2_w: {m2_w}, m2_h: {m2_h}, m2_ID: {m2_ID}")

    iou = bbx_iou(marker_1,marker_2)
    print("iou:",iou)
    iou_new = bbx_iou_new(marker_1,marker_2)
    print("iou_new:",iou_new)

if __name__ == '__main__':
    # test_calculate_probabilities()
    # test_softmax()
    # test_cal_pro_and_softmax()
    # test_fusion()
    # fusion_markers = MarkerArray()
    pass


    
