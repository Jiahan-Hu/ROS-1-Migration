#! /usr/bin/env python
# coding: utf-8

import rospy 
from testing_marker import create_a_testing_marker
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler

def marker_encode_ds(marker,new_ds):
    """
    This code is to replace ss in marker.text with new_ds 

    Parameters
    ----------
    marker : Marker from visualization_msgs.msg
        Example: Marker.text="6.14 m/s , <ID= 4>, <ds= 0.958259>, <ss= 0.0,0.0,0.0>, <kds= 0.0>"
    new_ds : float
        Example: new_ds = 0.35

    Returns
    -------
    ds_marker : Marker from visualization_msgs.msg
        ss_marker.text = "6.14 m/s , <ID= 4>, <ds= 0.3500>, <ss= 0.3,0.4,0.5>, <kds= 0.0>"

    """    
    # Find the start and end positions of <ds= ...> in marker.text
    ds_start = marker.text.find("<ds=")

    if ds_start != -1:
        ds_end = marker.text.find(">", ds_start)
        # Replace the existing ds value with new_ds formatted to four decimal places
        ds_value = marker.text[ds_start:ds_end+1]
        print(ds_value)
        new_ds_string = f"<ds= {new_ds:.4f}>"
        marker.text = marker.text.replace(ds_value, new_ds_string)
    else:
        assert ds_start != -1, "no DS in the text!"

    return marker

def marker_encode_ss(marker, new_ss):
    """
    This code is to replace ss in marker.text with new_ss 

    Parameters
    ----------
    marker : Marker from visualization_msgs.msg
        Example: Marker.text="6.14 m/s , <ID= 4>, <ds= 0.958259>, <ss= 0.0,0.0,0.0>, <kds= 0.0>"
    new_ss : list
        Example: new_ss = [0.3, 0.4, 0.5]

    Returns
    -------
    marker : Marker from visualization_msgs.msg
        ss_marker.text = "6.14 m/s , <ID= 4>, <ds= 0.958259>, <ss= 0.3,0.4,0.5>, <kds= 0.0>"

    """
    # Find the starting index of "<ss=" in the marker text
    ss_start = marker.text.find("<ss=")
    
    # If "<ss=" is found in the marker text
    if ss_start != -1:
        # Find the ending index of the current ss value
        ss_end = marker.text.find(">", ss_start)
        
        # Extract the current ss value
        current_ss = marker.text[ss_start + 5:ss_end]
        # print(current_ss)
        
        # Create the new ss value string
        # new_ss_str = ",".join([str(val) for val in new_ss])
        new_ss_str = ",".join([f"{val:.4f}" for val in new_ss])
        
        # Replace the current ss value with the new_ss value
        updated_text = marker.text[:ss_start + 5] + new_ss_str + marker.text[ss_end:]
        
        # Create a new Marker instance with the updated text
        marker.text = updated_text
    else:
        # If "<ss=" is not found, report error info 
        rospy.logerr("no SS in the text!")
        # ss_marker = marker
        assert ss_start != -1, "no SS in the text!"
    
    return marker

def marker_encode_kds(marker, new_kds):
    """
    This code is to replace ss in marker.text with new_kds

    Parameters
    ----------
    marker : Marker from visualization_msgs.msg
        Example: Marker.text= "4.23 m/s, <ID= 4>, <ds= 0.924100>, <ss= 0.3,0.4,0.5>, <kds= 0.0>"
    new_kds : float
        DESCRIPTION.

    Returns
    -------
    marker : Marker from visualization_msgs.msg
        Example:
            marker.text = "6.14 m/s , <ID= 4>, <ds= 0.958259>, <ss= 0.3,0.4,0.5>, <kds= -1>"
            or 
            marker.text = "6.14 m/s , <ID= 4>, <ds= 0.958259>, <ss= 0.3,0.4,0.5>, <kds= E123.1>"

    """

    kds_start = marker.text.find("<kds=")
    
    if kds_start != -1:
        kds_end = marker.text.find(">", kds_start)
        
        current_kds = marker.text[kds_start + 5:kds_end]
        
        if new_kds == -1:  
            new_kds = f"{new_kds:.4f}"
            updated_text = marker.text[:kds_start + 6] + new_kds + marker.text[kds_end:]
        else:
            minE = 'E' + str(new_kds)
            minE = f"E{new_kds:.4f}"
            updated_text = marker.text[:kds_start + 6] + minE + marker.text[kds_end:]
        
        marker.text = updated_text
    else:
        rospy.logerr("no KDS in the text!")
        marker.text = "no KDS in the text!"
        assert kds_start != -1, "no KDS in the text!"
    
    return marker 

def marker_encode_f_kds(marker, f_kds):
    """
    This code is to replace <kds> in marker.text with new_kds

    Parameters
    ----------
    marker : Marker from visualization_msgs.msg
        Example:
            kds_marker.text = "6.14 m/s , <ID= 4>, <ds= 0.958259>, <ss= 0.3,0.4,0.5>, <kds= -1>"
            or 
            kds_marker.text = "6.14 m/s , <ID= 4>, <ds= 0.958259>, <ss= 0.3,0.4,0.5>, <kds= E123.1>"

    f_kds : float <= 1.0
        Final kds.
        Example: 0.78

    Returns
    -------
    marker : Marker from visualization_msgs.msg
        Example:
            marker.text = "6.14 m/s , <ID= 4>, <ds= 0.958259>, <ss= 0.3,0.4,0.5>, <kds= 0.7800>"
    """

    # Find the start and end positions of <kds= ...> in marker.text
    kds_start = marker.text.find("<kds=")

    if kds_start != -1:
        kds_end = marker.text.find(">", kds_start)
        # Replace the existing <kds> value with f_kds formatted to four decimal places
        kds_value = marker.text[kds_start:kds_end]
        # print(kds_value)
        new_kds_string = f"<kds= {f_kds:.4f}"
        marker.text = marker.text.replace(kds_value, new_kds_string)
    else:
        assert kds_start != -1, "no KDS in the text!"

    return marker
  

def marker_encode_all(marker,x,y,z,yaw,l,w,h,ID_v,ds_v,ss_v,kds_v):

    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z

    quaternion = quaternion_from_euler(0, 0, yaw)
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]
    
    marker.scale.x = l
    marker.scale.y = w
    marker.scale.z = h

    # change marker color to yellow
    marker = change_marker_colors(marker,0.9,0.9,0.0,0.5)

    marker = change_marker_text(marker,ID_v,ds_v,ss_v,kds_v)
    return marker


def change_marker_colors(marker,r,g,b,a):
    # change marker color to yellow
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = a
    return marker


def change_marker_text(marker,ID_v,ds_v,ss_v,kds_v):
    marker.text = f"<ID= {ID_v}>, <ds= {ds_v:.4f}>, <ss= {ss_v[0]:.4f},{ss_v[1]:.4f},{ss_v[2]:.4f}>, <kds= {kds_v:.4f}>"
    return marker 


def test1():
    # call marker_encode_ss function for testing 
    example_marker = create_a_testing_marker()
    new_ss_values = [0.312331, 0.91, 0.9]
    updated_marker = marker_encode_ss(example_marker, new_ss_values)
    
    # print updated marker.text
    print(updated_marker.text)    

def test2():
    # call marker_encode_kds function for testing 
    marker = create_a_testing_marker()
    print(marker.text)
    kds = -1
    
    new_marker = marker_encode_kds(marker, kds)
    print(new_marker.text)

def test_encoder_ds():
    marker = create_a_testing_marker()  
    print(marker)
    print('-------------')
    new_ds = 0.46
    marker_encode_ds(marker,new_ds)  
    print(marker)
if __name__ == '__main__':
    test_encoder_ds()

    
    



