#! /usr/bin/env python
# coding: utf-8

import rospy
from tf.transformations import euler_from_quaternion

def decode_marker_text(marker_text):
    """
    This code is to decode the marker_text into velo, ID, ds, ss, kds.
    
    Parameters:
    marker_text (str): The marker text containing velocity, ID, ds, ss, and kds information.
        - Example1: "6.14 m/s , <ID= 4>, <ds= 0.958259>, <ss= 0.0,0.0,0.0>, <kds= 0.0>"
        - Example2: "6.14 m/s , <ID= 4>, <ds= 0.958259>, <ss= 0.0,0.0,0.0>, <kds= -1.0000>"
        - Example3: "6.14 m/s , <ID= 4>, <ds= 0.958259>, <ss= 0.0,0.0,0.0>, <kds= E123.4000>"

    Returns:
    velo (float): Velocity value. [m/s]
    ID (int): Marker ID.
    ds (float): ds value.
    ss (list): List of ss values.
    kds (float) or (string): kds value.
    """
    
    parts = marker_text.split(',')
    parts = [part.strip() for part in parts]

    # Extracting velocity from the first part
    # print(parts[0].split()[0])
    velo = float(parts[0].split()[0])  # Extracting the numerical value

    # Extracting ID from the second part
    ID = int(parts[1].split('=')[1].split(">")[0])  # Extracting the numerical value after '='

    # Extracting ds from the third part
    ds = float(parts[2].split('=')[1].split(">")[0]) # Extracting the numerical value after '='

    # Extracting ss from the fourth part and converting it to a list of floats
    ss_l = float(parts[3].split('=')[1])
    ss_w = float(parts[4])
    ss_h = float(parts[5].split('>')[0])
    ss = [ss_l,ss_w,ss_h]

    # Extracting kds from the fifth part
    kds_content = parts[6].split('=')[1].split('>')[0]
    if kds_content[1] == 'E':
        kds = kds_content[1:]
    else:
        kds = float(kds_content)  # Extracting the numerical value after '='
    # print(kds)
    return velo, ID, ds, ss, kds

def decode_maker_shape(marker_pose):
    """
    Decode the marker_pose into x, y, z, and yaw.

    Parameters
    ----------
    marker_pose : geometry_msgs/Pose
        The pose of visulization_msgs/markerArray

    Returns
    -------
    x : float
        x-coordinate of the marker's position.
    y : float
        y-coordinate of the marker's position.
    z : float
        z-coordinate of the marker's position.
    yaw : float
        Yaw angle of the marker, converted from marker_pose.orientation.

    """
    x = marker_pose.position.x
    y = marker_pose.position.y
    z = marker_pose.position.z
    
    # Assuming that marker_pose.orientation is represented using Quaternions
    quaternion = (
        marker_pose.orientation.x,
        marker_pose.orientation.y,
        marker_pose.orientation.z,
        marker_pose.orientation.w
    )
    
    # Convert Quaternion to Euler angles (roll, pitch, yaw)
    euler = euler_from_quaternion(quaternion)
    
    # roll = euler[0]
    # pitch = euler[0]
    yaw = euler[2]  # Yaw angle is the third element in Euler angles
    
    return x, y, z, yaw

def decode_marker_scale(marker_scale):
    """
    Convert marker_scale into length, width, height

    Parameters
    ----------
    marker_scale : geometry_msgs/Vector3
        The scale of the marker.

    Returns
    -------
    length : float
        Length of the marker.
    width : float
        Width of the marker.
    height : float
        Height of the marker.
    """
    
    length = marker_scale.x
    width = marker_scale.y
    height = marker_scale.z
    
    return length, width, height

def decode_marker_all(marker):
    """
    Decode various properties of a marker.

    Parameters
    ----------
    marker : visualization_msgs.msg/MarkerArray
        The marker to be decoded.

    Returns
    -------
    x : float
        x-coordinate of the marker's position.
    y : float
        y-coordinate of the marker's position.
    z : float
        z-coordinate of the marker's position.
    yaw : float
        Yaw angle of the marker.
    l : float
        Length of the marker.
    w : float
        Width of the marker.
    h : float
        Height of the marker.
    velo : float
        Velocity value.
    ID : int
        Marker ID.
    ds : float
        ds value.
    ss : list
        List of ss values.
    kds : (float) or (string)
        kds value.
    """
    x, y, z, yaw = decode_maker_shape(marker.pose)
    l, w, h = decode_marker_scale(marker.scale)
    velo, ID, ds, ss, kds = decode_marker_text(marker.text)
    
    return x, y, z, yaw, l, w, h, velo, ID, ds, ss, kds

def decode_marker_minE(marker_text):
    """
    This code is to get the minE from marker_text

    Parameters
    ----------
    marker_text : string
        text from Visualization_msgs/marker
        Example: 
            kds_marker.text = "6.14 m/s , <ID= 4>, <ds= 0.958259>, <ss= 0.3,0.4,0.5>, <kds= -1.0000>"
            or 
            kds_marker.text = "6.14 m/s , <ID= 4>, <ds= 0.958259>, <ss= 0.3,0.4,0.5>, <kds= E123.100>"

    Returns
    -------
    minE : float
        The minimum energy consumption during bbx state transfer

    """
    kds_start = marker_text.find("<kds=")
    if kds_start!= -1:
        # kds_start = text1.find("<kds=")
        kds_end = marker_text.find(">", kds_start)
        current_kds = marker_text[kds_start + 5:kds_end]
        if current_kds[1] == 'E':
            minE = float(current_kds[2:])
        elif current_kds[1] == '-':
            minE = -1.0
        else:
            minE = 0.0
    else:
        rospy.logerr("no KDS in the text!")
        minE = 0.0
    
    return minE 
    
    
def test1():
    marker_text = "5.71 m/s, <ID= 4>, <ds= 0.962920>, <ss= 0.0,0.0,0.0>, <kds= 0.0>"

    velo, ID, ds, ss, kds = decode_marker_text(marker_text)
    # Printing out the decoded values
    print("Velocity:", velo)
    print("ID:", ID)
    print("ds:", ds)
    print("ss:", ss)
    print("kds:", kds)
    
def test2():
    

    # Assuming quaternion is a tuple (x, y, z, w)
    quaternion = (0.0, 0.0, 0.401361664001, 0.915919655139)  # Example quaternion values
    euler_angles = euler_from_quaternion(quaternion)
    
    roll = euler_angles[0]  # Rotation around x-axis
    pitch = euler_angles[1]  # Rotation around y-axis
    yaw = euler_angles[2]  # Rotation around z-axis (yaw angle)
    
    print(euler_angles)

def test3():
    # Test the decode_marker_scale function
    from geometry_msgs.msg import Vector3
    
    # Create a Vector3 object representing marker scale
    marker_scale = Vector3(1.5, 0.8, 2.0)
    
    # Decode the marker scale
    length, width, height = decode_marker_scale(marker_scale)
    
    # Print the results
    print("Marker Length:", length)
    print("Marker Width:", width)
    print("Marker Height:", height)    

def test4():
    # Test the decode_marker_all function
    from visualization_msgs.msg import Marker
    from geometry_msgs.msg import Pose, Vector3

    # Create a sample MarkerArray
    marker = Marker()
    marker.pose = Pose()
    marker.scale = Vector3(1.5, 0.8, 2.0)
    marker.text = "6.14 m/s , <ID= 4>, <ds=0.958259>, <ss=0.1,0.2,0.3>, <kds=0.9>"

    # Decode the marker properties
    x, y, z, yaw, l, w, h, velo, ID, ds, ss, kds = decode_marker_all(marker)
    
    # Print the decoded properties
    print("Marker Position (x, y, z):", x, y, z)
    print("Marker Yaw:", yaw)
    print("Marker Dimensions (length, width, height):", l, w, h)
    print("Marker Velocity:", velo)
    print("Marker ID:", ID)
    print("ds:", ds)
    print("ss:", ss)
    print("kds:", kds)

def test_decode_marker_all():
    from visualization_msgs.msg import Marker
    from geometry_msgs.msg import Pose, Vector3
    # Create a sample Marker message
    marker = Marker()
    marker.header.stamp.secs = 1671230178
    marker.header.stamp.nsecs = 99892855
    marker.header.frame_id = "rslidar"
    marker.ns = "objects/rslidar"
    marker.id = 4
    marker.type = 1
    marker.action = 0
    marker.pose.position.x = 14.5323152542
    marker.pose.position.y = 16.8133773804
    marker.pose.position.z = -1.7916585207
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.401361664001
    marker.pose.orientation.w = 0.915919655139
    marker.scale.x = 3.91373252869
    marker.scale.y = 2.5068359375
    marker.scale.z = 1.62611460686
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 0.5
    marker.points.append(Vector3(15.8587137064, 18.2521243082, -2.60471582413))
    marker.points.append(Vector3(19.7265201411, 22.4475412356, -2.60471582413))
    marker.text = "5.71 m/s, <ID= 101>, <ds= 0.962920>, <ss= 0.0,0.0,0.0>, <kds= 0.6>"
    marker.mesh_resource = ''
    marker.mesh_use_embedded_materials = False

    # Call the decode_marker_all function
    x, y, z, yaw, l, w, h, velo, ID, ds, ss, kds = decode_marker_all(marker)
    
    # Print the decoded properties
    print("Marker Position (x, y, z):", x, y, z)
    print("Marker Yaw:", yaw)
    print("Marker Dimensions (length, width, height):", l, w, h)
    print("Marker Velocity:", velo)
    print("Marker ID:", ID)
    print("ds:", ds)
    print("ss:", ss)
    print("kds:", kds)   

def test_decode_marker_text2():
    from visualization_msgs.msg import Marker
    from geometry_msgs.msg import Pose, Vector3
    # Create a sample Marker message
    marker = Marker()
    marker.header.stamp.secs = 1671230178
    marker.header.stamp.nsecs = 99892855
    marker.header.frame_id = "rslidar"
    marker.ns = "objects/rslidar"
    marker.id = 4
    marker.type = 1
    marker.action = 0
    marker.pose.position.x = 14.5323152542
    marker.pose.position.y = 16.8133773804
    marker.pose.position.z = -1.7916585207
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.401361664001
    marker.pose.orientation.w = 0.915919655139
    marker.scale.x = 3.91373252869
    marker.scale.y = 2.5068359375
    marker.scale.z = 1.62611460686
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 0.5
    marker.points.append(Vector3(15.8587137064, 18.2521243082, -2.60471582413))
    marker.points.append(Vector3(19.7265201411, 22.4475412356, -2.60471582413))
    marker.text = "5.71 m/s, <ID= 101>, <ds= 0.962920>, <ss= 0.0,0.0,0.0>, <kds= E123.4000>"
    marker.mesh_resource = ''
    marker.mesh_use_embedded_materials = False   

    decode_marker_text(marker.text)

def test_decode_marker_minE():
    marker_text_1 = "6.14 m/s , <ID= 4>, <ds= 0.958259>, <ss= 0.3,0.4,0.5>, <kds= -1.0000>"
    marker_text_2 = "6.14 m/s , <ID= 4>, <ds= 0.958259>, <ss= 0.3,0.4,0.5>, <kds= E123.1000>"
    marker_text_3 = "6.14 m/s , <ID= 4>, <ds= 0.958259>, <ss= 0.3,0.4,0.5>"
    
    minE_1 = decode_marker_minE(marker_text_1)
    minE_2 = decode_marker_minE(marker_text_2)
    minE_3 = decode_marker_minE(marker_text_3)
    
    assert minE_1 == -1.0, "Test case 1 failed"
    assert minE_2 == 123.1, "Test case 2 failed"
    assert minE_3 == 0.0, "Test case 3 failed"
    
    print("All test cases passed!")
    
if __name__ == "__main__":
    # test_decode_marker_all()
    # test_decode_marker_minE()
    test_decode_marker_text2()


    


    
