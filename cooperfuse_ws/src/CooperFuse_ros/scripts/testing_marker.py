#! /usr/bin/env python
# coding: utf-8
import math 
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Vector3
import tf.transformations as tf_transformations

def create_a_testing_marker():
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
    marker.text = "4.23 m/s, <ID= 4>, <ds= 0.924100>, <ss= 0.0,0.0,0.0>, <kds= 0.0>"
    marker.mesh_resource = ''
    marker.mesh_use_embedded_materials = False
    return marker


def create_a_testing_marker_with_feature(x, y, z, yaw, l, w, h, velo, ID, ds, ss, kds):
    # Create a sample Marker message
    marker = Marker()
    marker.header.stamp.secs = 1671230178
    marker.header.stamp.nsecs = 99892855
    marker.header.frame_id = "rslidar"
    marker.ns = "objects/rslidar"
    marker.id = 4
    marker.type = 1
    marker.action = 0
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    # create orientation from yaw 
    quaternioin = yaw_to_quaternion(yaw) 
    
    marker.pose.orientation.x = quaternioin[0]
    marker.pose.orientation.y = quaternioin[1]
    marker.pose.orientation.z = quaternioin[2]
    marker.pose.orientation.w = quaternioin[3]

    marker.scale.x = l
    marker.scale.y = w
    marker.scale.z = h
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 0.5
    # marker.points.append(Vector3(15.8587137064, 18.2521243082, -2.60471582413))
    # marker.points.append(Vector3(19.7265201411, 22.4475412356, -2.60471582413))
    if type(kds)== str:
        text = text = str(velo)+" m/s" + ", <ID= " + str(ID) + ">, <ds= " + str(ds) + ">, <ss= " + str(ss[0])+','+ str(ss[1])+',' + str(ss[2])+">, <kds= "+ kds +'>'
    else:
        text = str(velo)+" m/s" + ", <ID= " + str(ID) + ">, <ds= " + str(ds) + ">, <ss= " + str(ss[0])+','+ str(ss[1])+',' + str(ss[2])+">, <kds= "+str(kds)+'>'
    marker.text = text # "5.71 m/s, <ID= 4>, <ds= 0.962920>, <ss= 0.0,0.0,0.0>, <kds= 0.0>"
    marker.mesh_resource = ''
    marker.mesh_use_embedded_materials = False
    return marker

def create_a_markerarray_1():
    # This function is to mimic agent 1 in previous time stamp 

    marker_array = MarkerArray()

    # Add multiple markers with different features to the MarkerArray
    marker1 = create_a_testing_marker_with_feature(1.4, 12.4, 0.0, math.radians(60.0), 5.5, 3.6, 2.2, 0.0, 3, 0.962920, [0.0, 0.0, 0.0], 0.0)
    marker_array.markers.append(marker1)

    marker2 = create_a_testing_marker_with_feature(2.0, 3.0, 0.0, math.radians(-30.0), 4.4, 3.3, 2.1, 6.5, 4, 0.8, [0.0, 0.0, 0.0], 0.0)
    marker_array.markers.append(marker2)

    # Add more markers with different features as needed

    return marker_array

def create_a_markerarray_2():
    # This function is to mimic agent 1 in current time stamp 

    marker_array = MarkerArray()

    # Add multiple markers with different features to the MarkerArray
    marker1 = create_a_testing_marker_with_feature(1.3, 12.3, 0.0, math.radians(45.0), 5.3, 3.8, 2.8, 0.0, 3, 0.975, [0.0, 0.0, 0.0], 0.0)
    marker_array.markers.append(marker1)

    marker2 = create_a_testing_marker_with_feature(22.0, 23.0, 0.0, math.radians(-30.0), 4.4, 3.3, 2.1, 6.6, 5, 0.8, [0.0, 0.0, 0.0], 0.0)
    marker_array.markers.append(marker2)

    # Add more markers with different features as needed

    return marker_array

def create_a_markerarray_3():
    # This function is to mimic agent 1 

    marker_array = MarkerArray()

    # Add multiple markers with different features to the MarkerArray
    marker1 = create_a_testing_marker_with_feature(1.4, 12.6, 0.0, math.radians(60.0), 5.5, 3.6, 2.2, 0.0, 3, 0.962920, [0.9, 0.9, 0.9], 'E1323.6000')
    marker_array.markers.append(marker1)

    marker2 = create_a_testing_marker_with_feature(2.0, 3.0, 0.0, math.radians(-30.0), 4.4, 3.3, 2.1, 6.5, 4, 0.8, [0.0, 0.0, 0.0], 0.0)
    marker_array.markers.append(marker2)

    # Add more markers with different features as needed

    return marker_array

def create_a_markerarray_4():
    # This function is to mimic agent 2

    marker_array = MarkerArray()

    # Add multiple markers with different features to the MarkerArray
    marker1 = create_a_testing_marker_with_feature(1.2, 11.8, 0.0, math.radians(45.0), 5.3, 3.8, 2.8, 0.0, 7, 0.975, [0.4, 0.4, 0.4], 'E1100.6000')
    marker_array.markers.append(marker1)

    marker2 = create_a_testing_marker_with_feature(22.0, 23.0, 0.0, math.radians(-30.0), 4.4, 3.3, 2.1, 6.5, 5, 0.8, [0.0, 0.0, 0.0], 0.0)
    marker_array.markers.append(marker2)

    # Add more markers with different features as needed

    return marker_array

def yaw_to_quaternion(yaw):
    """
    Convert a yaw angle (in radians) to a quaternion representation.

    Parameters
    ----------
    yaw : float
        The yaw angle in radians.

    Returns
    -------
    quaternion : list
        A list containing the quaternion representation of the rotation.
        The order of elements in the list is [x, y, z, w], where 'w' is the scalar part of the quaternion.

    Notes
    -----
    The quaternion representation of a rotation is commonly used in robotics and computer graphics.
    The yaw angle is the rotation around the vertical (up) axis.

    Example
    -------
    >>> yaw_angle = 1.57  # 90 degrees in radians
    >>> quaternion = yaw_to_quaternion(yaw_angle)
    >>> print(quaternion)
    [0.0, 0.0, 0.7071067811865476, 0.7071067811865476]
    """

    quaternion = tf_transformations.quaternion_from_euler(0, 0, yaw)
    return quaternion


def test_yaw_to_quaternion():
    test_cases_degrees = [0, 45, 90, 180]
    
    for yaw_degrees in test_cases_degrees:
        yaw_radians = math.radians(yaw_degrees)
        quaternion = yaw_to_quaternion(yaw_radians)
        print(f"Yaw angle (degrees): {yaw_degrees}")
        print(f"Quaternion: {quaternion}")
        print()

def test_create_a_testing_marker_with_feature():
    x = 1.0
    y = 2.0
    z = 0.0
    yaw = math.radians(45.0)
    l = 0.5
    w = 0.3
    h = 0.2
    velo = 5.71
    ID = 4
    ds = 0.962920
    ss = [0.0, 0.0, 0.0]
    kds = "0.0"
    
    marker = create_a_testing_marker_with_feature(x, y, z, yaw, l, w, h, velo, ID, ds, ss, kds)
    
    print("Generated Marker:")
    print(marker)

def create_two_markers_debug():
    # create marker1
    marker1 = Marker()
    marker1.header = Header()
    marker1.header.seq = 0
    marker1.header.stamp.secs = 1694557982
    marker1.header.stamp.nsecs = 562543154
    marker1.header.frame_id = "map"
    marker1.ns = "objects/veh2_rslidar"
    marker1.id = 12
    marker1.type = Marker.SPHERE
    marker1.action = Marker.ADD
    marker1.pose.position.x = 47.113874938781095
    marker1.pose.position.y = -204.3048342043482
    marker1.pose.position.z = 0.5810575647434737
    marker1.pose.orientation.x = -0.016862908254076932
    marker1.pose.orientation.y = -0.04257660989204505
    marker1.pose.orientation.z = 0.6616244525864502
    marker1.pose.orientation.w = 0.7484356741597737
    marker1.scale.x = 6.937153339385986
    marker1.scale.y = 2.821444511413574
    marker1.scale.z = 2.6525778770446777
    marker1.color.r = 1.0
    marker1.color.g = 0.0
    marker1.color.b = 0.0
    marker1.color.a = 0.5
    marker1.lifetime.secs = 0
    marker1.lifetime.nsecs = 0
    marker1.frame_locked = False

    # create marker2
    marker2 = Marker()
    marker2.header = Header()
    marker2.header.seq = 0
    marker2.header.stamp.secs = 1694557982
    marker2.header.stamp.nsecs = 562543154
    marker2.header.frame_id = "map"
    marker2.ns = "objects/veh2_rslidar"
    marker2.id = 30
    marker2.type = Marker.SPHERE
    marker2.action = Marker.ADD
    marker2.pose.position.x = 43.8382423259132
    marker2.pose.position.y = -205.16381439625917
    marker2.pose.position.z = 0.028799731118660077
    marker2.pose.orientation.x = -0.018651295780087847
    marker2.pose.orientation.y = -0.041824090550883264
    marker2.pose.orientation.z = 0.6927339478540665
    marker2.pose.orientation.w = 0.7197378356775713
    marker2.scale.x = 4.361310958862305
    marker2.scale.y = 2.2126331329345703
    marker2.scale.z = 1.6804648637771606
    marker2.color.r = 1.0
    marker2.color.g = 0.0
    marker2.color.b = 0.0
    marker2.color.a = 0.5
    marker2.lifetime.secs = 0
    marker2.lifetime.nsecs = 0
    marker2.frame_locked = False    

    marker1.text = "0.00 m/s, <ID= 26000>, <ds= 0.974132>, <ss= 0.0,0.0,0.0>, <kds= 0.0>"
    marker2.text = "3.55 m/s, <ID= 26062>, <ds= 0.989590>, <ss= 0.0,0.0,0.0>, <kds= 0.0>"

    return marker1,marker2

def create_two_markers_debug2():
    """
    This is false negative
    """
    marker1 = Marker()
    marker1.header.seq = 0
    marker1.header.stamp.secs = 1694557988
    marker1.header.stamp.nsecs = 562779188
    marker1.header.frame_id = "map"
    marker1.ns = "objects/veh2_rslidar"
    marker1.id = 9
    marker1.type = Marker.CUBE
    marker1.action = Marker.ADD
    marker1.pose.position.x = 43.08239940050596
    marker1.pose.position.y = -187.21411123951162
    marker1.pose.position.z = 0.4890363083709628
    marker1.pose.orientation.x = 0.04242496909704913
    marker1.pose.orientation.y = -0.01685304085833203
    marker1.pose.orientation.z = -0.7468647396873147
    marker1.pose.orientation.w = 0.6634072336225627
    marker1.scale.x = 4.491794109344482
    marker1.scale.y = 2.2491180896759033
    marker1.scale.z = 1.7093585729599
    marker1.color.r = 1.0
    marker1.color.g = 0.0
    marker1.color.b = 0.0
    marker1.color.a = 0.5
    marker1.lifetime.secs = 0
    marker1.lifetime.nsecs = 0
    marker1.frame_locked = False
    marker1.points = []
    marker1.colors = []
    marker1.text = "0.00 m/s, <ID= 26009>, <ds= 0.979519>, <ss= 0.0,0.0,0.0>, <kds= 0.0>"
    marker1.mesh_resource = ''
    marker1.mesh_use_embedded_materials = False

    marker2 = Marker()
    marker2.header.seq = 0
    marker2.header.stamp.secs = 1694557988
    marker2.header.stamp.nsecs = 562779188
    marker2.header.frame_id = "map"
    marker2.ns = "objects/veh2_rslidar"
    marker2.id = 19
    marker2.type = Marker.CUBE
    marker2.action = Marker.ADD
    marker2.pose.position.x = 46.004704903664965
    marker2.pose.position.y = -187.35691283770004
    marker2.pose.position.z = 0.6621619635825589
    marker2.pose.orientation.x = -0.017726528491261564
    marker2.pose.orientation.y = -0.042067483603213954
    marker2.pose.orientation.z = 0.6787060524369635
    marker2.pose.orientation.w = 0.7329898985636677
    marker2.scale.x = 4.026174545288086
    marker2.scale.y = 2.1880083084106445
    marker2.scale.z = 1.666746973991394
    marker2.color.r = 1.0
    marker2.color.g = 0.0
    marker2.color.b = 0.0
    marker2.color.a = 0.5
    marker2.lifetime.secs = 0
    marker2.lifetime.nsecs = 0
    marker2.frame_locked = False
    marker2.points = []
    marker2.colors = []
    marker2.text = "1.70 m/s, <ID= 26062>, <ds= 0.992027>, <ss= 0.0,0.0,0.0>, <kds= 0.0>"
    marker2.mesh_resource = ''
    marker2.mesh_use_embedded_materials = False
    return marker1,marker2

def create_two_markers_debug3():

    marker1 = Marker()
    marker1.header.seq = 0
    marker1.header.stamp.secs = 1694565210
    marker1.header.stamp.nsecs = 416768
    marker1.header.frame_id = "map"
    # marker1.ns = "objects/os_sensor"
    marker1.ns = "os_sensor/car"
    marker1.id = 5
    marker1.type = Marker.CUBE
    marker1.action = Marker.ADD
    marker1.pose.position.x = 16.44800832463512
    marker1.pose.position.y = -171.91808021933693
    marker1.pose.position.z = -0.12789120752917515
    marker1.pose.orientation.x = -0.025270584296745172
    marker1.pose.orientation.y = 0.015855988789673293
    marker1.pose.orientation.z = 0.9992854410600301
    marker1.pose.orientation.w = -0.023207595184855448
    marker1.scale.x = 4.445168972015381
    marker1.scale.y = 1.9494993686676025
    marker1.scale.z = 1.497753620147705
    marker1.color.r = 1.0
    marker1.color.g = 0.0
    marker1.color.b = 0.0
    marker1.color.a = 0.5
    marker1.lifetime.secs = 0
    marker1.lifetime.nsecs = 0
    marker1.frame_locked = False
    marker1.points = []
    marker1.colors = []
    marker1.text = "0.00 m/s, <ID= 65021>, <ds= 0.907080>, <ss= 0.0,0.0,0.0>, <kds= 0.0>"
    marker1.mesh_resource = ''
    marker1.mesh_use_embedded_materials = False

    marker2 = Marker()
    marker2.header.seq = 0
    marker2.header.stamp.secs = 1694557981
    marker2.header.stamp.nsecs = 362512350
    marker2.header.frame_id = "map"
    # marker2.ns = "objects/veh2_rslidar"
    marker2.ns = "os_sensor/pedestrian"
    marker2.id = 3
    marker2.type = Marker.CUBE
    marker2.action = Marker.ADD
    marker2.pose.position.x = 16.86283843768669
    marker2.pose.position.y = -171.95656359703204
    marker2.pose.position.z = -0.9833144783012719
    marker2.pose.orientation.x = -0.04163015726441266
    marker2.pose.orientation.y = -0.0192269438076593
    marker2.pose.orientation.z = 0.9953500929248982
    marker2.pose.orientation.w = 0.08470801114625566
    marker2.scale.x = 4.542710304260254
    marker2.scale.y = 2.46319317817688
    marker2.scale.z = 1.5317782163619995
    marker2.color.r = 1.0
    marker2.color.g = 0.0
    marker2.color.b = 0.0
    marker2.color.a = 0.5
    marker2.lifetime.secs = 0
    marker2.lifetime.nsecs = 0
    marker2.frame_locked = False
    marker2.points = []
    marker2.colors = []
    marker2.text = "0.00 m/s, <ID= 25909>, <ds= 0.973335>, <ss= 0.0,0.0,0.0>, <kds= 0.0>"
    marker2.mesh_resource = ''
    marker2.mesh_use_embedded_materials = False
    return marker1,marker2

if __name__ == "__main__":
    test_create_a_testing_marker_with_feature()
    marker_array = create_a_markerarray_1()

    for marker in marker_array.markers:
        print(marker)