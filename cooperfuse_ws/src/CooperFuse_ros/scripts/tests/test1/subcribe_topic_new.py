#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# Global variables to store the latest MarkerArrays from the two topics
latest_marker_array1 = None
latest_marker_array2 = None

# Global variable to store the timestamp of the last published MarkerArray
last_published_timestamp = None

def callback1(data):
    global latest_marker_array1
    latest_marker_array1 = data
    fusion_pub()

def callback2(data):
    global latest_marker_array2
    latest_marker_array2 = data

def fusion_pub():
    global latest_marker_array1, latest_marker_array2, last_published_timestamp

    if latest_marker_array1 is None or latest_marker_array2 is None:
        rospy.loginfo("Waiting for data from both topics...")
        return

    current_timestamp = latest_marker_array1.header.stamp

    # Check if we have already published for this timestamp
    if last_published_timestamp == current_timestamp:
        return

    # Create a new MarkerArray for fusion
    fused_marker_array = MarkerArray()

    # Create an empty marker with action=3
    empty_marker = Marker()
    empty_marker.header = latest_marker_array1.header  # Use the timestamp from the first topic
    empty_marker.action = 3  # DELETEALL action
    fused_marker_array.markers.append(empty_marker)

    # Process and append markers from the first MarkerArray
    for marker in latest_marker_array1.markers[1:]:
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        fused_marker_array.markers.append(marker)

    # Process and append markers from the second MarkerArray
    for marker in latest_marker_array2.markers[1:]:
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        fused_marker_array.markers.append(marker)

    # Publish the fused MarkerArray
    pub = rospy.Publisher('/fusion/bbx', MarkerArray, queue_size=10)
    pub.publish(fused_marker_array)

    # Update the last published timestamp
    last_published_timestamp = current_timestamp

def main():
    global latest_marker_array1, latest_marker_array2

    rospy.init_node('marker_fusion_node', anonymous=True)
    rospy.Subscriber('/tracking/transformed_objects_box_score', MarkerArray, callback1)
    rospy.Subscriber('/tracking2/transformed_objects_box_score', MarkerArray, callback2)

    rospy.spin()

if __name__ == '__main__':
    main()
