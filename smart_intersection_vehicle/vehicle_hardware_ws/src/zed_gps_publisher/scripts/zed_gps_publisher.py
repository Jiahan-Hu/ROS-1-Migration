#!/usr/bin/env python

import rospy
from sensor_msgs.msg import TimeReference, CompressedImage

class ImagePublisher:

    def __init__(self):
        self.nh = rospy.get_namespace()
        # gps_time_topic = rospy.get_param("~gps_time_topic", "/veh_2/gps_can_time_hz10")
        # image_topic = rospy.get_param("~image_topic", "/zed2i_v2_2/zed_node2/left/image_rect_color/compressed")
        # updated_image_topic = rospy.get_param("~updated_image_topic", "/zed2i_v2_2_update/zed_node2/left/image_rect_color/compressed")

        gps_time_topic = rospy.get_param("~gps_time_topic", "/gps_nw/gps_time_30hz")
        image_topic = rospy.get_param("~image_topic", "/axis214/image_raw/compressed")
        updated_image_topic = rospy.get_param("~updated_image_topic", "/axis214_update/image_raw/compressed")

        # Subscribe to the gps_can_time_hz10 topic
        self.gps_time_sub = rospy.Subscriber(gps_time_topic, TimeReference, self.time_callback)

        # Subscribe to the /zed2i_v2_2/zed_node2/left/image_rect_color/compressed topic
        self.image_sub = rospy.Subscriber(image_topic, CompressedImage, self.image_callback)

        # Publish the updated image on the "/updated_image" topic
        self.image_pub = rospy.Publisher(updated_image_topic, CompressedImage, queue_size=1)

        self.time_ref = rospy.Time()
        self.latest_time_ref_ = None  # Added member variable
        self.image_get = CompressedImage()

    def time_callback(self, time_ref_msg):
        time_ref_ = time_ref_msg.time_ref
        # self.latest_time_ref_ = self.time_ref
        rospy.loginfo("Time reference: %s", str(time_ref_))
        # Publish the updated message
        self.image_get.header.stamp = time_ref_
        self.image_pub.publish(self.image_get)
        rospy.loginfo("---------------------------------------------")
    
    def image_callback(self, image_msg):
        self.image_get = image_msg

    def image_callback0(self, image_msg):
        # Check if time_ref has been set
        if self.time_ref.is_zero():
            rospy.logwarn_once("Time reference not set yet!")
            
        # Create a new CompressedImage message
        updated_image_msg = CompressedImage()

        # Copy the original message
        updated_image_msg.header = image_msg.header
        updated_image_msg.format = image_msg.format
        updated_image_msg.data = image_msg.data

        # Update the header timestamp with the time_ref from the gps_can_time_hz10 topic
        updated_image_msg.header.stamp = self.time_ref
        # print("update_stamp:",updated_image_msg.header.stamp)
        rospy.loginfo("update_stamp  : %s", str(updated_image_msg.header.stamp))

        # Publish the updated message
        self.image_pub.publish(updated_image_msg)
        rospy.loginfo("---------------------------------------------")

    def time_callback1(self, time_ref_msg):
        self.time_ref = time_ref_msg.time_ref
        # self.latest_time_ref_ = self.time_ref
        rospy.loginfo("Time reference: %s", str(self.time_ref))
        # print("self.time_ref:",self.time_ref)

    def image_callback1(self, image_msg):
        # Check if time_ref has been set
        if self.time_ref.is_zero():
            rospy.logwarn_once("Time reference not set yet!")

        # Update the header timestamp with the time_ref from the gps_can_time_hz10 topic
        image_msg.header.stamp = self.time_ref
        # print("update_stamp:",updated_image_msg.header.stamp)
        rospy.loginfo("update_stamp  : %s", str(image_msg.header.stamp))

        # Publish the updated message
        self.image_pub.publish(image_msg)
        rospy.loginfo("---------------------------------------------")

if __name__ == '__main__':
    rospy.init_node("image_publisher")
    rospy.loginfo("This is image_publisher node")
    img_pub = ImagePublisher()
    rospy.spin()
