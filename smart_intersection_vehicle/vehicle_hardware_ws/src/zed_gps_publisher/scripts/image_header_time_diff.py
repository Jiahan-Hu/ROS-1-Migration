#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage

class ImageSubscriber:

    def __init__(self):
        self.prev_time = None
        self.time_diffs = []  # Initialize an empty list to store time differences
        self.sub = rospy.Subscriber('/zed2i_v2_1_update/zed_node1/left/image_rect_color/compressed', CompressedImage, self.callback)

    def callback(self, msg):
        current_time = msg.header.stamp
        if self.prev_time is not None:
            time_diff = current_time - self.prev_time
            rospy.loginfo("Time difference between frames: %f", time_diff.to_sec())
            self.time_diffs.append(time_diff.to_sec())  # Add the time difference to the list
            if time_diff.to_sec() > 0.11 or time_diff.to_sec() < 0.09:
                rospy.logwarn("Warning: Time difference outside the expected range!")
        self.prev_time = current_time

    def save_time_diffs(self, filename):
        with open(filename, 'w') as f:
            for time_diff in self.time_diffs:
                f.write(str(time_diff) + '\n')

if __name__ == '__main__':
    rospy.init_node('image_subscriber')
    image_subscriber = ImageSubscriber()
    rospy.on_shutdown(lambda: image_subscriber.save_time_diffs('time_diffs.txt'))
    rospy.spin()

