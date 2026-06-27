#! /usr/bin/env python
"""
A python script to convert transformation matrix to pitch, roll, yaw
"""
import rospy
import math
if __name__ == '__main__':
    rospy.init_node('tf2pry')
    rospy.loginfo('tf2pry node started')

    """ Transformation matrix

        |r11 r12 r13 |
        |r21 r22 r23 |
        |r31 r32 r33 |

        yaw: alpha=arctan(r21/r11)
        pitch: beta=arctan(-r31/sqrt( r32^2+r33^2 ) )
        roll: gamma=arctan(r32/r33)
    """
    r11 = rospy.get_param('~r11',default=0.0)
    r12 = rospy.get_param('~r12',default=0.0)
    r13 = rospy.get_param('~r13',default=0.0)
    r21 = rospy.get_param('~r21',default=0.0)
    r22 = rospy.get_param('~r22',default=0.0)
    r23 = rospy.get_param('~r23',default=0.0)
    r31 = rospy.get_param('~r31',default=0.0)
    r32 = rospy.get_param('~r32',default=0.0)
    r33 = rospy.get_param('~r33',default=0.0)
    """
    print out the transofrmation matrix in the following format
        |r11 r12 r13 |
        |r21 r22 r23 |
        |r31 r32 r33 |
    """
    rospy.loginfo('Transformation matrix R:')
    rospy.loginfo('|%.4f %.4f %.4f |',r11,r12,r13)
    rospy.loginfo('|%.4f %.4f %.4f |',r21,r22,r23)
    rospy.loginfo('|%.4f %.4f %.4f |',r31,r32,r33)
    
    yaw = math.atan2(r21,r11)
    pitch = math.atan2(-r31,math.sqrt(r32**2+r33**2))
    roll = math.atan2(r32,r33)

    # print yaw pitch roll to angle radian
    rospy.loginfo('yaw(rad): %f',yaw)
    rospy.loginfo('pitch(rad): %f',pitch)
    rospy.loginfo('roll(rad): %f',roll)
    
    # yaw pitch roll to angle degree
    yaw = yaw*180/math.pi
    pitch = pitch*180/math.pi
    roll = roll*180/math.pi

    rospy.loginfo('yaw(degeree): %f',yaw)
    rospy.loginfo('pitch(degree): %f',pitch)
    rospy.loginfo('roll(degree): %f',roll)


    rospy.spin()
