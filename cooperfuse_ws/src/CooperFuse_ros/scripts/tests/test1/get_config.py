#! /usr/bin/env python
# coding: utf-8

import rospy

def main():
    # Initialize the ROS node
    rospy.init_node('yaml_reader', anonymous=True)

    # Get the tracking parameter dictionary
    fusion_params = rospy.get_param('~v2xfusion')

    # Extract individual parameters
    sub_ego_bbx_topic = fusion_params['sub_ego_bbx']
    sub_agent1_bbx_topic = fusion_params['sub_agent1_bbx']
    pub_fusion_bbx_topic = fusion_params['pub_fusion_bbx']

    # Print the extracted parameters
    rospy.loginfo("config parameters are as follows:")
    rospy.loginfo(f"sub_infra1_bbx_topic: {sub_ego_bbx_topic}")
    rospy.loginfo(f"sub_veh1_bbx_topic: {sub_agent1_bbx_topic}")
    rospy.loginfo(f"pub_fusion_bbx_topic: {pub_fusion_bbx_topic}")
    

if __name__ == '__main__':
    main()

    