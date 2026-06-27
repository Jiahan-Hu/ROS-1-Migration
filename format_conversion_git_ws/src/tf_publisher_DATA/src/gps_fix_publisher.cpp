#include <ros/ros.h>
#include <gps_common/GPSFix.h>

void publishGPSFix(ros::Publisher &gps_pub) {
  gps_common::GPSFix gps_fix_msg;
  // populate the gps_fix_msg message
  gps_fix_msg.header.stamp = ros::Time::now();
  gps_fix_msg.header.frame_id = "gps";
  gps_fix_msg.latitude = 37.7858;
  gps_fix_msg.longitude = -122.399;
  gps_fix_msg.altitude = 15.0;
  gps_fix_msg.status.status = gps_common::GPSStatus::STATUS_FIX;

  gps_pub.publish(gps_fix_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "gps_publisher_node");
  ros::NodeHandle nh;
  ros::Publisher gps_pub = nh.advertise<gps_common::GPSFix>("gps/fix", 1);

  ros::Rate rate(1.0);
  while (nh.ok()) {
    publishGPSFix(gps_pub);
    rate.sleep();
  }
  return 0;
}