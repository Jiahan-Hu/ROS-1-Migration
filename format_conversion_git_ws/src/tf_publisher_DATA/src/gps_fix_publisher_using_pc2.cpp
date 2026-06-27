#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <gps_common/GPSFix.h>

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg, ros::Publisher &gps_pub){
  gps_common::GPSFix gps_fix_msg;
  // Extract the header from the PointCloud2 message
  gps_fix_msg.header = msg->header;
  //populate the gps_fix_msg message
  gps_fix_msg.latitude = 37.7858;
  gps_fix_msg.longitude = -122.399;
  gps_fix_msg.altitude = 15.0;
  gps_fix_msg.status.status = gps_common::GPSStatus::STATUS_FIX;
  
  // Publish the gps_fix_msg message
  gps_pub.publish(gps_fix_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "gps_publisher_node");
  ros::NodeHandle nh;

  ros::Publisher gps_pub = nh.advertise<gps_common::GPSFix>("gps/fix", 1);
  ros::Subscriber sub = nh.subscribe("point_cloud", 1, pointCloudCallback,gps_pub);

  ros::spin();
  return 0;
}
