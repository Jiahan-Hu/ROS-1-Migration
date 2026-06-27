#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <gps_common/GPSFix.h>

ros::Publisher gps_pub;

void pointCloudCallback(const sensor_msgs::PointCloud2 msg){
  gps_common::GPSFix gps_fix_msg;
  // Extract the header from the PointCloud2 message
  gps_fix_msg.header = msg.header;
  //populate the gps_fix_msg message
  gps_fix_msg.latitude = 37.7858;
  gps_fix_msg.longitude = -122.399;
  gps_fix_msg.altitude = 15.0;
  gps_fix_msg.status.status = gps_common::GPSStatus::STATUS_FIX;
  
  // Publish the gps_fix_msg message
  gps_pub.publish(gps_fix_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pc_gps_publisher_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  //load parameters
  std::string point_cloud_topic;
  std::string pub_gps_topic;
  private_nh.param<std::string>("point_cloud_topic", point_cloud_topic, "/rslidar_points");
  private_nh.param<std::string>("pub_gps_topic", pub_gps_topic, "/gps/fix");

  ros::Subscriber sub = nh.subscribe(point_cloud_topic, 10, pointCloudCallback);

  gps_pub = nh.advertise<gps_common::GPSFix>(pub_gps_topic, 1);

  ros::spin();
  return 0;
}
