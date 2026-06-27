
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher header_pub;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  std_msgs::Header header = cloud_msg->header;
  // publish header information
  header_pub.publish(header);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "header_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string lidar_topic,header_topic;
    nh_private.param<std::string>("lidar_topic", lidar_topic, "/input_pointcloud");
    nh_private.param<std::string>("header_topic", header_topic, "/header_topic");
  
  // subscribe to the PointCloud2 topic
  ros::Subscriber cloud_sub = nh.subscribe(lidar_topic, 1, pointCloudCallback);
  header_pub = nh.advertise<std_msgs::Header>(header_topic, 1);

  ros::spin();

  return 0;
}
