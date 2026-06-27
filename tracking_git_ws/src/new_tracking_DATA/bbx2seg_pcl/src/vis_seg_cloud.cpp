#include <ros/ros.h>
#include "common/msgs/autosense_msgs/PointCloud2Array.h"
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub;

void pointcloud2arrayCallback(const autosense_msgs::PointCloud2Array::ConstPtr& msg)
{
  // Iterate through each PointCloud2 in the array
  for (const sensor_msgs::PointCloud2& pc2 : msg->clouds)
  {
    // Publish each PointCloud2 separately
    pub.publish(pc2);
  }
  ROS_INFO("hello");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud2array_converter");

  ros::NodeHandle nh;

  // Retrieve the input and output topics from ROS parameters
  std::string input_topic, output_topic;
  nh.param<std::string>("input_topic", input_topic, "/input_topic");
  nh.param<std::string>("output_topic", output_topic, "/output_topic");

  // Create a subscriber to listen to the input PointCloud2Array messages
  ros::Subscriber sub = nh.subscribe(input_topic, 10, pointcloud2arrayCallback);

  // Create a publisher to publish the output PointCloud2 messages
  pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 10);

  // Spin the node to receive and process messages
  ros::spin();

  return 0;
}
