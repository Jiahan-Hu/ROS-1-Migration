#include <ros/ros.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/CompressedImage.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher pub;

void SyncCallback(const TimeReferenceConstPtr& time_ref_msg, const CompressedImageConstPtr& image_msg)
{  
  std::cout << "\033[1;32m Syn! \033[0m" << std::endl;
  // Create a new image message with the updated header
  CompressedImage new_image_msg = *image_msg;
  std_msgs::Header header = new_image_msg.header;
  header.stamp = time_ref_msg->time_ref;
  new_image_msg.header = header;

  // Publish the new message
  pub.publish(new_image_msg);

  ROS_INFO("Received synchronized messages with timestamp %f", header.stamp.toSec());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cam_gps sync_node");
  ROS_INFO("Starting cam_gps sync_node");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  // Retrieve the topic names from ROS parameters
  std::string time_ref_topic, image_topic,new_image_topic;
  nh.param<std::string>("time_ref_topic", time_ref_topic, "/gps_can_time_hz10");
  nh.param<std::string>("image_topic", image_topic, "/zed2i_v2_0/zed_node0/left/image_rect_color/compressed");
  nh.param<std::string>("new_image_topic", new_image_topic, "/zed2i_v2_0_update/zed_node0/left/image_rect_color/compressed");

  // Create subscribers for the two topics
  message_filters::Subscriber<TimeReference> time_ref_sub(node, time_ref_topic, 1);
  message_filters::Subscriber<CompressedImage> image_sub(node, image_topic, 1);

  // Define the synchronization policy
  typedef sync_policies::ApproximateTime<TimeReference, CompressedImage> SyncPolicy;
  Synchronizer<SyncPolicy> sync(SyncPolicy(10), time_ref_sub, image_sub);

  // Register the callback function to be called when a synchronized message is received
  sync.registerCallback(boost::bind(&SyncCallback, _1, _2));

  // Create a publisher for the new image topic
  pub = node.advertise<CompressedImage>(new_image_topic, 100);

  ros::spin();

  return 0;
}
