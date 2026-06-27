#include "ros/ros.h"    
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <visualization_header_msgs/MarkerArrayHeader.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Declare the message filters and sync policy
typedef message_filters::sync_policies::ApproximateTime<visualization_msgs::MarkerArray, visualization_msgs::MarkerArray> SyncPolicy;
message_filters::Subscriber<visualization_msgs::MarkerArray> sub1;
message_filters::Subscriber<visualization_msgs::MarkerArray> sub2;
message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub1, sub2);

// ros::Publisher bbx1_pub;
// ros::Publisher bbx2_pub;

// Callback function to process the synchronized messages
void callback(const visualization_msgs::MarkerArray::ConstPtr& msg1, const visualization_msgs::MarkerArray::ConstPtr& msg2)
{
  // Do something with the synchronized messages here
  // bbx1_pub.publish(msg1);
  // bbx2_pub.publish(msg2);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "two_bbx_syn_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");

    std::cout << "----- two_bbx_syn is running! ----" << std::endl;
    // std::string input_topic1;
    // std::string input_topic2;
    // std::string output_topic1;
    // std::string output_topic2;
    // nh_local.param<std::string>("input_topic1", input_topic1, "/detected_bbx1");
    // nh_local.param<std::string>("input_topic2", input_topic2, "/detected_bbx2");
    // nh_local.param<std::string>("output_topic1", output_topic1, "/sync_bbx1");
    // nh_local.param<std::string>("output_topic2", output_topic2, "/sync_bbx2");

  // Set the topics for the message filters
  sub1.subscribe(nh, "topic1", 10);
  sub2.subscribe(nh, "topic2", 10);

  // Register the callback function
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}

