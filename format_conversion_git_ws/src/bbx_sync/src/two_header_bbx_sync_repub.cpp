#include "ros/ros.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "visualization_header_msgs/MarkerArrayHeader.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"

ros::Publisher bbx1_pub;
ros::Publisher bbx2_pub;

using namespace std;
using namespace message_filters;

visualization_header_msgs::MarkerArrayHeader syn_msg1;
visualization_header_msgs::MarkerArrayHeader syn_msg2;
void callback(const visualization_header_msgs::MarkerArrayHeader::ConstPtr& msg1,
              const visualization_header_msgs::MarkerArrayHeader::ConstPtr& msg2)
{
  cout << "\033[1;32m Syn! \033[0m" << endl;
  syn_msg1 = *msg1;
  syn_msg2 = *msg2;
  cout << "syn msg1' timestamp : " << syn_msg1.header.stamp << endl;
  cout << "syn msg2's timestamp : " << syn_msg2.header.stamp << endl;

  // publish the synchronized messages in visualization_msgs::MarkerArray format
  visualization_msgs::MarkerArray bbx1;
  visualization_msgs::MarkerArray bbx2;
  bbx1.markers = msg1->markers;
  bbx2.markers = msg2->markers;

  visualization_msgs::MarkerArray empty_markers1;
  visualization_msgs::Marker clear_marker1;

  clear_marker1.header = bbx1.markers[0].header;
  clear_marker1.header.frame_id = "map";
  clear_marker1.ns = "/detection/clear_marker";
  clear_marker1.id = 0;
  clear_marker1.action = clear_marker1.DELETEALL;
  clear_marker1.lifetime = ros::Duration();
  empty_markers1.markers.push_back(clear_marker1);
  bbx1_pub.publish(empty_markers1);

  visualization_msgs::MarkerArray empty_markers2;
  visualization_msgs::Marker clear_marker2;

  clear_marker2.header = bbx2.markers[0].header;
  clear_marker2.header.frame_id = "map";
  clear_marker2.ns = "/detection/clear_marker";
  clear_marker2.id = 0;
  clear_marker2.action = clear_marker2.DELETEALL;
  clear_marker2.lifetime = ros::Duration();
  empty_markers2.markers.push_back(clear_marker1);
  bbx2_pub.publish(empty_markers2);


  // publish the bbx1 and bbx2
  bbx1_pub.publish(bbx1);
  bbx2_pub.publish(bbx2);
  
  // bbx1.markers.clear();
  // bbx2.markers.clear();

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "synchronize_marker_array_headers");
  ros::NodeHandle nh;

  ros::NodeHandle nh_local("~");

  std::cout << "----- two_bbx_header_syn is running! ----" << std::endl;

  std::string input_topic1;
  std::string input_topic2;
  std::string output_topic1;
  std::string output_topic2;
  nh_local.param<std::string>("input_topic1", input_topic1, "/detected_bbx1");
  nh_local.param<std::string>("input_topic2", input_topic2, "/detected_bbx2");
  nh_local.param<std::string>("output_topic1", output_topic1, "/sync_bbx1");
  nh_local.param<std::string>("output_topic2", output_topic2, "/sync_bbx2");

  // Create the message filter and register the callback function
  message_filters::Subscriber<visualization_header_msgs::MarkerArrayHeader> sub1(nh, input_topic1, 1);
  message_filters::Subscriber<visualization_header_msgs::MarkerArrayHeader> sub2(nh, input_topic2, 1);
  typedef sync_policies::ApproximateTime<visualization_header_msgs::MarkerArrayHeader, visualization_header_msgs::MarkerArrayHeader> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(200), sub1, sub2);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  bbx1_pub = nh.advertise<visualization_msgs::MarkerArray>(output_topic1, 10);
  bbx2_pub = nh.advertise<visualization_msgs::MarkerArray>(output_topic2, 10);

  // Spin the ROS loop to start receiving and synchronizing the messages
  ros::spin();

  return 0;
}
