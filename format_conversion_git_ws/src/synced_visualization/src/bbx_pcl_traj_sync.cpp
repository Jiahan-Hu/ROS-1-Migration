// #include "ros/ros.h"
// #include "message_filters/subscriber.h"
// #include "message_filters/synchronizer.h"
// #include "message_filters/sync_policies/approximate_time.h"

// #include "visualization_header_msgs/MarkerArrayHeader.h"
// #include "visualization_msgs/MarkerArray.h"
// #include "visualization_msgs/Marker.h"
// #include <string>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

// customize msgs
#include <visualization_header_msgs/MarkerArrayHeader.h> 

using namespace message_filters;
using namespace sensor_msgs;
using namespace visualization_header_msgs;

ros::Publisher pub_vis_bbx;
ros::Publisher pub_vis_pc;
ros::Publisher pub_vis_trajectory;

MarkerArrayHeader syn_bbx_msg1;
PointCloud2 syn_pc_msg2;
MarkerArrayHeader syn_traj_msg3;

void callback(const MarkerArrayHeader::ConstPtr& bbx_msg, const PointCloud2::ConstPtr& pc_msg, const MarkerArrayHeader::ConstPtr& trajectory_msg)
{
    std::cout << "\033[1;32m Syn! \033[0m" << std::endl;
    syn_bbx_msg1 = *bbx_msg;
    syn_pc_msg2 = *pc_msg;
    syn_traj_msg3 = *trajectory_msg;
    std::cout << "syn bbx_header msg's timestamp : " << syn_bbx_msg1.header.stamp << std::endl;
    std::cout << "syn pc msg's timestamp : " << syn_pc_msg2.header.stamp << std::endl;
    std::cout << "syn traj msg's timestamp :" << syn_traj_msg3.header.stamp << std::endl;

    // republish marker from bbx_msg and pc_msg in two new new topics. 
    visualization_msgs::MarkerArray bbx;
    bbx.markers = bbx_msg->markers;

    // Handle the new trajectory_msg and republish it
    visualization_msgs::MarkerArray trajectory;
    trajectory.markers = trajectory_msg->markers;

    pub_vis_bbx.publish(bbx);
    pub_vis_pc.publish(*pc_msg);
    pub_vis_trajectory.publish(trajectory);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string input_topic_bbx, input_topic_pc, output_topic_vis_bbx, output_topic_vis_pc;
    std::string input_topic_trajectory, output_topic_vis_trajectory;

    pnh.param("input_topic_bbx", input_topic_bbx, std::string("/detected_bbx"));
    pnh.param("input_topic_pc", input_topic_pc, std::string("/pc_subset"));
    pnh.param("output_topic_vis_bbx", output_topic_vis_bbx, std::string("/vis/detected_bbx"));
    pnh.param("output_topic_vis_pc", output_topic_vis_pc, std::string("/vis/pc_subset"));

    pnh.param("input_topic_trajectory", input_topic_trajectory, std::string("/tracking/trajectory"));
    pnh.param("output_topic_vis_trajectory", output_topic_vis_trajectory, std::string("/vis/tracjectory"));

    // create subscriber
    message_filters::Subscriber<MarkerArrayHeader> bbx_sub(nh, input_topic_bbx, 1);
    message_filters::Subscriber<PointCloud2> pc_sub(nh, input_topic_pc, 1);
    message_filters::Subscriber<MarkerArrayHeader> trajectory_sub(nh, input_topic_trajectory, 1);

    // create publisher
    pub_vis_bbx = nh.advertise<visualization_msgs::MarkerArray>(output_topic_vis_bbx, 1);
    pub_vis_pc = nh.advertise<PointCloud2>(output_topic_vis_pc, 1);
    pub_vis_trajectory = nh.advertise<visualization_msgs::MarkerArray>(output_topic_vis_trajectory, 1);

    // use ApproximateTime to sync
    typedef sync_policies::ApproximateTime<MarkerArrayHeader, PointCloud2, MarkerArrayHeader> MySyncPolicy;
    // MySyncPolicy(10) means to set a queue of size 10 for each subscribed message type.
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), bbx_sub, pc_sub, trajectory_sub); 
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));

    ros::spin();
    return 0;
}




