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

MarkerArrayHeader syn_bbx_msg1;
PointCloud2 syn_pc_msg2;

void callback(const MarkerArrayHeader::ConstPtr& bbx_msg, const PointCloud2::ConstPtr& pc_msg)
{

    syn_bbx_msg1 = *bbx_msg;
    syn_pc_msg2 = *pc_msg;

    visualization_msgs::MarkerArray empty_bbx;
    visualization_msgs::Marker clear_marker;
    clear_marker.header = syn_bbx_msg1.header;
    clear_marker.ns = "objects";
    clear_marker.id = 0;
    clear_marker.action = clear_marker.DELETEALL;
    clear_marker.lifetime = ros::Duration();
    empty_bbx.markers.push_back(clear_marker);
    
    std::cout << "\033[1;32m Syn! \033[0m" << std::endl;
    std::cout << "syn bbx_header msg's timestamp : " << syn_bbx_msg1.header.stamp << std::endl;
    std::cout << "syn pc msg's timestamp : " << syn_pc_msg2.header.stamp << std::endl;

    // republish marker from bbx_msg and pc_msg in two new new topics. 
    visualization_msgs::MarkerArray bbx;
    bbx.markers = bbx_msg->markers;

    
    if (syn_bbx_msg1.markers[0].action != 3){
        std::cout << "publish!"<< std::endl;
        pub_vis_bbx.publish(empty_bbx);
        
        pub_vis_bbx.publish(bbx);
        pub_vis_pc.publish(*pc_msg);
    }
        
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string input_topic_bbx, input_topic_pc, output_topic_vis_bbx, output_topic_vis_pc;
    pnh.param("input_topic_bbx", input_topic_bbx, std::string("/detected_bbx"));
    pnh.param("input_topic_pc", input_topic_pc, std::string("/pc_subset"));
    pnh.param("output_topic_vis_bbx", output_topic_vis_bbx, std::string("/vis/detected_bbx"));
    pnh.param("output_topic_vis_pc", output_topic_vis_pc, std::string("/vis/pc_subset"));

    // create subscriber
    message_filters::Subscriber<MarkerArrayHeader> bbx_sub(nh, input_topic_bbx, 10);
    message_filters::Subscriber<PointCloud2> pc_sub(nh, input_topic_pc, 10);

    // create publisher
    pub_vis_bbx = nh.advertise<visualization_msgs::MarkerArray>(output_topic_vis_bbx, 1);
    pub_vis_pc = nh.advertise<PointCloud2>(output_topic_vis_pc, 1);

    // use ApproximateTime to sync
    typedef sync_policies::ApproximateTime<MarkerArrayHeader, PointCloud2> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), bbx_sub, pc_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
    return 0;
}




