#include "ros/ros.h"    
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <sensor_msgs/MarkerArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher bbx1_pub;
ros::Publisher bbx2_pub;

void convertMarkerArray(const visualization_msgs::MarkerArray& vis_markers, sensor_msgs::MarkerArray& sensor_markers)
{
  sensor_markers.markers.resize(vis_markers.markers.size());
  for (size_t i = 0; i < vis_markers.markers.size(); ++i)
  {
    sensor_msgs::Marker& sensor_marker = sensor_markers.markers[i];
    const visualization_msgs::Marker& vis_marker = vis_markers.markers[i];
    sensor_marker.header = vis_marker.header;
    sensor_marker.ns = vis_marker.ns;
    sensor_marker.id = vis_marker.id;
    sensor_marker.type = vis_marker.type;
    sensor_marker.action = vis_marker.action;
    sensor_marker.pose = vis_marker.pose;
    sensor_marker.scale = vis_marker.scale;
    sensor_marker.color = vis_marker.color;
    sensor_marker.lifetime = vis_marker.lifetime;
    sensor_marker.frame_locked = vis_marker.frame_locked;
    sensor_marker.points = vis_marker.points;
    sensor_marker.colors = vis_marker.colors;
    sensor_marker.text = vis_marker.text;
    sensor_marker.mesh_resource = vis_marker.mesh_resource;
    sensor_marker.mesh_use_embedded_materials = vis_marker.mesh_use_embedded_materials;
  }
}


// void Syncallback(const visualization_msgs::MarkerArray& marker_array1, const visualization_msgs::MarkerArray& marker_array2)
// {
//   // process synchronized messages here
//   bbx1_pub.publish(marker_array1);
//   bbx2_pub.publish(marker_array2);
// }

// void Syncallback(const sensor_msgs::MarkerArray& marker_array1, const sensor_msgs::MarkerArray& marker_array2)
// {
//   // process synchronized messages here
//   bbx1_pub.publish(marker_array1);
//   bbx2_pub.publish(marker_array2);
// }


void Syncallback(const sensor_msgs::PointCloud2ConstPtr& marker_array1, const sensor_msgs::PointCloud2ConstPtr& marker_array2)
{
  // process synchronized messages here
  bbx1_pub.publish(marker_array1);
  bbx2_pub.publish(marker_array2);
}

// main function
int main(int argc, char **argv){
    ros::init(argc, argv, "two_bbx_syn_node");
    ros::NodeHandle node;
    ros::NodeHandle nh_local("~");

    std::cout << "----- two_bbx_syn is running! ----" << std::endl;
    std::string input_topic1;
    std::string input_topic2;
    std::string output_topic1;
    std::string output_topic2;
    nh_local.param<std::string>("input_topic1", input_topic1, "/detected_bbx1");
    nh_local.param<std::string>("input_topic2", input_topic2, "/detected_bbx2");
    nh_local.param<std::string>("output_topic1", output_topic1, "/sync_bbx1");
    nh_local.param<std::string>("output_topic2", output_topic2, "/sync_bbx2");

    message_filters::Subscriber<visualization_msgs::MarkerArray> marker_array_sub1(node, input_topic1, 1);
    message_filters::Subscriber<visualization_msgs::MarkerArray> marker_array_sub2(node, input_topic2, 1);
    // create a new sensor_msgs::MarkerArray variable
    sensor_msgs::MarkerArray sensor_marker_array1;
    sensor_msgs::MarkerArray sensor_marker_array2;
    // convert visualization_msgs::MarkerArray to sensor_msgs::MarkerArray
    convertMarkerArray(marker_array_sub1,sensor_marker_array1);
    convertMarkerArray(marker_array_sub2,sensor_marker_array2);

    // typedef sync_policies::ApproximateTime<sensor_msgs::MarkerArray, sensor_msgs::MarkerArray> MySyncPolicy;
    // typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    // // typedef sync_policies::ApproximateTime<visualization_msgs::MarkerArray, visualization_msgs::MarkerArray> MySyncPolicy; 
    // Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), marker_array_sub1, marker_array_sub2); //queue size=100
    // sync.registerCallback(boost::bind(&Syncallback, _1, _2));   

    // ros::Publisher bbx1_pub = node.advertise<visualization_msgs::MarkerArray>(output_topic1, 30);
    // ros::Publisher bbx2_pub = node.advertise<visualization_msgs::MarkerArray>(output_topic2, 30);

    ros::Publisher bbx1_pub = node.advertise<sensor_msgs::MarkerArray>(output_topic1, 30);
    ros::Publisher bbx2_pub = node.advertise<sensor_msgs::MarkerArray>(output_topic2, 30);

    ros::spin();    
    return 0;
}