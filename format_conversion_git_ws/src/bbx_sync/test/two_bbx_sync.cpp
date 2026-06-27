#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <iostream>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher bbx1_pub;
ros::Publisher bbx2_pub;

void Syncallback(const visualization_msgs::MarkerArray::ConstPtr& ori_bbx_new, const visualization_msgs::MarkerArray::ConstPtr& ori_bbx_rs)
{
    cout << "\033[1;32m Syn! \033[0m" << endl;
    bbx1_pub.publish(ori_bbx_new);
    bbx2_pub.publish(ori_bbx_rs);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "two_bbx_syn_node");
    ros::NodeHandle node;
    ros::NodeHandle nh_local("~");

    cout << "----- two_bbx_syn is running! ----" << endl;

    // Read the parameter from the parameter server
    std::string input_topic1;
    std::string input_topic2;
    std::string output_topic1;
    std::string output_topic2;
    nh_local.param<std::string>("input_topic1", input_topic1, "/detected_bbx1");
    nh_local.param<std::string>("input_topic2", input_topic2, "/detected_bbx2");
    nh_local.param<std::string>("output_topic1", output_topic1, "/sync_bbx1");
    nh_local.param<std::string>("output_topic2", output_topic2, "/sync_bbx2");

    message_filters::Subscriber<visualization_msgs::MarkerArray> bbx1_sub(node, input_topic1, 1);
    message_filters::Subscriber<visualization_msgs::MarkerArray> bbx2_sub(node, input_topic2, 1);

    typedef sync_policies::ApproximateTime<visualization_msgs::MarkerArray, visualization_msgs::MarkerArray> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), bbx1_sub, bbx2_sub);
    
    bbx1_pub = node.advertise<visualization_msgs::MarkerArray>(output_topic1, 30);
    bbx2_pub = node.advertise<visualization_msgs::MarkerArray>(output_topic2, 30);

    sync.registerCallback(boost::bind(&Syncallback, _1, _2));

    ros::spin();
    return 0;
}
