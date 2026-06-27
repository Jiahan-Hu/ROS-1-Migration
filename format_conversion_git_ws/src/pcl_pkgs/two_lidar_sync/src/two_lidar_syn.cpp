#include "ros/ros.h"    
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>


#include <iostream>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher PointCloudInfo_new_pub;
ros::Publisher PointCloudInfo_rs_pub;

PointCloud2 syn_pointcloud_new;
PointCloud2 syn_pointcloud_rs;


void Syncallback(const PointCloud2ConstPtr& ori_pointcloud_new,const PointCloud2ConstPtr& ori_pointcloud_rs)
{
    cout << "\033[1;32m Syn! \033[0m" << endl;
    syn_pointcloud_new = *ori_pointcloud_new;
    syn_pointcloud_rs = *ori_pointcloud_rs;
    cout << "syn pointcloud_new' timestamp : " << syn_pointcloud_new.header.stamp << endl;
    cout << "syn pointcloud_rs's timestamp : " << syn_pointcloud_rs.header.stamp << endl;
    PointCloudInfo_new_pub.publish(syn_pointcloud_new);
    PointCloudInfo_rs_pub.publish(syn_pointcloud_rs);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "two_lidar_syn");
    ros::NodeHandle node;
    ros::NodeHandle nh_local("~");

    cout << "----- two_lidar_syn is running! ----" << endl;

    // Read the parameter from the parameter server
    std::string input_topic1;
    std::string input_topic2;
    std::string output_topic1;
    std::string output_topic2;
    nh_local.param<std::string>("input_topic1", input_topic1, "/new/rslidar_points");
    nh_local.param<std::string>("input_topic2", input_topic2, "/rslidar_points");
    nh_local.param<std::string>("output_topic1", output_topic1, "/syn_pc_new");
    nh_local.param<std::string>("output_topic2", output_topic2, "/syn_pc_rs");


    message_filters::Subscriber<PointCloud2> PointCloudInfo_new_sub(node, input_topic1, 1);
    message_filters::Subscriber<PointCloud2> PointCloudInfo_rs_sub(node, input_topic2, 1);
    
    typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2> MySyncPolicy; 
    
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), PointCloudInfo_new_sub, PointCloudInfo_rs_sub); //queue size=30
    sync.registerCallback(boost::bind(&Syncallback, _1, _2));

    PointCloudInfo_new_pub = node.advertise<PointCloud2>(output_topic1, 100);
    PointCloudInfo_rs_pub = node.advertise<PointCloud2>(output_topic2, 100);

    ros::spin();
    return 0;
}
