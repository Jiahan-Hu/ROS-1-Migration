#include "ros/ros.h"    
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <ctime>
#include <mutex>
#include <atomic>
#include <memory>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <unordered_map>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_search.h>

#include <iostream>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
typedef pcl::PointXYZI PointT;

ros::Publisher syn_pointcloud_transformed_pub;

void Syncallback(const sensor_msgs::PointCloud2ConstPtr& ori_pointcloud_new, const sensor_msgs::PointCloud2ConstPtr& ori_pointcloud_rs)
{
	std::cout<<"ori_pointcloud_new->header.stamp.sec = "<<ori_pointcloud_new->header.stamp.sec<<std::endl;
	std::cout<<"ori_pointcloud_rs->header.stamp.sec = "<<ori_pointcloud_rs->header.stamp.sec<<std::endl;
	std::cout<<std::endl;
	std::cout<<"ori_pointcloud_new->header.stamp.nsec = "<<ori_pointcloud_new->header.stamp.nsec<<std::endl;
	std::cout<<"ori_pointcloud_rs->header.stamp.nsec = "<<ori_pointcloud_rs->header.stamp.nsec<<std::endl;
	std::cout<<std::endl;

    pcl::PointCloud<PointT>::Ptr cloud_new(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*ori_pointcloud_new, *cloud_new);

    for (int i = 0; i < cloud_new->size(); ++i)
    {
    	cloud_new->points[i].intensity=0;
    }

    pcl::PointCloud<PointT>::Ptr cloud_rs(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*ori_pointcloud_rs, *cloud_rs);

    for (int i = 0; i < cloud_rs->size(); ++i)
    {
    	cloud_rs->points[i].intensity=200;
    }

    pcl::PointCloud<PointT>::Ptr combined_cloud(new pcl::PointCloud<PointT>());

    *combined_cloud+=*cloud_new;
    *combined_cloud+=*cloud_rs;

	sensor_msgs::PointCloud2 lidar_twocars_msg;
    pcl::toROSMsg(*combined_cloud, lidar_twocars_msg);
    lidar_twocars_msg.header.stamp = ori_pointcloud_rs->header.stamp;
    lidar_twocars_msg.header.frame_id = "/map";
	syn_pointcloud_transformed_pub.publish(lidar_twocars_msg);

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "two_lidar_syn_comb");
    ros::NodeHandle node;
    ros::NodeHandle nh_local("~");

    cout << "----- two_lidar_syn is running! ----" << endl;

    // Read the parameter from the parameter server
    std::string input_topic1;
    std::string input_topic2;
    std::string comb_topic;

    nh_local.param<std::string>("input_topic1", input_topic1, "/transformed_new");
    nh_local.param<std::string>("input_topic2", input_topic2, "/transformed_rs");
    nh_local.param<std::string>("output_topic", comb_topic, "/pcd_transformed_synched");

    message_filters::Subscriber<sensor_msgs::PointCloud2> PointCloudInfo_new_sub(node, input_topic1, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> PointCloudInfo_rs_sub(node, input_topic2, 1);
    
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy; 
    
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(30), PointCloudInfo_new_sub, PointCloudInfo_rs_sub); //queue size=30
    sync.registerCallback(boost::bind(&Syncallback, _1, _2));

    syn_pointcloud_transformed_pub = node.advertise<sensor_msgs::PointCloud2>(comb_topic, 10,true);

    ros::spin();
    return 0;
}
