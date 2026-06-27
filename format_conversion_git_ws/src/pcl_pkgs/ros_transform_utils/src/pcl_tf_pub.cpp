/*

Author: Zhaoliang
Date: 2020-03-02
Description: This code is two transfer two pointclouds from two different frames to the same frame "map".
*/

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

using std::atan2;
using std::cos;
using std::sin;

using std::cout;
using std::endl;
using std::vector;
using std::setprecision;

typedef pcl::PointXYZI PointT;

double veh_x,veh_y,veh_z,veh_heading,veh_pitch,veh_roll;
double fix_heading=0,fix_pitch=0,fix_roll=0,fix_x=0,fix_y=0,fix_z=0;

Eigen::Matrix4f T_VEH2M = Eigen::Matrix4f::Identity();//transform from veh to fix_lidar
Eigen::Matrix4f T_FIX2M = Eigen::Matrix4f::Identity(); //transfrom from fix to fix_lidar frame

// publish two transformed pointclouds
ros::Publisher transformed_fix_pub,transformed_veh_pub;

/*Rotation matrix generation function*/
Eigen::Matrix3f Creat_Rt(Eigen::Vector3f euler_angle) 
{
    //heading, pitch and roll
	Eigen::Matrix3f Rz= Eigen::Matrix3f::Identity();
	Eigen::Matrix3f Ry= Eigen::Matrix3f::Identity();
	Eigen::Matrix3f Rx= Eigen::Matrix3f::Identity();

	float yaw = euler_angle(0), pitch = euler_angle(1), roll = euler_angle(2);
	Rz(0,0)=cos(yaw);
	Rz(0,1)=-sin(yaw);
	Rz(1,0)=sin(yaw);
	Rz(1,1)=cos(yaw);

	Ry(0,0)=cos(pitch);
	Ry(0,2)=sin(pitch);
	Ry(2,0)=-sin(pitch);
	Ry(2,2)=cos(pitch);

	Rx(1,1)=cos(roll);
	Rx(1,2)=-sin(roll);
	Rx(2,1)=sin(roll);
	Rx(2,2)=cos(roll);

	Eigen::Matrix3f Rt=Rz*Ry*Rx;
	return Rt;
}

void lidar1_handler(sensor_msgs::PointCloud2 laserCloudMsg)
{
	// cout<<"fix pcd has been received! "<<endl;

	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
	pcl::fromROSMsg(laserCloudMsg, *cloud);

	// cout<<"fix lidar cloud size () =  "<<cloud->size()<<endl;


	/*generating the rotation matrix*/
	Eigen::Vector3f euler_angle_fix(fix_heading,fix_pitch,fix_roll);
	Eigen::Matrix3f Rt_fix=Creat_Rt(euler_angle_fix);

	/*generating the transformation matrix from rslidar to the fix_lidar frame*/
	T_FIX2M.block<3,3>(0,0)=Rt_fix;
	T_FIX2M (0,3) = fix_x;
	T_FIX2M (1,3) = fix_y;
	T_FIX2M (2,3) = fix_z;

	pcl::PointCloud<PointT>::Ptr transformed_cloud_fix(new pcl::PointCloud<PointT>());
	pcl::transformPointCloud (*cloud, *transformed_cloud_fix, T_FIX2M);
	
	// cout<<"fix lidar published pcd size () =  "<<transformed_cloud_fix->size()<<endl;
	/*transform the pointcloud from rslidar in rslidar frame to fix_lidar frame*/
	sensor_msgs::PointCloud2 pcd_inter;
    pcl::toROSMsg(*transformed_cloud_fix, pcd_inter);
    pcd_inter.header.stamp = laserCloudMsg.header.stamp;
    pcd_inter.header.frame_id = "map";
	transformed_fix_pub.publish(pcd_inter);

}

// write a lidar2_handler function to handle the second lidar pointcloud
void lidar2_handler(sensor_msgs::PointCloud2 laserCloudMsg)
{
    // cout<<"veh pcd has been received! "<<endl;

	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
	pcl::fromROSMsg(laserCloudMsg, *cloud);

    // cout<<"veh lidar cloud size () =  "<<cloud->size()<<endl;    
    /*generating the rotation matrix*/
    Eigen::Vector3f euler_angle_veh(veh_heading,veh_pitch,veh_roll);
    Eigen::Matrix3f Rt_veh=Creat_Rt(euler_angle_veh);

    /*generating the transformation matrix from rslidar to the fix_lidar frame*/
    T_VEH2M.block<3,3>(0,0)=Rt_veh;
    T_VEH2M (0,3) = veh_x;
    T_VEH2M (1,3) = veh_y;
    T_VEH2M (2,3) = veh_z;

    pcl::PointCloud<PointT>::Ptr transformed_cloud_veh(new pcl::PointCloud<PointT>());
    pcl::transformPointCloud (*cloud, *transformed_cloud_veh, T_VEH2M);

    // cout<<"veh lidar published pcd size () =  "<<transformed_cloud_veh->size()<<endl;
    /*transform the pointcloud from rslidar in rslidar frame to fix_lidar frame*/
    sensor_msgs::PointCloud2 pcd_inter;
    pcl::toROSMsg(*transformed_cloud_veh, pcd_inter);
    pcd_inter.header.stamp = laserCloudMsg.header.stamp;
    pcd_inter.header.frame_id = "map";
    transformed_veh_pub.publish(pcd_inter);

}

int main(int argc, char **argv){
    ros::init(argc, argv, "pcl_tf_pub");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    fix_x = nh.param<double>("fix_x", 0);
    fix_y = nh.param<double>("fix_y", 0);
    fix_z = nh.param<double>("fix_z", 0);
    fix_heading = nh.param<double>("fix_heading", 0);
    fix_pitch = nh.param<double>("fix_pitch", 0);
    fix_roll = nh.param<double>("fix_roll", 0);

	fix_heading=fix_heading*M_PI/180;
	fix_pitch=fix_pitch*M_PI/180;
	fix_roll=fix_roll*M_PI/180;

    veh_x = nh.param<double>("veh_x", 0);
    veh_y = nh.param<double>("veh_y", 0);
    veh_z = nh.param<double>("veh_z", 0);
    veh_heading = nh.param<double>("veh_heading", 0);
    veh_pitch = nh.param<double>("veh_pitch", 0);
    veh_roll = nh.param<double>("veh_roll", 0);

	veh_heading=veh_heading*M_PI/180;
	veh_pitch=veh_pitch*M_PI/180;
	veh_roll=veh_roll*M_PI/180;

    // read topic name from parameter server
    std::string lidar1_topic, lidar2_topic;
    
    nh.param<std::string>("lidar1_topic", lidar1_topic, "/lidar1");
    nh.param<std::string>("lidar2_topic", lidar2_topic, "/lidar2");

    // read publish topic name from parameter server
    std::string transformed_lidar1_topic, transformed_lidar2_topic;
    nh.param<std::string>("transformed_lidar1_topic", transformed_lidar1_topic, "/transformed_lidar1");
    nh.param<std::string>("transformed_lidar2_topic", transformed_lidar2_topic, "/transformed_lidar2");

    // subscribe to two pointclouds
    // message_filters::Subscriber<sensor_msgs::PointCloud2> lidar1_sub(nh, lidar1_topic, 1);
    // message_filters::Subscriber<sensor_msgs::PointCloud2> lidar2_sub(nh, lidar2_topic, 1);

	ros::Subscriber lidar1_sub = nh.subscribe<sensor_msgs::PointCloud2>(lidar1_topic, 10, lidar1_handler);
    ros::Subscriber lidar2_sub = nh.subscribe<sensor_msgs::PointCloud2>(lidar2_topic, 10, lidar2_handler);

    // publish two transformed pointclouds using transformed topic name
    transformed_fix_pub = nh.advertise<sensor_msgs::PointCloud2>(transformed_lidar1_topic, 10);
    transformed_veh_pub = nh.advertise<sensor_msgs::PointCloud2>(transformed_lidar2_topic, 10);

    ros::spin();
    return 0;
}
