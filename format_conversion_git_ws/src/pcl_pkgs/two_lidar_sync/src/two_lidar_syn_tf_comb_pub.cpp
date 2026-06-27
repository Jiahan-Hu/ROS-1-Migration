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

ros::Publisher PointCloudInfo_new_pub;
ros::Publisher PointCloudInfo_rs_pub;

PointCloud2 syn_pointcloud_new;
PointCloud2 syn_pointcloud_rs;

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
ros::Publisher syn_pointcloud_transformed_pub;

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

void Syncallback(const PointCloud2ConstPtr& ori_pointcloud_new,const PointCloud2ConstPtr& ori_pointcloud_rs)
{
    cout << "\033[1;32m Syn! \033[0m" << endl;
    cout << "ori_pointcloud_new->header.stamp = " << ori_pointcloud_new->header.stamp << endl;
    cout << "ori_pointcloud_rs->header.stamp = " << ori_pointcloud_rs->header.stamp << endl;

    pcl::PointCloud<PointT>::Ptr cloud_new(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*ori_pointcloud_new, *cloud_new);

	/*generating the rotation matrix*/
	Eigen::Vector3f euler_angle_fix(fix_heading,fix_pitch,fix_roll);
	Eigen::Matrix3f Rt_fix=Creat_Rt(euler_angle_fix);

	/*generating the transformation matrix from rslidar to the fix_lidar frame*/
	T_FIX2M.block<3,3>(0,0)=Rt_fix;
	T_FIX2M (0,3) = fix_x;
	T_FIX2M (1,3) = fix_y;
	T_FIX2M (2,3) = fix_z;

    ros::Time begin = ros::Time::now();
    cout << "start:" << begin << endl;

	pcl::PointCloud<PointT>::Ptr transformed_cloud_fix(new pcl::PointCloud<PointT>());
	pcl::transformPointCloud (*cloud_new, *transformed_cloud_fix, T_FIX2M);

    // ros::Time end_time = ros::Time::now();
    // cout << "end:" << end_time << endl;
    // cout << "time elipse:" << end_time - begin << endl;

	// cout<<"fix lidar published pcd size () =  "<<transformed_cloud_fix->size()<<endl;
	/*transform the pointcloud from rslidar in rslidar frame to fix_lidar frame*/
	sensor_msgs::PointCloud2 pcd_inter1;
    pcl::toROSMsg(*transformed_cloud_fix, pcd_inter1);
    syn_pointcloud_new = *ori_pointcloud_new;
    pcd_inter1.header.stamp = syn_pointcloud_new.header.stamp;
    pcd_inter1.header.frame_id = "map";
	transformed_fix_pub.publish(pcd_inter1);   

    for (int i = 0; i < transformed_cloud_fix->size(); ++i)
    {
    	transformed_cloud_fix->points[i].intensity=0;
    }

    // -------------------------------------------------------------------------------------------- 
	
    pcl::PointCloud<PointT>::Ptr cloud_rs(new pcl::PointCloud<PointT>());
	pcl::fromROSMsg(*ori_pointcloud_rs, *cloud_rs);

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
    pcl::transformPointCloud (*cloud_rs, *transformed_cloud_veh, T_VEH2M);

    // cout<<"veh lidar published pcd size () =  "<<transformed_cloud_veh->size()<<endl;
    /*transform the pointcloud from rslidar in rslidar frame to fix_lidar frame*/
    sensor_msgs::PointCloud2 pcd_inter2;
    pcl::toROSMsg(*transformed_cloud_veh, pcd_inter2);
    syn_pointcloud_rs = *ori_pointcloud_rs;
    pcd_inter2.header.stamp = syn_pointcloud_rs.header.stamp;
    pcd_inter2.header.frame_id = "map"; 
    transformed_veh_pub.publish(pcd_inter2);

    for (int i = 0; i < transformed_cloud_veh->size(); ++i)
    {
    	transformed_cloud_veh->points[i].intensity=200;
    }

    pcl::PointCloud<PointT>::Ptr combined_cloud(new pcl::PointCloud<PointT>());
    *combined_cloud+=*transformed_cloud_fix;
    *combined_cloud+=*transformed_cloud_veh;

	sensor_msgs::PointCloud2 lidar_twocars_msg;
    pcl::toROSMsg(*combined_cloud, lidar_twocars_msg);
    lidar_twocars_msg.header.stamp = ori_pointcloud_rs->header.stamp;
    lidar_twocars_msg.header.frame_id = "/map";

    if (abs(double(pcd_inter1.header.stamp.nsec) - double(pcd_inter2.header.stamp.nsec)) < 100000000){
        syn_pointcloud_transformed_pub.publish(lidar_twocars_msg);
    }else{
        cout << "sync problematic!" << endl;
    }
	
    
            
    // -------------------------------------------------------------------------------------------- 
    // syn_pointcloud_new = *pcd_inter1;
    // syn_pointcloud_rs = *pcd_inter2;
    // cout << "syn pointcloud_new' timestamp : " << syn_pointcloud_new.header.stamp << endl;
    // cout << "syn pointcloud_rs's timestamp : " << syn_pointcloud_rs.header.stamp << endl;
    // PointCloudInfo_new_pub.publish(syn_pointcloud_new);
    // PointCloudInfo_rs_pub.publish(syn_pointcloud_rs);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "two_lidar_syn");
    ros::NodeHandle node;
    ros::NodeHandle nh_local("~");

    cout << "----- two_lidar_syn is running! ----" << endl;

    ros::Rate loop_rate(10);

    fix_x = node.param<double>("fix_x", 0);
    fix_y = node.param<double>("fix_y", 0);
    fix_z = node.param<double>("fix_z", 0);
    fix_heading = node.param<double>("fix_heading", 0);
    fix_pitch = node.param<double>("fix_pitch", 0);
    fix_roll = node.param<double>("fix_roll", 0);

	fix_heading=fix_heading*M_PI/180;
	fix_pitch=fix_pitch*M_PI/180;
	fix_roll=fix_roll*M_PI/180;

    veh_x = node.param<double>("veh_x", 0);
    veh_y = node.param<double>("veh_y", 0);
    veh_z = node.param<double>("veh_z", 0);
    veh_heading = node.param<double>("veh_heading", 0);
    veh_pitch = node.param<double>("veh_pitch", 0);
    veh_roll = node.param<double>("veh_roll", 0);

	veh_heading=veh_heading*M_PI/180;
	veh_pitch=veh_pitch*M_PI/180;
	veh_roll=veh_roll*M_PI/180;

    // Read the parameter from the parameter server
    std::string input_topic1;
    std::string input_topic2;
    std::string output_topic1;
    std::string output_topic2;
    std::string comb_topic;

    nh_local.param<std::string>("input_topic1", input_topic1, "/new/rslidar_points");
    nh_local.param<std::string>("input_topic2", input_topic2, "/rslidar_points");

    // read publish topic name from parameter server
    std::string transformed_lidar1_topic, transformed_lidar2_topic;
    nh_local.param<std::string>("transformed_lidar1_topic", transformed_lidar1_topic, "/transformed_lidar1");
    nh_local.param<std::string>("transformed_lidar2_topic", transformed_lidar2_topic, "/transformed_lidar2");

    nh_local.param<std::string>("output_topic1", output_topic1, "/syn_pc_new");
    nh_local.param<std::string>("output_topic2", output_topic2, "/syn_pc_rs");
    nh_local.param<std::string>("comb_topic", comb_topic, "/pcd_transformed_synched");

    // ros::Subscriber lidar1_sub = nh.subscribe<sensor_msgs::PointCloud2>(input_topic1, 100, lidar1_handler);
    // transformed_lidar1_pub=nh.advertise<sensor_msgs::PointCloud2>(transformed_output_topic1, 100);

    message_filters::Subscriber<PointCloud2> PointCloudInfo_new_sub(node, input_topic1, 100);
    message_filters::Subscriber<PointCloud2> PointCloudInfo_rs_sub(node, input_topic2, 100);
    
    typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2> MySyncPolicy; 
    
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), PointCloudInfo_new_sub, PointCloudInfo_rs_sub); //queue size=30
    sync.registerCallback(boost::bind(&Syncallback, _1, _2));

    // PointCloudInfo_new_pub = node.advertise<PointCloud2>(output_topic1, 30);
    // PointCloudInfo_rs_pub = node.advertise<PointCloud2>(output_topic2, 30);

    transformed_fix_pub = node.advertise<sensor_msgs::PointCloud2>(transformed_lidar1_topic, 100);
    transformed_veh_pub = node.advertise<sensor_msgs::PointCloud2>(transformed_lidar2_topic, 100);
    syn_pointcloud_transformed_pub = node.advertise<sensor_msgs::PointCloud2>(comb_topic, 10,true);
    ros::spin();
    return 0;
    
}
