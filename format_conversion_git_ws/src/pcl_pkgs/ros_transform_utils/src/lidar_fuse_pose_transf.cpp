#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <string>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Geometry>

typedef pcl::PointXYZI PointT;

ros::Publisher transformed_lidar_pub;

void callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg,
              const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
{
    // TODO: Implement your transformation logic here
    // sensor_msgs::PointCloud2 transformed_pointcloud = *pointcloud_msg;

    ROS_INFO_STREAM("\033[32mSync starts\033[0m");
    // Print pose_msg header stamp and pointcloud_msg header stamp
    ROS_INFO("pose_msg header.stamp: %f", pose_msg->header.stamp.toSec());
    ROS_INFO("pointcloud_msg header.stamp: %f", pointcloud_msg->header.stamp.toSec());

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
	pcl::fromROSMsg(*pointcloud_msg, *cloud);

    Eigen::Affine3d T_VEH2M = Eigen::Affine3d::Identity();

    // Extract translation
    Eigen::Translation3d translation(pose_msg->pose.position.x, 
                                    pose_msg->pose.position.y, 
                                    pose_msg->pose.position.z);

    // Extract rotation quaternion
    Eigen::Quaterniond quat(pose_msg->pose.orientation.w, 
                            pose_msg->pose.orientation.x, 
                            pose_msg->pose.orientation.y, 
                            pose_msg->pose.orientation.z);

    // Construct transformation matrix
    T_VEH2M = translation * quat;

    pcl::PointCloud<PointT>::Ptr transformed_pointcloud(new pcl::PointCloud<PointT>());
    // Transform point cloud
    pcl::transformPointCloud(*cloud, *transformed_pointcloud, T_VEH2M.matrix());

    // Convert back to sensor_msgs/PointCloud2 message if needed
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*transformed_pointcloud, output_msg);

    // output_msg.header.stamp = pointcloud_msg.header.stamp; 
    output_msg.header.stamp = pointcloud_msg->header.stamp;
    output_msg.header.frame_id = "map";
    
    // E.g., transformPointCloud(pose_msg, pointcloud_msg, transformed_pointcloud);
    transformed_lidar_pub.publish(output_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_lidar_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string pose_topic, source_lidar_topic, transformed_lidar_topic;

    // Getting topic names from rosparam
    private_nh.param<std::string>("fusion_pose_gps_topic", pose_topic, "/fusion_pose_gps");
    private_nh.param<std::string>("rslidar_points_topic", source_lidar_topic, "/veh2/rslidar_points");
    private_nh.param<std::string>("transformed_rslidar_points_topic", transformed_lidar_topic, "/veh2/transformed_rslidar_points");

    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh, pose_topic, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, source_lidar_topic, 1);

    typedef message_filters::sync_policies::ApproximateTime<
        geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pose_sub, lidar_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    transformed_lidar_pub = nh.advertise<sensor_msgs::PointCloud2>(transformed_lidar_topic, 1);

    ros::spin();

    return 0;
}
