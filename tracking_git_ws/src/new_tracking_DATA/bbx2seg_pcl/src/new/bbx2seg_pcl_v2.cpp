#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <random>

#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "common/msgs/autosense_msgs/PointCloud2Array.h"
#include "common/msgs/autosense_msgs/TrackingFixedTrajectoryArray.h"
#include "common/msgs/autosense_msgs/TrackingObjectArray.h"
#include "common/msgs/autosense_msgs/TrackingObjectArray_local_frame.h"

#include "common/common.hpp"
#include "common/geometry.hpp"      // common::geometry::calcYaw4DirectionVector
#include "common/transform.hpp"     // common::transform::transformPointCloud
#include "common/types/object.hpp"  // ObjectPtr
#include "common/types/type.h"

ros::Publisher segpclPub;
ros::Publisher pclPub;

// Define the create_3dpoints function
std::vector<Eigen::Vector3d> create_3dpoints(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2, double distance_duration = 0.2)
{
    std::vector<Eigen::Vector3d> points;
    
    // Calculate the distance between point1 and point2
    double dist = (point2 - point1).norm();
    
    // Calculate the number of points needed
    int num_points = static_cast<int>(dist / distance_duration);
    
    // Generate points in between
    for (int i = 0; i < num_points; ++i)
    {
        double t = static_cast<double>(i) / (num_points - 1);
        Eigen::Vector3d interpolated_point = (1 - t) * point1 + t * point2;
        points.push_back(interpolated_point);
    }
    
    return points;
}

void convert_marker2seg_pcl(const visualization_msgs::Marker& marker, sensor_msgs::PointCloud2& cloud)
{

    // Extract marker properties
    std_msgs::Header header = marker.header;
    double length = marker.scale.x;
    double width = marker.scale.y;
    double height = marker.scale.z;

    Eigen::Vector3d center;
    center[0] = marker.pose.position.x;
    center[1] = marker.pose.position.y;
    center[2] = marker.pose.position.z - height / 2;

    geometry_msgs::Quaternion quaternion = marker.pose.orientation;
    double yaw = tf::getYaw(quaternion);
    Eigen::Vector3d ldir(cos(yaw), sin(yaw), 0);
    Eigen::Vector3d odir(-ldir[1], ldir[0], 0);

    double half_l = length / 2.0;
    double half_w = width / 2.0;
    double h = height;

    /*
    * @note Apollo's Object Coordinate
    *          |x
    *      C   |   D-----------
    *          |              |
    *  y---------------     length
    *          |              |
    *      B   |   A-----------
    */

    // Calculate vertices of the bottom and top quad and mid quad 
    Eigen::Vector3d bottom_quad[12];
        // A(-half_l, -half_w)
    bottom_quad[0] = center + ldir * -half_l + odir * -half_w;
    // B(-half_l, half_w)
    bottom_quad[1] = center + ldir * -half_l + odir * half_w;
    // C(half_l, half_w)
    bottom_quad[2] = center + ldir * half_l + odir * half_w;
    // D(half_l, -half_w)
    bottom_quad[3] = center + ldir * half_l + odir * -half_w;
    // top 4 vertices
    // E(-half_l, -half_w)
    bottom_quad[4] = bottom_quad[0];
    bottom_quad[4](2) += h;
    // F(-half_l, half_w)
    bottom_quad[5] = bottom_quad[1];
    bottom_quad[5](2) += h;
    // G(half_l, half_w)
    bottom_quad[6] = bottom_quad[2];
    bottom_quad[6](2) += h;
    // H(half_l, -half_w)
    bottom_quad[7] = bottom_quad[3];
    bottom_quad[7](2) += h;
    // middle 4 vertices
    // I
    bottom_quad[8] = bottom_quad[0];
    bottom_quad[8](2) += h/2;
    // J
    bottom_quad[9] = bottom_quad[1];
    bottom_quad[9](2) += h/2;
    // K
    bottom_quad[10] = bottom_quad[2];
    bottom_quad[10](2) += h/2;
    // L 
    bottom_quad[11] = bottom_quad[3];
    bottom_quad[11](2) += h/2;   

    // Create points between vertices using create_3dpoints function
    std::vector<Eigen::Vector3d> cloud_point_raw;
    double distance_duration = 0.2; 
    // Create points between A and B
    std::vector<Eigen::Vector3d> segment_AB = create_3dpoints(bottom_quad[0], bottom_quad[1], distance_duration);
    cloud_point_raw.insert(cloud_point_raw.end(), segment_AB.begin(), segment_AB.end());

    // Create points between B and C
    std::vector<Eigen::Vector3d> segment_BC = create_3dpoints(bottom_quad[1], bottom_quad[2], distance_duration);
    cloud_point_raw.insert(cloud_point_raw.end(), segment_BC.begin(), segment_BC.end());

    // Create points between E and F
    std::vector<Eigen::Vector3d> segment_EF = create_3dpoints(bottom_quad[4], bottom_quad[5], distance_duration);
    cloud_point_raw.insert(cloud_point_raw.end(), segment_EF.begin(), segment_EF.end());

    // Create points between F and G
    std::vector<Eigen::Vector3d> segment_FG = create_3dpoints(bottom_quad[5], bottom_quad[6], distance_duration);
    cloud_point_raw.insert(cloud_point_raw.end(), segment_FG.begin(), segment_FG.end());


    // Create points between I and J
    std::vector<Eigen::Vector3d> segment_IJ = create_3dpoints(bottom_quad[8], bottom_quad[9], distance_duration);
    cloud_point_raw.insert(cloud_point_raw.end(), segment_IJ.begin(), segment_IJ.end());

    // Create points between J and K
    std::vector<Eigen::Vector3d> segment_JK = create_3dpoints(bottom_quad[9], bottom_quad[10], distance_duration);
    cloud_point_raw.insert(cloud_point_raw.end(), segment_JK.begin(), segment_JK.end());


    // Create points between A and E (connecting top and bottom vertices)
    std::vector<Eigen::Vector3d> segment_AE = create_3dpoints(bottom_quad[0], bottom_quad[4], distance_duration);
    cloud_point_raw.insert(cloud_point_raw.end(), segment_AE.begin(), segment_AE.end());

    // Create points between B and F (connecting top and bottom vertices)
    std::vector<Eigen::Vector3d> segment_BF = create_3dpoints(bottom_quad[1], bottom_quad[5], distance_duration);
    cloud_point_raw.insert(cloud_point_raw.end(), segment_BF.begin(), segment_BF.end());

    // Create points between C and G (connecting top and bottom vertices)
    std::vector<Eigen::Vector3d> segment_CG = create_3dpoints(bottom_quad[2], bottom_quad[6], distance_duration);
    cloud_point_raw.insert(cloud_point_raw.end(), segment_CG.begin(), segment_CG.end());

    // Create points between D and H (connecting top and bottom vertices)
    // std::vector<Eigen::Vector3d> segment_DH = create_3dpoints(bottom_quad[3], bottom_quad[7], distance_duration);
    // cloud_point_raw.insert(cloud_point_raw.end(), segment_DH.begin(), segment_DH.end());


    // Convert cloud_point_raw to pcl_cloud
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl_cloud.width = cloud_point_raw.size();
    pcl_cloud.height = 1;
    pcl_cloud.points.resize(pcl_cloud.width * pcl_cloud.height);

    double intensity = 255 * rand() / (RAND_MAX + 1.0);

    for (size_t i = 0; i < pcl_cloud.points.size(); i++)
    {
        pcl_cloud.points[i].x = cloud_point_raw[i](0);
        pcl_cloud.points[i].y = cloud_point_raw[i](1);
        pcl_cloud.points[i].z = cloud_point_raw[i](2);
        pcl_cloud.points[i].intensity = intensity;
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header = header;

}

void markerArrayCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{   
    if (msg->markers.empty()) {
        ROS_WARN("Received empty MarkerArray message. Skipping...");
        return;
    }


    autosense_msgs::PointCloud2Array seg_pcl;
    seg_pcl.header = msg->markers[0].header; // Assume all markers have the same header

    sensor_msgs::PointCloud2 all_cloud;
    sensor_msgs::PointCloud2 cloud;

    // Create a PointCloud2Modifier object to modify the all_cloud
    // sensor_msgs::PointCloud2Modifier modifier(all_cloud);

    for (const auto& marker : msg->markers)
    {
        // Check if the marker's action is 3, and skip processing if it is
        if (marker.action == 3)
        {
            ROS_INFO("Skipping marker with action == 3");
            continue;
        }

        if (marker.action != 3){

        
        // Convert marker to segmented pcl and append to seg_pcl
        convert_marker2seg_pcl(marker, cloud);
        seg_pcl.clouds.push_back(cloud);
        // Append the data of 'cloud' to 'all_cloud'
        // modifier.addPointCloud(cloud);
        pclPub.publish(cloud);
        }
    }

    if (!seg_pcl.clouds.empty())
    {
        // Publish the PointCloud2Array
        segpclPub.publish(seg_pcl);
        
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_array_converter");
    ros::NodeHandle nh;

    ros::Subscriber markerArraySub = nh.subscribe("/tracking/transformed_objects_box_score", 10, markerArrayCallback);
    segpclPub = nh.advertise<autosense_msgs::PointCloud2Array>("/seg/segment_msgs", 1);
    pclPub = nh.advertise<sensor_msgs::PointCloud2>("/seg/pointcloud", 1);

    ros::spin();

    return 0;
}
