#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> // Added for PointXYZI
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include "common/msgs/autosense_msgs/PointCloud2Array.h"
#include "common/msgs/autosense_msgs/TrackingFixedTrajectoryArray.h"
#include "common/msgs/autosense_msgs/TrackingObjectArray.h"
#include "common/msgs/autosense_msgs/TrackingObjectArray_local_frame.h"


// define a new function here, also pseudocode code 
/*
In this code, you will need to based on the two 3d vector, create points between them

function create_3dpoints(3dpoint1,3dpoint2,distance_duration = 0.2){
    step1: calculate the distance "dist" between 3dpoint1 and 3dpoint2
    step2: calculate the number of points needs to generate between them: dist / distance_duration
    step3: generate these points in the middle.
    return these middle points
}

*/

// define this new function here, also pseudocode code 
/*
void convert_marker2seg_pcl(marker,&cloud){
    // get the header of marker
    header = marker.header 

    // scond get the size of marker
    length  = marker.scale.x
    width = marker.scale.y
    height = marker.scale.z

    Eigen::Vector3d center[3];
    center(0) = marker.pose.position.x
    center(1) = marker.pose.position.y
    center(2) = marker.pose.position.z - height / 2;

    geometry_msgs::Quaternion quaternion;
    quaternion = marker.pose.orientation

    double yaw = tf::getYaw(quaternion);
    Eigen::Vector3d ldir(cos(yaw), sin(yaw), 0);
    Eigen::Vector3d odir(-ldir[1], ldir[0], 0);

    double half_l = length / 2;
    double half_w = width / 2;
    double h = height;

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
    bottom_quad[4] = bottom_quad[0];
    bottom_quad[4](2) += h;
    bottom_quad[5] = bottom_quad[1];
    bottom_quad[5](2) += h;
    bottom_quad[6] = bottom_quad[2];
    bottom_quad[6](2) += h;
    bottom_quad[7] = bottom_quad[3];
    bottom_quad[7](2) += h;
    // middle 4 vertices
    bottom_quad[8] = bottom_quad[0];
    bottom_quad[8](2) += h/2;
    bottom_quad[9] = bottom_quad[1];
    bottom_quad[9](2) += h/2;
    bottom_quad[10] = bottom_quad[2];
    bottom_quad[10](2) += h/2;
    bottom_quad[11] = bottom_quad[3];
    bottom_quad[11](2) += h/2;    

    // create a variable called "cloud_point_raw" to put all the points in bottom_quad

    cloud_point_raw = create_3dpoints(bottom_quad[0],bottom_quad[1],distance_duration = 0.2)
    cloud_point_raw = create_3dpoints(bottom_quad[1],bottom_quad[2],distance_duration = 0.2)
    cloud_point_raw = create_3dpoints(bottom_quad[4],bottom_quad[5],distance_duration = 0.2)
    cloud_point_raw = create_3dpoints(bottom_quad[5],bottom_quad[6],distance_duration = 0.2)

    cloud_point_raw = create_3dpoints(bottom_quad[8],bottom_quad[9],distance_duration = 0.2)
    cloud_point_raw = create_3dpoints(bottom_quad[9],bottom_quad[10],distance_duration = 0.2)

    cloud_point_raw = create_3dpoints(bottom_quad[0],bottom_quad[4],distance_duration = 0.2)
    cloud_point_raw = create_3dpoints(bottom_quad[1],bottom_quad[5],distance_duration = 0.2)
    cloud_point_raw = create_3dpoints(bottom_quad[2],bottom_quad[6],distance_duration = 0.2)

    // next, convert  cloud_point_raw into sensor_msgs::PointCloud2 cloud; here is the pseudocode code:
    
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud; 
    pcl_cloud.width = number of cloud_point_raw
    pcl_cloud.height = 1;
    pcl_cloud.points.resize(cloud.width * cloud.height);

    double intensity = 255 * rand() / (RAND_MAX + 1.0f);
    for (size_t i = 0; i < pcl_cloud.points.size(); i++){
        pcl_cloud.points[i].x = cloud_point_raw[i](0)
        pcl_cloud.points[i].y = cloud_point_raw[i](1)
        pcl_cloud.points[i].z = cloud_point_raw[i](2)
        pcl_cloud.points[i].intensity = intensity
    }

    pcl::toROSMsg(pcl_cloud, cloud);

}

*/


void markerArrayCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    // This function will be called whenever a new MarkerArray message is received
    // Convert markers to PointCloud2 messages and publish as PointCloud2Array
    autosense_msgs::PointCloud2Array seg_pcl;
    seg_pcl.header = msg.header;
    
    // the following is the pseudocode
    /*
    for each marker in the marker array do the following:{
        sensor_msgs::PointCloud2 cloud;
        convert_marker2seg_pcl(marker,&cloud);
        seg_pcl.clouds.push_back(cloud);
    }

	if (!msg.clouds.empty())
	{
		// Publish the PointCloud2Array
        pclPub.publish(seg_pcl);
	}

    */

    

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_array_converter");
    ros::NodeHandle nh;
    
    // Subscribe to the /tracking/objects_box_score topic
    ros::Subscriber markerArraySub = nh.subscribe("/tracking/objects_box_score", 10, markerArrayCallback);
    
    // Advertise the PointCloud2Array publisher
    ros::Publisher pclPub = nh.advertise<autosense_msgs::PointCloud2Array>("point_cloud_array", 1);
    
    // Spin and process incoming messages
    ros::spin();
    
    return 0;
}
