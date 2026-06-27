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

#include "common/time.hpp"          // calculate running time
#include "common/common.hpp"
#include "common/geometry.hpp"      // common::geometry::calcYaw4DirectionVector
#include "common/transform.hpp"     // common::transform::transformPointCloud
#include "common/types/object.hpp"  // ObjectPtr
#include "common/types/type.h"

ros::Publisher segpclPub;
ros::Publisher pclPub;

// Define the create_3dpoints function
std::vector<Eigen::Vector3d> create_3dpoints(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2, double distance_duration)
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

std::vector<Eigen::Vector3d> ellipsoidPoints_in_cude(const Eigen::Vector3d bottom_quad[12], const Eigen::Vector3d& center, double length, double width, double height, int point_num) {
        // create a function called ellipsoidPoints_in_cude,
    /*
    The input of this function is bottom_quad, center, point_num 
    The output of this function should be std::vector<Eigen::Vector3d> ellipsoidPoints. 

    In this function, you will need to based on the 8 vertices of bottom_quad, which are bottom_quad[0] ~ bottom_quad[7]
        and center of the cube, you generate a group of points that form the surface of the ellipsoid that fills the cube,
        what needs to be notice is that all the generated point should not exceed the cube.

    the number of point should be point_num.
        

    */
    
   // std::vector<Eigen::Vector3d> ellipsoidPoints = ellipsoidPoints_in_cude(bottom_quad,center,length,width,height,point_num);
   
    std::vector<Eigen::Vector3d> ellipsoidPoints;

    // Calculate semi-axes lengths of the ellipsoid
    double a = length / 2.0;
    double b = width / 2.0;
    double c = height / 2.0;

    // Generate ellipsoid surface points within the cube
    for (double theta = 0; theta < M_PI; theta += M_PI / point_num) {
        for (double phi = 0; phi < 2 * M_PI; phi += 2 * M_PI / point_num) {
            double x = a * sin(theta) * cos(phi);
            double y = b * sin(theta) * sin(phi);
            double z = c * cos(theta);

            // Convert ellipsoid coordinates to world coordinates
            Eigen::Vector3d ellipsoidPoint = center + Eigen::Vector3d(x, y, z);

            // Check if the ellipsoid point is inside the cube
            if (ellipsoidPoint(0) >= bottom_quad[0](0) && ellipsoidPoint(0) <= bottom_quad[2](0) &&
                ellipsoidPoint(1) >= bottom_quad[0](1) && ellipsoidPoint(1) <= bottom_quad[2](1) &&
                ellipsoidPoint(2) >= bottom_quad[0](2) && ellipsoidPoint(2) <= bottom_quad[4](2)) {
                ellipsoidPoints.push_back(ellipsoidPoint);
            }
        }
    }

    return ellipsoidPoints;
}

int cal_point_num(double length, double width, double height, double distance_duration){
    double max1 = 0;
    double max2 = 0;
    // compared length, width and height, select the first two biggest and put into max1 and max2 

    if (length >= width && length >= height) {
        max1 = length;
        max2 = std::max(width, height);
    } else if (width >= length && width >= height) {
        max1 = width;
        max2 = std::max(length, height);
    } else {
        max1 = height;
        max2 = std::max(length, width);
    }

    return static_cast<int>((max1 / distance_duration) * (max2 / distance_duration));

}

std::vector<Eigen::Vector3d> random_fill0(const Eigen::Vector3d bottom_quad[12], int point_num) {
    std::vector<Eigen::Vector3d> random_fill_points;

    // Generate random points within the cube defined by bottom_quad
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist_x(bottom_quad[0](0), bottom_quad[2](0));
    std::uniform_real_distribution<double> dist_y(bottom_quad[0](1), bottom_quad[2](1));
    std::uniform_real_distribution<double> dist_z(bottom_quad[0](2), bottom_quad[4](2));

    for (int i = 0; i < point_num; ++i) {
        double x = dist_x(gen);
        double y = dist_y(gen);
        double z = dist_z(gen);

        Eigen::Vector3d point(x, y, z);
        random_fill_points.push_back(point);
        /*
        if (point(0) >= bottom_quad[0](0) && point(0) <= bottom_quad[2](0) &&
            point(1) >= bottom_quad[0](1) && point(1) <= bottom_quad[2](1) &&
            point(2) >= bottom_quad[0](2) && point(2) <= bottom_quad[4](2)) {
            random_fill_points.push_back(point);
        }*/
    }

    return random_fill_points;
}

std::vector<Eigen::Vector3d> random_fill1(const Eigen::Vector3d bottom_quad[12], int point_num) {
    std::vector<Eigen::Vector3d> random_fill_points;

    // Generate random points within the cube defined by bottom_quad
    std::random_device rd;
    std::mt19937 gen(rd());

    for (int i = 0; i < point_num; ++i) {
        int idx = i % 8; // Cycle through the 8 vertices of bottom_quad
        double x_range = bottom_quad[2](0) - bottom_quad[0](0);
        double y_range = bottom_quad[2](1) - bottom_quad[0](1);
        double z_range = bottom_quad[4](2) - bottom_quad[0](2);
        
        double x = bottom_quad[0](0) + x_range * static_cast<double>(rand()) / RAND_MAX;
        double y = bottom_quad[0](1) + y_range * static_cast<double>(rand()) / RAND_MAX;
        double z = bottom_quad[0](2) + z_range * static_cast<double>(rand()) / RAND_MAX;

        Eigen::Vector3d point(x, y, z);
        // Check if the ellipsoid point is inside the cube
        if (point(0) >= bottom_quad[0](0) && point(0) <= bottom_quad[2](0) &&
            point(1) >= bottom_quad[0](1) && point(1) <= bottom_quad[2](1) &&
            point(2) >= bottom_quad[0](2) && point(2) <= bottom_quad[4](2)) {
            random_fill_points.push_back(point);
        }

        // random_fill_points.push_back(point);
    }

    return random_fill_points;
}

std::vector<Eigen::Vector3d> random_fill(const Eigen::Vector3d& center, double length, double width, double height, double yaw, int point_num) {
    std::vector<Eigen::Vector3d> random_points;

    double offset = 0.2;
    // Create random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist_x((-length / 2.0 + offset), (length / 2.0 - offset));
    std::uniform_real_distribution<double> dist_y((-width / 2.0 + offset), (width / 2.0-offset));
    std::uniform_real_distribution<double> dist_z((-height / 2.0+offset), (height / 2.0-offset));

    // Convert yaw to radians
    // double yaw_rad = yaw * M_PI / 180.0;

    // Generate random points within the specified cube
    for (int i = 0; i < point_num; ++i) {
        double x = dist_x(gen);
        double y = dist_y(gen);
        double z = dist_z(gen);

        // Rotate the point based on the yaw angle
        double rotated_x = x * cos(yaw) - y * sin(yaw);
        double rotated_y = x * sin(yaw) + y * cos(yaw);
        double rotated_z = z;

        Eigen::Vector3d point(rotated_x + center[0], rotated_y + center[1], rotated_z + center[2] + height/2);
        random_points.push_back(point);
    }

    return random_points;
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
    int point_num = 0;
    
    point_num = cal_point_num(length, width, height, distance_duration);

    for (const auto& point : bottom_quad) {
        cloud_point_raw.push_back(point);
    }

    // create a function called random_fill 
    /*
        the input of random_fill should be bottom_quad,center, point_num
        the output of random fill should be std::vector<Eigen::Vector3d> random_fill_points

        the goal of this function is to generate point_num of random points that fill in the cube, which is define by the vertices of cube bottom_quad[0]~bottom_quad[7]
        please note that all the generated point should be inside the cube. 

    */
    // Generate random points using random_fill function
    // std::vector<Eigen::Vector3d> random_fill_points = random_fill(bottom_quad,point_num);
    std::vector<Eigen::Vector3d> random_fill_points = random_fill(center,length,  width,  height,  yaw,  point_num);
   
   cloud_point_raw.insert(cloud_point_raw.end(), random_fill_points.begin(), random_fill_points.end());


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
    pcl::PointCloud<pcl::PointXYZI> pcl_all_cloud;  // PCL point cloud to hold all points

    // Create a PointCloud2Modifier object to modify the all_cloud
    // sensor_msgs::PointCloud2Modifier modifier(all_cloud);

    autosense::common::Clock clock;

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
        // how to Append the data of 'cloud' to 'all_cloud'

        // pclPub.publish(cloud);

        pcl::PointCloud<pcl::PointXYZI> pcl_cloud;  // Temporary PCL point cloud
        pcl::fromROSMsg(cloud, pcl_cloud);  // Convert to PCL point cloud

        pcl_all_cloud += pcl_cloud;  // Append to the main PCL point cloud

        }
    }



    if (!seg_pcl.clouds.empty())
    {
        // Publish the PointCloud2Array
        segpclPub.publish(seg_pcl);

        pcl::toROSMsg(pcl_all_cloud, all_cloud);  // Convert the combined PCL point cloud back to ROS msg
        all_cloud.header = seg_pcl.header;  // Set the header

        pclPub.publish(all_cloud);
    }

    ROS_INFO_STREAM("BBX to segment pointcloud conversion Took "
                << clock.takeRealTime() << "ms.");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_array_converter");
    ros::NodeHandle nh("~");

    std::string sub_bbx_topic,pub_segment_topic,pub_pointcloud_topic; 

    nh.getParam("bbx2segpcl/sub_bbx_score",sub_bbx_topic);
    nh.getParam("bbx2segpcl/pub_segment_msg",pub_segment_topic);
    nh.getParam("bbx2segpcl/pub_pointcloud",pub_pointcloud_topic);

    ros::Subscriber markerArraySub = nh.subscribe(sub_bbx_topic, 10, markerArrayCallback);
    segpclPub = nh.advertise<autosense_msgs::PointCloud2Array>(pub_segment_topic, 1);
    pclPub = nh.advertise<sensor_msgs::PointCloud2>(pub_pointcloud_topic, 1);

    ros::spin();

    return 0;
}
