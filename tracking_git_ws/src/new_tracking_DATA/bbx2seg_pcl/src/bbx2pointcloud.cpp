/*
 * Input: subscribe a bounding box topic "/tracking_objects_box_score" (data type: visualization_msgs/MarkerArray), 
 convert this cube shape bounding box into a autosense_msgs/PointCloud2Array data type topic: "/segment/clouds_segmented".
 * Output: /segment/clouds_segmented (data type: autosense_msgs/PointCloud2Array)
 * This code fill the four vertical faces of the bounding box with 24-line point cloud and then publish it as 
 autosense_msgs/PointCloud2Array data type topic: "/segment/clouds_segmented". 
 * The goal of this node is to convert bounding boxes topic into a dense point cloud topic
 * Zhaoliang Zheng <zhz03@g.ucla.edu>
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "common/msgs/autosense_msgs/PointCloud2Array.h"

ros::Publisher segmentedCloudsPub;
ros::Publisher pointCloudPub;

void boundingBoxCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    autosense_msgs::PointCloud2Array segmentedClouds;
    sensor_msgs::PointCloud2 combinedCloud;  // Combined point cloud for all markers

    // Initialize the combined point cloud
    combinedCloud.header = msg->markers[0].header;  // Use the header of the first marker
    combinedCloud.height = 1;
    combinedCloud.width = 0;
    combinedCloud.fields.resize(3);
    combinedCloud.fields[0].name = "x";
    combinedCloud.fields[0].offset = 0;
    combinedCloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    combinedCloud.fields[0].count = 1;
    combinedCloud.fields[1].name = "y";
    combinedCloud.fields[1].offset = 4;
    combinedCloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    combinedCloud.fields[1].count = 1;
    combinedCloud.fields[2].name = "z";
    combinedCloud.fields[2].offset = 8;
    combinedCloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    combinedCloud.fields[2].count = 1;
    combinedCloud.point_step = 12;
    combinedCloud.is_dense = true;
    combinedCloud.is_bigendian = false;

    int publish_yes = 0;

    // Iterate through each bounding box marker in the marker array
    for (const auto& marker : msg->markers)
    {
        // Check if the marker represents a bounding box (shape = 1)
        if (marker.type == visualization_msgs::Marker::CUBE && marker.action == visualization_msgs::Marker::ADD)
        {   publish_yes = 1;
            // Create a point cloud for the bounding box
            sensor_msgs::PointCloud2 boxCloud;
            boxCloud.header = marker.header;
            boxCloud.height = 1;
            boxCloud.width = 32 * 6 * 4;  // Total number of points for all faces
            boxCloud.fields.resize(3);
            boxCloud.fields[0].name = "x";
            boxCloud.fields[0].offset = 0;
            boxCloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
            boxCloud.fields[0].count = 1;
            boxCloud.fields[1].name = "y";
            boxCloud.fields[1].offset = 4;
            boxCloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
            boxCloud.fields[1].count = 1;
            boxCloud.fields[2].name = "z";
            boxCloud.fields[2].offset = 8;
            boxCloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
            boxCloud.fields[2].count = 1;
            boxCloud.point_step = 12;
            boxCloud.row_step = boxCloud.point_step * boxCloud.width;
            boxCloud.is_dense = true;
            boxCloud.is_bigendian = false;
            boxCloud.data.resize(boxCloud.row_step);
            boxCloud.header.frame_id = marker.header.frame_id;

            // new 
            // Compute the size and orientation of the bounding box
            geometry_msgs::Vector3 scale = marker.scale;
            geometry_msgs::Point position = marker.pose.position;
            geometry_msgs::Quaternion orientation = marker.pose.orientation;

            // Convert the orientation to a tf2::Quaternion
            tf2::Quaternion tfQuaternion;
            tf2::fromMsg(orientation, tfQuaternion);

            // Generate the point cloud for the bounding box
            float spacing_x = 0.1;  // Point spacing in x direction
            float spacing_z = 0.1;  // Point spacing in z direction

            // Calculate the number of points
            int numPoints = 0;
            for (float x = -scale.x / 2.0; x <= scale.x / 2.0; x += spacing_x)
            {
                for (float y = -scale.y / 2.0; y <= scale.y / 2.0; y += spacing_x)
                {
                    for (float z = -scale.z / 2.0; z <= scale.z / 2.0; z += spacing_z)
                    {
                        numPoints++;
                    }
                }
            }

            // Resize the data vector
            boxCloud.data.resize(boxCloud.point_step * numPoints);
            boxCloud.width = numPoints;
            boxCloud.row_step = boxCloud.point_step * boxCloud.width;

            int pointIdx = 0;
            for (float x = -scale.x / 2.0; x <= scale.x / 2.0; x += spacing_x)
            {
                for (float y = -scale.y / 2.0; y <= scale.y / 2.0; y += spacing_x)
                {
                    for (float z = -scale.z / 2.0; z <= scale.z / 2.0; z += spacing_z)
                    {
                        // Apply the orientation to the point
                        // tf2::Vector3 point(x, y, z);
                        // point = tfQuaternion * point;

                        // Apply the orientation to the point
                        tf2::Vector3 point(x, y, z);
                        tf2::Quaternion pointQuaternion(point.x(), point.y(), point.z(), 0);  // Convert the point to a quaternion

                        // Multiply the quaternions
                        tf2::Quaternion rotatedPointQuaternion = tfQuaternion * pointQuaternion * tfQuaternion.inverse();

                        // Convert the result back to a vector
                        tf2::Vector3 rotatedPoint(rotatedPointQuaternion.x(), rotatedPointQuaternion.y(), rotatedPointQuaternion.z());

                        // Add the point to the point cloud
                        // boxCloud.data.push_back(position.x + point.x());
                        // boxCloud.data.push_back(position.y + point.y());
                        // boxCloud.data.push_back(position.z + point.z());
                        // boxCloud.width++;

                        // Add the point to the point cloud
                        geometry_msgs::Point32 p;
                        p.x = position.x + rotatedPoint.x();
                        p.y = position.y + rotatedPoint.y();
                        p.z = position.z + rotatedPoint.z();
                        memcpy(&boxCloud.data[pointIdx * boxCloud.point_step], &p, sizeof(p));
                        pointIdx++;
                    }
                }
            }

            // new ends

            // Merge the box cloud into the combined point cloud

            combinedCloud.width += boxCloud.width;
            combinedCloud.row_step = combinedCloud.point_step * combinedCloud.width;    
            // combinedCloud.width += boxCloud.width;
            // combinedCloud.row_step += boxCloud.row_step;
            combinedCloud.data.insert(combinedCloud.data.end(), boxCloud.data.begin(), boxCloud.data.end());

            // Add the box cloud to the segmented clouds
            segmentedClouds.clouds.push_back(boxCloud);
        }
    }

    if (publish_yes == 1){
    // Use the timestamp from the first marker

    combinedCloud.header.stamp = msg->markers[0].header.stamp;

    // Publish the combined point cloud
    pointCloudPub.publish(combinedCloud);

    // Publish the segmented clouds
    segmentedCloudsPub.publish(segmentedClouds);
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bounding_box_to_segmented1_clouds_node");
    ros::NodeHandle nh("~");

    std::string subscribeTopic;
    std::string outputTopic;
    std::string pc2Topic;
    nh.param<std::string>("subscribe_topic", subscribeTopic, "/tracking_objects_box_score");
    nh.param<std::string>("output_topic", outputTopic, "/segment/clouds_segmented");
    nh.param<std::string>("pc2_topic",pc2Topic,"/segment/point_cloud");

    ros::Subscriber boundingBoxSub = nh.subscribe(subscribeTopic, 10, boundingBoxCallback);
    segmentedCloudsPub = nh.advertise<autosense_msgs::PointCloud2Array>(outputTopic, 10);
    pointCloudPub = nh.advertise<sensor_msgs::PointCloud2>(pc2Topic, 10);

    ros::spin();

    return 0;
}
