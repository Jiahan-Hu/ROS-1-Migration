/*
 * Input: subscribe a bounding box topic "/tracking_objects_box_score" (data type: /segment/clouds_segmented), 
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
#include "common/msgs/autosense_msgs/PointCloud2Array.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

ros::Publisher segmentedCloudsPub;

void boundingBoxCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    autosense_msgs::PointCloud2Array segmentedClouds;

    // Iterate through each bounding box marker in the marker array
    for (const auto& marker : msg->markers)
    {
        // Check if the marker represents a bounding box (shape = 1)
        if (marker.type == visualization_msgs::Marker::CUBE && marker.action == visualization_msgs::Marker::ADD)
        {
            // Create a point cloud for the four vertical faces of the bounding box
            sensor_msgs::PointCloud2 faceCloud;
            faceCloud.header = marker.header;
            faceCloud.height = 1;
            faceCloud.width = 24;
            faceCloud.fields.resize(3);
            faceCloud.fields[0].name = "x";
            faceCloud.fields[0].offset = 0;
            faceCloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
            faceCloud.fields[0].count = 1;
            faceCloud.fields[1].name = "y";
            faceCloud.fields[1].offset = 4;
            faceCloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
            faceCloud.fields[1].count = 1;
            faceCloud.fields[2].name = "z";
            faceCloud.fields[2].offset = 8;
            faceCloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
            faceCloud.fields[2].count = 1;
            faceCloud.point_step = 12;
            faceCloud.row_step = faceCloud.point_step * faceCloud.width;
            faceCloud.is_dense = true;
            faceCloud.is_bigendian = false;
            faceCloud.data.resize(faceCloud.row_step);
            faceCloud.header.frame_id = marker.header.frame_id;

            // Compute the size and orientation of the bounding box
            geometry_msgs::Vector3 scale = marker.scale;
            geometry_msgs::Point32 position;
            position.x = marker.pose.position.x;
            position.y = marker.pose.position.y;
            position.z = marker.pose.position.z;

            // Get the orientation of the bounding box
            tf2::Quaternion quaternion(marker.pose.orientation.x,
                                       marker.pose.orientation.y,
                                       marker.pose.orientation.z,
                                       marker.pose.orientation.w);
            
            tf2::Matrix3x3 rotationMatrix(quaternion);

            // Generate the 24-line point cloud for each vertical face
            for (int i = 0; i < 6; ++i)
            {
                for (int j = 0; j < 4; ++j)
                {
                    int pointIdx = i * 4 + j;
                    geometry_msgs::Point32 point;
                    point.x = position.x + scale.x * (pointIdx % 4 < 2 ? -0.5 : 0.5);
                    point.y = position.y + scale.y * (pointIdx / 4 < 2 ? -0.5 : 0.5);
                    point.z = position.z + scale.z * (i < 2 ? -0.5 : 0.5);

                    // Apply rotation to each point
                    tf2::Vector3 point_tf(point.x, point.y, point.z);
                    point_tf = rotationMatrix * point_tf;

                    // Update point coordinates after rotation
                    point.x = point_tf.getX();
                    point.y = point_tf.getY();
                    point.z = point_tf.getZ();

                    memcpy(&faceCloud.data[pointIdx * 12], &point, sizeof(point));
                }
            }

            // Add the face cloud to the segmented clouds
            segmentedClouds.clouds.push_back(faceCloud);
        }
    }

    // Publish the segmented clouds
    segmentedCloudsPub.publish(segmentedClouds);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bounding_box_to_segmented_clouds");
    ros::NodeHandle nh("~");

    std::string subscribeTopic;
    std::string outputTopic;
    nh.param<std::string>("subscribe_topic", subscribeTopic, "/tracking_objects_box_score");
    nh.param<std::string>("output_topic", outputTopic, "/segment/clouds_segmented");

    ros::Subscriber boundingBoxSub = nh.subscribe(subscribeTopic, 10, boundingBoxCallback);
    segmentedCloudsPub = nh.advertise<autosense_msgs::PointCloud2Array>(outputTopic, 10);

    ros::spin();

    return 0;
}