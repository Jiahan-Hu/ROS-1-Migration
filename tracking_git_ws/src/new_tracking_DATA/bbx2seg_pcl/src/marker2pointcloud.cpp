#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "common/msgs/autosense_msgs/PointCloud2Array.h"

ros::Publisher pointCloudPub;
ros::Publisher pointCloudMergedPub;

void markerArrayCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    autosense_msgs::PointCloud2Array pointCloudArrayMsg;
    sensor_msgs::PointCloud2 pointCloudMsg;
    
    pointCloudArrayMsg.header = msg->markers[0].header;
    pointCloudMsg.header = msg->markers[0].header;
    
    for (const auto& marker : msg->markers)
    {
        // Check if the marker represents a bounding box (shape = 1)
        if (marker.type == visualization_msgs::Marker::CUBE && marker.action == visualization_msgs::Marker::ADD)
        {
        sensor_msgs::PointCloud2 pointCloud;
        pointCloud.header = marker.header;
        pointCloud.height = 1;
        pointCloud.is_bigendian = false;
        pointCloud.point_step = 32;
        pointCloud.is_dense = true;
        pointCloud.fields.resize(4);
        
        pointCloud.fields[0].name = "x";
        pointCloud.fields[0].offset = 0;
        pointCloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        pointCloud.fields[0].count = 1;
        
        pointCloud.fields[1].name = "y";
        pointCloud.fields[1].offset = 4;
        pointCloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        pointCloud.fields[1].count = 1;
        
        pointCloud.fields[2].name = "z";
        pointCloud.fields[2].offset = 8;
        pointCloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        pointCloud.fields[2].count = 1;
        
        pointCloud.fields[3].name = "intensity";
        pointCloud.fields[3].offset = 16;
        pointCloud.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
        pointCloud.fields[3].count = 1;
        
        tf2::Quaternion orientation;
        tf2::convert(marker.pose.orientation, orientation);
        
        tf2::Matrix3x3 rotationMatrix(orientation);
        tf2::Vector3 xAxis = rotationMatrix * tf2::Vector3(1, 0, 0);
        tf2::Vector3 yAxis = rotationMatrix * tf2::Vector3(0, 1, 0);
        tf2::Vector3 zAxis = rotationMatrix * tf2::Vector3(0, 0, 1);
        
        float minX = -marker.scale.x / 2.0;
        float maxX = marker.scale.x / 2.0;
        float minY = -marker.scale.y / 2.0;
        float maxY = marker.scale.y / 2.0;
        float minZ = -marker.scale.z / 2.0;
        float maxZ = marker.scale.z / 2.0;
        
        for (float x = minX; x <= maxX; x += 0.01)
        {
            for (float y = minY; y <= maxY; y += 0.01)
            {
                // Front face
                tf2::Vector3 pointOnFrontFace = tf2::Vector3(
                    marker.pose.position.x + (x * xAxis.x()) + (y * yAxis.x()) + (minZ * zAxis.x()),
                    marker.pose.position.y + (x * xAxis.y()) + (y * yAxis.y()) + (minZ * zAxis.y()),
                    marker.pose.position.z + (x * xAxis.z()) + (y * yAxis.z()) + (minZ * zAxis.z())
                );
                pointCloud.data.push_back(pointOnFrontFace.x());
                pointCloud.data.push_back(pointOnFrontFace.y());
                pointCloud.data.push_back(pointOnFrontFace.z());
                pointCloud.data.push_back(0.0);

                // Back face
                tf2::Vector3 pointOnBackFace = tf2::Vector3(
                    marker.pose.position.x + (x * xAxis.x()) + (y * yAxis.x()) + (maxZ * zAxis.x()),
                    marker.pose.position.y + (x * xAxis.y()) + (y * yAxis.y()) + (maxZ * zAxis.y()),
                    marker.pose.position.z + (x * xAxis.z()) + (y * yAxis.z()) + (maxZ * zAxis.z())
                );
                pointCloud.data.push_back(pointOnBackFace.x());
                pointCloud.data.push_back(pointOnBackFace.y());
                pointCloud.data.push_back(pointOnBackFace.z());
                pointCloud.data.push_back(0.0);
            }
        }
        
        /*
        for (float z = minZ; z <= maxZ; z += 0.01)
        {
            for (float x = minX; x <= maxX; x += 0.01)
            {
                // Left face
                tf2::Vector3 pointOnLeftFace = marker.pose.position +
                    (x * xAxis) +
                    (minY * yAxis) +
                    (z * zAxis);
                pointCloud.data.push_back(pointOnLeftFace.x());
                pointCloud.data.push_back(pointOnLeftFace.y());
                pointCloud.data.push_back(pointOnLeftFace.z());
                pointCloud.data.push_back(0.0);
                
                // Right face
                tf2::Vector3 pointOnRightFace = marker.pose.position +
                    (x * xAxis) +
                    (maxY * yAxis) +
                    (z * zAxis);
                pointCloud.data.push_back(pointOnRightFace.x());
                pointCloud.data.push_back(pointOnRightFace.y());
                pointCloud.data.push_back(pointOnRightFace.z());
                pointCloud.data.push_back(0.0);
            }
            
            for (float y = minY; y <= maxY; y += 0.01)
            {
                // Top face
                tf2::Vector3 pointOnTopFace = marker.pose.position +
                    (maxX * xAxis) +
                    (y * yAxis) +
                    (z * zAxis);
                pointCloud.data.push_back(pointOnTopFace.x());
                pointCloud.data.push_back(pointOnTopFace.y());
                pointCloud.data.push_back(pointOnTopFace.z());
                pointCloud.data.push_back(0.0);
                
                // Bottom face
                tf2::Vector3 pointOnBottomFace = marker.pose.position +
                    (minX * xAxis) +
                    (y * yAxis) +
                    (z * zAxis);
                pointCloud.data.push_back(pointOnBottomFace.x());
                pointCloud.data.push_back(pointOnBottomFace.y());
                pointCloud.data.push_back(pointOnBottomFace.z());
                pointCloud.data.push_back(0.0);
            }
            
        }
        */
        
        pointCloud.width = static_cast<uint32_t>(pointCloud.data.size() / pointCloud.point_step);
        pointCloud.row_step = pointCloud.width * pointCloud.point_step;
        
        pointCloudArrayMsg.clouds.push_back(pointCloud);
        }
    }
    
    // Publish segmented point clouds
    pointCloudPub.publish(pointCloudArrayMsg);
    
    // Publish merged point cloud
    pointCloudPub.publish(pointCloudMsg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bounding_box_to_point_cloud_converter");
    
    ros::NodeHandle nh;

    std::string subscribeTopic;
    std::string outputTopic;
    std::string pc2Topic;
    nh.param<std::string>("subscribe_topic", subscribeTopic, "/tracking_objects_box_score");
    nh.param<std::string>("output_topic", outputTopic, "/segment/clouds_segmented");
    nh.param<std::string>("pc2_topic",pc2Topic,"/segment/point_cloud");

    ros::Subscriber boundingBoxSub = nh.subscribe(subscribeTopic, 10, markerArrayCallback);
    pointCloudPub = nh.advertise<autosense_msgs::PointCloud2Array>(outputTopic, 10);
    pointCloudMergedPub = nh.advertise<sensor_msgs::PointCloud2>(pc2Topic, 10);

    // ros::Subscriber markerArraySub = nh.subscribe("/tracking_objects_box_score", 1, markerArrayCallback);
    // pointCloudPub = nh.advertise<autosense_msgs::PointCloud2Array>("/segment/clouds_segmented", 1);
    // ros::Publisher pointCloudMergedPub = nh.advertise<sensor_msgs::PointCloud2>("/segment/point_cloud", 1);
    
    ros::spin();
    
    return 0;
}
