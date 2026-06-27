#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    // Create a new pcl/PointCloud to store the selected lines
    pcl::PointCloud<pcl::PointXYZ> new_cloud;
    new_cloud.header = cloud.header;
    new_cloud.height = cloud.height;
    new_cloud.width = 32; //cloud.width;

    ROS_INFO("cloud.width: %d", cloud.width);

    // Iterate through the original cloud and select desired lines based on a specific criterion
    for (int i = 0; i < cloud.width; i++) {
        // Select every 4th line
        if (i % 4 == 0) {
            // output i 
            ROS_INFO("i: %d", i);
            for (int j = 0; j < cloud.height; j++) {
                new_cloud.push_back(cloud.at(i,j));
            }
        }
    }
    // new_cloud.width = 32; //cloud.width;
    // Convert the new pcl/PointCloud to sensor_msgs/PointCloud2
    sensor_msgs::PointCloud2 new_cloud_msg;
    pcl::toROSMsg(new_cloud, new_cloud_msg);

    // Publish the new sensor_msgs/PointCloud2 message
    pub.publish(new_cloud_msg);
}

void cloud_cb_sp (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    // Create a new pcl/PointCloud to store the selected lines
    pcl::PointCloud<pcl::PointXYZI> new_cloud;
    new_cloud.header = cloud.header;
    new_cloud.height = cloud.height;
    new_cloud.width = 37; //cloud.width;

    // keep the intensity value in the new cloud
    new_cloud.is_dense = false;

    // Iterate through the original cloud and select desired lines based on a specific criterion
    // map the 128 line to fake 64 line (37)
    
    for (int i = 0; i < cloud.width; i++) {
        
        // if i in [1,2,3,4,5,6,7,8,9,13,14,15,20,21,24,26,31,33,36,38,43,48,50,55,57,60,61,67,71,74,78,84,85,88,91,95,102] - 1 
        if (i == 0 || i == 1 || i == 2 || i == 3 || i == 4 || i == 5 || i == 6 || i == 7 || i == 8 || i == 12 || i == 13 || i == 14 || i == 19 || i == 20 || i == 23 || i == 25 || i == 30 || i == 32 || i == 35 || i == 37 || i == 42 || i == 47 || i == 49 || i == 54 || i == 56 || i == 59 || i == 60 || i == 66 || i == 70 || i == 73 || i == 77 || i == 83 || i == 84 || i == 87 || i == 90 || i == 94 || i == 101) {
            for (int j = 0; j < cloud.height; j++) {
                // push back the point to the perticular line of new_cloud
                new_cloud.push_back(cloud.at(i,j));
            }
        }

    }
    // new_cloud.width = 32; //cloud.width;
    // Convert the new pcl/PointCloud to sensor_msgs/PointCloud2
    sensor_msgs::PointCloud2 new_cloud_msg;
    pcl::toROSMsg(new_cloud, new_cloud_msg);

    // Publish the new sensor_msgs/PointCloud2 message
    pub.publish(new_cloud_msg);
}


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "pointcloud_subset");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // get parameters
    std::string input_topic;
    nh_private.param<std::string>("input_topic", input_topic, "/rslidar_points");
    // output topic 
    std::string output_topic;
    nh_private.param<std::string>("output_topic", output_topic, "/rslidar_points_subset");

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe (input_topic, 1, cloud_cb_sp);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> (output_topic, 1);

    // Spin
    ros::spin ();
}
