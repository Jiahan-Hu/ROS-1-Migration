#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

ros::Publisher pointcloud_pub;
ros::Publisher image_pub;

void callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg,
              const sensor_msgs::CompressedImageConstPtr& image_msg)
{
    std::cout << "\033[1;32m Syn! \033[0m" << std::endl;
    // Publish the synchronized messages to the output topics
    sensor_msgs::PointCloud2 output_pointcloud_msg(*pointcloud_msg);
    sensor_msgs::CompressedImage output_image_msg(*image_msg);
    output_pointcloud_msg.header.stamp = pointcloud_msg->header.stamp;
    output_image_msg.header.stamp = image_msg->header.stamp;
    pointcloud_pub.publish(output_pointcloud_msg);
    image_pub.publish(output_image_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sync_and_republish_node");
    ros::NodeHandle nh("~");

    // Get topic names from ROS parameters
    std::string pointcloud_topic, image_topic, pointcloud_output_topic, image_output_topic;
    nh.getParam("pointcloud_topic", pointcloud_topic);
    nh.getParam("image_topic", image_topic);
    nh.getParam("pointcloud_output_topic", pointcloud_output_topic);
    nh.getParam("image_output_topic", image_output_topic);

    // Create subscribers to the input topics
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub(nh, pointcloud_topic, 1);
    message_filters::Subscriber<sensor_msgs::CompressedImage> image_sub(nh, image_topic, 1);

    // Create publishers to the output topics
    pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>(pointcloud_output_topic, 1);
    image_pub = nh.advertise<sensor_msgs::CompressedImage>(image_output_topic, 1);

    // Create synchronizer to synchronize the input topics
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::CompressedImage> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), pointcloud_sub, image_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
    return 0;
}
