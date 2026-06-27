/*
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/TimeReference.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

ros::Publisher fusion_pose_gps_pub;

void fusionPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& fusion_pose_msg,
                       const sensor_msgs::TimeReference::ConstPtr& gps_time_msg)
{
    // Create a new PoseStamped message
    geometry_msgs::PoseStamped fusion_pose_gps_msg = *fusion_pose_msg;

    // Assign the timestamp from /gps_can_time_topic to /fusion_pose's header
    fusion_pose_gps_msg.header.stamp = gps_time_msg->time_ref;

    // Publish the modified /fusion_pose message to /fusion_pose_gps
    fusion_pose_gps_pub.publish(fusion_pose_gps_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "loc_gps_time_node");
    ros::NodeHandle nh;

    // Create a Publisher to publish /fusion_pose_gps messages
    fusion_pose_gps_pub = nh.advertise<geometry_msgs::PoseStamped>("/fusion_pose_gps", 10);

    // Create Subscribers to subscribe to /fusion_pose and /gps_can_time_topic
    message_filters::Subscriber<geometry_msgs::PoseStamped> fusion_pose_sub(nh, "/fusion_pose", 10);
    message_filters::Subscriber<sensor_msgs::TimeReference> gps_time_sub(nh, "/gps_can_time_topic", 10);

    // ApproximateTime synchronizer to synchronize /fusion_pose and /gps_can_time_topic
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::TimeReference> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), fusion_pose_sub, gps_time_sub);
    sync.registerCallback(boost::bind(&fusionPoseCallback, _1, _2));

    ros::spin();

    return 0;
}
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/TimeReference.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <string>

ros::Publisher fusion_pose_gps_pub;

void fusionPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& fusion_pose_msg,
                       const sensor_msgs::TimeReference::ConstPtr& gps_time_msg)
{
    // Print out header to show that synchronization starts
    ROS_INFO_STREAM("\033[32mSync starts\033[0m");

    // Output the timestamp from gps_time_msg's header
    ROS_INFO("gps_time_msg header.stamp: %f", gps_time_msg->header.stamp.toSec());

    // Output the timestamp from fusion_pose_msg's header
    ROS_INFO("fusion_pose_msg header.stamp: %f", fusion_pose_msg->header.stamp.toSec());

    // Create a new PoseStamped message
    geometry_msgs::PoseStamped fusion_pose_gps_msg = *fusion_pose_msg;

    // Assign the timestamp from /gps_can_time_topic to /fusion_pose's header
    fusion_pose_gps_msg.header.stamp = gps_time_msg->time_ref;

    // Publish the modified /fusion_pose message to /fusion_pose_gps
    fusion_pose_gps_pub.publish(fusion_pose_gps_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "loc_gps_time_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Declare variables to store topic names
    std::string fusion_pose_topic;
    std::string gps_time_topic;
    std::string fusion_pose_gps_topic;

    // Get topic names from rosparam, with default values if not specified
    private_nh.param<std::string>("fusion_pose_topic", fusion_pose_topic, "/fusion_pose");
    private_nh.param<std::string>("gps_time_topic", gps_time_topic, "/gps_can_time_topic");
    private_nh.param<std::string>("fusion_pose_gps_topic", fusion_pose_gps_topic, "/fusion_pose_gps");

    // Create a Publisher to publish /fusion_pose_gps messages
    fusion_pose_gps_pub = nh.advertise<geometry_msgs::PoseStamped>(fusion_pose_gps_topic, 10);

    // Create Subscribers to subscribe to /fusion_pose and /gps_can_time_topic
    message_filters::Subscriber<geometry_msgs::PoseStamped> fusion_pose_sub(nh, fusion_pose_topic, 10);
    message_filters::Subscriber<sensor_msgs::TimeReference> gps_time_sub(nh, gps_time_topic, 10);

    // ApproximateTime synchronizer to synchronize /fusion_pose and /gps_can_time_topic
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::TimeReference> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), fusion_pose_sub, gps_time_sub);
    sync.registerCallback(boost::bind(&fusionPoseCallback, _1, _2));

    ros::spin();

    return 0;
}

/*
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/TimeReference.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <string>

ros::Publisher fusion_pose_gps_pub;

void fusionPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& fusion_pose_msg,
                        const sensor_msgs::TimeReference::ConstPtr& gps_time_msg)
{
    geometry_msgs::PoseStamped fusion_pose_gps_msg = *fusion_pose_msg;
    fusion_pose_gps_msg.header.stamp = gps_time_msg->time_ref;
    fusion_pose_gps_pub.publish(fusion_pose_gps_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fusion_pose_gps_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string fusion_pose_topic;
    std::string gps_time_topic;
    std::string fusion_pose_gps_topic;

    private_nh.param<std::string>("fusion_pose_topic", fusion_pose_topic, "/fusion_pose");
    private_nh.param<std::string>("gps_time_topic", gps_time_topic, "/gps_can_time_topic");
    private_nh.param<std::string>("fusion_pose_gps_topic", fusion_pose_gps_topic, "/fusion_pose_gps");

    message_filters::Subscriber<geometry_msgs::PoseStamped> fusion_pose_sub(nh, fusion_pose_topic, 10);
    message_filters::Subscriber<sensor_msgs::TimeReference> gps_time_sub(nh, gps_time_topic, 10);

    typedef message_filters::sync_policies::ApproximateTime<
        geometry_msgs::PoseStamped, sensor_msgs::TimeReference> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), fusion_pose_sub, gps_time_sub);
    sync.registerCallback(boost::bind(&fusionPoseCallback, _1, _2));

    fusion_pose_gps_pub = nh.advertise<geometry_msgs::PoseStamped>(fusion_pose_gps_topic, 10);

    ros::spin();

    return 0;
}
*/

