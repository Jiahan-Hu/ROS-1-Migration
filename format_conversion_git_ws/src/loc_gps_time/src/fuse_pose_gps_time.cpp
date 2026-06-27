#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/TimeReference.h>
#include <string>

ros::Publisher fusion_pose_gps_pub;
ros::Subscriber fusion_pose_sub;
ros::Subscriber gps_time_sub; // 新的Subscriber

void fusionPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& fusion_pose_msg,
                       const sensor_msgs::TimeReference::ConstPtr& gps_time_msg)
{
    // Create a new PoseStamped message
    geometry_msgs::PoseStamped fusion_pose_gps_msg = *fusion_pose_msg;

    // Assign the /gps_can_time_topic timestamp to the /header field
    fusion_pose_gps_msg.header.stamp = gps_time_msg->time_ref;

    // Publish the new message to the /fusion_pose_gps topic
    fusion_pose_gps_pub.publish(fusion_pose_gps_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fusion_pose_gps_node");
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

    // Create a Subscriber for fusion_pose_topic
    fusion_pose_sub = nh.subscribe(fusion_pose_topic, 10, fusionPoseCallback);

    // Create a Subscriber for gps_time_topic
    gps_time_sub = nh.subscribe(gps_time_topic, 10, fusionPoseCallback);

    // Create a Publisher, publishing to the /fusion_pose_gps topic
    fusion_pose_gps_pub = nh.advertise<geometry_msgs::PoseStamped>(fusion_pose_gps_topic, 10);

    ros::spin();

    return 0;
}
