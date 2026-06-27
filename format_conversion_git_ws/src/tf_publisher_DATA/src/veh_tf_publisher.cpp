#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

// Global variables
std::string map_frame_id = "/map";
std::string child_frame_id = "/veh1_rslidar";
std::string ndt_pose_topic = "/ndt_pose";

// Callback function for the /ndt_pose topic
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  static tf::TransformBroadcaster br;

  // Extract the pose information from the message
  tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
                   msg->pose.orientation.z, msg->pose.orientation.w);
  tf::Vector3 t(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

  // Create a transform from the pose
  tf::Transform transform(q, t);

  // Set the timestamp
  ros::Time stamp(msg->header.stamp);

  // Publish the transform
  br.sendTransform(tf::StampedTransform(transform, stamp, map_frame_id, child_frame_id));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_to_tf_node");
  ros::NodeHandle nh("~");

  // Retrieve the parameters from the ROS parameter server
  nh.param<std::string>("map_frame_id", map_frame_id, "/map");
  nh.param<std::string>("child_frame_id", child_frame_id, "/veh1_rslidar");
  nh.param<std::string>("ndt_pose_topic", ndt_pose_topic, "/ndt_pose");

  // Subscribe to the specified ndt_pose_topic
  ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>(ndt_pose_topic, 10, poseCallback);

  // Spin the ROS node
  ros::spin();

  return 0;
}
