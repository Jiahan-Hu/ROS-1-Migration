#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

ros::Publisher marker_pub;
std::string fusion_pose_topic;
std::string visualization_marker_topic;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Create a visualization_msgs/Marker message
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp. See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = msg->header.stamp;

    // Set the namespace and id for this marker. This serves to create a unique ID
    marker.ns = "bounding_box";
    marker.id = 0;

    // Set the marker type. CUBE is chosen here to represent the bounding box.
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action. In this case, ADD to add a marker.
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker. This will be the pose received from /fusion_pose.
    marker.pose = msg->pose;

    // Set the scale of the marker. Assuming dimensions are in inches and converting to meters.
    // Note: 1 inch = 0.0254 meter
    marker.scale.x = 194 * 0.0254;
    marker.scale.y = 73 * 0.0254;
    marker.scale.z = 58 * 0.0254;

    // Set the color. Green is chosen here.
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Publish the marker to the specified topic (visualization_marker_topic)
    marker_pub.publish(marker);
}

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "pose_visualization_node");
    ros::NodeHandle nh;

    // Get fusion_pose and visualization_marker topics from rosparams
    nh.getParam("fusion_pose", fusion_pose_topic);
    nh.getParam("visualization_marker", visualization_marker_topic);

    // Subscribe to the fusion_pose topic
    ros::Subscriber pose_sub = nh.subscribe(fusion_pose_topic, 10, poseCallback);

    // Advertise a new publisher for the visualization marker topic
    marker_pub = nh.advertise<visualization_msgs::Marker>(visualization_marker_topic, 10);

    // Keep spinning
    ros::spin();

    return 0;
}
