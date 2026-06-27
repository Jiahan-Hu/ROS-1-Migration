/*
 * This ROS node subscribes to a customized message type `visualization_header_msgs::MarkerArrayHeader` which 
        includes an additional header, and the topic name for this subscription is obtained from ROS parameters. 
 * The node also subscribes to the `/tf` topic to retrieve transformation information. 
 * It then performs transformations on markers contained in the incoming messages, converting them from the frame 
        specified in the received message header to the "map" frame. The transformed markers, along with the updated 
        header information, are published to another topic, also specified via ROS parameters.
 *
 * The FusionBbxTransformer class initializes the ROS node, sets up subscriptions, and performs the necessary transformations. 
 * The callback function `fusionBbxHeaderCallback` is triggered upon receiving messages. 
 * It transforms the marker poses using the provided transformation information and updates the header accordingly. 
 * The transformed markers with the updated header are then published to the specified topic. 
 * In case of any transformation exceptions, a warning is displayed.
 *
 * Node parameters:
 * - `fusion_bbx_topic`: The topic name for subscribing to `visualization_header_msgs::MarkerArrayHeader` messages.
 * - `transformed_bbx_topic`: The topic name for publishing the transformed `visualization_header_msgs::MarkerArrayHeader` messages.
 *
 * This node is initialized with the name "header_bbx_transformer_node" and uses the `FusionBbxTransformer` class to handle the subscription, transformation, and publication tasks. The node continues to spin and process callbacks as long as the ROS master is available.
 */
/*
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "visualization_msgs/MarkerArray.h"

class FusionBbxTransformer {
public:
    FusionBbxTransformer() : nh("~") {
        // Get the subscribed topic name from ROS parameter
        std::string fusion_bbx_topic;
        nh.param<std::string>("bbx_topic", fusion_bbx_topic, "/tracking/fusion_bbx_score");

        // Subscribe to the fusion_bbx_score_header topic
        fusion_bbx_header_sub = nh.subscribe(fusion_bbx_topic, 1, &FusionBbxTransformer::fusionBbxHeaderCallback, this);

        // Setup TF listener
        tf_listener = new tf2_ros::TransformListener(tf_buffer);

        // Get the advertised topic name from ROS parameter
        std::string transformed_bbx_topic;
        nh.param<std::string>("transformed_bbx_topic", transformed_bbx_topic, "/tracking/transformed_fusion_bbx_score");

        // Advertise the transformed_fusion_bbx_score_header topic
        transformed_bbx_pub = nh.advertise<visualization_msgs::MarkerArray>(transformed_bbx_topic, 1);
    }

    void fusionBbxHeaderCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
        try {
            // Get the latest transform for the desired frame (e.g., "map")
            geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform("map", msg->markers[0].header.frame_id, ros::Time(0));

            // Transform each marker in the array
            visualization_msgs::MarkerArray transformed_markers = *msg;
            for (auto& marker : transformed_markers.markers) {
                marker.header.frame_id = "map";
                tf2::doTransform(marker.pose, marker.pose, transformStamped);
            }

            // Add a custom header
            // transformed_markers_header.header.stamp = ros::Time::now();
            // transformed_markers_header.header.stamp = msg->header.stamp;
            // transformed_markers.header.frame_id = "map";

            // Publish the transformed markers header
            transformed_bbx_pub.publish(transformed_markers);
        } catch (tf2::TransformException& ex) {
            ROS_WARN("Transform exception: %s", ex.what());
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber fusion_bbx_header_sub;
    ros::Publisher transformed_bbx_pub;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener* tf_listener;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "bbx_transformer_node");

    FusionBbxTransformer bbx_transformer;

    ros::spin();

    return 0;
}
*/

#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

class FusionBbxTransformer {
public:
    FusionBbxTransformer() : nh("~"), tf_listener(tf_buffer) {  // Stack-allocated listener
        // Get the subscribed topic name from ROS parameter
        std::string fusion_bbx_topic;
        nh.param<std::string>("bbx_topic", fusion_bbx_topic, "/tracking/fusion_bbx_score");

        // Subscribe to the fusion_bbx_score_header topic
        fusion_bbx_header_sub = nh.subscribe(fusion_bbx_topic, 1, &FusionBbxTransformer::fusionBbxHeaderCallback, this);

        // Get the advertised topic name from ROS parameter
        std::string transformed_bbx_topic;
        nh.param<std::string>("transformed_bbx_topic", transformed_bbx_topic, "/tracking/transformed_fusion_bbx_score");

        // Advertise the transformed_fusion_bbx_score_header topic
        transformed_bbx_pub = nh.advertise<visualization_msgs::MarkerArray>(transformed_bbx_topic, 1);
    }

    void fusionBbxHeaderCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
        if (msg->markers.empty()) {
            ROS_WARN("Received empty MarkerArray.");
            return;
        }

        try {
            // Get the latest transform for the desired frame (e.g., "map")
            geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform("map", msg->markers[0].header.frame_id, ros::Time(0));

            // Transform each marker in the array
            visualization_msgs::MarkerArray transformed_markers = *msg;
            for (auto& marker : transformed_markers.markers) {
                marker.header.frame_id = "map";
                if (marker.action !=3) {
                    tf2::doTransform(marker.pose, marker.pose, transformStamped);
                }
                
            }

            // Publish the transformed markers
            transformed_bbx_pub.publish(transformed_markers);
        } catch (tf2::TransformException& ex) {
            ROS_WARN("Transform exception: %s", ex.what());
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber fusion_bbx_header_sub;
    ros::Publisher transformed_bbx_pub;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;  // Stack-allocated listener
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "bbx_transformer_node");

    FusionBbxTransformer bbx_transformer;

    ros::spin();

    return 0;
}
