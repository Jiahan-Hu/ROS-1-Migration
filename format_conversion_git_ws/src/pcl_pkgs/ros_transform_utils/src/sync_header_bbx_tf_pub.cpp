#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/message_filter.h>
#include "visualization_header_msgs/MarkerArrayHeader.h"
#include <memory>
#include <message_filters/subscriber.h>

class FusionBbxTransformer {
public:
    FusionBbxTransformer()
    : nh("~"), tf_listener(std::make_shared<tf2_ros::TransformListener>(tf_buffer)),
      fusion_bbx_header_sub(nh, fusion_bbx_topic, 1), tf_filter(fusion_bbx_header_sub, tf_buffer, "map", 10, nh) {

        if (!nh.getParam("fusion_bbx_topic", fusion_bbx_topic)) {
            ROS_WARN("Using default topic: /tracking/fusion_bbx_score_header");
            fusion_bbx_topic = "/tracking/fusion_bbx_score_header";
        }

        if (!nh.getParam("transformed_bbx_topic", transformed_bbx_topic)) {
            ROS_WARN("Using default topic: /tracking/transformed_fusion_bbx_score_header");
            transformed_bbx_topic = "/tracking/transformed_fusion_bbx_score_header";
        }

        transformed_bbx_header_pub = nh.advertise<visualization_header_msgs::MarkerArrayHeader>(transformed_bbx_topic, 1);
        tf_filter.registerCallback(boost::bind(&FusionBbxTransformer::fusionBbxHeaderCallback, this, _1));
    }

    void fusionBbxHeaderCallback(const visualization_header_msgs::MarkerArrayHeader::ConstPtr& msg) {
        try {
            geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform("map", msg->header.frame_id, ros::Time(0));

            visualization_header_msgs::MarkerArrayHeader transformed_markers_header = *msg;
            for (auto& marker : transformed_markers_header.markers) {
                marker.header.frame_id = "map";
                tf2::doTransform(marker.pose, marker.pose, transformStamped);
            }

            transformed_markers_header.header.stamp = msg->header.stamp;
            transformed_markers_header.header.frame_id = "map";

            transformed_bbx_header_pub.publish(transformed_markers_header);
        } catch (tf2::TransformException& ex) {
            ROS_WARN("Transform exception: %s", ex.what());
            // Add more error-handling logic here if needed
        }
    }

private:
    ros::NodeHandle nh;
    std::string fusion_bbx_topic;
    std::string transformed_bbx_topic;
    message_filters::Subscriber<visualization_header_msgs::MarkerArrayHeader> fusion_bbx_header_sub;
    tf2_ros::MessageFilter<visualization_header_msgs::MarkerArrayHeader> tf_filter;
    ros::Publisher transformed_bbx_header_pub;
    tf2_ros::Buffer tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "header_bbx_transformer_node");
    FusionBbxTransformer header_bbx_transformer;
    ros::spin();
    return 0;
}
