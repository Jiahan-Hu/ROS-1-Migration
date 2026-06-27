#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>

class FusionNode {
private:
    visualization_msgs::MarkerArray ego_markers;
    visualization_msgs::MarkerArray agent1_markers;
    visualization_msgs::Marker first_ego_marker;
    visualization_msgs::Marker first_agent1_marker;

    ros::Publisher fusion_pub;
    ros::Subscriber sub, sub1;

    std::string sub_ego_bbx_topic;
    std::string sub_agent1_bbx_topic;
    std::string pub_fusion_bbx_topic;

public:
    FusionNode() {
        ros::NodeHandle nh("~");
        nh.getParam("v2xfusion/sub_ego_bbx", sub_ego_bbx_topic);
        nh.getParam("v2xfusion/sub_agent1_bbx", sub_agent1_bbx_topic);
        nh.getParam("v2xfusion/pub_fusion_bbx", pub_fusion_bbx_topic);

        ROS_INFO("Config parameters are as follows:");
        ROS_INFO("sub_infra1_bbx_topic: %s", sub_ego_bbx_topic.c_str());
        ROS_INFO("sub_veh1_bbx_topic: %s", sub_agent1_bbx_topic.c_str());
        ROS_INFO("pub_fusion_bbx_topic: %s", pub_fusion_bbx_topic.c_str());

        fusion_pub = nh.advertise<visualization_msgs::MarkerArray>(pub_fusion_bbx_topic, 10);
        sub = nh.subscribe(sub_ego_bbx_topic, 10, &FusionNode::callback0, this);
        sub1 = nh.subscribe(sub_agent1_bbx_topic, 10, &FusionNode::callback1, this);
    }

    void callback0(const visualization_msgs::MarkerArray::ConstPtr& msg) {
        ego_markers = *msg;
        first_ego_marker = ego_markers.markers[0];
        if (first_ego_marker.action == 3) {
            clear_markerarray(first_ego_marker, fusion_pub);
        }
        // ROS_INFO("data received!");
    }

    void callback1(const visualization_msgs::MarkerArray::ConstPtr& msg) {
        agent1_markers = *msg;
        first_agent1_marker = agent1_markers.markers[0];
    }
    /*
    bool subscribe_to_topic(const std::string& topic_name) {
        if (ros::topic::exists(topic_name)) {
            ROS_INFO("Subscribed to topic %s.", topic_name.c_str());
            return true;
        } else {
            ROS_WARN("Topic %s does not exist.", topic_name.c_str());
            return false;
        }
    }
    */

    bool subscribe_to_topic(const std::string& topic_name) {
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);
    for (const auto& topic_info : topic_infos) {
        if (topic_info.name == topic_name) {
            ROS_INFO("Subscribed to topic %s.", topic_name.c_str());
            return true;
        }
    }
    ROS_WARN("Topic %s does not exist.", topic_name.c_str());
    return false;
    }


    void clear_markerarray(const visualization_msgs::Marker& ego_maker, const ros::Publisher& ma_pub) {
        visualization_msgs::MarkerArray empty_markers;
        visualization_msgs::Marker clear_marker;

        clear_marker.header = ego_maker.header;
        clear_marker.ns = "objects";
        clear_marker.id = 0;
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        clear_marker.lifetime = ros::Duration(0);

        empty_markers.markers.push_back(clear_marker);
        ma_pub.publish(empty_markers);

        ROS_INFO("%d", visualization_msgs::Marker::DELETEALL);
    }

    void fusion_add() {
        visualization_msgs::MarkerArray fusion_markers;

        if (!subscribe_to_topic(sub_agent1_bbx_topic)) {
            agent1_markers.markers.clear();
            fusion_markers.markers.insert(fusion_markers.markers.end(), agent1_markers.markers.begin(), agent1_markers.markers.end());
        }

        ROS_INFO("-----------------------------------------");
        ROS_INFO("ego_markers length: %lu", ego_markers.markers.size());
        ROS_INFO("agent1_markers length: %lu", agent1_markers.markers.size());

        if (ego_markers.markers.size() == 1) {
            ROS_ERROR("ego_marker empty!");
            ROS_INFO("Action: %d", ego_markers.markers[0].action);
        }

        if (first_ego_marker.action != 3 && !ego_markers.markers.empty()) {
            fusion_markers.markers.insert(fusion_markers.markers.end(), ego_markers.markers.begin(), ego_markers.markers.end());
        }

        if (first_agent1_marker.action != 3 && !agent1_markers.markers.empty()) {
            fusion_markers.markers.insert(fusion_markers.markers.end(), agent1_markers.markers.begin(), agent1_markers.markers.end());
        }

        if (!fusion_markers.markers.empty()) {
            for (auto& marker : fusion_markers.markers) {
                marker.color.r = 0.9;
                marker.color.g = 0.9;
                marker.color.b = 0.0;
                marker.color.a = 0.5;
            }

            if (first_ego_marker.action != 3) {
                fusion_pub.publish(fusion_markers);
            }
        }
    }

    void run() {
        ros::Rate rate(10);
        while (ros::ok()) {
            fusion_add();
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "subscribe_test");
    FusionNode fusion_node;
    fusion_node.run();
    return 0;
}
