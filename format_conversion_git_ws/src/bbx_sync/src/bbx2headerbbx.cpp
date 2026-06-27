// #include <ros/ros.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <visualization_header_msgs/MarkerArrayHeader.h>
// #include <string>

// class MarkerArrayConverter
// {
// public:
//     MarkerArrayConverter() : nh_("~")
//     {
//         // Get the input and output topic names from ROS parameters
//         nh_.param<std::string>("input_topic", input_topic_, "/tracking/bbx1");
//         nh_.param<std::string>("output_topic", output_topic_, "/tracking/header_bbx1");

//         // Subscribe to the input topic
//         sub_ = nh_.subscribe(input_topic_, 1, &MarkerArrayConverter::markerArrayCallback, this);

//         // Publish on the output topic
//         pub_ = nh_.advertise<visualization_header_msgs::MarkerArrayHeader>(output_topic_, 1);
//     }

//     void markerArrayCallback(const visualization_msgs::MarkerArray::ConstPtr& input_msg)
//     {
//         visualization_header_msgs::MarkerArrayHeader output_msg;

//         // Copy the header from the input message
//         output_msg.header = input_msg->header;

//         // Copy the markers from the input message
//         output_msg.markers = input_msg->markers;

//         // Publish the converted message
//         pub_.publish(output_msg);
//     }

// private:
//     ros::NodeHandle nh_;
//     ros::Subscriber sub_;
//     ros::Publisher pub_;
//     std::string input_topic_;
//     std::string output_topic_;
// };

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "marker_array_converter");
//     MarkerArrayConverter converter;
//     ros::spin();
//     return 0;
// }

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_header_msgs/MarkerArrayHeader.h>

class ConverterNode {
public:
    ConverterNode() {
        // Get parameters for input and output topic names
        ros::NodeHandle private_nh("~");
        private_nh.param<std::string>("input_topic", input_topic_, "/tracking/bbx1");
        private_nh.param<std::string>("output_topic", output_topic_, "/tracking/header_bbx1");

        // Publisher and Subscriber
        pub_ = nh_.advertise<visualization_header_msgs::MarkerArrayHeader>(output_topic_, 10);
        sub_ = nh_.subscribe(input_topic_, 10, &ConverterNode::callback, this);
    }

    void callback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
        visualization_header_msgs::MarkerArrayHeader output_msg;
        output_msg.header = msg->markers.front().header;  // Using the header of the first marker
        output_msg.markers = msg->markers;

        pub_.publish(output_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    std::string input_topic_;
    std::string output_topic_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "marker_converter_node");
    ConverterNode converter;
    ros::spin();
    return 0;
}

