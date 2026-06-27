#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

ros::Time stamp1;
ros::Time stamp2;
ros::Publisher time_diff_pub;

void callback1(const std_msgs::Header::ConstPtr& msg)
{
    stamp1 = msg->stamp;
}

void callback2(const std_msgs::Header::ConstPtr& msg)
{
    stamp2 = msg->stamp;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "time_difference_node");
    ros::NodeHandle nh("~");

    // Get topic names and time difference topic from ROS parameters
    std::string ego_topic1, topic2, time_difference_topic;
    nh.param<std::string>("ego_topic1", ego_topic1, "/rs_header");
    nh.param<std::string>("topic2", topic2, "/os_header");
    nh.param<std::string>("time_difference_topic", time_difference_topic, "time_difference");

    ros::Subscriber sub1 = nh.subscribe(ego_topic1, 1000, callback1);
    ros::Subscriber sub2 = nh.subscribe(topic2, 1000, callback2);

    time_diff_pub = nh.advertise<std_msgs::Float32>(time_difference_topic, 1);

    ros::Rate loop_rate(10);  // Adjust as necessary

    ROS_INFO("Hello time_difference_node");

    while (ros::ok())
    {
        ros::spinOnce();

        ros::Duration time_diff = stamp1 - stamp2;

        std_msgs::Float32 time_diff_msg;

        std_msgs::Float32 time_diff_msg_ms;

        // time_diff_msg.data = time_diff.toSec() * 1e9;  // Convert to nanoseconds

        time_diff_msg_ms.data = time_diff.toSec() * 1000;  // Convert to milliseconds

        time_diff_msg.data = time_diff.toNSec();  // Get time difference in nanoseconds

        time_diff_pub.publish(time_diff_msg);
        
        // Print the time difference in milliseconds
        ROS_INFO("Time Difference (ms): %.2f", time_diff_msg_ms.data);
        ROS_INFO("Time Difference (ns): %.0f", time_diff_msg.data);

        loop_rate.sleep();
    }

    return 0;
}

