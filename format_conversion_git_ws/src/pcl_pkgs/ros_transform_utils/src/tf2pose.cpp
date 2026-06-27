#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_to_pose");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // get paraemters 
    std::string parent_frame, child_frame, output_topic;
    pnh.param("parent_frame", parent_frame, std::string("map"));
    pnh.param("child_frame", child_frame, std::string("/veh2_lidar"));
    pnh.param("output_topic", output_topic, std::string("/infra_pose"));

    // create tf listener and publisher 
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>(output_topic, 10);

    ros::Rate rate(10.0); // 10 Hz
    while(nh.ok()){
        geometry_msgs::TransformStamped transformStamped;
        try{
            transformStamped = tfBuffer.lookupTransform(parent_frame, child_frame, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        // convert into PoseStamped and publish
        geometry_msgs::PoseStamped poseStamped;
        // poseStamped.header.stamp = ros::Time::now();
        poseStamped.header.stamp = transformStamped.header.stamp;
        poseStamped.header.frame_id = parent_frame;
        poseStamped.pose.position.x = transformStamped.transform.translation.x;
        poseStamped.pose.position.y = transformStamped.transform.translation.y;
        poseStamped.pose.position.z = transformStamped.transform.translation.z;
        poseStamped.pose.orientation = transformStamped.transform.rotation;

        pose_pub.publish(poseStamped);

        rate.sleep();
    }
    return 0;
};
