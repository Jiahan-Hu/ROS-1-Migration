#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub;
int time_threshold;

void callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  sensor_msgs::PointCloud2 new_msg = *msg;
  ROS_INFO("original pointcloud time: %d.%d", new_msg.header.stamp.sec, new_msg.header.stamp.nsec);
  if (new_msg.header.stamp.nsec < time_threshold) {
    new_msg.header.stamp.sec -= 1;
    new_msg.header.stamp.nsec += 1000000000 - time_threshold;  // add 0.6 seconds
  }else{
    new_msg.header.stamp.nsec -= time_threshold;  // subtract 0.4 seconds
  }

  pub.publish(new_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pointcloud_modifier");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string input_topic, output_topic;
    nh_private.param<std::string>("input_topic", input_topic, "/input_pointcloud");
    nh_private.param<std::string>("output_topic", output_topic, "/output_pointcloud");
    nh_private.param<int>("time_threshold", time_threshold, 400000000);

  ROS_INFO("pointcloud_TIME_modifier node started");
  pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 1);
  ros::Subscriber sub = nh.subscribe(input_topic, 1, callback);

  ros::spin();
  return 0;
}
