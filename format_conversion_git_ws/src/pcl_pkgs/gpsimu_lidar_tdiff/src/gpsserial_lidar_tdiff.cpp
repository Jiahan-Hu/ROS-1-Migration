#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/PointCloud2.h>

ros::Time gps_time,lidar_time;

void gpsCallback(const sensor_msgs::TimeReference::ConstPtr& msg)
{ // gps frequency depends on the /gps_time topic publish node
  gps_time = msg->time_ref;
  // print imu_time and msg->header.stamp
  
  std::cout<<"gps_time = "<<gps_time<<std::endl;
  std::cout<<"lidar time stamp = "<<lidar_time<<std::endl;

  ros::Duration time_diff = lidar_time - gps_time;
  ROS_INFO("Time difference: %f seconds", time_diff.toSec());
}

void laserCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{   // 10 hz
    lidar_time = msg -> header.stamp;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "time_diff_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_local("~");

  //get the parameters
    std::string gps_time_topic, lidar_topic;
    nh_local.param<std::string>("gps_time_topic", gps_time_topic, "/gps_time");
    nh_local.param<std::string>("lidar_topic", lidar_topic, "/rslidar_point");

  ros::Subscriber imu_sub = nh.subscribe(gps_time_topic, 100, gpsCallback);
  ros::Subscriber laser_sub = nh.subscribe(lidar_topic, 10, laserCallback);

  ros::spin();

  return 0;
}
