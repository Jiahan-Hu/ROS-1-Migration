#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

ros::Time imu_time;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_time = msg->header.stamp;
}

void laserCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{ 
  // print imu_time and msg->header.stamp
  std::cout<<"imu_time = "<<imu_time<<std::endl;
  std::cout<<"msg->header.stamp = "<<msg->header.stamp<<std::endl;

  ros::Duration time_diff = msg->header.stamp - imu_time;
  ROS_INFO("Time difference: %f seconds", time_diff.toSec());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "time_diff_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_local("~");

  //get the parameters
    std::string imu_topic, lidar_topic;
    nh_local.param<std::string>("imu_topic", imu_topic, "/imu");
    nh_local.param<std::string>("lidar_topic", lidar_topic, "/rslidar_point");

  ros::Subscriber imu_sub = nh.subscribe(imu_topic, 100, imuCallback);
  ros::Subscriber laser_sub = nh.subscribe(lidar_topic, 10, laserCallback);

  ros::spin();

  return 0;
}
