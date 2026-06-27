#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <can_msgs/Frame.h>

ros::Time imu_time;
ros::Time can_time;
float gnss_week;
float gnss_seconds;
float ins_status;
float postype;

void canCallback(const can_msgs::Frame::ConstPtr& ros_msg)
{ // print can_msgs data
  can_time = ros_msg->header.stamp;
	if (ros_msg->id == 0x630)
	{
		unsigned short inter_variable=ros_msg->data[1]*256+ros_msg->data[0];
		gnss_week=inter_variable;
		inter_variable=ros_msg->data[3]*256+ros_msg->data[2];
		gnss_seconds=static_cast<float>(inter_variable)*0.001;
		inter_variable=ros_msg->data[6];
		ins_status=inter_variable;
		inter_variable=ros_msg->data[7];
		postype=inter_variable;
    //print gnss_week gnss_seconds ins_status postype
    std::cout<<"gnss_week = "<<gnss_week<<std::endl;
    std::cout<<"gnss_seconds = "<<gnss_seconds<<std::endl;
  }
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_time = msg->header.stamp;
}

void laserCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{ 
  // print can_time imu_time and msg->header.stamp
  std::cout<<"can_time = "<<can_time<<std::endl;
  std::cout<<"imu_time = "<<imu_time<<std::endl;
  std::cout<<"msg->header.stamp = "<<msg->header.stamp<<std::endl;

  // calculate the time difference
  ros::Duration can_lidar_time_diff = msg->header.stamp - can_time;
  ros::Duration imu_lidar_time_diff = msg->header.stamp - imu_time;
  // ROS_INFO("Time difference: %f seconds", can_lidar_time_diff.toSec());
  // ROS_INFO("Time difference: %f seconds", imu_lidar_time_diff.toSec());
  ROS_INFO("--------------------------------------------");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "time_diff_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_local("~");

  //get the parameters
    std::string can_topic,imu_topic, lidar_topic;
    nh_local.param<std::string>("can_topic", can_topic, "/can_tx");
    nh_local.param<std::string>("imu_topic", imu_topic, "/imu");
    nh_local.param<std::string>("lidar_topic", lidar_topic, "/rslidar_point");
  
  ros::Subscriber can_sub = nh.subscribe(can_topic, 1000, canCallback);
  ros::Subscriber imu_sub = nh.subscribe(imu_topic, 100, imuCallback);
  ros::Subscriber laser_sub = nh.subscribe(lidar_topic, 10, laserCallback);

  ros::spin();

  return 0;
}
