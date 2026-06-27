#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher gps_pub;
//load parameters
std::string point_cloud_topic;
std::string map_frame;
std::string child_frame;
double fix_x, fix_y, fix_z;
double fix_roll, fix_pitch, fix_yaw;

void pointCloudCallback(const sensor_msgs::PointCloud2 msg){

  std_msgs::Header header = msg.header;
  ROS_INFO("Received PointCloud2 with timestamp %d sec, %d nsec", 
             header.stamp.sec, header.stamp.nsec);

  tf::Transform transform;
	static tf::TransformBroadcaster br;
  // set quaternion from roll pitch yaw
  tf::Quaternion tf_Q;
  tf_Q.setRPY(fix_roll, fix_pitch, fix_yaw);
  // set translation and rotation
  transform.setOrigin( tf::Vector3(fix_x,fix_y,fix_z) );
	// transform.setRotation( tf::Quaternion(0, 0, 0, 1));
  transform.setRotation(tf_Q);
	br.sendTransform(tf::StampedTransform(transform, msg.header.stamp,map_frame,child_frame));  

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pc_gps_publisher_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.param<std::string>("point_cloud_topic", point_cloud_topic, "/rslidar_points");
  private_nh.param<std::string>("map_frame", map_frame, "map");
  private_nh.param<std::string>("child_frame", child_frame, "rslidar");
  private_nh.param<double>("fix_x", fix_x, 0.0);
  private_nh.param<double>("fix_y", fix_y, 0.0);
  private_nh.param<double>("fix_z", fix_z, 0.0);
  private_nh.param<double>("fix_roll", fix_roll, 0.0);
  private_nh.param<double>("fix_pitch", fix_pitch, 0.0);
  private_nh.param<double>("fix_yaw", fix_yaw, 0.0);

  // convert pitch,roll,yaw from degree to radius
  fix_roll = fix_roll * M_PI / 180;
  fix_pitch = fix_pitch * M_PI / 180;
  fix_yaw = fix_yaw * M_PI / 180;

  ros::Subscriber sub = nh.subscribe(point_cloud_topic, 10, pointCloudCallback);

  ros::spin();
  return 0;
}

// #include <ros/ros.h>
// #include <tf/transform_broadcaster.h>
// #include <sensor_msgs/PointCloud2.h>

// void publish_tf(tf::Vector3 translation,tf::Quaternion rotation)
// {
//     // Create a TransformBroadcaster object
//     tf::TransformBroadcaster br;
//     while (ros::ok()) 
//     {
//         // Broadcast the transform
//         br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "imu"));
//         ros::Duration(0.1).sleep();
//     }
// }

// void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
//     // extract the header information from the PointCloud2 message
//     std_msgs::Header header = msg->header;
//     ROS_INFO("Received PointCloud2 with timestamp %d sec, %d nsec", 
//              header.stamp.sec, header.stamp.nsec);
//     // Define the translation and rotation data from the YAML file
//     tf::Vector3 translation(175.587300635, 181.411803641, -0.32539367676);
//     tf::Quaternion rotation(-0.00365339913944, -0.00208248063148, 
//                             0.968257754353, 0.24991846087);

//     // construct the transform
//     tf::Transform transform(rotation, translation);
//     //update the timestamp
//     transform.setData(rotation,translation, header.stamp);
//     //call the publishing function
//     publish_tf(transform);
// }

// int main(int argc, char** argv) 
// {
//     ros::init(argc, argv, "tf_publisher_example");
//     ros::NodeHandle nh;
//     // Subscribe to the pointcloud topic
//     ros::Subscriber sub = nh.subscribe("pointcloud", 1, pointcloudCallback);

//     ros::spin();
//     return 0;
// }
