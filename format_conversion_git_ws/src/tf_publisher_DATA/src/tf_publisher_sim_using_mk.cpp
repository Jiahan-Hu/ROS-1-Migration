#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

ros::Publisher gps_pub;
//load parameters
std::string markerarray_topic;
std::string map_frame;
std::string child_frame;
double fix_x, fix_y, fix_z;
double fix_roll, fix_pitch, fix_yaw;

void markerArrayCallback(const visualization_msgs::MarkerArray::ConstPtr& msg){

  if(msg->markers.empty()){
    ROS_WARN("Received empty MarkerArray!");
    return;
  }
  const visualization_msgs::Marker& first_marker = msg->markers[0];
  std_msgs::Header header = first_marker.header;
  ROS_INFO("Received markerArray with timestamp %d sec, %d nsec", 
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
  ros::init(argc, argv, "pc_tf_publisher_using_mkarray_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.param<std::string>("makerarray_topic", markerarray_topic, "/tracking/transformed_objects_box_score");
  private_nh.param<std::string>("map_frame", map_frame, "map");
  private_nh.param<std::string>("child_frame", child_frame, "map");
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

  ros::Subscriber sub = nh.subscribe(markerarray_topic, 10, markerArrayCallback);

  ros::spin();
  return 0;
}