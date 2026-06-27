#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class LidarTransformer {
public:
  LidarTransformer() {
    // Initialize ROS node
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Load parameters
    private_nh.param<std::string>("velodyne_1_side_topic", velodyne_1_side_topic_, "/hardware_interface/velodyne_1_side/velodyne_points");
    private_nh.param<std::string>("velodyne_1_front_topic", velodyne_1_front_topic_, "/hardware_interface/velodyne_1_front/velodyne_points");
    private_nh.param("x", x_, 0.0);
    private_nh.param("y", y_, 0.0);
    private_nh.param("z", z_, 0.0);
    private_nh.param("heading", heading_degrees_, 0.0);
    private_nh.param("pitch", pitch_degrees_, 0.0);
    private_nh.param("roll", roll_degrees_, 0.0);

    // Convert angles to radians
    const double heading = heading_degrees_ * M_PI / 180.0;
    const double pitch = pitch_degrees_ * M_PI / 180.0;
    const double roll = roll_degrees_ * M_PI / 180.0;    

    // Initialize subscribers and publishers
    velodyne_1_side_sub_ = nh.subscribe(velodyne_1_side_topic_, 1, &LidarTransformer::velodyne_1_side_callback, this);
    velodyne_1_front_sub_ = nh.subscribe(velodyne_1_front_topic_, 1, &LidarTransformer::velodyne_1_front_callback, this);
    combined_pub_ = nh.advertise<sensor_msgs::PointCloud2>("combined_velodyne_points", 1);

    // (For radian)Initialize transformation
    // transform_.setOrigin(tf::Vector3(x_, y_, z_));
    // transform_.setRotation(tf::createQuaternionFromRPY(roll_, pitch_, heading_));

    // Initialize transformation
    tf::Quaternion q;
    q.setRPY(roll, pitch, heading);
    tf::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);
    transform_.setOrigin(tf::Vector3(x_, y_, z_));
    transform_.setRotation(q);
  }

private:
  void velodyne_1_front_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*msg, *cloud_in);
    pcl_ros::transformPointCloud(*cloud_in, *cloud_transformed, transform_);

    sensor_msgs::PointCloud2 cloud_out;
    pcl::toROSMsg(*cloud_transformed, cloud_out);
    cloud_out.header.frame_id = "velodyne";
    cloud_out.header.stamp = msg->header.stamp;

    combined_pub_.publish(cloud_out);
  }

  void velodyne_1_side_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

    sensor_msgs::PointCloud2 msg_copy = *msg;
    // Set the frame ID of the header to "velodyne"
    msg_copy.header.frame_id = "velodyne";
    combined_pub_.publish(msg_copy);
  }

  std::string velodyne_1_side_topic_;
  std::string velodyne_1_front_topic_;
  double x_, y_, z_, heading_degrees_, pitch_degrees_, roll_degrees_;
  ros::Subscriber velodyne_1_side_sub_;
  ros::Subscriber velodyne_1_front_sub_;
  ros::Publisher combined_pub_;
  tf::Transform transform_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_transformer_node");
  LidarTransformer lidar_transformer;

  ros::spin();

  return 0;
}
