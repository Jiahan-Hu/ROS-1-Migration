/*This rosnode is for generating the point cloud map given the pose (position&attitude) from RT3000. 
With this accurate pose, the current lidar point cloud will be transformed into the map coordinates. 
Then, the point cloud is stacked to generate the map.
If you have any questions, please feel free to contact Xin Xia (xiaxin2000@gmail.com)*/
#include <ctime>
#include <mutex>
#include <atomic>
#include <memory>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <unordered_map>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_search.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <gps_common/GPSFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <boost/shared_ptr.hpp>

#include "common/msgs/autosense_msgs/PointCloud2Array.h"
#include "common/msgs/autosense_msgs/TrackingFixedTrajectoryArray.h"
#include "common/msgs/autosense_msgs/TrackingObjectArray.h"
#include "common/msgs/autosense_msgs/TrackingObjectArray_local_frame.h"



using namespace std;
using std::atan2;
using std::cos;
using std::sin;

using std::cout;
using std::endl;
using std::vector;
using std::setprecision;


double deg_rad=M_PI/180,rad_deg=180/M_PI;

ros::Publisher to_tracking_objects_pub,to_autoware_tracking_pub;


/*Transform quaternion to euler angle*/
Eigen::Vector3f Quaternion2EulerAngle(Eigen::Quaternionf Quaternion_ToBeTransformed) //XX 四元数转化成欧拉角
{

	Eigen::Vector3f EulerAngle_FromQuaternion;
	Eigen::Vector4f q_coeffs = Quaternion_ToBeTransformed.coeffs();
	float yaw_from_Quaternion,pitch_from_Quaternion,roll_from_Quaternion,qw,qx,qy,qz;

	qw=q_coeffs[3];
	qx=q_coeffs[0];
	qy=q_coeffs[1];
	qz=q_coeffs[2];

	yaw_from_Quaternion=std::atan2(2*(qx*qy+qw*qz),(1-2*(qy*qy+qz*qz)));
	pitch_from_Quaternion=std::asin(2*(qw*qy-qz*qx));
	roll_from_Quaternion=std::atan2(2*(qw*qx+qy*qz),(1-2*(qx*qx+qy*qy)));

	EulerAngle_FromQuaternion[0]=yaw_from_Quaternion;
	EulerAngle_FromQuaternion[1]=pitch_from_Quaternion;
	EulerAngle_FromQuaternion[2]=roll_from_Quaternion;

	return EulerAngle_FromQuaternion;

}


/*Transform euler angle to quaternion*/
Eigen::Quaternionf EulerAngle2Quaternion(Eigen::Vector3f EulerAngle_ToBeTransformed)
{

	Eigen::Quaternionf Quaternion_FromEulerAngle;

	float yaw=EulerAngle_ToBeTransformed[0], pitch=EulerAngle_ToBeTransformed[1], roll=EulerAngle_ToBeTransformed[2];

	float cy = std::cos(yaw*0.5), sy = std::sin(yaw*0.5);
	float cp = std::cos(pitch*0.5), sp = std::sin(pitch*0.5);
	float cr = std::cos(roll*0.5), sr = std::sin(roll*0.5);

	Eigen::Vector3f v;
	v[0] = sr * cp * cy - cr * sp * sy;
	v[1] = cr * sp * cy + sr * cp * sy;
	v[2] = cr * cp * sy - sr * sp * cy;

	Quaternion_FromEulerAngle.w()=cr * cp * cy + sr * sp * sy;
	Quaternion_FromEulerAngle.vec() = v; 

	return Quaternion_FromEulerAngle;

}

/*Rotation matrix generation function*/
Eigen::Matrix3f Creat_Rt(Eigen::Vector3f euler_angle) 
{

	Eigen::Matrix3f Rz= Eigen::Matrix3f::Identity();
	Eigen::Matrix3f Ry= Eigen::Matrix3f::Identity();
	Eigen::Matrix3f Rx= Eigen::Matrix3f::Identity();

	float alpha = euler_angle(0), beta = euler_angle(1), gamma = euler_angle(2);
	Rz(0,0)=cos(alpha);
	Rz(0,1)=-sin(alpha);
	Rz(1,0)=sin(alpha);
	Rz(1,1)=cos(alpha);

	Ry(0,0)=cos(beta);
	Ry(0,2)=sin(beta);
	Ry(2,0)=-sin(beta);
	Ry(2,2)=cos(beta);

	Rx(1,1)=cos(gamma);
	Rx(1,2)=-sin(gamma);
	Rx(2,1)=sin(gamma);
	Rx(2,2)=cos(gamma);

	Eigen::Matrix3f Rt=Rz*Ry*Rx;
	return Rt;
}

void autoware_lidar_detector_sub_callback(autoware_msgs::DetectedObjectArray input)
{
	autosense_msgs::PointCloud2Array msg;
	msg.header=input.header;

	for (int i = 0; i < input.objects.size(); ++i)
	{
		sensor_msgs::PointCloud2 cloud=input.objects[i].pointcloud;//obj_array.objects[i].pointcloud is sensor_msgs/PointCloud2
		msg.clouds.push_back(cloud);
	}

	if (!msg.clouds.empty())
	{
		to_tracking_objects_pub.publish(msg);
	}

}

void tracking_lib_sub_callback1(autosense_msgs::TrackingObjectArray_local_frame input)
{
	autoware_msgs::DetectedObject autoware_obj;
	autoware_msgs::DetectedObjectArray autoware_obj_array;

	autoware_obj_array.header=input.header;
	// autoware_obj_array.header.frame_id="map";

	for (int i = 0; i < input.ids.size(); ++i)
	{

		autoware_obj.header=input.header;
		autoware_obj.id=input.ids[i];

		autoware_obj.valid=true;

		autoware_obj.label="car";

		autoware_obj.pose_reliable=true;
		autoware_obj.velocity_reliable=true;
		autoware_obj.acceleration_reliable=true;	

		autoware_obj.pose.position.x=input.positions[i].x;
		autoware_obj.pose.position.y=input.positions[i].y;
		autoware_obj.pose.position.z=input.positions[i].z;

		double heading=input.headings[i];

		Eigen::Vector3f euler_angle(heading,0,0);

		Eigen::Quaternionf q=EulerAngle2Quaternion(euler_angle);

		autoware_obj.pose.orientation.x=q.x();
		autoware_obj.pose.orientation.y=q.y();
		autoware_obj.pose.orientation.z=q.z();
		autoware_obj.pose.orientation.w=q.w();

		// autoware_obj.pose.orientation.x=0;
		// autoware_obj.pose.orientation.y=0;
		// autoware_obj.pose.orientation.z=0;
		// autoware_obj.pose.orientation.w=1;

		autoware_obj.dimensions.x=input.sizes[i].x;
		autoware_obj.dimensions.y=input.sizes[i].y;
		autoware_obj.dimensions.z=input.sizes[i].z;

		autoware_obj.velocity.linear.x=input.velocities[i].x;
		autoware_obj.velocity.linear.y=input.velocities[i].y;
		autoware_obj.velocity.linear.z=input.velocities[i].z;

		autoware_obj_array.objects.push_back(autoware_obj);
	}

	if (!autoware_obj_array.objects.empty())
	{
		to_autoware_tracking_pub.publish(autoware_obj_array);
	}

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "api_tracking_lib_autoware");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

	std::string detection_object_topic;
	std::string segment_cloud_topic;
	std::string tracking_topic;
	std::string tracking_objects_topic;

	private_nh.param<std::string>("detection_object_topic", detection_object_topic, "/detection/lidar_detector/objects");
	private_nh.param<std::string>("segment_cloud_topic", segment_cloud_topic, "/segment/clouds_segmented");
	private_nh.param<std::string>("tracking_topic", tracking_topic, "/tracking/tracking_objects_local");
	private_nh.param<std::string>("tracking_objects_topic", tracking_objects_topic, "/detection/object_tracker/objects");

	ros::Subscriber autoware_lidar_detector_sub = nh.subscribe(detection_object_topic, 100,autoware_lidar_detector_sub_callback);//subscribe the detected objects from apollo cnn method
	to_tracking_objects_pub=nh.advertise<autosense_msgs::PointCloud2Array>(segment_cloud_topic, 10);//convert the objects from apollo method to the objects accepted by tracking_lib


	ros::Subscriber tracking_lib_sub = nh.subscribe(tracking_topic, 100,tracking_lib_sub_callback1);	
	to_autoware_tracking_pub=nh.advertise<autoware_msgs::DetectedObjectArray>(tracking_objects_topic, 10);



    ros::spin();

    return 0;
}
