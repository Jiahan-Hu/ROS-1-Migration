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
#include <tf_publisher/ros_utils.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_search.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <wgs84_utils/wgs84_utils.h>
#include <tf2/LinearMath/Transform.h>
#include <boost/optional.hpp>
// #include <lanelet2_extension/projection/local_frame_projector.h>


using std::atan2;
using std::cos;
using std::sin;

using std::cout;
using std::endl;
using std::vector;
using std::setprecision;

Eigen::Matrix4f T_V2R = Eigen::Matrix4f::Identity();//transform from velodyne to RT3000
Eigen::Matrix4f T_RT2M = Eigen::Matrix4f::Identity();//transfrom from RT3000 to map frame
double RT3000_latitude,RT3000_longitude,RT3000_heading,RT3000_velocity,RT3000_pitch,RT3000_roll,RT3000_altitude;
Eigen::Vector3d latest_pos(0,0,0);

double yaw_misalignment=0,pitch_misalignment=0,roll_misalignment=0;
double lever_x=0.15,lever_y=0,lever_z=1.2;//the translation between RT3000 and velodyne lidar, the position that velodyne is relative to RT3000, here is (0.15,0,1.2)

// double ref_easting = 282928.02,ref_northing = 4465603.2,ref_up = 297.295;//the origin of the TRC test 1
// double ref_easting = 282578.81 ,ref_northing = 4466190.3,ref_up = 299.35;//the origin of the TRC test 13

// double ref_easting = 282536.74 ,ref_northing = 4466293.4,ref_up = 299.582;//the origin of the TRC test 13 new map

double ref_easting = 301119.21 ,ref_northing = 4456058.9,ref_up = 268.007;//the origin of road test 1 second half

// RT3000_latitude = 40.312759 
// RT3000_longitude = -83.554581
// RT3000_altitude = 297.295
// xyz = 282928.02
// 4465603.2
//   297.295

Eigen::Vector3d xyz_ref(ref_easting,ref_northing,ref_up);
int I_Count=0;


// ros::Publisher transformed_velodyne_pub,transformed_velodyne_map_pub;
ros::Publisher local_map_pub;
ros::Publisher map_origin_m_pub,map_origin_ecef_pub;
ros::Publisher road_origin_pub;
ros::Publisher pcd_map_pub,transformed_velodyne_pub;

std::string Nav_topic_name,lidar_pcd_name;


ros::Publisher ego_vehicle_position;
ros::Publisher map_pose_pub;


double geo_longi,geo_lati,geo_alti;
bool if_georef_is_processed=false,alt_initialized=false,if_georef_is_converted2UTM=false;

std::string base_link_frame;
std::string map_frame;
std::string lidar_frame;

std::string pcd_map_path;

// std::shared_ptr<lanelet::projection::LocalFrameProjector> map_projector_;

void load_map()
{
	//load PCD file to check the intensity
	std::cout<<"XX pcd map starts to be loaded !"<<std::endl;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::io::loadPCDFile<pcl::PointXYZI> (pcd_map_path, *cloud_); //* load the file
	std::cout<<"XX pcd map is loaded !"<<std::endl;
	sensor_msgs::PointCloud2 loaded_map_;
    pcl::toROSMsg(*cloud_, loaded_map_);
    loaded_map_.header.stamp = ros::Time::now();
    loaded_map_.header.frame_id = "map";
	pcd_map_pub.publish(loaded_map_);

	std::cout<<"XX map cloud_->size ="<<cloud_->size()<<std::endl;
}

void parse_geoRef(std::string georef)
{
	geo_longi=0;
	geo_lati=0;
	cout<<"georef =  "<<georef<<endl;
	auto equal = georef.find("=");
	georef.erase(0, equal + 1);
	// cout<<"georef =  "<<georef<<endl;
	equal = georef.find("=");
	georef.erase(0, equal + 1);
	// cout<<"georef =  "<<georef<<endl;
	auto space = georef.find(" ");
	geo_lati=std::stod(georef.substr(0, space));
	cout<<"geo_lati =  "<<setprecision(8)<<geo_lati<<endl;
	georef.erase(0, space + 1);
	// cout<<"georef =  "<<georef<<endl;
	equal = georef.find("=");
	georef.erase(0, equal + 1);
	// cout<<"georef =  "<<georef<<endl;
	space = georef.find(" ");
	geo_longi=std::stod(georef.substr(0, space));
	cout<<"geo_longi =  "<<setprecision(8)<<geo_longi<<endl;

	if_georef_is_processed=true;

}

void geoReferenceCallback(const std_msgs::String& geo_ref)
{
	std::string geo_data=geo_ref.data.c_str();

	std::string ccc="+proj=tmerc +lat_0=40.317952 +lon_0=-83.558886 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";

	parse_geoRef(geo_data);

	std::cout<<"receivece geo data = "<<geo_data<<std::endl;

	// map_projector_ = std::make_shared<lanelet::projection::LocalFrameProjector>(geo_ref.data.c_str());  // Build projector from proj string

}



visualization_msgs::Marker PosToBox(Eigen::Vector3d pos, tf2::Quaternion orientation_)
{
	visualization_msgs::Marker box;
    box.header.frame_id = "map";
	// box.lifetime = ros::Duration(2);
	box.type = visualization_msgs::Marker::CUBE;
	box.action = visualization_msgs::Marker::ADD;
	box.id = 0;
	box.scale.x=5;
	box.scale.y=2;
	box.scale.z=1;
	box.pose.position.x = pos[0];
	box.pose.position.y = pos[1];
	box.pose.position.z = pos[2];
    box.pose.orientation.x = orientation_.getX();
    box.pose.orientation.y = orientation_.getY();
    box.pose.orientation.z = orientation_.getZ();
    box.pose.orientation.w = orientation_.getW();

    box.color.r = 0.0f;
    box.color.g = 1.0f;
    box.color.b = 0.0f;
    box.color.a = 1.0;

	return box;
}


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

/* Processing the message from RT3000 */
void imu_handler(sensor_msgs::Imu msg_input)
{

	Eigen::Quaternionf q_;
	q_.x()=msg_input.orientation.x;
	q_.y()=msg_input.orientation.y;
	q_.z()=msg_input.orientation.z;
	q_.w()=msg_input.orientation.w;

	Eigen::Vector3f euler_angle=Quaternion2EulerAngle(q_);

	if (msg_input.angular_velocity.z<=270)//transform the heading with respect to north direction to that with respect to east direction.
	{
		msg_input.angular_velocity.z=(90-msg_input.angular_velocity.z);
	}
	else
	{
		msg_input.angular_velocity.z=(450-msg_input.angular_velocity.z);
	}

	RT3000_heading=msg_input.angular_velocity.z*M_PI/180;
	RT3000_pitch=-msg_input.angular_velocity.y*M_PI/180;
	RT3000_roll=msg_input.angular_velocity.x*M_PI/180;

	// cout<<"RT3000_heading = "<<RT3000_heading*180/M_PI<<endl;
	// cout<<"RT3000_pitch = "<<RT3000_pitch*180/M_PI<<endl;
	// cout<<"RT3000_roll = "<<RT3000_roll*180/M_PI<<endl;
	




	// if (!Lv5OrLvx)
	// {
	// 	if (Applanix_heading>=0)//for applanix lv5, since the difference between the LVX and LV5 is 180 degrees
	// 	{
	// 		Applanix_heading=Applanix_heading-180;
	// 	}else
	// 	{
	// 		Applanix_heading=Applanix_heading+180;
	// 	}
	// }

	// Applanix_heading=Applanix_heading-90;//turn to velodyne coordinates since the x direction of velodyne is toward right hand side of the vehicle 
	// if (Applanix_heading<-270)
	// {
	// 	Applanix_heading=Applanix_heading+180;
	// }


}

/**
* @brief received gps data is added to #gps_queue
* @param gps_msg
*/
void gps_callback(const geographic_msgs::GeoPointStampedPtr& gps_msg) {
	gps_msg->header.stamp += ros::Duration(0);
}

void navsat_callback(const sensor_msgs::NavSatFixConstPtr& navsat_msg) {
	geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
	gps_msg->header = navsat_msg->header;
	gps_msg->position.latitude = navsat_msg->latitude;
	gps_msg->position.longitude = navsat_msg->longitude;
	gps_msg->position.altitude = navsat_msg->altitude;
	gps_callback(gps_msg);
}

void navsat_callback1(sensor_msgs::NavSatFix navsat_msg) {

	I_Count++;
	// cout<<"I_Count = "<<I_Count<<endl;


	RT3000_latitude=navsat_msg.latitude;
	RT3000_longitude=navsat_msg.longitude;
	RT3000_altitude=navsat_msg.altitude;

	


	if (if_georef_is_processed && !if_georef_is_converted2UTM)
	{
		/*transform the longitude, latitude, and height to utm*/
		if (!alt_initialized)
		{
			geo_alti=navsat_msg.altitude;
			alt_initialized=true;
		}		
		geographic_msgs::GeoPointStampedPtr gps_utm_ref(new geographic_msgs::GeoPointStamped());
		gps_utm_ref->position.longitude=geo_longi;
		gps_utm_ref->position.latitude=geo_lati;
		gps_utm_ref->position.altitude=geo_alti;
		geodesy::UTMPoint utm_ref;
		geodesy::fromMsg((*gps_utm_ref).position, utm_ref);
		xyz_ref(0)=utm_ref.easting;
		xyz_ref(1)=utm_ref.northing;
		xyz_ref(2)=utm_ref.altitude;
		std::cout<<"xyz_ref = " << setprecision(8)<< xyz_ref<<endl;
		if_georef_is_converted2UTM=true;
	}

	geographic_msgs::GeoPointStampedPtr gps_utm(new geographic_msgs::GeoPointStamped());
	gps_utm->position.longitude=RT3000_longitude;
	gps_utm->position.latitude=RT3000_latitude;
	gps_utm->position.altitude=RT3000_altitude;
	geodesy::UTMPoint utm;
	geodesy::fromMsg((*gps_utm).position, utm);
	Eigen::Vector3d xyz(utm.easting, utm.northing, utm.altitude);

	xyz -= xyz_ref;
	// std::cout<<"xyz_ref = " << setprecision(8)<< xyz_ref<<endl;
	// std::cout<<"xyz = " << setprecision(8)<< xyz<<endl;

	// cout<<"RT3000_heading = "<<RT3000_heading*180/M_PI<<endl;
	// cout<<"RT3000_pitch = "<<-RT3000_pitch*180/M_PI<<endl;
	// cout<<"RT3000_roll = "<<RT3000_roll*180/M_PI<<endl;

	// cout<<"RT3000_latitude = "<<setprecision(8)<<RT3000_latitude<<endl;
	// cout<<"RT3000_longitude = "<<setprecision(8)<<RT3000_longitude<<endl;
	// cout<<"RT3000_altitude = "<<RT3000_altitude<<endl;


	/*published UTM pose*/
	tf2::Quaternion R_n_b;
	R_n_b.setRPY(0, 0, RT3000_heading);
	geometry_msgs::PoseStamped msg;  // TODO until covariance is added drop it from the output
	msg.header = navsat_msg.header;
	msg.header.frame_id = map_frame;
	msg.pose.position.x = xyz[0];
	msg.pose.position.y = xyz[1];
	msg.pose.position.z = xyz[2];
	msg.pose.orientation.x=R_n_b.getX();
	msg.pose.orientation.y=R_n_b.getY();
	msg.pose.orientation.z=R_n_b.getZ();
	msg.pose.orientation.w=R_n_b.getW();
	map_pose_pub.publish(msg);
	// std::cout<<"RT3000_heading = "<<RT3000_heading*180/M_PI<<std::endl;
	/**/

	// clear all markers before
	visualization_msgs::Marker clear_marker;
	clear_marker.header = navsat_msg.header;
	clear_marker.id = 0;
	clear_marker.action = clear_marker.DELETEALL;
	clear_marker.lifetime = ros::Duration();
	ego_vehicle_position.publish(clear_marker);
	visualization_msgs::Marker marker=PosToBox(xyz,R_n_b);
	marker.header.stamp = navsat_msg.header.stamp;
	ego_vehicle_position.publish(marker);

	// cout<<"box has been published "<<endl;

	// geographic_msgs::GeoPointStampedPtr gps_utm_(new geographic_msgs::GeoPointStamped());
	// gps_utm_->position.longitude=-83.55941563234308;
	// gps_utm_->position.latitude=40.31886920336057;
	// gps_utm_->position.altitude=RT3000_altitude;
	// geodesy::UTMPoint utm_;
	// geodesy::fromMsg((*gps_utm_).position, utm_);
	// Eigen::Vector3d xyz__(utm_.easting, utm_.northing, utm_.altitude);
	// std::cout<<"xyz__ = " << setprecision(8)<< xyz__<<endl;

	if (if_georef_is_converted2UTM)
	{
		geometry_msgs::Vector3 map_origin_m,map_origin_ecef;
		geometry_msgs::Vector3 road_origin;
		map_origin_m.x=xyz_ref(0);
		map_origin_m.y=xyz_ref(1);
		map_origin_m.z=xyz_ref(2);

		map_origin_ecef.x=geo_longi;
		map_origin_ecef.y=geo_lati;
		map_origin_ecef.z=geo_alti;

		road_origin.x=0;
		road_origin.y=0;
		road_origin.z=0;

		if (I_Count%10<0.5)
		{
			map_origin_m_pub.publish(map_origin_m);
			map_origin_ecef_pub.publish(map_origin_ecef);
			road_origin_pub.publish(road_origin);

			// load_map();
		}
	}


	/*sending the transformation from velodyne to map frame for visualization*/
	Eigen::Vector3f Euler_angle_RT3000(RT3000_heading,0,0);
	Eigen::Quaternionf q=EulerAngle2Quaternion(Euler_angle_RT3000);


	// tf::StampedTransform::StampedTransform(tf::Transform & input,ros::Time &timestamp,const std::string & frame_id,const std::string &child_frame_id) 	

	//one child frame can only have only one parent frame!!!!

	/*sending the transformation from velodyne to baselink frame for visualization*/
	
	tf::Transform transform_1;
    transform_1.setOrigin( tf::Vector3(lever_x, lever_y, lever_z) );
    tf::Quaternion R_n_h;
    R_n_h.setRPY(0,0,yaw_misalignment);
	transform_1.setRotation( R_n_h);
	tf::Transform transform_1_=transform_1.inverse();

	static tf::TransformBroadcaster br_1;
	br_1.sendTransform(tf::StampedTransform(transform_1, navsat_msg.header.stamp,base_link_frame,lidar_frame));
	/*end of sending transformation*/

	/*sending the transformation from map to base_link frame for visualization*/
	static tf::TransformBroadcaster br_2;
	tf::Transform transform_2;
    transform_2.setOrigin( tf::Vector3(xyz(0), xyz(1), xyz(2)) );
	transform_2.setRotation( tf::Quaternion(R_n_b.getX(), R_n_b.getY(), R_n_b.getZ(), R_n_b.getW()));
	br_2.sendTransform(tf::StampedTransform(transform_2, navsat_msg.header.stamp, map_frame, base_link_frame));
	/*end of sending transformation*/

	std::string novatel_gnss="novatel_gnss";
	std::string novatel_imu="novatel_imu";

	/*sending the transformation from novatel_gnss to base_link frame for visualization*/
	static tf::TransformBroadcaster br_3;
	tf::Transform transform_3;
    transform_3.setOrigin( tf::Vector3(0, 0, 0) );
	transform_3.setRotation( tf::Quaternion(0, 0, 0, 1));
	br_3.sendTransform(tf::StampedTransform(transform_3, navsat_msg.header.stamp,base_link_frame,novatel_gnss ));
	/*end of sending transformation*/

	/*sending the transformation from novatel_gnss to novatel_imu frame for visualization*/
	static tf::TransformBroadcaster br_4;
	tf::Transform transform_4;
    transform_4.setOrigin( tf::Vector3(0, 0, 0) );
	transform_4.setRotation( tf::Quaternion(0, 0, 0, 1));
	br_4.sendTransform(tf::StampedTransform(transform_4, navsat_msg.header.stamp, novatel_gnss, novatel_imu));
	/*end of sending transformation*/

	// std::cout<<"map_frame = "<<map_frame<<std::endl;
	// std::cout<<"base_link_frame = "<<base_link_frame<<std::endl;
	// std::cout<<"lidar_frame = "<<lidar_frame<<std::endl;


}

void vel_callback1(geometry_msgs::TwistWithCovarianceStamped vel_topic)
{
	float ve,vn,vu;
	ve=vel_topic.twist.twist.linear.x;
	vn=vel_topic.twist.twist.linear.y;
	vu=vel_topic.twist.twist.linear.z;
	RT3000_velocity=sqrt(pow(ve,2)+pow(vn,2));
}


/*processing the pointcloud from velodyne*/
void velydyne_handler(sensor_msgs::PointCloud2 laserCloudMsg)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::fromROSMsg(laserCloudMsg, *cloud);
	pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_RT3000 (new pcl::PointCloud<pcl::PointXYZI> ());
	pcl::transformPointCloud (*cloud, *transformed_cloud_RT3000, T_V2R);
	/*generating the rotation matrix*/
	Eigen::Vector3f euler_angle_RT3000(RT3000_heading,-RT3000_pitch,RT3000_roll);
	Eigen::Matrix3f Rt_RT3000=Creat_Rt(euler_angle_RT3000);
	/**/

	/*transform the longitude, latitude, and height to utm*/
	geographic_msgs::GeoPointStampedPtr gps_utm(new geographic_msgs::GeoPointStamped());
	gps_utm->position.longitude=RT3000_longitude;
	gps_utm->position.latitude=RT3000_latitude;
	gps_utm->position.altitude=RT3000_altitude;
	geodesy::UTMPoint utm;
	geodesy::fromMsg((*gps_utm).position, utm);
	Eigen::Vector3d xyz(utm.easting, utm.northing, utm.altitude);
	// std::cout<<"xyz = " << setprecision(12)<< xyz<<endl;
	xyz -= xyz_ref;
	/**/

	/*generating the transformation matrix from RT3000 to the map frame*/
	T_RT2M.block<3,3>(0,0)=Rt_RT3000;
	T_RT2M (0,3) = xyz(0);
	T_RT2M (1,3) = xyz(1);
	T_RT2M (2,3) = xyz(2);

	pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_map (new pcl::PointCloud<pcl::PointXYZI> ());
	pcl::transformPointCloud (*transformed_cloud_RT3000, *transformed_cloud_map, T_RT2M);

	/*transform the pointcloud from velodyne in velodyne frame to RT3000 frame*/
	sensor_msgs::PointCloud2 pcd_inter;
    pcl::toROSMsg(*transformed_cloud_map, pcd_inter);
    pcd_inter.header.stamp = laserCloudMsg.header.stamp;
    pcd_inter.header.frame_id = "map";
	transformed_velodyne_pub.publish(pcd_inter);
	/**/


}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    yaw_misalignment = private_nh.param<double>("yaw_misalignment", 2);
    pitch_misalignment = private_nh.param<double>("pitch_misalignment", 2);
    roll_misalignment = private_nh.param<double>("roll_misalignment", 2);
    lever_x = private_nh.param<double>("lever_x", -0.3);
    lever_y = private_nh.param<double>("lever_y", 1.36);
    lever_z = private_nh.param<double>("lever_z", 1.31);

    std::cout<<"yaw_misalignment = "<<yaw_misalignment  <<std::endl;
    std::cout<<"lever_x = "<<lever_x  <<std::endl;
    std::cout<<"lever_y = "<<lever_y  <<std::endl;

    yaw_misalignment=yaw_misalignment*M_PI/180;
    pitch_misalignment=pitch_misalignment*M_PI/180;
    roll_misalignment=roll_misalignment*M_PI/180;
    Eigen::Vector3f euler_angle_misalignment(yaw_misalignment,pitch_misalignment,roll_misalignment);
    Eigen::Matrix3f T_V2R_=Creat_Rt(euler_angle_misalignment);
    T_V2R.block<3,3>(0,0)=T_V2R_;
	T_V2R(0,3)=lever_x;
	T_V2R(1,3)=lever_y;
	T_V2R(2,3)=lever_z;

    // Load parameters
    private_nh.param<std::string>("base_link_frame_id", base_link_frame, "base_link");
    private_nh.param<std::string>("map_frame_id", map_frame, "map");
    private_nh.param<std::string>("lidar_frame_id", lidar_frame, "rslidar");

    private_nh.param<std::string>("pcd_map_path", pcd_map_path, "/home/carma/catkin_ws_carma_DATA_proj/carma_load/pcd_maps/TRC_test13_2.pcd");
    
	

	ros::Subscriber Imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 100, imu_handler);//rt3000

	ros::Subscriber navsat_sub = nh.subscribe("/gps/fix", 100,navsat_callback1); // this is where publish tf
	ros::Subscriber vel_sub = nh.subscribe("/gps/vel", 100,vel_callback1);
	ros::Subscriber geo_sub = nh.subscribe("georeference", 1, geoReferenceCallback);

	// ros::Subscriber velodyne_sub = nh.subscribe<sensor_msgs::PointCloud2>("/points_raw", 100, velydyne_handler);
	ros::Subscriber velodyne_sub = nh.subscribe<sensor_msgs::PointCloud2>("/syn_pc_rs", 100, velydyne_handler);

	// transformed_velodyne_pub=nh.advertise<sensor_msgs::PointCloud2>("/transformed_velodyne_pub", 100);//xx
	transformed_velodyne_pub=nh.advertise<sensor_msgs::PointCloud2>("/transformed_rs_pub", 100);//xx

	pcd_map_pub=nh.advertise<sensor_msgs::PointCloud2>("/pcd_map_pub", 100);//xx
	map_origin_m_pub=nh.advertise<geometry_msgs::Vector3>("/map_origin_m_pub", 100);//xx
	map_origin_ecef_pub=nh.advertise<geometry_msgs::Vector3>("/map_origin_ecef_pub", 100);//xx
	road_origin_pub=nh.advertise<geometry_msgs::Vector3>("/road_origin_pub", 100);//xx
	ego_vehicle_position = nh.advertise<visualization_msgs::Marker>("ego_vehicle_pos", 1);

    // pose publisher
    map_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("gnss_pose", 10, true);

    // load_map();


    ros::spin();

    return 0;
}
