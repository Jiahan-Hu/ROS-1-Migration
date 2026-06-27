#include <thread>
#include <mutex>
#include <chrono>
#include <algorithm>
#include <ros/ros.h>
#include <kvaser_interface/kvaser_interface.h>
#include <can_msgs/Frame.h>
#include <ucla_can_parser/ucla_can_parser.h>
#include <novatel_oem7_msgs/INSPVAX.h>
#include <sensor_msgs/Imu.h>

#include <iostream>
#include <math.h>

using namespace AS::CAN;
using namespace std;

ros::Publisher can_rx_pub,inspvax_pub,imu_pub;
ros::Subscriber can_tx_sub;

typedef unsigned char uint8_t;

void test();

float acc_x;
float acc_y;
float acc_z;//g
float ang_x;
float ang_y;
float ang_z;

float heading;
float roll;
float pitch;
float heading_sigma;
float roll_sigma;
float pitch_sigma;

float vel_e;
float vel_n;
float vel_u;
float vel_e_sigma;
float vel_n_sigma;
float vel_u_sigma;

double latitudedouble ;
double longitudedouble;
double latitude;
double longitude;
double height;

float latitude_sigma ;
float longitude_sigma;
float height_sigma;

unsigned short int gnss_week;
unsigned int gnss_week_milliseconds;
float ins_status;
float postype;
float checksum;

void inspvax_publisher()
{
	novatel_oem7_msgs::INSPVAX msg;
  msg.header.stamp=ros::Time::now();
  msg.header.frame_id="gps";
  // msg.nov_header=inspva_msg->nov_header;
  msg.nov_header.gps_week_number=gnss_week;
  msg.nov_header.gps_week_milliseconds=gnss_week_milliseconds;
  msg.latitude=latitude;
  msg.longitude=longitude;
  msg.height=height;
  msg.north_velocity=vel_n;
  msg.east_velocity=vel_e;
  msg.up_velocity=vel_u;
  msg.roll=roll;
  msg.pitch=pitch;
  msg.azimuth=heading;
  msg.ins_status.status=postype;

  msg.latitude_stdev=latitude_sigma;
  msg.longitude_stdev=longitude_sigma;
  msg.height_stdev=height_sigma;
  msg.north_velocity_stdev=vel_e_sigma;
  msg.east_velocity_stdev=vel_n_sigma;
  msg.up_velocity_stdev=vel_u_sigma;
  msg.roll_stdev=roll_sigma;
  msg.pitch_stdev=pitch_sigma;
  msg.azimuth_stdev=heading_sigma;
  inspvax_pub.publish(msg);

}

void imu_publisher()
{
	sensor_msgs::Imu msg;
  	msg.header.stamp=ros::Time::now();
 	msg.header.frame_id="gps";
	msg.angular_velocity.x=ang_x;
	msg.angular_velocity.y=ang_y;
	msg.angular_velocity.z=ang_z;
	msg.linear_acceleration.x=acc_x;
	msg.linear_acceleration.y=acc_y;
	msg.linear_acceleration.z=acc_z;
	imu_pub.publish(msg);
}



void can_tx_sub_callback(const can_msgs::Frame::ConstPtr& ros_msg)
{
	if (ros_msg->id == 0x630)
	{
		unsigned int inter_variable=ros_msg->data[1]*256+ros_msg->data[0];
		gnss_week=inter_variable;
		inter_variable=ros_msg->data[5]*256*256*256+ros_msg->data[4]*256*256+ros_msg->data[3]*256+ros_msg->data[2];
		gnss_week_milliseconds=inter_variable;
		inter_variable=ros_msg->data[6];
		ins_status=inter_variable;
		inter_variable=ros_msg->data[7];
		postype=inter_variable;
	}else if (ros_msg->id == 0x631)
	{
		signed long int inter_variable=ros_msg->data[3]*256*256*256+ros_msg->data[2]*256*256+ros_msg->data[1]*256+ros_msg->data[0];
		// cout<<"latitude : "<<setprecision(12)<<inter_variable<<endl;
		latitude=static_cast<double>(inter_variable)*1e-7;
		inter_variable=ros_msg->data[7]*256*256*256+ros_msg->data[6]*256*256+ros_msg->data[5]*256+ros_msg->data[4];
		// cout<<"longitude : "<<setprecision(12)<<inter_variable<<endl;
		longitude=static_cast<double>(inter_variable)*1e-7;
	}else if (ros_msg->id == 0x632)
	{
		signed long int inter_variable=ros_msg->data[3]*256*256*256+ros_msg->data[2]*256*256+ros_msg->data[1]*256+ros_msg->data[0];
		height=static_cast<double>(inter_variable)*0.001;
		unsigned short inter_variable_1=ros_msg->data[5]*256+ros_msg->data[4];
		height_sigma=inter_variable_1*0.001;
		inter_variable_1=ros_msg->data[7]*256+ros_msg->data[6];
		roll_sigma=inter_variable_1*0.01;
	}else if (ros_msg->id == 0x633)
	{
		signed short inter_variable=ros_msg->data[1]*256+ros_msg->data[0];
		vel_n=static_cast<float>(inter_variable)*0.01;
		inter_variable=ros_msg->data[3]*256+ros_msg->data[2];
		vel_e=static_cast<float>(inter_variable)*0.01;
		inter_variable=ros_msg->data[5]*256+ros_msg->data[4];
		vel_u=static_cast<float>(inter_variable)*0.01;
		unsigned short inter_variable_1=ros_msg->data[7]*256+ros_msg->data[6];
		vel_u_sigma=static_cast<float>(inter_variable_1)*0.01;
	}else if (ros_msg->id == 0x634)
	{
		signed short inter_variable=ros_msg->data[1]*256+ros_msg->data[0];
		roll=static_cast<float>(inter_variable)*0.01;
		inter_variable=ros_msg->data[3]*256+ros_msg->data[2];
		pitch=static_cast<float>(inter_variable)*0.01;
		inter_variable=ros_msg->data[5]*256+ros_msg->data[4];
		heading=static_cast<float>(inter_variable)*0.01;
		unsigned short inter_variable_1=ros_msg->data[7]*256+ros_msg->data[6];
		heading_sigma=static_cast<float>(inter_variable_1)*0.01;

	}else if (ros_msg->id == 0x635)
	{
		unsigned short inter_variable=ros_msg->data[1]*256+ros_msg->data[0];
		latitude_sigma=static_cast<float>(inter_variable)*0.001;
		inter_variable=ros_msg->data[3]*256+ros_msg->data[2];
		longitude_sigma=static_cast<float>(inter_variable)*0.001;
		inter_variable=ros_msg->data[5]*256+ros_msg->data[4];
		vel_e_sigma=static_cast<float>(inter_variable)*0.01;
		inter_variable=ros_msg->data[7]*256+ros_msg->data[6];
		vel_n_sigma=static_cast<float>(inter_variable)*0.01;
	}else if (ros_msg->id == 0x636)
	{
		signed short inter_variable=ros_msg->data[1]*256+ros_msg->data[0];
		acc_x=static_cast<float>(inter_variable)*0.000244;
		inter_variable=ros_msg->data[3]*256+ros_msg->data[2];
		acc_y=static_cast<float>(inter_variable)*0.000244;
		inter_variable=ros_msg->data[5]*256+ros_msg->data[4];
		acc_z=static_cast<float>(inter_variable)*0.000244;
		unsigned short inter_variable_1=ros_msg->data[7]*256+ros_msg->data[6];
		checksum=static_cast<float>(inter_variable_1);

	}else if (ros_msg->id == 0x637)
	{
		signed short inter_variable=ros_msg->data[1]*256+ros_msg->data[0];
		ang_x=static_cast<float>(inter_variable)*0.00875;
		inter_variable=ros_msg->data[3]*256+ros_msg->data[2];
		ang_y=static_cast<float>(inter_variable)*0.00875;
		inter_variable=ros_msg->data[5]*256+ros_msg->data[4];
		ang_z=static_cast<float>(inter_variable)*0.00875;
		unsigned short inter_variable_1=ros_msg->data[7]*256+ros_msg->data[6];
		pitch_sigma=static_cast<float>(inter_variable_1)*0.01;
		imu_publisher();
		inspvax_publisher();
	}

}




void test()
{

}


int main(int argc, char** argv)
{
  bool exit = false;

  // ROS initialization
  ros::init(argc, argv, "ucla_can_parser");
  ros::NodeHandle n;
  ros::NodeHandle priv("~");
  ros::AsyncSpinner spinner(1);
  // get parameters
  std::string imu_topic, inspvax_topic,can_tx_topic,can_rx_topic;
  priv.param<std::string>("can_tx_topic", can_tx_topic, "/can_tx");
  priv.param<std::string>("can_rx_topic", can_rx_topic, "/can_rx");
  priv.param<std::string>("imu_topic", imu_topic, "/gps/imu");
  priv.param<std::string>("inspvax_topic", inspvax_topic, "/novatel/oem7/inspva");
  // Wait for time to be valid
  ros::Time::waitForValid();

  if (exit)
  {
    ros::shutdown();
    return 0;
  }

  can_tx_sub = n.subscribe(can_tx_topic, 500, can_tx_sub_callback);//subscrib messages from Kvaser driver
  can_rx_pub = n.advertise<can_msgs::Frame>(can_rx_topic, 500);//publish the request from Mehdi's code for kvaser driver
  inspvax_pub = n.advertise<novatel_oem7_msgs::INSPVAX>(inspvax_topic, 500);
  imu_pub = n.advertise<sensor_msgs::Imu>(imu_topic, 500);

  spinner.start();

  ros::waitForShutdown();

  return 0;
}
