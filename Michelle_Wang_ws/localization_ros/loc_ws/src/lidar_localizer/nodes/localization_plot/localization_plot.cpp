#define SAVEDATA 0

//#include </home/letiangao18/autoware.ai/src/drivers/awf_drivers/as/nodes/ssc_interface/ssc_interface.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pthread.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <boost/filesystem.hpp>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>  
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ndt_cpu/NormalDistributionsTransform.h>
#include <pcl/registration/ndt.h>
#ifdef CUDA_FOUND
#include <ndt_gpu/NormalDistributionsTransform.h>
#endif
#ifdef USE_PCL_OPENMP
#include <pcl_omp_registration/ndt.h>
#endif

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <autoware_config_msgs/ConfigNDT.h>

#include <autoware_msgs/NDTStat.h>

#include <novatel_oem7_msgs/INSPVA.h>
#include <novatel_oem7_msgs/INSPVAX.h>

//headers in Autoware Health Checker
//#include <autoware_health_checker/node_status_publisher.h>

#include "ros/time.h"
//following is added for geodesy, glt
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <nmea_msgs/Sentence.h>
#include <eigen3/Eigen/Dense>
#include <boost/optional.hpp>

#include <dbw_mkz_msgs/WheelSpeedReport.h>

using namespace message_filters;
//above is added for geodesy glt
std::ofstream outdata_;


/* original contents
ros::Time gnss_time, ndt_time, fusion_time, kferror_time;
double gnss_x = 0, gnss_y = 0, gnss_z = 0;
double ndt_x = 0, ndt_y = 0, ndt_z = 0;
double fusion_x = 0, fusion_y = 0, fusion_z = 0;
double kferror_x = 0, kferror_y = 0, kferror_z = 0;

static void callback(const geometry_msgs::PoseStampedConstPtr& input_gnsslocal,
                                      //const sensor_msgs::PointCloud2ConstPtr& input_lidar,
                                      const geometry_msgs::PoseStampedConstPtr& input_dr_fusion,
                                      const geometry_msgs::PoseStampedConstPtr& input_ndtpose,
                                      const autoware_msgs::NDTStatConstPtr& input_ndtstat,
                                      const geometry_msgs::PoseStampedConstPtr& input_kfPoseError
                                      //const autoware_msgs::NDTStatConstPtr& input_ndtstat
                                      )
{
  gnss_time = input_gnsslocal -> header.stamp;
  gnss_x = input_gnsslocal -> pose.position.x;
  gnss_y = input_gnsslocal -> pose.position.y;
  gnss_z = input_gnsslocal -> pose.position.z;

  ndt_time = input_ndtpose -> header.stamp;
  ndt_x = input_ndtpose -> pose.position.x;
  ndt_y = input_ndtpose -> pose.position.y;
  ndt_z = input_ndtpose -> pose.position.z;

  fusion_time = input_dr_fusion -> header.stamp;
  fusion_x = input_dr_fusion -> pose.position.x;
  fusion_y = input_dr_fusion -> pose.position.y;
  fusion_z = input_dr_fusion -> pose.position.z;

  kferror_time = input_kfPoseError -> header.stamp;
  kferror_x = input_kfPoseError -> pose.position.x;
  kferror_y = input_kfPoseError -> pose.position.y;
  kferror_z = input_kfPoseError -> pose.position.z;

  if (SAVEDATA)
  {
    outdata_ << std::setprecision(13)<< "time " << gnss_time;
    outdata_ << std::setprecision(5)<< " fusion_x " << fusion_x << " fusion_y " << fusion_y << " fusion_z " << fusion_z;
    outdata_ << std::setprecision(5)<< " gnss_x " << gnss_x<< " gnss_y " <<gnss_y << " gnss_z " << gnss_z;
    //outdata_ << std::setprecision(5)<< " gnss_x " << gnss_x<< " gnss_y " <<gnss_y << " gnss_azimuth " << gnss_z;
    outdata_ << std::setprecision(5)<< " ndt_x " << ndt_x<< " ndt_y " << ndt_y << " ndt_z " << ndt_z<< std::endl;
  }

}
int main(int argc, char** argv)
{
  std:: cout<< "plot node launched"<< std::endl;
  if (SAVEDATA)
  {
    outdata_.open("/home/letiangao18/Bags/results and analysis/savedData.txt", std::ios::app | std::ios::out);
  }
  
  ros::init(argc, argv, "localization_plot");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  //  
  // // Publishers
  // fusion_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/fusion_pose", 10);
  
  // std::cout<<"fusion node launched"<<std::endl;

  // // Subscribers
  // ros::Subscriber gnss_sub = nh.subscribe("/novatel/oem7/inspva", 10, gnss_callback);
  // //ros::Subscriber points_sub = nh.subscribe("velodyne_points", _queue_size, points_callback);
  // //ros::Subscriber ndt_pos_sub = nh.subscribe("/ndt_pose", 10, ndt_pose_callback);
  // //ros::Subscriber ndt_stat_sub = nh.subscribe("/ndt_stat", 10, ndt_stat_callback);
  // ros::Subscriber kf_PoseError_sub = nh.subscribe("/kf_PoseError", 10, kf_PoseError_callback);
  // 
 

//   message_filters::Subscriber<novatel_oem7_msgs::INSPVA> gnss_sub(nh, "/novatel/oem7/inspva", 1);
//   message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub(nh, "/velodyne_points", 1);
//   message_filters::Subscriber<geometry_msgs::PoseStamped> ndt_pos_sub(nh, "/ndt_pose", 1);
//   message_filters::Subscriber<autoware_msgs::NDTStat> ndt_stat_sub(nh, "/ndt_stat", 1);
//   message_filters::Subscriber<geometry_msgs::PoseStamped> kf_PoseError_sub(nh, "/kf_PoseError", 1);
//   message_filters::Subscriber<geometry_msgs::PoseStamped> fusion_pose_sub(nh, "/fusion_pose", 1);
//   typedef sync_policies::ApproximateTime<novatel_oem7_msgs::INSPVA, sensor_msgs::PointCloud2, geometry_msgs::PoseStamped, autoware_msgs::NDTStat, geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> MySyncPolicy_locaplot;
//   Synchronizer<MySyncPolicy_locaplot> sync_localizationplot(MySyncPolicy_locaplot(30), gnss_sub, points_sub,ndt_pos_sub,ndt_stat_sub, kf_PoseError_sub, fusion_pose_sub);
//   //TimeSynchronizer<novatel_oem7_msgs::INSPVA, sensor_msgs::PointCloud2,geometry_msgs::PoseStamped,autoware_msgs::NDTStat> sync(gnss_sub, points_sub,ndt_pos_sub,ndt_stat_sub, 10);
//   sync_localizationplot.registerCallback(boost::bind(&plot_callback, _1, _2, _3, _4, _5, _6));


  message_filters::Subscriber<geometry_msgs::PoseStamped> gnss_sub(nh, "/gnss_localframe", 1);
  //message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub(nh, "/velodyne_points", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> dr_fusion_sub(nh, "/fusion_pose", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> ndt_pos_sub(nh, "/ndt_pose", 1);
  message_filters::Subscriber<autoware_msgs::NDTStat> ndt_stat_sub(nh, "/ndt_stat", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> kf_PoseError_sub(nh, "/kf_PoseError", 1);
  typedef sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped, geometry_msgs::PoseStamped, autoware_msgs::NDTStat,geometry_msgs::PoseStamped> MySyncPolicy_locafusion;
  Synchronizer<MySyncPolicy_locafusion> sync_localizationfusion(MySyncPolicy_locafusion(30), gnss_sub, dr_fusion_sub, ndt_pos_sub,ndt_stat_sub, kf_PoseError_sub);
  //TimeSynchronizer<novatel_oem7_msgs::INSPVA, sensor_msgs::PointCloud2,geometry_msgs::PoseStamped,autoware_msgs::NDTStat> sync(gnss_sub, points_sub,ndt_pos_sub,ndt_stat_sub, 10);
  sync_localizationfusion.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));
  


  ros::spin();
  return 0;
}
*/ //original contents end



// below for 20230821 test
bool init_flg = false;
float yaw =0;
ros::Time wheelspeed_time_cur;
ros::Time wheelspeed_time_pre;
ros::Time imu_time_cur;
ros::Time imu_time_pre;
ros::Time gongji_time_cur;
static float yaw_normal(float yaw_gongji)
{
  //input unit should be degree, output is degree
  // transform gongji yaw to ENU yaw. gongji yaw is -180-180, north0, counter-clock wise positive. 
  //ENU yaw is -180-180, east 0, counter-clock wise is positive
  float yaw = yaw_gongji+90;
  
  if (yaw >= 180)
  {
    yaw = yaw-360;
  }
  return yaw;
}

static float yaw_normal_rad(float yaw)
{
  //input is rad make yaw within -pi~pi
  
  if (yaw >= M_PI)
  {
    yaw = yaw-2*M_PI;
  }
  else if (yaw<-M_PI)
  {
    yaw=yaw+2*M_PI;
  }
  return yaw;
}
static void callback(const dbw_mkz_msgs::WheelSpeedReportConstPtr& input_wheelspeed,
                         const sensor_msgs::ImuConstPtr& input_imu,
                         const novatel_oem7_msgs::INSPVAXConstPtr& input_gongji)
{
  // gongji input
  gongji_time_cur = input_gongji->header.stamp;
  int gongji_status = input_gongji->ins_status.status;
  float gongji_heading = yaw_normal(input_gongji->azimuth)/180*M_PI;
  float gongji_ve = input_gongji->east_velocity;
  float gongji_vn = input_gongji->north_velocity;
  float gongji_speed = sqrt(gongji_ve*gongji_ve+gongji_vn*gongji_vn);
  
  if (!init_flg)
  {
    yaw = gongji_heading;
  }
  std::cout<<"gongji_time:"<<gongji_time_cur<<",gongji heading:" <<gongji_heading<<std::endl;
  //imu input
  imu_time_cur = input_imu->header.stamp;
  
  if (!init_flg)
  {
    imu_time_pre = imu_time_cur;
  }
  double ts_imu = (imu_time_cur - imu_time_pre).toSec();
  double yaw_rate_deg = input_imu->angular_velocity.z;
  double yaw_rate_rad = yaw_rate_deg *M_PI/180;
  yaw+=yaw_rate_rad*ts_imu;
  yaw = yaw_normal_rad(yaw);
  //std::cout<<"imu timestamp:"<<imu_time_cur<<",yaw error:"<<pose_error.yaw;
  // yaw + error
  //fusion_pose.yaw += pose_error.yaw;
  std::cout<<"imu_time:"<<imu_time_cur<<",yawrate"<<yaw_rate_rad<<",ts"<<ts_imu<<",DR_yaw:"<<yaw<<std::endl;
  imu_time_pre = imu_time_cur;

  //wheelspeed input
  wheelspeed_time_cur = input_wheelspeed->header.stamp;
   if (!init_flg)
  {
    wheelspeed_time_pre = wheelspeed_time_cur;
  }
  double ts_whs = (wheelspeed_time_cur - wheelspeed_time_pre).toSec();
  //std::cout<<"ts"<<ts_whs<<",wheelspeed_time_cur:"<<wheelspeed_time_cur<<",wheelspeed_time_pre:"<<wheelspeed_time_pre<<std::endl;
  
  //std::cout<<"wheelspeed_callback"<<std::endl;
  float whs_rl = input_wheelspeed->rear_left;
  float whs_rr = input_wheelspeed->rear_right;
  float whl_scalefactor = 1.21;
  
  float veh_speed = (whs_rl+whs_rr)*0.5/3.6*whl_scalefactor;
  float ve_whs = veh_speed * cos(yaw);
  float vn_whs = veh_speed * sin(yaw);
  std::cout<<"whs_time:"<<wheelspeed_time_cur<<",whs:"<<veh_speed<<",gongji_speed:"<<gongji_speed<<", err:"<<veh_speed-gongji_speed<<",k:"<<gongji_speed/veh_speed<<std::endl;
  std::cout<<"ve_whs:"<<ve_whs<<",ve_gongji:"<<gongji_ve<<", err:"<<ve_whs-gongji_ve<<std::endl;
  std::cout<<"vn_whs:"<<vn_whs<<",vn_gongji:"<<gongji_vn<<", err:"<<vn_whs-gongji_vn<<std::endl;
  std::cout<<"_____________________________________"<<std::endl;
  wheelspeed_time_pre = wheelspeed_time_cur;
  
  init_flg = true;

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "localization_plot");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  std:: cout<< "plot node launched"<< std::endl;

 // Subscribers
  //ros::Subscriber gnss_sub = nh.subscribe("/novatel/oem7/inspva", 10, gnss_callback);
  //ros::Subscriber points_sub = nh.subscribe("velodyne_points", _queue_size, points_callback);
  //ros::Subscriber ndt_pos_sub = nh.subscribe("/ndt_pose", 10, ndt_pose_callback);
  //ros::Subscriber ndt_stat_sub = nh.subscribe("/ndt_stat", 10, ndt_stat_callback);
  //ros::Subscriber kf_PoseError_sub = nh.subscribe("/kf_PoseError", 10, kf_PoseError_callback);
  //ros::Subscriber wheelspeed_sub = nh.subscribe("/vehicle/wheel_speed_report", 10, wheelspeed_callback);
  //ros::Subscriber imu_sub = nh.subscribe("/veh_2/gps/imu", 10, imu_callback);

  //message_filters::Subscriber<geometry_msgs::PoseStamped> ndt_pos_sub(nh, "/ndt_pose", 5);
  //message_filters::Subscriber<autoware_msgs::NDTStat> ndt_stat_sub(nh, "/ndt_stat", 5);
  //message_filters::Subscriber<geometry_msgs::PoseStamped> fusion_pos_sub(nh, "/fusion_pose", 100);
  message_filters::Subscriber<dbw_mkz_msgs::WheelSpeedReport> wheelspeed_sub(nh, "/vehicle/wheel_speed_report", 10);
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/veh_2/gps/imu", 10);
  message_filters::Subscriber<novatel_oem7_msgs::INSPVAX> gongji_sub(nh, "/veh_2/novatel/oem7/inspva", 10);
  
  typedef sync_policies::ApproximateTime<dbw_mkz_msgs::WheelSpeedReport,sensor_msgs::Imu,novatel_oem7_msgs::INSPVAX> MySyncPolicy_locaplot;
  Synchronizer<MySyncPolicy_locaplot> sync_localizationplot(MySyncPolicy_locaplot(10),wheelspeed_sub,imu_sub,gongji_sub);
  //TimeSynchronizer<novatel_oem7_msgs::INSPVA, sensor_msgs::PointCloud2,geometry_msgs::PoseStamped,autoware_msgs::NDTStat> sync(gnss_sub, points_sub,ndt_pos_sub,ndt_stat_sub, 10);
  sync_localizationplot.registerCallback(boost::bind(&callback, _1, _2, _3));
  
  ros::spin();
  return 0;
}