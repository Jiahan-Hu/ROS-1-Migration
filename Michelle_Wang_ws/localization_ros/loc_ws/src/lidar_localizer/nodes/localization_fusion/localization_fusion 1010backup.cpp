#define SAVEDATA 0

// #include </home/letiangao18/autoware.ai/src/drivers/awf_drivers/as/nodes/ssc_interface/ssc_interface.h>
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

#include <dbw_mkz_msgs/WheelSpeedReport.h>

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
#include <cmath>
using namespace message_filters;
//above is added for geodesy glt
std::ofstream outdata_;

struct pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};
static int _queue_size = 1000;
static ros::Publisher fusion_pose_pub;
static geometry_msgs::PoseStamped fusion_pose_msg;

bool gnss_init_ = false;
bool kfinitial_ = false;
bool ndt_change_flg_ = false;

ros::Time ndt_stat_timestamp_;
static double ndt_stat_iteration_ = 0;
static double ndt_stat_score_ = 0;


// definition
Eigen::Vector3d Xkk_1(0,0,0), Xk_1(0,0,0), Zk(0,0,0),Xk(0,0,0);
Eigen::Matrix3d Hk, Pkk_1= Eigen::Matrix3d::Identity(), Pk_1=Eigen::Matrix3d::Identity(),Qk= Eigen::Matrix3d::Zero();
Eigen::Matrix3d Rk = Eigen::Matrix3d::Zero(),Kk=Eigen::Matrix3d::Zero(),Pk=Eigen::Matrix3d::Identity();
ros::Time gnss_time_cur_fusion;
ros::Time gnss_time_pre_fusion;
uint insstatus_ = 0;
uint fusionStatus = 0;
Eigen::Vector3d errorResult(.0,.0,.0);
Eigen::Vector3d xyz_lever(0, 0, 0),ndt_xyz(0,0,0),ndt_xyz_pre(0,0,0),fusion_ins_xyz(0,0,0);
Eigen::Vector3d zero_utm(0,0,0);
static pose fusion_pose;
bool DR_init_flg = false;

static void filterUpdate()
{
    Eigen::Matrix< double, 3, 3 > PHT;    // 3*3,3*3
    Eigen::Matrix< double, 3, 3 > HPHTR;  // 3*3,3*3,3*3,3*3
    //std::cout << "filterUPdate:"<< std::endl;
    PHT = Pkk_1 * Hk.transpose();  // Pkk_1*H';
    HPHTR = Hk * PHT + Rk;         // H*Pkk_1*H'+R;
    Kk = PHT * HPHTR.inverse();         // K=P*H'*(H*P*H'+R)^-1;
    //std::cout << "Pkk_1:"<< std::endl<<Pkk_1<<std::endl;
    //std::cout << "Hk:"<< std::endl<<Hk<<std::endl;
    //std::cout << "Rk:"<< std::endl<<Rk<<std::endl;
    //std::cout << "PHT:"<< std::endl<<PHT<<std::endl;
    //std::cout << "HPHTR:"<< std::endl<<HPHTR<<std::endl;
    //std::cout << "Kk:"<< std::endl<<Kk<<std::endl;
    
    Eigen::Vector3d innovation;
    innovation = Zk - Hk * Xkk_1;  // Zk-Hk*Xkk_1
    std::cout << "innovation:"<< std::endl<<innovation<<std::endl;
    // Xk, Pk
    Xk = Xkk_1 + Kk * innovation;
    Pk = Pkk_1 - Kk * Hk * Pkk_1;
    Pk = 0.5 * (Pk + Pk.transpose());
    std::cout << "Xk:"<< std::endl<<Xk<<std::endl;
}

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
//static void callback(const novatel_oem7_msgs::INSPVAConstPtr& input_gnss,
//                                      const sensor_msgs::PointCloud2ConstPtr& input_lidar,
//                                      const geometry_msgs::PoseStampedConstPtr& input_ndtpose,
//                                      const autoware_msgs::NDTStatConstPtr& input_ndtstat
//                                      )
//{
static pose pose_error;
static void kf_PoseError_callback(const geometry_msgs::PoseStamped::Ptr& input_PoseError)
{
  pose_error.x = input_PoseError-> pose.position.x;
  pose_error.y = input_PoseError-> pose.position.y;
  pose_error.z = input_PoseError-> pose.position.z;
  pose_error.roll = input_PoseError-> pose.orientation.x;
  pose_error.pitch = input_PoseError-> pose.orientation.y;
  pose_error.yaw = input_PoseError-> pose.orientation.z;
}

ros::Time imu_time_cur;
ros::Time imu_time_pre;
static void imu_callback(const sensor_msgs::Imu::Ptr& input_imu)
{
  
  imu_time_cur = input_imu->header.stamp;
  double ts_imu = (imu_time_cur - imu_time_pre).toSec();
  if (ts_imu>1)
  {
    ts_imu=0.01;
  }
  imu_time_pre = imu_time_cur;
  double yaw_rate_deg = input_imu->angular_velocity.z;
  double yaw_rate_rad = yaw_rate_deg *M_PI/180;
  fusion_pose.yaw+=yaw_rate_rad*ts_imu;
  std::cout<<"imu timestamp:"<<imu_time_cur<<",yaw error:"<<pose_error.yaw;
  // attitude + error
  fusion_pose.yaw += pose_error.yaw;
  fusion_pose.pitch += pose_error.pitch;
  fusion_pose.roll += pose_error.roll;
  fusion_pose.yaw = yaw_normal_rad(fusion_pose.yaw);
  std::cout<<",yaw:"<<fusion_pose.yaw<<std::endl;
  pose_error.yaw = 0;
  pose_error.pitch = 0;
  pose_error.roll = 0;
}


ros::Time wheelspeed_time_cur;
ros::Time wheelspeed_time_pre;
static void wheelspeed_callback(const dbw_mkz_msgs::WheelSpeedReport::Ptr& input_wheelspeed)
{
  wheelspeed_time_cur = input_wheelspeed->header.stamp;
  double ts_whs = (wheelspeed_time_cur - wheelspeed_time_pre).toSec();
  if (ts_whs>1)
  {
    ts_whs=0.01;
  }
  //std::cout<<"ts"<<ts_whs<<",wheelspeed_time_cur:"<<wheelspeed_time_cur<<",wheelspeed_time_pre:"<<wheelspeed_time_pre<<std::endl;
  wheelspeed_time_pre = wheelspeed_time_cur;
  
  //std::cout<<"wheelspeed_callback"<<std::endl;
  float whs_rl = input_wheelspeed->rear_left;
  float whs_rr = input_wheelspeed->rear_right;
  float whl_scalefactor = 1.21;
  
  float veh_speed = (whs_rl+whs_rr)*0.5/3.6*whl_scalefactor;
  float ve_whs = veh_speed * cos(fusion_pose.yaw);
  float vn_whs = veh_speed * sin(fusion_pose.yaw);

  //DR
  if (DR_init_flg)
  {
    std::cout<<"fusion:whs_callback:pose error x:"<<pose_error.x<<",y:"<<pose_error.y<<",z:"<<pose_error.z<<std::endl;
    fusion_pose.x += ve_whs*ts_whs;
    fusion_pose.y += vn_whs*ts_whs;
    fusion_pose.z += 0*ts_whs;
    
    //DR+error:
    fusion_pose.x += pose_error.x;
    fusion_pose.y += pose_error.y;
    fusion_pose.z += pose_error.z;
  
    pose_error.x = 0;
    pose_error.y = 0;
    pose_error.z = 0;
  }
  else
  {
    fusion_pose.x = 0;
    fusion_pose.y = 0;
    fusion_pose.z = 0;
    fusion_pose.roll = 0;
    fusion_pose.pitch = 0;
    fusion_pose.yaw = 0;
  }
  std::cout<<"fusion time stamp:"<<input_wheelspeed-> header.stamp<<", DR init Flg:"<<DR_init_flg<<std::endl;
  std::cout<<"fusion:whs_callback:fusion_pose x:"<<fusion_pose.x<<",y:"<<fusion_pose.y<<",z:"<<fusion_pose.z<<std::endl;
  std::cout<<"fusion:whs_callback:fusion_roll:"<<fusion_pose.roll<<",pitch:"<<fusion_pose.pitch<<",yaw:"<<fusion_pose.yaw<<std::endl;
  std::cout<<"ve_whs:"<<ve_whs<<",vn_whs:"<<vn_whs<<std::endl;
  
  tf::Quaternion fusion_q;
  fusion_q.setRPY(fusion_pose.roll, fusion_pose.pitch, fusion_pose.yaw);

  fusion_pose_msg.header.stamp = input_wheelspeed-> header.stamp;
  fusion_pose_msg.pose.position.x = fusion_pose.x;
  fusion_pose_msg.pose.position.y = fusion_pose.y;
  fusion_pose_msg.pose.position.z = fusion_pose.z;
  fusion_pose_msg.pose.orientation.x = fusion_q.x();
  fusion_pose_msg.pose.orientation.y = fusion_q.y();
  fusion_pose_msg.pose.orientation.z = fusion_q.z();
  fusion_pose_msg.pose.orientation.w = fusion_q.w();
  fusion_pose_pub.publish(fusion_pose_msg);

}

//ndt callback only used for initialization
static void ndt_callback(const geometry_msgs::PoseStampedConstPtr& input_ndtpose,
                         const autoware_msgs::NDTStatConstPtr& input_ndtstat,
                         //const geometry_msgs::PoseStampedConstPtr& input_fusion_pose,
                         const novatel_oem7_msgs::INSPVAXConstPtr& input_gongji)
{
   if (DR_init_flg&&kfinitial_)
   {
    return;
   }
   
   std::cout<<"!!!!!!!!!!!!!ENTER MULTI-CALLBACK!!!!!!!!!"<<std::endl;
   static pose ndt_pose;
   static pose fusion_ins_pose;
    //ndtpose input
   ros::Time current_scan_time = input_ndtpose->header.stamp;
   ndt_pose.x =  input_ndtpose->pose.position.x;
   ndt_pose.y =  input_ndtpose->pose.position.y;
   ndt_pose.z =  input_ndtpose->pose.position.z;
   tf::Quaternion ndt_q(input_ndtpose->pose.orientation.x, input_ndtpose->pose.orientation.y,input_ndtpose->pose.orientation.z,
                        input_ndtpose->pose.orientation.w);
   tf::Matrix3x3 ndt_m(ndt_q);
   ndt_m.getRPY(ndt_pose.roll,ndt_pose.pitch, ndt_pose.yaw);
   std::cout<<"NDTscanTime:"<<current_scan_time<<std::endl;
   std::cout<<"fusion:ndt_callback:ndt_roll:"<<ndt_pose.roll<<",pitch:"<<ndt_pose.pitch<<",yaw:"<<ndt_pose.yaw<<std::endl;
   
    //ndt stat input
   float ndt_exeTime = input_ndtstat->exe_time;
   float ndt_iteration = input_ndtstat->iteration;
   float ndt_fitness_score = input_ndtstat->score;

   //Eigen::Vector3d errorResult(.0,.0,.0);
   //Eigen::Vector3d ndt_xyz(0,0,0),ndt_xyz_pre(0,0,0);
   ndt_xyz << ndt_pose.x, ndt_pose.y, ndt_pose.z;
   
    /*
    //fusion pose input
   ros::Time input_fusion_pos_time = input_fusion_pose->header.stamp;
   std::cout<<"InputFusionTime:"<<input_fusion_pos_time<<std::endl;
   fusion_ins_pose.x =  input_fusion_pose->pose.position.x;
   fusion_ins_pose.y =  input_fusion_pose->pose.position.y;
   fusion_ins_pose.z =  input_fusion_pose->pose.position.z;
   tf::Quaternion fusion_ins_q(input_fusion_pose->pose.orientation.x, input_fusion_pose->pose.orientation.y,input_fusion_pose->pose.orientation.z,
                        input_fusion_pose->pose.orientation.w);
   tf::Matrix3x3 fusion_ins_m(fusion_ins_q);
   fusion_ins_m.getRPY(fusion_ins_pose.roll,fusion_ins_pose.pitch, fusion_ins_pose.yaw);
    */
   //gongji input
  int gongji_status = input_gongji->ins_status.status;
  float gongji_heading = yaw_normal(input_gongji->azimuth)/180*M_PI;
  std::cout<<"fusion heading:" <<fusion_ins_pose.yaw<<std::endl;
  std::cout<<"gongji heading:" <<gongji_heading<<std::endl;
  std::cout<<"gongji ve:" <<input_gongji->east_velocity<<",vn:"<<input_gongji->north_velocity<<std::endl;

  if (!DR_init_flg) //DR initial
  {
    if (ndt_fitness_score < 0.9 && ndt_fitness_score > 0.01 && ndt_iteration < 20)
    {
      fusion_pose.x = ndt_pose.x;
      fusion_pose.y = ndt_pose.y;
      fusion_pose.z = ndt_pose.z;
      fusion_pose.roll = ndt_pose.roll;
      fusion_pose.pitch = ndt_pose.pitch;
      fusion_pose.yaw = ndt_pose.yaw;
      /*
      fusion_ins_pose.x = ndt_pose.x;
      fusion_ins_pose.y = ndt_pose.y;
      fusion_ins_pose.z = ndt_pose.z;
      fusion_ins_pose.roll = ndt_pose.roll;
      fusion_ins_pose.pitch = ndt_pose.pitch;
      fusion_ins_pose.yaw = ndt_pose.yaw;
      */
      DR_init_flg = true;
    }
    else
    {
      fusion_pose.x = 0;
      fusion_pose.y = 0;
      fusion_pose.z = 0;
      fusion_pose.roll = 0;
      fusion_pose.pitch = 0;
      fusion_pose.yaw = 0;
    }
  }
  
  if (!kfinitial_) //KF initial
  {
    Xk_1 << 0., 0., 0.;
    kfinitial_ = true;
  }
/*
  if (DR_init_flg && kfinitial_ )
  {
    //Xkk_1 prediction
    Xkk_1(0)=0;
    Xkk_1(1)=0;
    Xkk_1(2)=0;
    //Fk
    Eigen::Matrix< double, 3, 3 > Fkk_1 = Eigen::Matrix3d::Identity();
    //Qk
    Eigen::Matrix3d temp;
    temp << 0.3, 0., 0., 0.,0.3, 0., 0., 0.,0.3;
    Qk +=temp;
    //Pkk_1
    Pkk_1 = Fkk_1*Pk_1*Fkk_1.transpose()+Qk;
    //Zk
    //Eigen::Vector3d fusion_pose_3d(0.,0.,0.);
    //fusion_pose_3d << fusion_pose.x, fusion_pose.y, fusion_pose.z;
    Hk = Eigen::Matrix3d::Identity();
    if ((ndt_fitness_score < 1 && ndt_fitness_score > 0.01)||(ndt_fitness_score < 2 && ndt_fitness_score > 0.01 && ndt_iteration <30))
    {
      fusion_pose.roll = ndt_pose.roll;
      fusion_pose.pitch = ndt_pose.pitch;
      // fusion_pose.yaw = ndt_pose.yaw;
      //Zk
      fusion_ins_xyz << fusion_ins_pose.x, fusion_ins_pose.y, fusion_ins_pose.z;
      Zk = ndt_xyz - fusion_ins_xyz;
      if (abs(fusion_ins_pose.x)<0.01) // due to message filter, although the pose is initialized by ndt, 
                                  //the found fusion results may be previous stamp, value is 0, then the results can not be used
      {
        Zk << 0, 0, 0;
      }

      std::cout<<"Zk:"<<Zk<<std::endl;
      //Rk
      Rk(0,0) = 0.02;
      Rk(1,1) = 0.02;
      Rk(2,2) = 0.05; // RTK
      filterUpdate();
    }
    else
    {
      Xk = Xkk_1 ;
      Pk = Pkk_1 ;
      Pk = 0.5 * (Pk + Pk.transpose());
    }


    Qk = Eigen::Matrix3d::Zero();
    //Xk_1 = Xk;
    Xk_1 << 0., 0., 0.;
    Pk_1 = Pk;

    pose_error.x=Xk(0);
    pose_error.y=Xk(1);
    pose_error.z=Xk(2);

    // deal with yaw error
    float Zk_yaw = ndt_pose.yaw - fusion_ins_pose.yaw;
    if (abs(fusion_ins_pose.yaw)<0.001) // due to message filter, although the pose is initialized by ndt, 
                                  //the found fusion results may be previous stamp, value is 0, then the results can not be used
      {
        Zk_yaw = 0;
      }
    pose_error.yaw = Zk_yaw * 0.5;


  }
  else
  {
    pose_error.x = 0;
    pose_error.y = 0;
    pose_error.z = 0;
    pose_error.yaw = 0;
  }
  */
}

static void gnss_callback(const novatel_oem7_msgs::INSPVA::Ptr& input_gnss)
{
  /*
  ndt_xyz_pre = ndt_xyz;
  ndt_change_flg_ = false;
  //std::cout << "gnss callback test"<< std::setprecision(13)<< input->latitude<<std::endl;
  */
  geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
 
  insstatus_ = input_gnss->status.status; // novatel ins status, 3 means INS solution good
  //ndt_xyz << current_pose.x, current_pose.y, current_pose.z ;
  /*
  if (ndt_xyz(1) != ndt_xyz_pre(1))
  {
    ndt_change_flg_ = true;
  }
  */
  gps_msg->header = input_gnss->header;
  gps_msg->position.latitude = input_gnss->latitude;
  gps_msg->position.longitude = input_gnss->longitude;
  gps_msg->position.altitude = input_gnss->height;
  
  geodesy::UTMPoint utm;
  geodesy::fromMsg(gps_msg->position, utm);
  Eigen::Vector3d xyz(utm.easting, utm.northing, utm.altitude);
  Eigen::Vector3d gnss_velocities (input_gnss->east_velocity, input_gnss->north_velocity, input_gnss->up_velocity);
  
  //insstatus_ = input->status.status; // novatel ins status, 3 means INS solution good
  
  if ((!gnss_init_)&&(insstatus_==3)) //never initial, ins in RTK, start initialization
  {
    zero_utm = xyz; 
    std::cout << "LFzero_utm_init  x:" << zero_utm[0] << "  y:" << zero_utm[1] << "  z:" << zero_utm[2] << std::endl;
    gnss_time_cur_fusion = input_gnss->header.stamp;
    gnss_time_pre_fusion = gnss_time_cur_fusion;
    gnss_init_ = true;
  }
    xyz -= zero_utm;
    std::cout << "LFxyz  x:" << xyz[0] << "  y:" << xyz[1] << "  z:" << xyz[2] << std::endl;
  //following, lever arm compensation, lidar 90cm ahead of GNSS, transform Lidar to GNSS glt
  
  const float lever_arm_l2g=0.9; //m
  xyz_lever[0]=xyz[0]+lever_arm_l2g*sin((input_gnss->azimuth)/180*M_PI);
  xyz_lever[1]=xyz[1]-lever_arm_l2g*cos((input_gnss->azimuth)/180*M_PI);
  xyz_lever[2]=xyz[2];
  std::cout << "LFxyz_lever  x:" << xyz_lever[0] << "  y:" << xyz_lever[1] << "  z:" << xyz_lever[2] << std::endl;
  // above is for lever comp.
  //below is for fusion
  gnss_time_cur_fusion = input_gnss->header.stamp;
  const double ts_gnss_fusion = (gnss_time_cur_fusion-gnss_time_pre_fusion).toSec();
  gnss_time_pre_fusion = gnss_time_cur_fusion;
  
  if ((!kfinitial_) && gnss_init_) //DR initialization
  {
    fusion_pose.x = xyz_lever(0);
    fusion_pose.y = xyz_lever(1);
    fusion_pose.z = xyz_lever(2);
    //Xk_1 << 0., 0., 0.;
    kfinitial_=true;
    //std::cout<<"LFiiiiiiiiiiiii"<<std::endl;
  }
  /*
  //if (kfinitial_)
  //{//Xkk_1 prediction
    //std::cout<<"dddddddddddddd"<<std::endl;
    Xkk_1(0)=Xk_1(0)+gnss_velocities(0)*ts_gnss_fusion;
    Xkk_1(1)=Xk_1(1)+gnss_velocities(1)*ts_gnss_fusion;
    Xkk_1(2)=Xk_1(2)+gnss_velocities(2)*ts_gnss_fusion;
    */
    std::cout<<"LF ts gnss"<<ts_gnss_fusion<<std::endl;
     /*
    //Fk
    Eigen::Matrix< double, 3, 3 > Fkk_1 = Eigen::Matrix3d::Identity();
    //Qk
    Eigen::Matrix3d temp;
    temp << 0.3, 0., 0., 0.,0.3, 0., 0., 0.,0.3;
    Qk +=temp;
    //Pkk_1
    Pkk_1 = Fkk_1*Pk_1*Fkk_1.transpose()+Qk;
    std::cout << "LF Fkk_1 outside:"<< std::endl<<Fkk_1<<std::endl;
    std::cout << "LF Pk_1 outside:"<< std::endl<<Pk_1<<std::endl;
    std::cout << "LF Fkk_1.transpose outside:"<< std::endl<<Fkk_1.transpose()<<std::endl;
    std::cout << "LF Qk outside:"<< std::endl<<Qk<<std::endl;
    std::cout << "LF Pkk_1 outside:"<< std::endl<<Pkk_1<<std::endl;
    //Zk
    Eigen::Vector3d fusion_pose_3d(0.,0.,0.);
    */

    //DR:
    fusion_pose.x += gnss_velocities(0)*ts_gnss_fusion;
    fusion_pose.y += gnss_velocities(1)*ts_gnss_fusion;
    fusion_pose.z += gnss_velocities(2)*ts_gnss_fusion;
    //fusion_pose_3d << fusion_pose.x, fusion_pose.y, fusion_pose.z;
    /*
    if (insstatus_==3)
    {
      std::cout<<"LF aaaaaaaaaaaaaaaa"<<std::endl;
      std::cout<<"LF xyz_lever:"<<xyz_lever<<"LF fusion_pose_3d:"<<fusion_pose_3d<<std::endl;
      Zk = xyz_lever - fusion_pose_3d;// gnss as measurements
      //Rk
  
      Rk(0,0) = 0.02;
      Rk(1,1) = 0.02;
      Rk(2,2) = 0.05; // RTK
    }
    else if (ndt_stat_score_ < 50)
    {
      std::cout<<"LF bbbbbbbbbb"<<std::endl;
      Zk = ndt_xyz - fusion_pose_3d; //ndt as measurements
      //Rk
  
      Rk(0,0) = 0.1;
      Rk(1,1) = 0.1;
      Rk(2,2) = 0.2; // RTK
    }
    std:: cout<<"LF Zk x:"<< Zk(0) << " y:"<<Zk(1)<<" z:"<< Zk(2)<<std::endl;
  //Zk = xyz_lever - fusion_pose_3d; // gnss as measurements
  
  //Zk = ndt_xyz; // ndt results as measurements
  
  
    Hk = Eigen::Matrix3d::Identity();
  
  //if (insstatus_==3 && ndt_change_flg_)  // 3 means ins solution good
    if (insstatus_==3)
    {
      fusionStatus = 1;
      filterUpdate();
    }
    else if (ndt_stat_score_ < 50)
    {
      fusionStatus = 2;
      filterUpdate();
    }
    else
    {
      fusionStatus = 3;
      Xk = Xkk_1 ;
      Pk = Pkk_1 ;
      Pk = 0.5 * (Pk + Pk.transpose());
    }
  

    Qk = Eigen::Matrix3d::Zero();
    Xk_1 << 0., 0., 0.;
    Pk_1 = Pk;

    errorResult(0)=Xk(0);
    errorResult(1)=Xk(1);
    errorResult(2)=Xk(2);
  //}
  std::cout << "LF error state:"<< fusionStatus<<std::endl;
  std::cout << "LF error  x:" << errorResult(0)<< "  y:" << errorResult(1) << "  z:" << errorResult(2) << std::endl;
  */

  //DR+error:
  std::cout << "fusion time:"<< input_gnss->header.stamp<<std::endl;
  std::cout << "LF fusion results pre x:" << fusion_pose.x<< "  y:" << fusion_pose.y << "  z:" << fusion_pose.z << std::endl;
  fusion_pose.x += pose_error.x;
  fusion_pose.y += pose_error.y;
  fusion_pose.z += pose_error.z;
  std::cout << "LF pose error  x:" << pose_error.x<< "  y:" << pose_error.y << "  z:" << pose_error.z << std::endl;
  std::cout << "LF fusion results  x:" << fusion_pose.x<< "  y:" << fusion_pose.y << "  z:" << fusion_pose.z << std::endl;
  pose_error.x = 0;
  pose_error.y = 0;
  pose_error.z = 0;
  /*
  if (SAVEDATA)
  {
    outdata_ << std::setprecision(13)<< "time " << gnss_time_cur_fusion;
    outdata_ << std::setprecision(5)<< " fusion_x " << errorResult(0)<< " fusion_y " << errorResult(1) << " fusion_z " << errorResult(2);
    outdata_ << std::setprecision(5)<< " gnss_x " << xyz_lever(0)<< " gnss_y " << xyz_lever(1) << " gnss_z " << xyz_lever(2)<< std::endl;
  }
  */
  fusion_pose_msg.header.stamp = input_gnss-> header.stamp;
  fusion_pose_msg.pose.position.x = fusion_pose.x;
  fusion_pose_msg.pose.position.y = fusion_pose.y;
  fusion_pose_msg.pose.position.z = fusion_pose.z;
  fusion_pose_pub.publish(fusion_pose_msg);
  
}
//static void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
//{
//}

//static void ndt_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& input)
//{
  
  //ndt_xyz << input_ndtpose->pose.position.x, input_ndtpose->pose.position.y, input_ndtpose->pose.position.z ;
//}
/*
static void ndt_stat_callback(const autoware_msgs::NDTStat::ConstPtr& input)
{
  ndt_stat_timestamp_ = input_ndtstat->header.stamp;
  ndt_stat_iteration_ = input_ndtstat->iteration;
  ndt_stat_score_ = input_ndtstat->score;
  if (SAVEDATA)
  {
    outdata_ << std::setprecision(13)<< "time " << fusion_pose_msg.header.stamp;
    outdata_ << std::setprecision(1)<< " fusion_state " << fusionStatus;
    outdata_ << std::setprecision(5)<< " fusion_x " << fusion_pose.x<< " fusion_y " << fusion_pose.y << " fusion_z " << fusion_pose.z;
    outdata_ << std::setprecision(5)<< " gnss_x " << xyz_lever(0)<< " gnss_y " << xyz_lever(1) << " gnss_z " << xyz_lever(2);
    outdata_ << std::setprecision(5)<< " ndt_x " << ndt_xyz(0)<< " ndt_y " << ndt_xyz(1) << " ndt_z " << ndt_xyz(2)<< std::endl;
  }
}
*/
int main(int argc, char** argv)
{
  if (SAVEDATA)
  {
    outdata_.open("/home/letiangao18/Bags/results and analysis/savedData.txt", std::ios::app | std::ios::out);
  }
  
  ros::init(argc, argv, "localization_fusion");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  
  // Publishers
  fusion_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/fusion_pose", 20);
  
  std::cout<<"fusion node launched"<<std::endl;

  
  // Subscribers
  //ros::Subscriber gnss_sub = nh.subscribe("/novatel/oem7/inspva", 10, gnss_callback);

  //ros::Subscriber points_sub = nh.subscribe("velodyne_points", _queue_size, points_callback);
  //ros::Subscriber ndt_pos_sub = nh.subscribe("/ndt_pose", 10, ndt_pose_callback);
  //ros::Subscriber ndt_stat_sub = nh.subscribe("/ndt_stat", 10, ndt_stat_callback);
  ros::Subscriber kf_PoseError_sub = nh.subscribe("/kf_PoseError", 10, kf_PoseError_callback);
  ros::Subscriber wheelspeed_sub = nh.subscribe("/vehicle/wheel_speed_report", 10, wheelspeed_callback);
  ros::Subscriber imu_sub = nh.subscribe("/veh_2/gps/imu", 10, imu_callback);

  message_filters::Subscriber<geometry_msgs::PoseStamped> ndt_pos_sub(nh, "/ndt_pose", 2);
  message_filters::Subscriber<autoware_msgs::NDTStat> ndt_stat_sub(nh, "/ndt_stat", 2);
  //message_filters::Subscriber<geometry_msgs::PoseStamped> fusion_pos_sub(nh, "/fusion_pose", 100);
  message_filters::Subscriber<novatel_oem7_msgs::INSPVAX> gongji_sub(nh, "/veh_2/novatel/oem7/inspva", 100);
  
  typedef sync_policies::ApproximateTime<geometry_msgs::PoseStamped,autoware_msgs::NDTStat,/*geometry_msgs::PoseStamped,*/novatel_oem7_msgs::INSPVAX> MySyncPolicy_locafusion;
  Synchronizer<MySyncPolicy_locafusion> sync_localizationfusion(MySyncPolicy_locafusion(20),ndt_pos_sub,ndt_stat_sub,/*fusion_pos_sub,*/gongji_sub);
  //TimeSynchronizer<novatel_oem7_msgs::INSPVA, sensor_msgs::PointCloud2,geometry_msgs::PoseStamped,autoware_msgs::NDTStat> sync(gnss_sub, points_sub,ndt_pos_sub,ndt_stat_sub, 10);
  sync_localizationfusion.registerCallback(boost::bind(&ndt_callback, _1, _2, _3));
  /*
  message_filters::Subscriber<novatel_oem7_msgs::INSPVA> gnss_sub(nh, "/novatel/oem7/inspva", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub(nh, "/velodyne_points", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> ndt_pos_sub(nh, "/ndt_pose", 1);
  message_filters::Subscriber<autoware_msgs::NDTStat> ndt_stat_sub(nh, "/ndt_stat", 1);
  typedef sync_policies::ApproximateTime<novatel_oem7_msgs::INSPVA, sensor_msgs::PointCloud2,geometry_msgs::PoseStamped,autoware_msgs::NDTStat> MySyncPolicy_locafusion;
  Synchronizer<MySyncPolicy_locafusion> sync_localizationfusion(MySyncPolicy_locafusion(30), gnss_sub, points_sub,ndt_pos_sub,ndt_stat_sub);
  //TimeSynchronizer<novatel_oem7_msgs::INSPVA, sensor_msgs::PointCloud2,geometry_msgs::PoseStamped,autoware_msgs::NDTStat> sync(gnss_sub, points_sub,ndt_pos_sub,ndt_stat_sub, 10);
  sync_localizationfusion.registerCallback(boost::bind(&callback, _1, _2, _3, _4));
  */
  ros::spin();
  return 0;
}