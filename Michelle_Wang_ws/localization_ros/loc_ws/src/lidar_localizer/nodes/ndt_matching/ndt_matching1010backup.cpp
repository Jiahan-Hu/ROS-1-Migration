/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 Localization program using Normal Distributions Transform

 Yuki KITSUKAWA
 */
#define SAVEDATA 0
#define SAVEDATA_datasetprocess 0


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

#include <dbw_mkz_msgs/WheelSpeedReport.h>

//headers in Autoware Health Checker
//include <autoware_health_checker/node_status_publisher.h>

#include "ros/time.h"
//following is added for geodesy, glt
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <gps_common/GPSFix.h>
#include <sensor_msgs/NavSatFix.h>
#include <nmea_msgs/Sentence.h>
#include <eigen3/Eigen/Dense>
#include <boost/optional.hpp>

using namespace message_filters;
//above is added for geodesy glt
#define PREDICT_POSE_THRESHOLD 0.5

#define Wa 0.4
#define Wb 0.3
#define Wc 0.3
int a =0;
//static std::shared_ptr<autoware_health_checker::NodeStatusPublisher> node_status_publisher_ptr_;

struct pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

enum class MethodType
{
  PCL_GENERIC = 0,
  PCL_ANH = 1,
  PCL_ANH_GPU = 2,
  PCL_OPENMP = 3,
};
static MethodType _method_type = MethodType::PCL_GENERIC;

static pose initial_pose, predict_pose, predict_pose_imu, predict_pose_odom, predict_pose_imu_odom, previous_pose,
    ndt_pose, current_pose, current_pose_imu, current_pose_odom, current_pose_imu_odom, localizer_pose;

static double offset_x, offset_y, offset_z, offset_yaw;  // current_pos - previous_pose
static double offset_imu_x, offset_imu_y, offset_imu_z, offset_imu_roll, offset_imu_pitch, offset_imu_yaw;
static double offset_odom_x, offset_odom_y, offset_odom_z, offset_odom_roll, offset_odom_pitch, offset_odom_yaw;
static double offset_imu_odom_x, offset_imu_odom_y, offset_imu_odom_z, offset_imu_odom_roll, offset_imu_odom_pitch,
    offset_imu_odom_yaw;

static double _initial_pose_x, _initial_pose_y,_initial_pose_z, _initial_pose_roll, _initial_pose_pitch, _initial_pose_yaw;
static double _map_ori_utm_easting, _map_ori_utm_northing, _map_ori_utm_altitude;
static double gps_raw_longitude, gps_raw_latitude, gps_raw_altitude, gps_raw_heading;
static int _init_post_set_fromGNSS, _init_post_set_fromLAUNCH;

// Can't load if typed "pcl::PointCloud<pcl::PointXYZRGB> map, add;"
static pcl::PointCloud<pcl::PointXYZ> map, add;

// If the map is loaded, map_loaded will be 1.
/* original
static int map_loaded = 0;
static int _use_gnss = 1;
static int init_pos_set = 0;
*/
static int map_loaded = 1;
static int _use_gnss = 0;
static int init_pos_set = 0;

pcl::PointCloud<pcl::PointXYZ>::Ptr map_saved_cloud (new pcl::PointCloud<pcl::PointXYZ>);

bool map_pub=true;

std::ofstream outdata_;
std::ofstream outdata_datasetprocessing_;

static pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
static cpu::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> anh_ndt;
#ifdef CUDA_FOUND
static std::shared_ptr<gpu::GNormalDistributionsTransform> anh_gpu_ndt_ptr =
    std::make_shared<gpu::GNormalDistributionsTransform>();
#endif
#ifdef USE_PCL_OPENMP
static pcl_omp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> omp_ndt;
#endif

// Default values
static int max_iter = 25;        // Maximum iterations default 30
static float ndt_res = 1.5;      // Resolution default 1.0
static double step_size = 0.1;   // Step size default 0.1
static double trans_eps = 0.015;  // Transformation epsilon default 0.01

static ros::Publisher predict_pose_pub;
static geometry_msgs::PoseStamped predict_pose_msg;

static ros::Publisher predict_pose_imu_pub;
static geometry_msgs::PoseStamped predict_pose_imu_msg;

static ros::Publisher predict_pose_odom_pub;
static geometry_msgs::PoseStamped predict_pose_odom_msg;

static ros::Publisher predict_pose_imu_odom_pub;
static geometry_msgs::PoseStamped predict_pose_imu_odom_msg;

static ros::Publisher ndt_pose_pub;
static geometry_msgs::PoseStamped ndt_pose_msg;

// current_pose is published by vel_pose_mux
/*
static ros::Publisher current_pose_pub;
static geometry_msgs::PoseStamped current_pose_msg;
 */

static ros::Publisher localizer_pose_pub;
static geometry_msgs::PoseStamped localizer_pose_msg;

static ros::Publisher estimate_twist_pub;
static geometry_msgs::TwistStamped estimate_twist_msg;

static ros::Publisher ndt_final_cloud_pub;
static sensor_msgs::PointCloud2::Ptr ndt_final_cloud_ptr(new sensor_msgs::PointCloud2);

static ros::Publisher saved_map_cloud_pub;
//static sensor_msgs::PointCloud2::Ptr saved_map_cloud_ptr(new sensor_msgs::PointCloud2);
static sensor_msgs::PointCloud2 saved_map_cloud;
static ros::Duration scan_duration;

static ros::Publisher kf_PoseError_pub;
static geometry_msgs::PoseStamped kf_PoseError_msg;

static ros::Publisher gnss_localframe_pub;
static geometry_msgs::PoseStamped gnss_localframe_msg;

static double exe_time = 0.0;
static bool has_converged;
static int iteration = 0;
static double fitness_score = 0.0;
static double trans_probability = 0.0;

static double diff = 0.0;
static double diff_x = 0.0, diff_y = 0.0, diff_z = 0.0, diff_yaw;

static double current_velocity = 0.0, previous_velocity = 0.0, previous_previous_velocity = 0.0;  // [m/s]
static double current_velocity_x = 0.0, previous_velocity_x = 0.0;
static double current_velocity_y = 0.0, previous_velocity_y = 0.0;
static double current_velocity_z = 0.0, previous_velocity_z = 0.0;
// static double current_velocity_yaw = 0.0, previous_velocity_yaw = 0.0;
static double current_velocity_smooth = 0.0;

static double current_velocity_imu_x = 0.0;
static double current_velocity_imu_y = 0.0;
static double current_velocity_imu_z = 0.0;

static double current_accel = 0.0, previous_accel = 0.0;  // [m/s^2]
static double current_accel_x = 0.0;
static double current_accel_y = 0.0;
static double current_accel_z = 0.0;
// static double current_accel_yaw = 0.0;

static double angular_velocity = 0.0;

static int use_predict_pose = 0;

static ros::Publisher estimated_vel_mps_pub, estimated_vel_kmph_pub, estimated_vel_pub;
static std_msgs::Float32 estimated_vel_mps, estimated_vel_kmph, previous_estimated_vel_kmph;

static std::chrono::time_point<std::chrono::system_clock> matching_start, matching_end;

static ros::Publisher time_ndt_matching_pub;
static std_msgs::Float32 time_ndt_matching;

static int _queue_size = 1000;

static ros::Publisher ndt_stat_pub;
static autoware_msgs::NDTStat ndt_stat_msg;

static double predict_pose_error = 0.0;

static double _tf_x=0, _tf_y=0, _tf_z=0, _tf_roll=0, _tf_pitch=0, _tf_yaw=0;
static Eigen::Matrix4f tf_btol;

//static std::string _localizer = "velodyne";
static std::string _localizer = "rslidar";
static std::string _offset = "zero";  // linear, zero, quadratic

static ros::Publisher ndt_reliability_pub;
static std_msgs::Float32 ndt_reliability;

static bool _get_height = false;
static bool _use_local_transform = false;
static bool _use_imu = false;
static bool _use_odom = false;
static bool _imu_upside_down = false;
static bool _output_log_data = false;

static std::string _imu_topic = "/imu_raw";

static std::ofstream ofs;
static std::string filename;

static sensor_msgs::Imu imu;
static nav_msgs::Odometry odom;

// static tf::TransformListener local_transform_listener;
static tf::StampedTransform local_transform;

static unsigned int points_map_num = 0;

static double gps_fusion_roll, gps_fusion_pitch;

Eigen::Vector3d map_ori_utm;

pthread_mutex_t mutex;

ros::Time rawPointCouldStamp;
double rawPointCouldStamp_db;
Eigen::Vector3d gps_raw_local(0,0,0), gps_attitude_local, initial_pose_local, initial_attitude_local;
static pose convertPoseIntoRelativeCoordinate(const pose &target_pose, const pose &reference_pose)
{
    tf::Quaternion target_q;
    target_q.setRPY(target_pose.roll, target_pose.pitch, target_pose.yaw);
    tf::Vector3 target_v(target_pose.x, target_pose.y, target_pose.z);
    tf::Transform target_tf(target_q, target_v);

    tf::Quaternion reference_q;
    reference_q.setRPY(reference_pose.roll, reference_pose.pitch, reference_pose.yaw);
    tf::Vector3 reference_v(reference_pose.x, reference_pose.y, reference_pose.z);
    tf::Transform reference_tf(reference_q, reference_v);

    tf::Transform trans_target_tf = reference_tf.inverse() * target_tf;

    pose trans_target_pose;
    trans_target_pose.x = trans_target_tf.getOrigin().getX();
    trans_target_pose.y = trans_target_tf.getOrigin().getY();
    trans_target_pose.z = trans_target_tf.getOrigin().getZ();
    tf::Matrix3x3 tmp_m(trans_target_tf.getRotation());
    tmp_m.getRPY(trans_target_pose.roll, trans_target_pose.pitch, trans_target_pose.yaw);

    return trans_target_pose;
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

/*
static void param_callback(const autoware_config_msgs::ConfigNDT::ConstPtr& input)
{
  std::cout<< "enter param_callback"<<std::endl;
  std::cout<< input->x<<input->y<<input->z<<input->roll<<input->pitch<<input->yaw;
  if (_use_gnss != input->init_pos_gnss)
  {
    init_pos_set = 0;
  }
  else if (_use_gnss == 0 &&
           (initial_pose.x != input->x || initial_pose.y != input->y || initial_pose.z != input->z ||
            initial_pose.roll != input->roll || initial_pose.pitch != input->pitch || initial_pose.yaw != input->yaw))
  {
    init_pos_set = 0;
  }

  _use_gnss = input->init_pos_gnss;

  // Setting parameters
  if (input->resolution != ndt_res)
  {
    ndt_res = input->resolution;

    if (_method_type == MethodType::PCL_GENERIC)
      ndt.setResolution(ndt_res);
    else if (_method_type == MethodType::PCL_ANH)
      anh_ndt.setResolution(ndt_res);
#ifdef CUDA_FOUND
    else if (_method_type == MethodType::PCL_ANH_GPU)
      anh_gpu_ndt_ptr->setResolution(ndt_res);
#endif
#ifdef USE_PCL_OPENMP
    else if (_method_type == MethodType::PCL_OPENMP)
      omp_ndt.setResolution(ndt_res);
#endif
  }

  if (input->step_size != step_size)
  {
    step_size = input->step_size;

    if (_method_type == MethodType::PCL_GENERIC)
      ndt.setStepSize(step_size);
    else if (_method_type == MethodType::PCL_ANH)
      anh_ndt.setStepSize(step_size);
#ifdef CUDA_FOUND
    else if (_method_type == MethodType::PCL_ANH_GPU)
      anh_gpu_ndt_ptr->setStepSize(step_size);
#endif
#ifdef USE_PCL_OPENMP
    else if (_method_type == MethodType::PCL_OPENMP)
      omp_ndt.setStepSize(ndt_res);
#endif
  }

  if (input->trans_epsilon != trans_eps)
  {
    trans_eps = input->trans_epsilon;

    if (_method_type == MethodType::PCL_GENERIC)
      ndt.setTransformationEpsilon(trans_eps);
    else if (_method_type == MethodType::PCL_ANH)
      anh_ndt.setTransformationEpsilon(trans_eps);
#ifdef CUDA_FOUND
    else if (_method_type == MethodType::PCL_ANH_GPU)
      anh_gpu_ndt_ptr->setTransformationEpsilon(trans_eps);
#endif
#ifdef USE_PCL_OPENMP
    else if (_method_type == MethodType::PCL_OPENMP)
      omp_ndt.setTransformationEpsilon(ndt_res);
#endif
  }

  if (input->max_iterations != max_iter)
  {
    max_iter = input->max_iterations;

    if (_method_type == MethodType::PCL_GENERIC)
      ndt.setMaximumIterations(max_iter);
    else if (_method_type == MethodType::PCL_ANH)
      anh_ndt.setMaximumIterations(max_iter);
#ifdef CUDA_FOUND
    else if (_method_type == MethodType::PCL_ANH_GPU)
      anh_gpu_ndt_ptr->setMaximumIterations(max_iter);
#endif
#ifdef USE_PCL_OPENMP
    else if (_method_type == MethodType::PCL_OPENMP)
      omp_ndt.setMaximumIterations(ndt_res);
#endif
  }

  if (_use_gnss == 0 && init_pos_set == 0)
  {
    initial_pose.x = input->x;
    initial_pose.y = input->y;
    initial_pose.z = input->z;
    initial_pose.roll = input->roll;
    initial_pose.pitch = input->pitch;
    initial_pose.yaw = input->yaw;

    if (_use_local_transform == true)
    {
      tf::Vector3 v(input->x, input->y, input->z);
      tf::Quaternion q;
      q.setRPY(input->roll, input->pitch, input->yaw);
      tf::Transform transform(q, v);
      initial_pose.x = (local_transform.inverse() * transform).getOrigin().getX();
      initial_pose.y = (local_transform.inverse() * transform).getOrigin().getY();
      initial_pose.z = (local_transform.inverse() * transform).getOrigin().getZ();

      tf::Matrix3x3 m(q);
      m.getRPY(initial_pose.roll, initial_pose.pitch, initial_pose.yaw);

      std::cout << "initial_pose.x: " << initial_pose.x << std::endl;
      std::cout << "initial_pose.y: " << initial_pose.y << std::endl;
      std::cout << "initial_pose.z: " << initial_pose.z << std::endl;
      std::cout << "initial_pose.roll: " << initial_pose.roll << std::endl;
      std::cout << "initial_pose.pitch: " << initial_pose.pitch << std::endl;
      std::cout << "initial_pose.yaw: " << initial_pose.yaw << std::endl;
    }

    // Setting position and posture for the first time.
    localizer_pose.x = initial_pose.x;
    localizer_pose.y = initial_pose.y;
    localizer_pose.z = initial_pose.z;
    localizer_pose.roll = initial_pose.roll;
    localizer_pose.pitch = initial_pose.pitch;
    localizer_pose.yaw = initial_pose.yaw;

    previous_pose.x = initial_pose.x;
    previous_pose.y = initial_pose.y;
    previous_pose.z = initial_pose.z;
    previous_pose.roll = initial_pose.roll;
    previous_pose.pitch = initial_pose.pitch;
    previous_pose.yaw = initial_pose.yaw;

    current_pose.x = initial_pose.x;
    current_pose.y = initial_pose.y;
    current_pose.z = initial_pose.z;
    current_pose.roll = initial_pose.roll;
    current_pose.pitch = initial_pose.pitch;
    current_pose.yaw = initial_pose.yaw;

    current_velocity = 0;
    current_velocity_x = 0;
    current_velocity_y = 0;
    current_velocity_z = 0;
    angular_velocity = 0;

    current_pose_imu.x = 0;
    current_pose_imu.y = 0;
    current_pose_imu.z = 0;
    current_pose_imu.roll = 0;
    current_pose_imu.pitch = 0;
    current_pose_imu.yaw = 0;

    current_velocity_imu_x = current_velocity_x;
    current_velocity_imu_y = current_velocity_y;
    current_velocity_imu_z = current_velocity_z;
    init_pos_set = 1;
  }
}

static void map_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  // if (map_loaded == 0)
  if (points_map_num != input->width)
  {
    std::cout << "Update points_map." << std::endl;

    points_map_num = input->width;

    // Convert the data type(from sensor_msgs to pcl).
    pcl::fromROSMsg(*input, map);

    if (_use_local_transform == true)
    {
      tf::TransformListener local_transform_listener;
      try
      {
        ros::Time now = ros::Time(0);
        local_transform_listener.waitForTransform("/map", "/world", now, ros::Duration(10.0));
        local_transform_listener.lookupTransform("/map", "world", now, local_transform);
      }
      catch (tf::TransformException& ex)
      {
        ROS_ERROR("%s", ex.what());
      }

      pcl_ros::transformPointCloud(map, map, local_transform.inverse());
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZ>(map));

    // Setting point cloud to be aligned to.
    if (_method_type == MethodType::PCL_GENERIC)
    {
      pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> new_ndt;
      pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      new_ndt.setResolution(ndt_res);
      new_ndt.setInputTarget(map_ptr);
      new_ndt.setMaximumIterations(max_iter);
      new_ndt.setStepSize(step_size);
      new_ndt.setTransformationEpsilon(trans_eps);

      new_ndt.align(*output_cloud, Eigen::Matrix4f::Identity());

      pthread_mutex_lock(&mutex);
      ndt = new_ndt;
      pthread_mutex_unlock(&mutex);
    }
    else if (_method_type == MethodType::PCL_ANH)
    {
      cpu::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> new_anh_ndt;
      new_anh_ndt.setResolution(ndt_res);
      new_anh_ndt.setInputTarget(map_ptr);
      new_anh_ndt.setMaximumIterations(max_iter);
      new_anh_ndt.setStepSize(step_size);
      new_anh_ndt.setTransformationEpsilon(trans_eps);

      pcl::PointCloud<pcl::PointXYZ>::Ptr dummy_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::PointXYZ dummy_point;
      dummy_scan_ptr->push_back(dummy_point);
      new_anh_ndt.setInputSource(dummy_scan_ptr);

      new_anh_ndt.align(Eigen::Matrix4f::Identity());

      pthread_mutex_lock(&mutex);
      anh_ndt = new_anh_ndt;
      pthread_mutex_unlock(&mutex);
    }
#ifdef CUDA_FOUND
    else if (_method_type == MethodType::PCL_ANH_GPU)
    {
      std::shared_ptr<gpu::GNormalDistributionsTransform> new_anh_gpu_ndt_ptr =
          std::make_shared<gpu::GNormalDistributionsTransform>();
      new_anh_gpu_ndt_ptr->setResolution(ndt_res);
      new_anh_gpu_ndt_ptr->setInputTarget(map_ptr);
      new_anh_gpu_ndt_ptr->setMaximumIterations(max_iter);
      new_anh_gpu_ndt_ptr->setStepSize(step_size);
      new_anh_gpu_ndt_ptr->setTransformationEpsilon(trans_eps);

      pcl::PointCloud<pcl::PointXYZ>::Ptr dummy_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::PointXYZ dummy_point;
      dummy_scan_ptr->push_back(dummy_point);
      new_anh_gpu_ndt_ptr->setInputSource(dummy_scan_ptr);

      new_anh_gpu_ndt_ptr->align(Eigen::Matrix4f::Identity());

      pthread_mutex_lock(&mutex);
      anh_gpu_ndt_ptr = new_anh_gpu_ndt_ptr;
      pthread_mutex_unlock(&mutex);
    }
#endif
#ifdef USE_PCL_OPENMP
    else if (_method_type == MethodType::PCL_OPENMP)
    {
      pcl_omp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> new_omp_ndt;
      pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      new_omp_ndt.setResolution(ndt_res);
      new_omp_ndt.setInputTarget(map_ptr);
      new_omp_ndt.setMaximumIterations(max_iter);
      new_omp_ndt.setStepSize(step_size);
      new_omp_ndt.setTransformationEpsilon(trans_eps);

      new_omp_ndt.align(*output_cloud, Eigen::Matrix4f::Identity());

      pthread_mutex_lock(&mutex);
      omp_ndt = new_omp_ndt;
      pthread_mutex_unlock(&mutex);
    }
#endif
    map_loaded = 1;
  }
}
*/



/*
static void gnss_callback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
  tf::Quaternion gnss_q(input->pose.orientation.x, input->pose.orientation.y, input->pose.orientation.z,
                        input->pose.orientation.w);
  tf::Matrix3x3 gnss_m(gnss_q);

  pose current_gnss_pose;
  current_gnss_pose.x = input->pose.position.x;
  current_gnss_pose.y = input->pose.position.y;
  current_gnss_pose.z = input->pose.position.z;
  gnss_m.getRPY(current_gnss_pose.roll, current_gnss_pose.pitch, current_gnss_pose.yaw);

  static pose previous_gnss_pose = current_gnss_pose;
  ros::Time current_gnss_time = input->header.stamp;
  static ros::Time previous_gnss_time = current_gnss_time;

  if ((_use_gnss == 1 && init_pos_set == 0) || fitness_score >= 500.0)
  {
    previous_pose.x = previous_gnss_pose.x;
    previous_pose.y = previous_gnss_pose.y;
    previous_pose.z = previous_gnss_pose.z;
    previous_pose.roll = previous_gnss_pose.roll;
    previous_pose.pitch = previous_gnss_pose.pitch;
    previous_pose.yaw = previous_gnss_pose.yaw;

    current_pose.x = current_gnss_pose.x;
    current_pose.y = current_gnss_pose.y;
    current_pose.z = current_gnss_pose.z;
    current_pose.roll = current_gnss_pose.roll;
    current_pose.pitch = current_gnss_pose.pitch;
    current_pose.yaw = current_gnss_pose.yaw;

    current_pose_imu = current_pose_odom = current_pose_imu_odom = current_pose;

    diff_x = current_pose.x - previous_pose.x;
    diff_y = current_pose.y - previous_pose.y;
    diff_z = current_pose.z - previous_pose.z;
    diff_yaw = current_pose.yaw - previous_pose.yaw;
    diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

    const pose trans_current_pose = convertPoseIntoRelativeCoordinate(current_pose, previous_pose);

    const double diff_time = (current_gnss_time - previous_gnss_time).toSec();
    current_velocity = (diff_time > 0) ? (diff / diff_time) : 0;
    current_velocity =  (trans_current_pose.x >= 0) ? current_velocity : -current_velocity;
    current_velocity_x = (diff_time > 0) ? (diff_x / diff_time) : 0;
    current_velocity_y = (diff_time > 0) ? (diff_y / diff_time) : 0;
    current_velocity_z = (diff_time > 0) ? (diff_z / diff_time) : 0;
    angular_velocity = (diff_time > 0) ? (diff_yaw / diff_time) : 0;

    current_accel = 0.0;
    current_accel_x = 0.0;
    current_accel_y = 0.0;
    current_accel_z = 0.0;

    init_pos_set = 1;
  }

  previous_gnss_pose.x = current_gnss_pose.x;
  previous_gnss_pose.y = current_gnss_pose.y;
  previous_gnss_pose.z = current_gnss_pose.z;
  previous_gnss_pose.roll = current_gnss_pose.roll;
  previous_gnss_pose.pitch = current_gnss_pose.pitch;
  previous_gnss_pose.yaw = current_gnss_pose.yaw;
  previous_gnss_time = current_gnss_time;
}
*/
static void initialpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input)
{
  tf::TransformListener listener;
  tf::StampedTransform transform;
  try
  {
    ros::Time now = ros::Time(0);
    listener.waitForTransform("/map", input->header.frame_id, now, ros::Duration(10.0));
    listener.lookupTransform("/map", input->header.frame_id, now, transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  tf::Quaternion q(input->pose.pose.orientation.x, input->pose.pose.orientation.y, input->pose.pose.orientation.z,
                   input->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);

  if (_use_local_transform == true)
  {
    current_pose.x = input->pose.pose.position.x;
    current_pose.y = input->pose.pose.position.y;
    current_pose.z = input->pose.pose.position.z;
  }
  else
  {
    current_pose.x = input->pose.pose.position.x + transform.getOrigin().x();
    current_pose.y = input->pose.pose.position.y + transform.getOrigin().y();
    current_pose.z = input->pose.pose.position.z + transform.getOrigin().z();
  }
  m.getRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);

  if (_get_height == true && map_loaded == 1)
  {
    double min_distance = DBL_MAX;
    double nearest_z = current_pose.z;
    for (const auto& p : map)
    {
      double distance = hypot(current_pose.x - p.x, current_pose.y - p.y);
      if (distance < min_distance)
      {
        min_distance = distance;
        nearest_z = p.z;
      }
    }
    current_pose.z = nearest_z;
  }

  current_pose_imu = current_pose_odom = current_pose_imu_odom = current_pose;
  previous_pose.x = current_pose.x;
  previous_pose.y = current_pose.y;
  previous_pose.z = current_pose.z;
  previous_pose.roll = current_pose.roll;
  previous_pose.pitch = current_pose.pitch;
  previous_pose.yaw = current_pose.yaw;

  current_velocity = 0.0;
  current_velocity_x = 0.0;
  current_velocity_y = 0.0;
  current_velocity_z = 0.0;
  angular_velocity = 0.0;

  current_accel = 0.0;
  current_accel_x = 0.0;
  current_accel_y = 0.0;
  current_accel_z = 0.0;

  offset_x = 0.0;
  offset_y = 0.0;
  offset_z = 0.0;
  offset_yaw = 0.0;

  offset_imu_x = 0.0;
  offset_imu_y = 0.0;
  offset_imu_z = 0.0;
  offset_imu_roll = 0.0;
  offset_imu_pitch = 0.0;
  offset_imu_yaw = 0.0;

  offset_odom_x = 0.0;
  offset_odom_y = 0.0;
  offset_odom_z = 0.0;
  offset_odom_roll = 0.0;
  offset_odom_pitch = 0.0;
  offset_odom_yaw = 0.0;

  offset_imu_odom_x = 0.0;
  offset_imu_odom_y = 0.0;
  offset_imu_odom_z = 0.0;
  offset_imu_odom_roll = 0.0;
  offset_imu_odom_pitch = 0.0;
  offset_imu_odom_yaw = 0.0;

  init_pos_set = 1;
}

static void set_initial_parameters(Eigen::Vector3d initial_pose_input, Eigen::Vector3d initial_attitude_input)
{
//previous_pose.yaw = -180/180*M_PI; // initial yaw definition glt
    previous_pose.x = initial_pose_input(0);
    previous_pose.y = initial_pose_input(1);
    previous_pose.z = initial_pose_input(2);
    previous_pose.roll = initial_attitude_input(0);
    previous_pose.pitch = initial_attitude_input(1);
    previous_pose.yaw = initial_attitude_input(2);

    current_pose.x = initial_pose_input(0);
    current_pose.y = initial_pose_input(1);
    current_pose.z = initial_pose_input(2);
    current_pose.roll = initial_attitude_input(0);
    current_pose.pitch = initial_attitude_input(1);
    current_pose.yaw = initial_attitude_input(2);
    previous_pose.yaw = 0/180*M_PI;//0312
}

static void imu_odom_calc(ros::Time current_time)
{
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_imu_roll = imu.angular_velocity.x * diff_time;
  double diff_imu_pitch = imu.angular_velocity.y * diff_time;
  double diff_imu_yaw = imu.angular_velocity.z * diff_time;

  current_pose_imu_odom.roll += diff_imu_roll;
  current_pose_imu_odom.pitch += diff_imu_pitch;
  current_pose_imu_odom.yaw += diff_imu_yaw;

  double diff_distance = odom.twist.twist.linear.x * diff_time;
  offset_imu_odom_x += diff_distance * cos(-current_pose_imu_odom.pitch) * cos(current_pose_imu_odom.yaw);
  offset_imu_odom_y += diff_distance * cos(-current_pose_imu_odom.pitch) * sin(current_pose_imu_odom.yaw);
  offset_imu_odom_z += diff_distance * sin(-current_pose_imu_odom.pitch);

  offset_imu_odom_roll += diff_imu_roll;
  offset_imu_odom_pitch += diff_imu_pitch;
  offset_imu_odom_yaw += diff_imu_yaw;

  predict_pose_imu_odom.x = previous_pose.x + offset_imu_odom_x;
  predict_pose_imu_odom.y = previous_pose.y + offset_imu_odom_y;
  predict_pose_imu_odom.z = previous_pose.z + offset_imu_odom_z;
  predict_pose_imu_odom.roll = previous_pose.roll + offset_imu_odom_roll;
  predict_pose_imu_odom.pitch = previous_pose.pitch + offset_imu_odom_pitch;
  predict_pose_imu_odom.yaw = previous_pose.yaw + offset_imu_odom_yaw;

  previous_time = current_time;
}

static void odom_calc(ros::Time current_time)
{
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_odom_roll = odom.twist.twist.angular.x * diff_time;
  double diff_odom_pitch = odom.twist.twist.angular.y * diff_time;
  double diff_odom_yaw = odom.twist.twist.angular.z * diff_time;

  current_pose_odom.roll += diff_odom_roll;
  current_pose_odom.pitch += diff_odom_pitch;
  current_pose_odom.yaw += diff_odom_yaw;

  double diff_distance = odom.twist.twist.linear.x * diff_time;
  offset_odom_x += diff_distance * cos(-current_pose_odom.pitch) * cos(current_pose_odom.yaw);
  offset_odom_y += diff_distance * cos(-current_pose_odom.pitch) * sin(current_pose_odom.yaw);
  offset_odom_z += diff_distance * sin(-current_pose_odom.pitch);

  offset_odom_roll += diff_odom_roll;
  offset_odom_pitch += diff_odom_pitch;
  offset_odom_yaw += diff_odom_yaw;

  predict_pose_odom.x = previous_pose.x + offset_odom_x;
  predict_pose_odom.y = previous_pose.y + offset_odom_y;
  predict_pose_odom.z = previous_pose.z + offset_odom_z;
  predict_pose_odom.roll = previous_pose.roll + offset_odom_roll;
  predict_pose_odom.pitch = previous_pose.pitch + offset_odom_pitch;
  predict_pose_odom.yaw = previous_pose.yaw + offset_odom_yaw;

  previous_time = current_time;
}

static void imu_calc(ros::Time current_time)
{
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_imu_roll = imu.angular_velocity.x * diff_time;
  double diff_imu_pitch = imu.angular_velocity.y * diff_time;
  double diff_imu_yaw = imu.angular_velocity.z * diff_time;

  current_pose_imu.roll += diff_imu_roll;
  current_pose_imu.pitch += diff_imu_pitch;
  current_pose_imu.yaw += diff_imu_yaw;

  double accX1 = imu.linear_acceleration.x;
  double accY1 = std::cos(current_pose_imu.roll) * imu.linear_acceleration.y -
                 std::sin(current_pose_imu.roll) * imu.linear_acceleration.z;
  double accZ1 = std::sin(current_pose_imu.roll) * imu.linear_acceleration.y +
                 std::cos(current_pose_imu.roll) * imu.linear_acceleration.z;

  double accX2 = std::sin(current_pose_imu.pitch) * accZ1 + std::cos(current_pose_imu.pitch) * accX1;
  double accY2 = accY1;
  double accZ2 = std::cos(current_pose_imu.pitch) * accZ1 - std::sin(current_pose_imu.pitch) * accX1;

  double accX = std::cos(current_pose_imu.yaw) * accX2 - std::sin(current_pose_imu.yaw) * accY2;
  double accY = std::sin(current_pose_imu.yaw) * accX2 + std::cos(current_pose_imu.yaw) * accY2;
  double accZ = accZ2;

  offset_imu_x += current_velocity_imu_x * diff_time + accX * diff_time * diff_time / 2.0;
  offset_imu_y += current_velocity_imu_y * diff_time + accY * diff_time * diff_time / 2.0;
  offset_imu_z += current_velocity_imu_z * diff_time + accZ * diff_time * diff_time / 2.0;

  current_velocity_imu_x += accX * diff_time;
  current_velocity_imu_y += accY * diff_time;
  current_velocity_imu_z += accZ * diff_time;

  offset_imu_roll += diff_imu_roll;
  offset_imu_pitch += diff_imu_pitch;
  offset_imu_yaw += diff_imu_yaw;

  predict_pose_imu.x = previous_pose.x + offset_imu_x;
  predict_pose_imu.y = previous_pose.y + offset_imu_y;
  predict_pose_imu.z = previous_pose.z + offset_imu_z;
  predict_pose_imu.roll = previous_pose.roll + offset_imu_roll;
  predict_pose_imu.pitch = previous_pose.pitch + offset_imu_pitch;
  predict_pose_imu.yaw = previous_pose.yaw + offset_imu_yaw;

  previous_time = current_time;
}

static double wrapToPm(double a_num, const double a_max)
{
  if (a_num >= a_max)
  {
    a_num -= 2.0 * a_max;
  }
  return a_num;
}

static double wrapToPmPi(const double a_angle_rad)
{
  return wrapToPm(a_angle_rad, M_PI);
}

static double calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
  double diff_rad = lhs_rad - rhs_rad;
  if (diff_rad >= M_PI)
    diff_rad = diff_rad - 2 * M_PI;
  else if (diff_rad < -M_PI)
    diff_rad = diff_rad + 2 * M_PI;
  return diff_rad;
}
/*
static void odom_callback(const nav_msgs::Odometry::ConstPtr& input)
{
  // std::cout << __func__ << std::endl;

  odom = *input;
  odom_calc(input->header.stamp);
}

static void imuUpsideDown(const sensor_msgs::Imu::Ptr input)
{
  double input_roll, input_pitch, input_yaw;

  tf::Quaternion input_orientation;
  tf::quaternionMsgToTF(input->orientation, input_orientation);
  tf::Matrix3x3(input_orientation).getRPY(input_roll, input_pitch, input_yaw);

  input->angular_velocity.x *= -1;
  input->angular_velocity.y *= -1;
  input->angular_velocity.z *= -1;

  input->linear_acceleration.x *= -1;
  input->linear_acceleration.y *= -1;
  input->linear_acceleration.z *= -1;

  input_roll *= -1;
  input_pitch *= -1;
  input_yaw *= -1;

  input->orientation = tf::createQuaternionMsgFromRollPitchYaw(input_roll, input_pitch, input_yaw);
}

static void imu_callback(const sensor_msgs::Imu::Ptr& input)
{
  // std::cout << __func__ << std::endl;

  if (_imu_upside_down)
    imuUpsideDown(input);

  const ros::Time current_time = input->header.stamp;
  static ros::Time previous_time = current_time;
  const double diff_time = (current_time - previous_time).toSec();

  double imu_roll, imu_pitch, imu_yaw;
  tf::Quaternion imu_orientation;
  tf::quaternionMsgToTF(input->orientation, imu_orientation);
  tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);

  imu_roll = wrapToPmPi(imu_roll);
  imu_pitch = wrapToPmPi(imu_pitch);
  imu_yaw = wrapToPmPi(imu_yaw);

  static double previous_imu_roll = imu_roll, previous_imu_pitch = imu_pitch, previous_imu_yaw = imu_yaw;
  const double diff_imu_roll = calcDiffForRadian(imu_roll, previous_imu_roll);
  const double diff_imu_pitch = calcDiffForRadian(imu_pitch, previous_imu_pitch);
  const double diff_imu_yaw = calcDiffForRadian(imu_yaw, previous_imu_yaw);

  imu.header = input->header;
  imu.linear_acceleration.x = input->linear_acceleration.x;
  // imu.linear_acceleration.y = input->linear_acceleration.y;
  // imu.linear_acceleration.z = input->linear_acceleration.z;
  imu.linear_acceleration.y = 0;
  imu.linear_acceleration.z = 0;

  if (diff_time != 0)
  {
    imu.angular_velocity.x = diff_imu_roll / diff_time;
    imu.angular_velocity.y = diff_imu_pitch / diff_time;
    imu.angular_velocity.z = diff_imu_yaw / diff_time;
  }
  else
  {
    imu.angular_velocity.x = 0;
    imu.angular_velocity.y = 0;
    imu.angular_velocity.z = 0;
  }

  imu_calc(input->header.stamp);

  previous_time = current_time;
  previous_imu_roll = imu_roll;
  previous_imu_pitch = imu_pitch;
  previous_imu_yaw = imu_yaw;
}
*/

bool gnss_init_ = false;
bool kfinitial_ = false;
bool ndt_change_flg = false;
Eigen::Vector3d zero_utm(0,0,0);
// definition
Eigen::Vector3d Xkk_1(0,0,0), Xk_1(0,0,0), Zk(0,0,0),Xk(0,0,0);
Eigen::Matrix3d Hk, Pkk_1= Eigen::Matrix3d::Identity(), Pk_1=Eigen::Matrix3d::Identity(),Qk= Eigen::Matrix3d::Zero();
Eigen::Matrix3d Rk = Eigen::Matrix3d::Zero(),Kk=Eigen::Matrix3d::Zero(),Pk=Eigen::Matrix3d::Identity();
ros::Time gnss_time_cur_fusion;
ros::Time gnss_time_pre_fusion;
uint insstatus_ = 0;
uint fusionStatus = 0;
//Eigen::Vector3d fusionResult(.0,.0,.0);
Eigen::Vector3d errorResult(.0,.0,.0);
Eigen::Vector3d xyz_lever(0, 0, 0),ndt_xyz(0,0,0),ndt_xyz_pre(0,0,0),fusion_ins_xyz(0,0,0);
static pose fusion_pose, fusion_pose_subed;
static double yaw_normal(double yaw_gnss)
{
  //input unit should be degree, output is degree
  // transform GNSS yaw to ENU yaw. Novatel GNSS yaw is 0-360, north0,clock wise positive. 
  //ENU yaw is -180-180, east 0, counter-clock wise is positive
  double yaw = 0;
  if (yaw_gnss <= 270)
  {
    yaw = 90 - yaw_gnss;
  }
  else
  {
    yaw = 450-yaw_gnss;
  }
  return yaw;
}
static void filterUpdate()
{
    Eigen::Matrix< double, 3, 3 > PHT;    // 3*3,3*3
    Eigen::Matrix< double, 3, 3 > HPHTR;  // 3*3,3*3,3*3,3*3

    PHT = Pkk_1 * Hk.transpose();  // Pkk_1*H';
    HPHTR = Hk * PHT + Rk;         // H*Pkk_1*H'+R;
    Kk = PHT * HPHTR.inverse();         // K=P*H'*(H*P*H'+R)^-1;

    Eigen::Vector3d innovation;
    innovation = Zk - Hk * Xkk_1;  // Zk-Hk*Xkk_1

    // Xk, Pk
    Xk = Xkk_1 + Kk * innovation;
    Pk = Pkk_1 - Kk * Hk * Pkk_1;
    Pk = 0.5 * (Pk + Pk.transpose());
}

//static void callback(const novatel_oem7_msgs::INSPVAConstPtr& input_gnss,
//                                      const sensor_msgs::PointCloud2ConstPtr& input_lidar,
//                                      const geometry_msgs::PoseStampedConstPtr& input_dr_fusion
//                                      //const geometry_msgs::PoseStampedConstPtr& input_ndtpose,
//                                      //const autoware_msgs::NDTStatConstPtr& input_ndtstat
//                                      )
//static void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input_lidar)

ros::Time lidar_current_time;
ros::Time lidar_previous_time;
static void rs_callback(const sensor_msgs::PointCloud2ConstPtr& input_lidar,
                        const geometry_msgs::PoseStampedConstPtr& input_fusion_pose) //0312 datacollection pretest
{
   lidar_current_time = input_lidar->header.stamp;
   float lidar_diff_time = (lidar_current_time-lidar_previous_time).toSec();
   if (lidar_diff_time<0.8)
   {
    return;
   }
   else
   {
    lidar_previous_time = lidar_current_time;
   }


   if (a<2)
  {
    if (fitness_score < 0.9 && fitness_score>0.01 && iteration <20)
    {
      a++;
    }
  }
  else
  {
    a=4;
    
  }
   static pose fusion_pose_subed;
   ros::Time input_fusion_pos_time = input_fusion_pose->header.stamp;
   fusion_pose_subed.x =  input_fusion_pose->pose.position.x;
   fusion_pose_subed.y =  input_fusion_pose->pose.position.y;
   fusion_pose_subed.z =  input_fusion_pose->pose.position.z;
   tf::Quaternion fusion_pose_subed_q(input_fusion_pose->pose.orientation.x, input_fusion_pose->pose.orientation.y,input_fusion_pose->pose.orientation.z,
                        input_fusion_pose->pose.orientation.w);
   tf::Matrix3x3 fusion_pose_subed_m(fusion_pose_subed_q);
   fusion_pose_subed_m.getRPY(fusion_pose_subed.roll,fusion_pose_subed.pitch, fusion_pose_subed.yaw);
   //tf::Quaternion q_fusion_subed;
   std::cout<<"fusion pose subed x in NDT:"<< fusion_pose_subed.x<<std::endl;
   std::cout<<"a:"<<a<<std::endl;
  if (/*a>3 &&*/ abs(fusion_pose_subed.x)>0.01)
  {
    current_pose.x = fusion_pose_subed.x;
    current_pose.y = fusion_pose_subed.y;
    current_pose.z = fusion_pose_subed.z;
    current_pose.roll = fusion_pose_subed.roll;
    current_pose.pitch = fusion_pose_subed.pitch;
    current_pose.yaw = fusion_pose_subed.yaw;
    
    
    //q_fusion_subed.setRPY(fusion_pose_subed.roll, fusion_pose_subed.pitch, fusion_pose_subed.yaw);

    // previous_pose.x = current_pose.x;
    // previous_pose.y = current_pose.y;
    // previous_pose.z = current_pose.z;
    // previous_pose.roll = current_pose.roll;
    // previous_pose.pitch = current_pose.pitch;
    // previous_pose.yaw = current_pose.yaw; 
  }
  
  
  
  // set initial parameters
  if (init_pos_set==0)
  {
    //std::cout<<"enter init pos set"<<std::endl;
    if (_init_post_set_fromGNSS==1)
    {
      if (insstatus_ == 4 || insstatus_ == 5)
      {
        set_initial_parameters(gps_raw_local,gps_attitude_local);
        init_pos_set=1;
      }
    }
    else if (_init_post_set_fromLAUNCH==1) // read initial values from launch file
    {
      set_initial_parameters(initial_pose_local,initial_attitude_local);
      init_pos_set=1;
    }
  }
  //std::cout<<"init_pos_set="<<init_pos_set<<std::endl;
  //std::cout << "after paraInit previous_pose_x:"<<previous_pose.x<<" y:"<<previous_pose.y<<" z:"<<previous_pose.z<<" yaw:"<<previous_pose.yaw<<std::endl;
  
  std::cout << "At points call back first line previous_pose_x:"<<previous_pose.x<<";previous_pose_yaw:"<<previous_pose.yaw<<std::endl;
  
  if  (map_pub)
  {
    pcl::toROSMsg(*map_saved_cloud, saved_map_cloud);
    saved_map_cloud.header.frame_id = "/map";
    saved_map_cloud.header.stamp = ros::Time::now();
    std::cout<<"stamp"<<rawPointCouldStamp<<std::endl;
    saved_map_cloud_pub.publish(saved_map_cloud);
    map_pub=false;

  }
  

  rawPointCouldStamp = input_lidar->header.stamp;
  rawPointCouldStamp_db=rawPointCouldStamp.toSec();
  //node_status_publisher_ptr_->CHECK_RATE("/topic/rate/points_raw/slow",8,5,1,"topic points_raw subscribe rate low.");
  //std::cout << "enter points callback" << std::endl;
  if (map_loaded == 1 && init_pos_set == 1)
  {
    //std::cout << "enter points callback processing" << std::endl;
    matching_start = std::chrono::system_clock::now();

    static tf::TransformBroadcaster br;
    tf::Transform transform;

    static tf::TransformBroadcaster br_velodyne;
    tf::Transform transform_velodyne;

    tf::Quaternion predict_q, ndt_q, current_q, localizer_q;

    pcl::PointXYZ p;
    pcl::PointCloud<pcl::PointXYZ> filtered_scan;

    ros::Time current_scan_time = input_lidar->header.stamp;
    static ros::Time previous_scan_time = current_scan_time;

    std::vector<int> indices;  //remove nan
    
    
    //down sample 0324
    pcl::PCLPointCloud2* ori_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr ori_cloudPtr(ori_cloud);
    pcl::PCLPointCloud2 msg_downsampled;
    pcl_conversions::toPCL(*input_lidar, *ori_cloud);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(ori_cloudPtr);
    sor.setLeafSize(0.6f,0.6f,0.6f);
    sor.filter(msg_downsampled);
    sensor_msgs::PointCloud2 point_downsampled_output;
    pcl_conversions::fromPCL(msg_downsampled, point_downsampled_output);
    //points_pub.publish(point_downsampled_output);

    //pcl::fromROSMsg(*input_lidar, filtered_scan);
    pcl::fromROSMsg(point_downsampled_output, filtered_scan);
    //filtered_scan = msg_downsampled;
    //pcl::fromPCLPointCloud2(const pcl::PCLPointCloud2 &msg_downsampled pcl::PointCloud<pcl::PointXYZ> filtered_scan)
    //down sample 0324 end
    

    //if do not downsample, comment out the above section, use the following one line:
    //pcl::fromROSMsg(*input_lidar, filtered_scan);// if no downsample, only use this, if downsample, comment out this line

    pcl::removeNaNFromPointCloud(filtered_scan, filtered_scan, indices); //remove nan
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(filtered_scan));
    int scan_points_num = filtered_scan_ptr->size();

    Eigen::Matrix4f t(Eigen::Matrix4f::Identity());   // base_link
    Eigen::Matrix4f t2(Eigen::Matrix4f::Identity());  // localizer

    std::chrono::time_point<std::chrono::system_clock> align_start, align_end, getFitnessScore_start,
        getFitnessScore_end;
    static double align_time, getFitnessScore_time = 0.0;

    pthread_mutex_lock(&mutex);

    if (_method_type == MethodType::PCL_GENERIC)
      ndt.setInputSource(filtered_scan_ptr);
    else if (_method_type == MethodType::PCL_ANH)
      anh_ndt.setInputSource(filtered_scan_ptr);
#ifdef CUDA_FOUND
    else if (_method_type == MethodType::PCL_ANH_GPU)
      anh_gpu_ndt_ptr->setInputSource(filtered_scan_ptr);
#endif
#ifdef USE_PCL_OPENMP
    else if (_method_type == MethodType::PCL_OPENMP)
      omp_ndt.setInputSource(filtered_scan_ptr);
#endif

    // Guess the initial gross estimation of the transformation
    double diff_time = (current_scan_time - previous_scan_time).toSec();

    if (_offset == "linear")
    {
      offset_x = current_velocity_x * diff_time;
      offset_y = current_velocity_y * diff_time;
      offset_z = current_velocity_z * diff_time;
      offset_yaw = angular_velocity * diff_time;
    }
    else if (_offset == "quadratic")
    {
      offset_x = (current_velocity_x + current_accel_x * diff_time) * diff_time;
      offset_y = (current_velocity_y + current_accel_y * diff_time) * diff_time;
      offset_z = current_velocity_z * diff_time;
      offset_yaw = angular_velocity * diff_time;
    }
    else if (_offset == "zero")
    {
      offset_x = 0.0;
      offset_y = 0.0;
      offset_z = 0.0;
      offset_yaw = 0.0;
    }

    predict_pose.x = previous_pose.x + offset_x;
    predict_pose.y = previous_pose.y + offset_y;
    predict_pose.z = previous_pose.z + offset_z;
    predict_pose.roll = previous_pose.roll;
    predict_pose.pitch = previous_pose.pitch;
    predict_pose.yaw = previous_pose.yaw + offset_yaw;

    if (_use_imu == true && _use_odom == true)
      imu_odom_calc(current_scan_time);
    if (_use_imu == true && _use_odom == false)
      imu_calc(current_scan_time);
    if (_use_imu == false && _use_odom == true)
      odom_calc(current_scan_time);

    pose predict_pose_for_ndt;
    if (_use_imu == true && _use_odom == true)
      predict_pose_for_ndt = predict_pose_imu_odom;
    else if (_use_imu == true && _use_odom == false)
      predict_pose_for_ndt = predict_pose_imu;
    else if (_use_imu == false && _use_odom == true)
      predict_pose_for_ndt = predict_pose_odom;
    else
      predict_pose_for_ndt = predict_pose;
/*
    Eigen::Translation3f init_translation(predict_pose_for_ndt.x, predict_pose_for_ndt.y, predict_pose_for_ndt.z);
    Eigen::AngleAxisf init_rotation_x(predict_pose_for_ndt.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(predict_pose_for_ndt.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(predict_pose_for_ndt.yaw, Eigen::Vector3f::UnitZ());
*/
    Eigen::Translation3f init_translation(current_pose.x, current_pose.y, current_pose.z);
    Eigen::AngleAxisf init_rotation_x(current_pose.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(current_pose.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(current_pose.yaw, Eigen::Vector3f::UnitZ());

    Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x) * tf_btol;

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    if (_method_type == MethodType::PCL_GENERIC)
    {
      
      align_start = std::chrono::system_clock::now();
      //std::cout << "breakpoint3" << std::endl;
      std::cout << "init_guess:"<< init_guess << std::endl;
      std::cout << "*output_cloud:"<< *output_cloud << std::endl;
      ndt.align(*output_cloud, init_guess);
      
      //std::cout << "breakpoint4" << std::endl;
      align_end = std::chrono::system_clock::now();

      has_converged = ndt.hasConverged();

      t = ndt.getFinalTransformation();
      iteration = ndt.getFinalNumIteration();

      getFitnessScore_start = std::chrono::system_clock::now();
      fitness_score = ndt.getFitnessScore();
      getFitnessScore_end = std::chrono::system_clock::now();

      trans_probability = ndt.getTransformationProbability();
    }
    else if (_method_type == MethodType::PCL_ANH)
    {
      std::cout << "breakpoint3333" << std::endl;
      align_start = std::chrono::system_clock::now();
      anh_ndt.align(init_guess);
      align_end = std::chrono::system_clock::now();

      has_converged = anh_ndt.hasConverged();

      t = anh_ndt.getFinalTransformation();
      iteration = anh_ndt.getFinalNumIteration();

      getFitnessScore_start = std::chrono::system_clock::now();
      fitness_score = anh_ndt.getFitnessScore();
      getFitnessScore_end = std::chrono::system_clock::now();

      trans_probability = anh_ndt.getTransformationProbability();
    }
    
#ifdef CUDA_FOUND
    else if (_method_type == MethodType::PCL_ANH_GPU)
    {
      align_start = std::chrono::system_clock::now();
      anh_gpu_ndt_ptr->align(init_guess);
      align_end = std::chrono::system_clock::now();

      has_converged = anh_gpu_ndt_ptr->hasConverged();

      t = anh_gpu_ndt_ptr->getFinalTransformation();
      iteration = anh_gpu_ndt_ptr->getFinalNumIteration();

      getFitnessScore_start = std::chrono::system_clock::now();
      fitness_score = anh_gpu_ndt_ptr->getFitnessScore();
      getFitnessScore_end = std::chrono::system_clock::now();

      trans_probability = anh_gpu_ndt_ptr->getTransformationProbability();
    }
#endif
#ifdef USE_PCL_OPENMP
    else if (_method_type == MethodType::PCL_OPENMP)
    {
      align_start = std::chrono::system_clock::now();
      omp_ndt.align(*output_cloud, init_guess);
      align_end = std::chrono::system_clock::now();

      has_converged = omp_ndt.hasConverged();

      t = omp_ndt.getFinalTransformation();
      iteration = omp_ndt.getFinalNumIteration();

      getFitnessScore_start = std::chrono::system_clock::now();
      fitness_score = omp_ndt.getFitnessScore();
      getFitnessScore_end = std::chrono::system_clock::now();

      trans_probability = omp_ndt.getTransformationProbability();
    }
#endif
    align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end - align_start).count() / 1000.0;

    t2 = t * tf_btol.inverse();

    getFitnessScore_time =
        std::chrono::duration_cast<std::chrono::microseconds>(getFitnessScore_end - getFitnessScore_start).count() /
        1000.0;

    pthread_mutex_unlock(&mutex);
    
    tf::Matrix3x3 mat_l;  // localizer
    mat_l.setValue(static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)),
                   static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
                   static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));

    // Update localizer_pose
    localizer_pose.x = t(0, 3);
    localizer_pose.y = t(1, 3);
    localizer_pose.z = t(2, 3);
    mat_l.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw, 1);

    tf::Matrix3x3 mat_b;  // base_link
    mat_b.setValue(static_cast<double>(t2(0, 0)), static_cast<double>(t2(0, 1)), static_cast<double>(t2(0, 2)),
                   static_cast<double>(t2(1, 0)), static_cast<double>(t2(1, 1)), static_cast<double>(t2(1, 2)),
                   static_cast<double>(t2(2, 0)), static_cast<double>(t2(2, 1)), static_cast<double>(t2(2, 2)));

    // Update ndt_pose
    ndt_pose.x = t2(0, 3);
    ndt_pose.y = t2(1, 3);
    ndt_pose.z = t2(2, 3);
    mat_b.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1);

    // Calculate the difference between ndt_pose and predict_pose
    predict_pose_error = sqrt((ndt_pose.x - predict_pose_for_ndt.x) * (ndt_pose.x - predict_pose_for_ndt.x) +
                              (ndt_pose.y - predict_pose_for_ndt.y) * (ndt_pose.y - predict_pose_for_ndt.y) +
                              (ndt_pose.z - predict_pose_for_ndt.z) * (ndt_pose.z - predict_pose_for_ndt.z));

    if (predict_pose_error <= PREDICT_POSE_THRESHOLD)
    {
      use_predict_pose = 0;
    }
    else
    {
      use_predict_pose = 1;
    }
    use_predict_pose = 0;

    if (use_predict_pose == 0)
    {
      current_pose.x = ndt_pose.x;
      current_pose.y = ndt_pose.y;
      current_pose.z = ndt_pose.z;
      current_pose.roll = ndt_pose.roll;
      current_pose.pitch = ndt_pose.pitch;
      current_pose.yaw = ndt_pose.yaw;
    }
    else
    {
      current_pose.x = predict_pose_for_ndt.x;
      current_pose.y = predict_pose_for_ndt.y;
      current_pose.z = predict_pose_for_ndt.z;
      current_pose.roll = predict_pose_for_ndt.roll;
      current_pose.pitch = predict_pose_for_ndt.pitch;
      current_pose.yaw = predict_pose_for_ndt.yaw;
    }

    // Compute the velocity and acceleration
    diff_x = current_pose.x - previous_pose.x;
    diff_y = current_pose.y - previous_pose.y;
    diff_z = current_pose.z - previous_pose.z;
    diff_yaw = calcDiffForRadian(current_pose.yaw, previous_pose.yaw);
    diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
    
    const pose trans_current_pose = convertPoseIntoRelativeCoordinate(current_pose, previous_pose);

    current_velocity = (diff_time > 0) ? (diff / diff_time) : 0;
    current_velocity =  (trans_current_pose.x >= 0) ? current_velocity : -current_velocity;
    current_velocity_x = (diff_time > 0) ? (diff_x / diff_time) : 0;
    current_velocity_y = (diff_time > 0) ? (diff_y / diff_time) : 0;
    current_velocity_z = (diff_time > 0) ? (diff_z / diff_time) : 0;
    angular_velocity = (diff_time > 0) ? (diff_yaw / diff_time) : 0;

    current_pose_imu.x = current_pose.x;
    current_pose_imu.y = current_pose.y;
    current_pose_imu.z = current_pose.z;
    current_pose_imu.roll = current_pose.roll;
    current_pose_imu.pitch = current_pose.pitch;
    current_pose_imu.yaw = current_pose.yaw;

    current_velocity_imu_x = current_velocity_x;
    current_velocity_imu_y = current_velocity_y;
    current_velocity_imu_z = current_velocity_z;

    current_pose_odom.x = current_pose.x;
    current_pose_odom.y = current_pose.y;
    current_pose_odom.z = current_pose.z;
    current_pose_odom.roll = current_pose.roll;
    current_pose_odom.pitch = current_pose.pitch;
    current_pose_odom.yaw = current_pose.yaw;

    current_pose_imu_odom.x = current_pose.x;
    current_pose_imu_odom.y = current_pose.y;
    current_pose_imu_odom.z = current_pose.z;
    current_pose_imu_odom.roll = current_pose.roll;
    current_pose_imu_odom.pitch = current_pose.pitch;
    current_pose_imu_odom.yaw = current_pose.yaw;

    current_velocity_smooth = (current_velocity + previous_velocity + previous_previous_velocity) / 3.0;
    if (std::fabs(current_velocity_smooth) < 0.2)
    {
      current_velocity_smooth = 0.0;
    }

    current_accel = (diff_time > 0) ? ((current_velocity - previous_velocity) / diff_time) : 0;
    current_accel_x = (diff_time > 0) ? ((current_velocity_x - previous_velocity_x) / diff_time) : 0;
    current_accel_y = (diff_time > 0) ? ((current_velocity_y - previous_velocity_y) / diff_time) : 0;
    current_accel_z = (diff_time > 0) ? ((current_velocity_z - previous_velocity_z) / diff_time) : 0;

    estimated_vel_mps.data = current_velocity;
    estimated_vel_kmph.data = current_velocity * 3.6;

    estimated_vel_mps_pub.publish(estimated_vel_mps);
    estimated_vel_kmph_pub.publish(estimated_vel_kmph);
    
    // Set values for publishing pose
    predict_q.setRPY(predict_pose.roll, predict_pose.pitch, predict_pose.yaw);
    if (_use_local_transform == true)
    {
      tf::Vector3 v(predict_pose.x, predict_pose.y, predict_pose.z);
      tf::Transform transform(predict_q, v);
      predict_pose_msg.header.frame_id = "/map";
      predict_pose_msg.header.stamp = current_scan_time;
      predict_pose_msg.pose.position.x = (local_transform * transform).getOrigin().getX();
      predict_pose_msg.pose.position.y = (local_transform * transform).getOrigin().getY();
      predict_pose_msg.pose.position.z = (local_transform * transform).getOrigin().getZ();
      predict_pose_msg.pose.orientation.x = (local_transform * transform).getRotation().x();
      predict_pose_msg.pose.orientation.y = (local_transform * transform).getRotation().y();
      predict_pose_msg.pose.orientation.z = (local_transform * transform).getRotation().z();
      predict_pose_msg.pose.orientation.w = (local_transform * transform).getRotation().w();
    }
    else
    {
      predict_pose_msg.header.frame_id = "/map";
      predict_pose_msg.header.stamp = current_scan_time;
      predict_pose_msg.pose.position.x = predict_pose.x;
      predict_pose_msg.pose.position.y = predict_pose.y;
      predict_pose_msg.pose.position.z = predict_pose.z;
      predict_pose_msg.pose.orientation.x = predict_q.x();
      predict_pose_msg.pose.orientation.y = predict_q.y();
      predict_pose_msg.pose.orientation.z = predict_q.z();
      predict_pose_msg.pose.orientation.w = predict_q.w();
    }

    tf::Quaternion predict_q_imu;
    predict_q_imu.setRPY(predict_pose_imu.roll, predict_pose_imu.pitch, predict_pose_imu.yaw);
    predict_pose_imu_msg.header.frame_id = "map";
    predict_pose_imu_msg.header.stamp = input_lidar->header.stamp;
    predict_pose_imu_msg.pose.position.x = predict_pose_imu.x;
    predict_pose_imu_msg.pose.position.y = predict_pose_imu.y;
    predict_pose_imu_msg.pose.position.z = predict_pose_imu.z;
    predict_pose_imu_msg.pose.orientation.x = predict_q_imu.x();
    predict_pose_imu_msg.pose.orientation.y = predict_q_imu.y();
    predict_pose_imu_msg.pose.orientation.z = predict_q_imu.z();
    predict_pose_imu_msg.pose.orientation.w = predict_q_imu.w();
    predict_pose_imu_pub.publish(predict_pose_imu_msg);

    tf::Quaternion predict_q_odom;
    predict_q_odom.setRPY(predict_pose_odom.roll, predict_pose_odom.pitch, predict_pose_odom.yaw);
    predict_pose_odom_msg.header.frame_id = "map";
    predict_pose_odom_msg.header.stamp = input_lidar->header.stamp;
    predict_pose_odom_msg.pose.position.x = predict_pose_odom.x;
    predict_pose_odom_msg.pose.position.y = predict_pose_odom.y;
    predict_pose_odom_msg.pose.position.z = predict_pose_odom.z;
    predict_pose_odom_msg.pose.orientation.x = predict_q_odom.x();
    predict_pose_odom_msg.pose.orientation.y = predict_q_odom.y();
    predict_pose_odom_msg.pose.orientation.z = predict_q_odom.z();
    predict_pose_odom_msg.pose.orientation.w = predict_q_odom.w();
    predict_pose_odom_pub.publish(predict_pose_odom_msg);

    tf::Quaternion predict_q_imu_odom;
    predict_q_imu_odom.setRPY(predict_pose_imu_odom.roll, predict_pose_imu_odom.pitch, predict_pose_imu_odom.yaw);
    predict_pose_imu_odom_msg.header.frame_id = "map";
    predict_pose_imu_odom_msg.header.stamp = input_lidar->header.stamp;
    predict_pose_imu_odom_msg.pose.position.x = predict_pose_imu_odom.x;
    predict_pose_imu_odom_msg.pose.position.y = predict_pose_imu_odom.y;
    predict_pose_imu_odom_msg.pose.position.z = predict_pose_imu_odom.z;
    predict_pose_imu_odom_msg.pose.orientation.x = predict_q_imu_odom.x();
    predict_pose_imu_odom_msg.pose.orientation.y = predict_q_imu_odom.y();
    predict_pose_imu_odom_msg.pose.orientation.z = predict_q_imu_odom.z();
    predict_pose_imu_odom_msg.pose.orientation.w = predict_q_imu_odom.w();
    predict_pose_imu_odom_pub.publish(predict_pose_imu_odom_msg);

    ndt_q.setRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw);
    if (_use_local_transform == true)
    {
      tf::Vector3 v(ndt_pose.x, ndt_pose.y, ndt_pose.z);
      tf::Transform transform(ndt_q, v);
      ndt_pose_msg.header.frame_id = "/map";
      ndt_pose_msg.header.stamp = current_scan_time;
      ndt_pose_msg.pose.position.x = (local_transform * transform).getOrigin().getX();
      ndt_pose_msg.pose.position.y = (local_transform * transform).getOrigin().getY();
      ndt_pose_msg.pose.position.z = (local_transform * transform).getOrigin().getZ();
      ndt_pose_msg.pose.orientation.x = (local_transform * transform).getRotation().x();
      ndt_pose_msg.pose.orientation.y = (local_transform * transform).getRotation().y();
      ndt_pose_msg.pose.orientation.z = (local_transform * transform).getRotation().z();
      ndt_pose_msg.pose.orientation.w = (local_transform * transform).getRotation().w();
    }
    else
    {
      ndt_pose_msg.header.frame_id = "/map";
      ndt_pose_msg.header.stamp = current_scan_time;
      ndt_pose_msg.pose.position.x = ndt_pose.x;
      ndt_pose_msg.pose.position.y = ndt_pose.y;
      ndt_pose_msg.pose.position.z = ndt_pose.z;
      ndt_pose_msg.pose.orientation.x = ndt_q.x();
      ndt_pose_msg.pose.orientation.y = ndt_q.y();
      ndt_pose_msg.pose.orientation.z = ndt_q.z();
      ndt_pose_msg.pose.orientation.w = ndt_q.w();
    }

    current_q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
    // current_pose is published by vel_pose_mux
    /*
    current_pose_msg.header.frame_id = "/map";
    current_pose_msg.header.stamp = current_scan_time;
    current_pose_msg.pose.position.x = current_pose.x;
    current_pose_msg.pose.position.y = current_pose.y;
    current_pose_msg.pose.position.z = current_pose.z;
    current_pose_msg.pose.orientation.x = current_q.x();
    current_pose_msg.pose.orientation.y = current_q.y();
    current_pose_msg.pose.orientation.z = current_q.z();
    current_pose_msg.pose.orientation.w = current_q.w();
    */

    localizer_q.setRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw);
    if (_use_local_transform == true)
    {
      tf::Vector3 v(localizer_pose.x, localizer_pose.y, localizer_pose.z);
      tf::Transform transform(localizer_q, v);
      localizer_pose_msg.header.frame_id = "/map";
      localizer_pose_msg.header.stamp = current_scan_time;
      localizer_pose_msg.pose.position.x = (local_transform * transform).getOrigin().getX();
      localizer_pose_msg.pose.position.y = (local_transform * transform).getOrigin().getY();
      localizer_pose_msg.pose.position.z = (local_transform * transform).getOrigin().getZ();
      localizer_pose_msg.pose.orientation.x = (local_transform * transform).getRotation().x();
      localizer_pose_msg.pose.orientation.y = (local_transform * transform).getRotation().y();
      localizer_pose_msg.pose.orientation.z = (local_transform * transform).getRotation().z();
      localizer_pose_msg.pose.orientation.w = (local_transform * transform).getRotation().w();
    }
    else
    {
      localizer_pose_msg.header.frame_id = "/map";
      localizer_pose_msg.header.stamp = current_scan_time;
      localizer_pose_msg.pose.position.x = localizer_pose.x;
      localizer_pose_msg.pose.position.y = localizer_pose.y;
      localizer_pose_msg.pose.position.z = localizer_pose.z;
      localizer_pose_msg.pose.orientation.x = localizer_q.x();
      localizer_pose_msg.pose.orientation.y = localizer_q.y();
      localizer_pose_msg.pose.orientation.z = localizer_q.z();
      localizer_pose_msg.pose.orientation.w = localizer_q.w();
    }
    //std::cout << "next enter predict pose publish";
    //pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::toROSMsg(*output_cloud, *ndt_final_cloud_ptr);
    ndt_final_cloud_ptr->header.stamp=input_lidar->header.stamp;
    ndt_final_cloud_ptr->header.frame_id="map";
    ndt_final_cloud_pub.publish(*ndt_final_cloud_ptr);




    predict_pose_pub.publish(predict_pose_msg);
    //node_status_publisher_ptr_->CHECK_RATE("/topic/rate/ndt_pose/slow",8,5,1,"topic points_raw publish rate low.");
    ndt_pose_pub.publish(ndt_pose_msg);
    // current_pose is published by vel_pose_mux
    //    current_pose_pub.publish(current_pose_msg);
    localizer_pose_pub.publish(localizer_pose_msg);

    // Send TF "/base_link" to "/map"
    if ((fitness_score < 1 && fitness_score > 0.01)||(fitness_score < 2 && fitness_score>0.01 && iteration <30))
    {
      transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
      transform.setRotation(current_q);
    }
    else
    {
      transform.setOrigin(tf::Vector3(fusion_pose_subed.x, fusion_pose_subed.y, fusion_pose_subed.z));
      transform.setRotation(fusion_pose_subed_q);
    }
    
    //    br.sendTransform(tf::StampedTransform(transform, current_scan_time, "/map", "/base_link"));
    if (_use_local_transform == true)
    {
      br.sendTransform(tf::StampedTransform(local_transform * transform, current_scan_time, "/map", "/base_link"));
    }
    else
    {
      br.sendTransform(tf::StampedTransform(transform, current_scan_time, "/map", "/base_link"));
    }



// Send TF "/velodyne" to "/map"
    transform_velodyne.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
    transform_velodyne.setRotation(current_q);
    //    br.sendTransform(tf::StampedTransform(transform, current_scan_time, "/map", "/base_link"));
    if (_use_local_transform == true)
    {
      br_velodyne.sendTransform(tf::StampedTransform(local_transform * transform, current_scan_time, "/map", "/rslidar"));
    }
    else
    {
      //br_velodyne.sendTransform(tf::StampedTransform(transform, current_scan_time, "/map", "/rslidar"));
      br_velodyne.sendTransform(tf::StampedTransform(transform, current_scan_time, "/map", "/veh2_rslidar"));
    }




    matching_end = std::chrono::system_clock::now();
    exe_time = std::chrono::duration_cast<std::chrono::microseconds>(matching_end - matching_start).count() / 1000.0;
    time_ndt_matching.data = exe_time;
    //node_status_publisher_ptr_->CHECK_MAX_VALUE("/value/time_ndt_matching",time_ndt_matching.data,50,70,100,"value time_ndt_matching is too high.");
    time_ndt_matching_pub.publish(time_ndt_matching);

    // Set values for /estimate_twist
    estimate_twist_msg.header.stamp = current_scan_time;
    estimate_twist_msg.header.frame_id = "/base_link";
    estimate_twist_msg.twist.linear.x = current_velocity;
    estimate_twist_msg.twist.linear.y = 0.0;
    estimate_twist_msg.twist.linear.z = 0.0;
    estimate_twist_msg.twist.angular.x = 0.0;
    estimate_twist_msg.twist.angular.y = 0.0;
    estimate_twist_msg.twist.angular.z = angular_velocity;

    estimate_twist_pub.publish(estimate_twist_msg);

    geometry_msgs::Vector3Stamped estimate_vel_msg;
    estimate_vel_msg.header.stamp = current_scan_time;
    estimate_vel_msg.vector.x = current_velocity;
    //node_status_publisher_ptr_->CHECK_MAX_VALUE("/value/estimate_twist/linear",current_velocity,5,10,15,"value linear estimated twist is too high.");
    //node_status_publisher_ptr_->CHECK_MAX_VALUE("/value/estimate_twist/angular",angular_velocity,5,10,15,"value linear angular twist is too high.");
    estimated_vel_pub.publish(estimate_vel_msg);

    // Set values for /ndt_stat
    ndt_stat_msg.header.stamp = current_scan_time;
    ndt_stat_msg.exe_time = time_ndt_matching.data;
    ndt_stat_msg.iteration = iteration;
    ndt_stat_msg.score = fitness_score;
    ndt_stat_msg.velocity = current_velocity;
    ndt_stat_msg.acceleration = current_accel;
    ndt_stat_msg.use_predict_pose = 0;

    ndt_stat_pub.publish(ndt_stat_msg);
    /* Compute NDT_Reliability */
    ndt_reliability.data = Wa * (exe_time / 100.0) * 100.0 + Wb * (iteration / 10.0) * 100.0 +
                           Wc * ((2.0 - trans_probability) / 2.0) * 100.0;
    ndt_reliability_pub.publish(ndt_reliability);
    /*
    // Write log
    if(_output_log_data)
    {
      if (!ofs)
      {
        std::cerr << "Could not open " << filename << "." << std::endl;
      }
      else
      {ndt_final_cloud
        ofs << input_lidar->header.seq << "," << scan_points_num << "," << step_size << "," << trans_eps << "," << std::fixed
            << std::setprecision(5) << current_pose.x << "," << std::fixed << std::setprecision(5) << current_pose.y << ","
            << std::fixed << std::setprecision(5) << current_pose.z << "," << current_pose.roll << "," << current_pose.pitch
            << "," << current_pose.yaw << "," << predict_pose.x << "," << predict_pose.y << "," << predict_pose.z << ","
            << predict_pose.roll << "," << predict_pose.pitch << "," << predict_pose.yaw << ","
            << current_pose.x - predict_pose.x << "," << current_pose.y - predict_pose.y << ","
            << current_pose.z - predict_pose.z << "," << current_pose.roll - predict_pose.roll << ","
            << current_pose.pitch - predict_pose.pitch << "," << current_pose.yaw - predict_pose.yaw << ","
            << predict_pose_error << "," << iteration << "," << fitness_score << "," << trans_probability << ","
            << ndt_reliability.data << "," << current_velocity << "," << current_velocity_smooth << "," << current_accel
            << "," << angular_velocity << "," << time_ndt_matching.data << "," << align_time << "," << getFitnessScore_time
            << std::endl;
      }
    }
    */

    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "Sequence: " << input_lidar->header.seq << std::endl;
    std::cout << "Timestamp: " << input_lidar->header.stamp << std::endl;

    std::cout << "PoseFusionSubed Timestamp: " << input_fusion_pos_time << std::endl;

    std::cout << "Frame ID: " << input_lidar->header.frame_id << std::endl;
    //		std::cout << "Number of Scan Points: " << scan_ptr->size() << " points." << std::endl;
    std::cout << "Number of Filtered Scan Points: " << scan_points_num << " points." << std::endl;
    std::cout << "NDT has converged: " << has_converged << std::endl;
    std::cout << "Fitness Score: " << fitness_score << std::endl;
    std::cout << "Transformation Probability: " << trans_probability << std::endl;
    std::cout << "Execution Time: " << exe_time << " ms." << std::endl;
    std::cout << "Number of Iterations: " << iteration << std::endl;
    std::cout << "NDT Reliability: " << ndt_reliability.data << std::endl;
    std::cout << "(x,y,z,roll,pitch,yaw): " << std::endl;
    std::cout << "(" << current_pose.x << ", " << current_pose.y << ", " << current_pose.z << ", " << current_pose.roll
              << ", " << current_pose.pitch << ", " << current_pose.yaw << ")" << std::endl;
    std::cout << "Transformation Matrix: " << std::endl;
    std::cout << t << std::endl;
    std::cout << "Align time: " << align_time << std::endl;
    std::cout << "Get fitness score time: " << getFitnessScore_time << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;

    offset_imu_x = 0.0;
    offset_imu_y = 0.0;
    offset_imu_z = 0.0;
    offset_imu_roll = 0.0;
    offset_imu_pitch = 0.0;
    offset_imu_yaw = 0.0;

    offset_odom_x = 0.0;
    offset_odom_y = 0.0;
    offset_odom_z = 0.0;
    offset_odom_roll = 0.0;
    offset_odom_pitch = 0.0;
    offset_odom_yaw = 0.0;

    offset_imu_odom_x = 0.0;
    offset_imu_odom_y = 0.0;
    offset_imu_odom_z = 0.0;
    offset_imu_odom_roll = 0.0;
    offset_imu_odom_pitch = 0.0;
    offset_imu_odom_yaw = 0.0;

    // Update previous_***
    previous_pose.x = current_pose.x;
    previous_pose.y = current_pose.y;
    previous_pose.z = current_pose.z;
    previous_pose.roll = current_pose.roll;
    previous_pose.pitch = current_pose.pitch;
    previous_pose.yaw = current_pose.yaw;

    previous_scan_time = current_scan_time;

    previous_previous_velocity = previous_velocity;
    previous_velocity = current_velocity;
    previous_velocity_x = current_velocity_x;
    previous_velocity_y = current_velocity_y;
    previous_velocity_z = current_velocity_z;
    previous_accel = current_accel;

    previous_estimated_vel_kmph.data = estimated_vel_kmph.data;
  }

  // ros::Time dr_fusion_timestamp = input_dr_fusion -> header.stamp;
  // double dr_fusion_x = input_dr_fusion -> pose.position.x;
  // double dr_fusion_y = input_dr_fusion -> pose.position.y;
  // double dr_fusion_z = input_dr_fusion -> pose.position.z;


/*
//static void gnss_callback(const novatel_oem7_msgs::INSPVA::Ptr& input_gnss)
//{
  ndt_xyz_pre = ndt_xyz;
  //std::cout << "gnss callback test"<< std::setprecision(13)<< input->latitude<<std::endl;
  geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
  gps_msg->header = input_gnss->header;
  gps_msg->position.latitude = input_gnss->latitude;
  gps_msg->position.longitude = input_gnss->longitude;
  gps_msg->position.altitude = input_gnss->height;

  insstatus_ = input_gnss->status.status; // novatel ins status, 3 means INS solution good
  //input_gnss->north_velocity;
  ndt_xyz << current_pose.x, current_pose.y, current_pose.z ;
  if (ndt_xyz(1) != ndt_xyz_pre(1))
  {
    ndt_change_flg = true;
  }
  geodesy::UTMPoint utm;
  geodesy::fromMsg(gps_msg->position, utm);
  Eigen::Vector3d xyz(utm.easting, utm.northing, utm.altitude);
  Eigen::Vector3d gnss_velocities (input_gnss->east_velocity, input_gnss->north_velocity, input_gnss->up_velocity);
  
  
  if ((!gnss_init_)&&(insstatus_==3)) //never initial, ins in RTK, start initialization
  {
    zero_utm = xyz; 
    //std::cout << "zero_utm_init  x:" << zero_utm[0] << "  y:" << zero_utm[1] << "  z:" << zero_utm[2] << std::endl;
    gnss_time_cur_fusion = input_gnss->header.stamp;
    gnss_time_pre_fusion = gnss_time_cur_fusion;
    gnss_init_ = true;
  }
  xyz -= zero_utm;
  std::cout << "xyz  x:" << xyz[0] << "  y:" << xyz[1] << "  z:" << xyz[2] << std::endl;
  
  //following, lever arm compensation, lidar 90cm ahead of GNSS, transform GNSS to lidar glt


  const float lever_arm_l2g=0.9; //m
  xyz_lever[0]=xyz[0]+lever_arm_l2g*cos(yaw_normal(input_gnss->azimuth)/180*M_PI);
  xyz_lever[1]=xyz[1]+lever_arm_l2g*sin(yaw_normal(input_gnss->azimuth)/180*M_PI);
  xyz_lever[2]=xyz[2];
  std::cout << "xyz_lever  x:" << xyz_lever[0] << "  y:" << xyz_lever[1] << "  z:" << xyz_lever[2] << std::endl;
  // above is for lever comp.
  
  
// publish GNSS results in local frame, lever arm is also compensated
  gnss_localframe_msg.header.frame_id = "/map";
  gnss_localframe_msg.header.stamp = input_gnss->header.stamp;
  gnss_localframe_msg.pose.position.x = xyz_lever[0];
  gnss_localframe_msg.pose.position.y = xyz_lever[1];
  gnss_localframe_msg.pose.position.z = xyz_lever[2];
  //for debug:
  //gnss_localframe_msg.pose.position.x = xyz[0];
  //gnss_localframe_msg.pose.position.y = xyz[1];
  //gnss_localframe_msg.pose.position.z = input_gnss->azimuth;
  gnss_localframe_pub.publish(gnss_localframe_msg);




  
  //below is for fusion
  gnss_time_cur_fusion = input_gnss->header.stamp;
  const double ts_gnss_fusion = (gnss_time_cur_fusion-gnss_time_pre_fusion).toSec();
  gnss_time_pre_fusion = gnss_time_cur_fusion;
  
  //if ((!kfinitial_) && gnss_init_) //DR initialization
  //{
    //fusion_pose.x = xyz_lever(0);
    //fusion_pose.y = xyz_lever(1);
    //fusion_pose.z = xyz_lever(2);
    Xk_1 << 0., 0., 0.;
    //kfinitial_=true;
    //std::cout<<"iiiiiiiiiiiii"<<std::endl;
  //}
  //if (!kfinitial_)
  //{
  //  //Xk_1 = xyz_lever;
  //  Xk_1 << 0., 0., 0.;
  //  kfinitial_=true;
  //}
  
  
  //Xkk_1 prediction
  Xkk_1(0)=Xk_1(0)+gnss_velocities(0)*ts_gnss_fusion;
  Xkk_1(1)=Xk_1(1)+gnss_velocities(1)*ts_gnss_fusion;
  Xkk_1(2)=Xk_1(2)+gnss_velocities(2)*ts_gnss_fusion;
  std::cout<<"ts gnss"<<ts_gnss_fusion<<std::endl;
  //Fk
  Eigen::Matrix< double, 3, 3 > Fkk_1 = Eigen::Matrix3d::Identity();
  //Qk
  Eigen::Matrix3d temp;
  temp << 0.3, 0., 0., 0.,0.3, 0., 0., 0.,0.3;
  Qk +=temp;
  //Pkk_1
  Pkk_1 = Fkk_1*Pk_1*Fkk_1.transpose()+Qk;
    //std::cout << "Fkk_1 outside:"<< std::endl<<Fkk_1<<std::endl;
    //std::cout << "Pk_1 outside:"<< std::endl<<Pk_1<<std::endl;
    //std::cout << "Fkk_1.transpose outside:"<< std::endl<<Fkk_1.transpose()<<std::endl;
    //std::cout << "Qk outside:"<< std::endl<<Qk<<std::endl;
    //std::cout << "Pkk_1 outside:"<< std::endl<<Pkk_1<<std::endl;
  //Zk
  Eigen::Vector3d fusion_pose_3d(0.,0.,0.);

  //DR:
    fusion_pose.x += gnss_velocities(0)*ts_gnss_fusion;
    fusion_pose.y += gnss_velocities(1)*ts_gnss_fusion;
    fusion_pose.z += gnss_velocities(2)*ts_gnss_fusion;
    //fusion_pose_3d << fusion_pose.x, fusion_pose.y, fusion_pose.z;
    fusion_pose_3d << dr_fusion_x, dr_fusion_y, dr_fusion_z;
  if (insstatus_==3)
    {
      std::cout<<"aaaaaaaaaaaaaaaa"<<std::endl;
      std::cout<<"xyz_lever:   fusion_pose_3d:"<<std::endl<<xyz_lever << fusion_pose_3d<<std::endl;
      Zk = xyz_lever - fusion_pose_3d;// gnss as measurements
      //Rk
  
      Rk(0,0) = 0.02;
      Rk(1,1) = 0.02;
      Rk(2,2) = 0.05; // RTK
    }
    else if (fitness_score < 50)
    {
      std::cout<<"bbbbbbbbbb"<<std::endl;
      Zk = ndt_xyz - fusion_pose_3d; //ndt as measurements
      //Rk
  
      Rk(0,0) = 0.1;
      Rk(1,1) = 0.1;
      Rk(2,2) = 0.2; // RTK
    }
    std:: cout<<"Zk x:"<< Zk(0) << " y:"<<Zk(1)<<" z:"<< Zk(2)<<std::endl;
  //Zk = ndt_xyz; // ndt results as measurements
  Hk = Eigen::Matrix3d::Identity();
  //Rk
 
  //if (insstatus_==3 && ndt_change_flg)  // 3 means ins solution good
  if (insstatus_==3)
    {
      fusionStatus = 1;
      filterUpdate();
    }
    else if (fitness_score < 50)
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
  //Xk_1 = Xk;
  Xk_1 << 0., 0., 0.;
  Pk_1 = Pk;

  errorResult(0)=Xk(0);
  errorResult(1)=Xk(1);
  errorResult(2)=Xk(2);
  //std::cout << "error state:"<< fusionStatus<<std::endl;
  std::cout << "gnss time:"<<input_gnss->header.stamp<<std::endl;
  std::cout << "error time"<< input_dr_fusion->header.stamp<<std::endl;
  std::cout << "error  x:" << errorResult(0)<< "  y:" << errorResult(1) << "  z:" << errorResult(2) << std::endl;
 /*
 //DR+error:
  fusion_pose.x += errorResult(0);
  fusion_pose.y += errorResult(1);
  fusion_pose.z += errorResult(2);
  std::cout << "fusion results  x:" << fusion_pose.x<< "  y:" << fusion_pose.y << "  z:" << fusion_pose.z << std::endl;
 */ 
/*
  kf_PoseError_msg.header.stamp = input_gnss-> header.stamp;
  kf_PoseError_msg.pose.position.x = errorResult(0);
  kf_PoseError_msg.pose.position.y = errorResult(1);
  kf_PoseError_msg.pose.position.z = errorResult(2);
  kf_PoseError_pub.publish(kf_PoseError_msg);
*/  
  /*
/*
  fusionResult(0)=Xk(0);
  fusionResult(1)=Xk(1);
  fusionResult(2)=Xk(2);

  //std::cout << "fusion state:"<< fusionStatus<<std::endl;
  //std::cout << "fusion  x:" << fusionResult(0)<< "  y:" << fusionResult(1) << "  z:" << fusionResult(2) << std::endl;
  
  if (SAVEDATA)
  {
    outdata_ << std::setprecision(13)<< "time " << gnss_time_cur_fusion;
    outdata_ << std::setprecision(5)<< " fusion_x " << fusionResult(0)<< " fusion_y " << fusionResult(1) << " fusion_z " << fusionResult(2);
    outdata_ << std::setprecision(5)<< " gnss_x " << xyz_lever(0)<< " gnss_y " << xyz_lever(1) << " gnss_z " << xyz_lever(2)<< std::endl;
  }
*/  
 
 //below is for NDT/DR fusion
 //if (DR_init_flg && kfinitial_ )
 static pose pose_error;
 std::cout<<"breakpoint 0822-1"<<std::endl;
 if (abs(fusion_pose_subed.x)>0.01 && abs(fusion_pose_subed.y)>0.01 ) //DR initialized
  {
    std::cout<<"breakpoint 0822-2"<<std::endl;
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
    std::cout<<"!!!!!!!!!!!!!KF INITIAL1!!!!!!!!!!!!!!!!!!!!"<<std::endl;
    if ((fitness_score < 1 && fitness_score > 0.01)||(fitness_score < 2 && fitness_score > 0.01 && iteration <30))
    {
      std::cout<<"!!!!!!!!!!!!!start ZK !!!!!!!!!!!!!!!!!!!"<<std::endl;
      // fusion_pose.roll = ndt_pose.roll;
      // fusion_pose.pitch = ndt_pose.pitch;
      // fusion_pose.yaw = ndt_pose.yaw;
      //Zk
      ndt_xyz << ndt_pose.x, ndt_pose.y, ndt_pose.z ;
      fusion_ins_xyz << fusion_pose_subed.x, fusion_pose_subed.y, fusion_pose_subed.z;
      Zk = ndt_xyz - fusion_ins_xyz;
      // if (abs(fusion_pose_subed.x)<0.01) // due to message filter, although the pose is initialized by ndt, 
      //                             //the found fusion results may be previous stamp, value is 0, then the results can not be used
      // {
      //   Zk << 0, 0, 0;
      // }

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
    float Zk_yaw = ndt_pose.yaw - fusion_pose_subed.yaw;
    Zk_yaw = yaw_normal_rad(Zk_yaw);  

    if (abs(fusion_pose_subed.yaw)<0.001) // due to message filter, although the pose is initialized by ndt, 
                                  //the found fusion results may be previous stamp, value is 0, then the results can not be used
      {
        Zk_yaw = 0;
      }
    pose_error.yaw = Zk_yaw * 0.5;
    pose_error.pitch = ndt_pose.pitch-fusion_pose_subed.pitch;
    pose_error.roll = ndt_pose.roll-fusion_pose_subed.roll;


  }
  else
  {
    pose_error.x = 0;
    pose_error.y = 0;
    pose_error.z = 0;
    pose_error.yaw = 0;
    pose_error.pitch = 0;
    pose_error.roll = 0;
  }
  kf_PoseError_msg.header.stamp = input_lidar-> header.stamp;
  kf_PoseError_msg.pose.position.x = pose_error.x;
  kf_PoseError_msg.pose.position.y = pose_error.y;
  kf_PoseError_msg.pose.position.z = pose_error.z;
  kf_PoseError_msg.pose.orientation.x = pose_error.roll; //attention: not quanterion, it's euler angle
  kf_PoseError_msg.pose.orientation.y = pose_error.pitch; //attention: not quanterion, it's euler angle
  kf_PoseError_msg.pose.orientation.z = pose_error.yaw; //attention: not quanterion, it's euler angle
  
  kf_PoseError_pub.publish(kf_PoseError_msg);
 
 
 if (SAVEDATA)
      {
        outdata_ << std::setprecision(13)<< "time " << input_lidar->header.stamp;
        outdata_ << std::setprecision(13)<< "time_ndt " << input_lidar->header.stamp;//
        //outdata_ << std::setprecision(5)<< " fusion_x " << dr_fusion_x<< " fusion_y " << dr_fusion_y << " fusion_z " << dr_fusion_z;
        //outdata_ << std::setprecision(5)<< " gnss_x " << xyz_lever(0)<< " gnss_y " << xyz_lever(1) << " gnss_z " << xyz_lever(2);
        outdata_ << std::setprecision(5)<< " ndt_x " << current_pose.x<< " ndt_y " << current_pose.y << " ndt_z " << current_pose.z;
        outdata_ << std::setprecision(5)<< " ndt_score " << fitness_score << " ndt_iteration " << iteration << " ndt_reliability " << ndt_reliability.data;
        //outdata_ << std::setprecision(5)<< " err_x " << errorResult(0)<< " err_y " << errorResult(1) << " err_z " << errorResult(2);
        //outdata_ << std::setprecision(5)<< " Zk_x " << Zk(0)<< " Zk_y " << Zk(1) << " Zk_z " << Zk(2)<< std::endl;
      }
  
  if (SAVEDATA_datasetprocess)
      {
        outdata_datasetprocessing_ << std::setprecision(13)<<input_lidar->header.stamp<<",";
        outdata_datasetprocessing_ << std::setprecision(5)<<current_pose.x<< "," << current_pose.y << "," << current_pose.z<<",";//
        outdata_datasetprocessing_ << std::setprecision(5)<<current_pose.roll<< "," << current_pose.yaw << "," << current_pose.pitch<<",";
        outdata_datasetprocessing_ << std::setprecision(5)<< fitness_score << "," << iteration << "," << ndt_reliability.data<<std::endl;
        
      }


}

/*
static void fusion_callback(const geometry_msgs::PoseStamped::Ptr& input_fusion)
{
  tf::Quaternion q_fusion(input_fusion->pose.orientation.x, input_fusion->pose.orientation.y, input_fusion->pose.orientation.z,
                   input_fusion->pose.orientation.w);
  tf::Matrix3x3 m_fusion(q_fusion);
  fusion_pose_subed.x = input_fusion->pose.position.x;
  fusion_pose_subed.y = input_fusion->pose.position.y;
  fusion_pose_subed.z = input_fusion->pose.position.z;
  
  m_fusion.getRPY(fusion_pose_subed.roll, fusion_pose_subed.pitch, fusion_pose_subed.yaw);
  if (a<3)
  {
    if (fitness_score < 0.9 && fitness_score>0.01 && iteration <20)
    {
      a++;
    }
  }
  else
  {
    a=4;
    
  }
  
}
*/


void* thread_func(void* args)
{
  ros::NodeHandle nh_map;
  ros::CallbackQueue map_callback_queue;
  nh_map.setCallbackQueue(&map_callback_queue);

  //ros::Subscriber map_sub = nh_map.subscribe("points_map", 10, map_callback);
  ros::Rate ros_rate(10);
  while (nh_map.ok())
  {
    map_callback_queue.callAvailable(ros::WallDuration());
    ros_rate.sleep();
  }

  return nullptr;
}



static void gps_raw_callback(const gps_common::GPSFixConstPtr& input_gps_raw) //0312 datacollection pretest
{ 
  gps_raw_heading = input_gps_raw->dip; //gnss raw heading
  geographic_msgs::GeoPointStampedPtr gps_raw_msg(new geographic_msgs::GeoPointStamped());
  gps_raw_msg->position.longitude = input_gps_raw->longitude;
  gps_raw_msg->position.latitude = input_gps_raw->latitude;
  gps_raw_msg->position.altitude = input_gps_raw->altitude;

  geodesy::UTMPoint gps_raw_UTMPoint;
  geodesy::fromMsg(gps_raw_msg->position, gps_raw_UTMPoint);
  Eigen::Vector3d gps_raw_UTM(gps_raw_UTMPoint.easting, gps_raw_UTMPoint.northing, gps_raw_UTMPoint.altitude);
  gps_raw_local= gps_raw_UTM-map_ori_utm;
}

static void novatel_inspva_callback(const novatel_oem7_msgs::INSPVA::Ptr& input_novatel_inspva)
{
  insstatus_ = input_novatel_inspva->status.status; // gongji ins status, 4 rtk, 5 float rtk
  gps_fusion_roll = input_novatel_inspva-> roll;
  gps_fusion_pitch = input_novatel_inspva-> pitch;
  //Eigen::Vector3d gps_attitude_local(gps_fusion_roll,gps_fusion_pitch,gps_raw_heading);
  Eigen::Vector3d gps_attitude_local(0,0,gps_raw_heading);
  
}
  
int main(int argc, char** argv)
{
  //std::cout << "start" << std::endl;
  if (SAVEDATA)
  {
    outdata_.open("/home/jiaqi/catkin_ws_glt/ForMatlabNDTsavedData.txt", std::ios::app | std::ios::out);
  }
  if (SAVEDATA_datasetprocess)
  {
    outdata_datasetprocessing_.open("/home/jiaqi/catkin_ws_glt/pose_cor_2023-04-04-14-29-53_46_0zzl.txt", std::ios::app | std::ios::out);
  }
  std::cout << "At first line previous_pose_x:"<<previous_pose.x<<";previous_pose_yaw:"<<previous_pose.yaw<<std::endl;
  
  
  ros::init(argc, argv, "ndt_matching");
  pthread_mutex_init(&mutex, NULL);

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  //node_status_publisher_ptr_ = std::make_shared<autoware_health_checker::NodeStatusPublisher>(nh,private_nh);
  //node_status_publisher_ptr_->ENABLE();
  //node_status_publisher_ptr_->NODE_ACTIVATE();
/*
  // Set log file name.
  private_nh.getParam("output_log_data", _output_log_data);
  if(_output_log_data)
  {
    char buffer[80];
    std::time_t now = std::time(NULL);
    std::tm* pnow = std::localtime(&now);
    std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
    std::string directory_name = "/tmp/Autoware/log/ndt_matching";
    filename = directory_name + "/" + std::string(buffer) + ".csv";
    boost::filesystem::create_directories(boost::filesystem::path(directory_name));
    ofs.open(filename.c_str(), std::ios::app);
  }
*/
// read the pcd map file glt
  
  //static std::string map_pcd_file_name = "/home/letiangao18/autoware.ai/test.pcd";
  //static std::string map_pcd_file_name = "/home/letiangao18/Bags/parking_lot_undistorted.pcd"; // map path definition glt
  //static std::string map_pcd_file_name = "/home/letiangao18/Bags/intersection_data1/smart_intersection_1.pcd"; // map path definition glt
  //static std::string map_pcd_file_name = "/home/letiangao18/Bags/intersection_data2/smart_intersection_2.pcd"; // map path definition glt
  //static std::string map_pcd_file_name = "/home/letiangao18/Bags/intersection_data3/smart_intersection_westeast.pcd"; // map path definition glt
  //static std::string map_pcd_file_name = "/home/letiangao18/Bags/map_for_dataprocessing/Westwood_NS_0/westwood_ns.pcd"; // 
  //static std::string map_pcd_file_name = "/home/letiangao18/Bags/map_for_dataprocessing/Charlse_young_we/charlse_young_ew.pcd"; // map path definition glt
  static std::string map_pcd_file_name = "/home/jiaqi/catkin_ws_glt/merged.pcd"; // map path definition glt
  
  //std::cout << "status1";
 
  //std::cout << "status2";
  //pcl::PCLPointCloud2 cloud_blob;
  //pcl::io::loadPCDFile (map_pcd_file_name, cloud_blob);
  //pcl::fromPCLPointCloud2 (cloud_blob, *map_saved_cloud); //* convert from pcl/PCLPointCloud2 to pcl::PointCloud<T>

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (map_pcd_file_name, *map_saved_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file pcd map \n");
    //return (-1);
  }
  //std::cout << "status3";
  std::cout << "Loaded "
            << map_saved_cloud->width * map_saved_cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  // for (const auto& point: *cloud)
  //   std::cout << "    " << point.x
  //             << " "    << point.y
  //             << " "    << point.z << std::endl;

  map_loaded = 1; 
  // if  (map_pub)
  // {
  //   pcl::toROSMsg(*map_saved_cloud, saved_map_cloud);
  //   saved_map_cloud.header.frame_id = "/map";
  //   saved_map_cloud.header.stamp = ros::Time::now();
  //   std::cout<<"stamp"<<rawPointCouldStamp<<std::endl;
  //   saved_map_cloud_pub.publish(saved_map_cloud);
  //   map_pub=false;
  //   std::cout << "the map is published!!! "<<std::endl;

  // }
  saved_map_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt_saved_map_cloud", 10);
  //pcl::toROSMsg(*map_saved_cloud, *saved_map_cloud_ptr);

  
  // ros::Time begin = ros::Time::now();

// end of read map

/*
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*nearHistorySurfKeyFrameCloudDS, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
            cloudMsgTemp.header.frame_id = "/camera_init";
            pubHistoryKeyFrames.publish(cloudMsgTemp);
*/
  // Geting parameters
  int method_type_tmp = 0;
  private_nh.getParam("method_type", method_type_tmp);
  _method_type = static_cast<MethodType>(method_type_tmp);
  private_nh.getParam("use_gnss", _use_gnss);
  private_nh.getParam("queue_size", _queue_size);
  private_nh.getParam("offset", _offset);
  private_nh.getParam("get_height", _get_height);
  private_nh.getParam("use_local_transform", _use_local_transform);
  private_nh.getParam("use_imu", _use_imu);
  private_nh.getParam("use_odom", _use_odom);
  private_nh.getParam("imu_upside_down", _imu_upside_down);
  private_nh.getParam("imu_topic", _imu_topic);

  private_nh.getParam("initial_pose_x", _initial_pose_x);
  private_nh.getParam("initial_pose_y", _initial_pose_y);
  private_nh.getParam("initial_pose_z", _initial_pose_z);
  private_nh.getParam("initial_pose_roll", _initial_pose_roll);
  private_nh.getParam("initial_pose_pitch", _initial_pose_pitch);
  private_nh.getParam("initial_pose_yaw", _initial_pose_yaw);

  private_nh.getParam("map_ori_utm_easting", _map_ori_utm_easting);
  private_nh.getParam("map_ori_utm_northing", _map_ori_utm_northing);
  private_nh.getParam("map_ori_utm_altitude", _map_ori_utm_altitude);

  private_nh.getParam("init_post_set_fromGNSS", _init_post_set_fromGNSS);
  private_nh.getParam("init_post_set_fromLAUNCH", _init_post_set_fromLAUNCH);

  //std::cout<<"_initial_pose_x:"<<_initial_pose_x<<std::endl;
  Eigen::Vector3d map_ori_utm(_map_ori_utm_easting, _map_ori_utm_northing, _map_ori_utm_altitude);
  //map_ori_utm is the zero point, the point from GPS need to minus this value to get the relative position in map coordinate
  Eigen::Vector3d initial_pose_local(_initial_pose_x,_initial_pose_y,_initial_pose_z);
  Eigen::Vector3d initial_attitude_local(_initial_pose_roll,_initial_pose_pitch,_initial_pose_yaw);

  // if (nh.getParam("localizer", _localizer) == false)
  // {
  //   std::cout << "localizer is not set." << std::endl;
  //   return 1;
  // }
  // if (nh.getParam("tf_x", _tf_x) == false)
  // {
  //   std::cout << "tf_x is not set." << std::endl;
  //   return 1;
  // }
  // if (nh.getParam("tf_y", _tf_y) == false)
  // {
  //   std::cout << "tf_y is not set." << std::endl;
  //   return 1;
  // }
  // if (nh.getParam("tf_z", _tf_z) == false)
  // {
  //   std::cout << "tf_z is not set." << std::endl;
  //   return 1;
  // }
  // if (nh.getParam("tf_roll", _tf_roll) == false)
  // {
  //   std::cout << "tf_roll is not set." << std::endl;
  //   return 1;
  // }
  // if (nh.getParam("tf_pitch", _tf_pitch) == false)
  // {
  //   std::cout << "tf_pitch is not set." << std::endl;
  //   return 1;
  // }
  // if (nh.getParam("tf_yaw", _tf_yaw) == false)
  // {
  //   std::cout << "tf_yaw is not set." << std::endl;
  //   return 1;
  // }

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Log file: " << filename << std::endl;
  std::cout << "method_type: " << static_cast<int>(_method_type) << std::endl;
  std::cout << "use_gnss: " << _use_gnss << std::endl;
  std::cout << "queue_size: " << _queue_size << std::endl;
  std::cout << "offset: " << _offset << std::endl;
  std::cout << "get_height: " << _get_height << std::endl;
  std::cout << "use_local_transform: " << _use_local_transform << std::endl;
  std::cout << "use_odom: " << _use_odom << std::endl;
  std::cout << "use_imu: " << _use_imu << std::endl;
  std::cout << "imu_upside_down: " << _imu_upside_down << std::endl;
  std::cout << "imu_topic: " << _imu_topic << std::endl;
  std::cout << "localizer: " << _localizer << std::endl;
  std::cout << "(tf_x,tf_y,tf_z,tf_roll,tf_pitch,tf_yaw): (" << _tf_x << ", " << _tf_y << ", " << _tf_z << ", "
            << _tf_roll << ", " << _tf_pitch << ", " << _tf_yaw << ")" << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
  /*
#ifndef CUDA_FOUND
  if (_method_type == MethodType::PCL_ANH_GPU)
  {
    std::cerr << "**************************************************************" << std::endl;
    std::cerr << "[ERROR]PCL_ANH_GPU is not built. Please use other method type." << std::endl;
    std::cerr << "**************************************************************" << std::endl;
    exit(1);
  }
#endif
#ifndef USE_PCL_OPENMP
  if (_method_type == MethodType::PCL_OPENMP)
  {
    std::cerr << "**************************************************************" << std::endl;
    std::cerr << "[ERROR]PCL_OPENMP is not built. Please use other method type." << std::endl;
    std::cerr << "**************************************************************" << std::endl;
    exit(1);
  }
#endif
*/
  ndt.setResolution(ndt_res);
  ndt.setInputTarget(map_saved_cloud);
  ndt.setMaximumIterations(max_iter);
  ndt.setStepSize(step_size);
  ndt.setTransformationEpsilon(trans_eps);



  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //ndt.align(*output_cloud, Eigen::Matrix4f::Identity());
  //pcl::toROSMsg(*output_cloud, *ndt_final_cloud_ptr);
  //ndt_final_cloud_pub.publish(*ndt_final_cloud_ptr);

  Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);                 // tl: translation
  Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
  tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

  /*
  // Updated in initialpose_callback or gnss_callback
  initial_pose.x = 0.0;
  initial_pose.y = 0.0;
  initial_pose.z = 0.0;
  initial_pose.roll = 0.0; 
  initial_pose.pitch = 0.0;
  initial_pose.yaw = 0.0;
  initial_pose.yaw = 0.0;
  */

  // Publishers
  predict_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/predict_pose", 10);
  std:: cout << "prepare to publish"<<std::endl;
  predict_pose_imu_pub = nh.advertise<geometry_msgs::PoseStamped>("/predict_pose_imu", 10);
  predict_pose_odom_pub = nh.advertise<geometry_msgs::PoseStamped>("/predict_pose_odom", 10);
  predict_pose_imu_odom_pub = nh.advertise<geometry_msgs::PoseStamped>("/predict_pose_imu_odom", 10);
  ndt_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ndt_pose", 1);
  // current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);
  localizer_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/localizer_pose", 10);
  estimate_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("/estimate_twist", 10);
  estimated_vel_mps_pub = nh.advertise<std_msgs::Float32>("/estimated_vel_mps", 10);
  estimated_vel_kmph_pub = nh.advertise<std_msgs::Float32>("/estimated_vel_kmph", 10);
  estimated_vel_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/estimated_vel", 10);
  time_ndt_matching_pub = nh.advertise<std_msgs::Float32>("/time_ndt_matching", 1);
  ndt_stat_pub = nh.advertise<autoware_msgs::NDTStat>("/ndt_stat", 1);
  ndt_reliability_pub = nh.advertise<std_msgs::Float32>("/ndt_reliability", 1);

  ndt_final_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt_final_cloud", 1);
  kf_PoseError_pub = nh.advertise<geometry_msgs::PoseStamped>("/kf_PoseError", 10);
  gnss_localframe_pub = nh.advertise<geometry_msgs::PoseStamped>("/gnss_localframe", 1);
  // Subscribers
  //ros::Subscriber param_sub = nh.subscribe("config/ndt", 10, param_callback);
  // ros::Subscriber gnss_sub = nh.subscribe("gnss_pose", 10, gnss_callback);
  //ros::Subscriber gnss_sub = nh.subscribe("/novatel/oem7/inspva", 10, gnss_callback);
  //  ros::Subscriber map_sub = nh.subscribe("points_map", 1, map_callback);
  ros::Subscriber initialpose_sub = nh.subscribe("initialpose", 10, initialpose_callback);
  //std:: cout << "_use_gnss="<<_use_gnss<<std::endl;
  _use_gnss=0;
  //ros::Subscriber points_sub = nh.subscribe("filtered_points", _queue_size, points_callback);
  //ros::Subscriber points_sub = nh.subscribe("points_raw", _queue_size, points_callback); // input points cloud message definition glt
  //ros::Subscriber points_sub = nh.subscribe("velodyne_points", _queue_size, points_callback);

  //std:: cout << "prepare to publish2"<<std::endl;
  // ros::Subscriber odom_sub = nh.subscribe("/vehicle/odom", _queue_size * 10, odom_callback);
  // ros::Subscriber imu_sub = nh.subscribe(_imu_topic.c_str(), _queue_size * 10, imu_callback);

  //message_filters::Subscriber<novatel_oem7_msgs::INSPVA> gnss_sub(nh, "/novatel/oem7/inspva", 1);
  //message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub(nh, "/velodyne_points", 1);
  //message_filters::Subscriber<geometry_msgs::PoseStamped> dr_fusion_sub(nh, "/fusion_pose", 1);
  //typedef sync_policies::ApproximateTime<novatel_oem7_msgs::INSPVA, sensor_msgs::PointCloud2,geometry_msgs::PoseStamped> MySyncPolicy_locafusion;
  //Synchronizer<MySyncPolicy_locafusion> sync_localizationfusion(MySyncPolicy_locafusion(30), gnss_sub, points_sub, dr_fusion_sub);
  //sync_localizationfusion.registerCallback(boost::bind(&callback, _1, _2, _3));
  
  //ros::Subscriber points_sub = nh.subscribe("/pc_subset", 1,rs_callback);
    //ros::Subscriber points_sub = nh.subscribe("/rslidar_points", 1,rs_callback);
    //ros::Subscriber points_sub = nh.subscribe("/veh_2/rslidar_points", 1,rs_callback);
  //}
  message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub(nh, "/pc_subset", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> fusion_pos_sub(nh, "/fusion_pose", 100);
  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2,geometry_msgs::PoseStamped> MySyncPolicy_locafusion;
  Synchronizer<MySyncPolicy_locafusion> sync_localizationfusion(MySyncPolicy_locafusion(20),points_sub,fusion_pos_sub);
  //TimeSynchronizer<novatel_oem7_msgs::INSPVA, sensor_msgs::PointCloud2,geometry_msgs::PoseStamped,autoware_msgs::NDTStat> sync(gnss_sub, points_sub,ndt_pos_sub,ndt_stat_sub, 10);
  sync_localizationfusion.registerCallback(boost::bind(&rs_callback, _1, _2));
  
  
  ros::Subscriber gps_raw_sub = nh.subscribe("/gps/raw", 10,gps_raw_callback);
  ros::Subscriber novatel_inspva_sub = nh.subscribe("/veh1/novatel/oem7/inspva", 10,novatel_inspva_callback);

  

 
  pthread_t thread;
  pthread_create(&thread, NULL, thread_func, NULL);


  ros::spin();

  return 0;
}
