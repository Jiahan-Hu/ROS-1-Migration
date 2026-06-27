/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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
 *
 */
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include "autoware_msgs/ProjectionMatrix.h"
#include <iostream>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <math.h>

static cv::Mat CameraExtrinsicMat;
static cv::Mat CameraMat;
static cv::Mat DistCoeff;
static cv::Size ImageSize;
static std::string DistModel;

ros::Publisher camera_info_pub;
ros::Publisher projection_matrix_pub;
ros::Subscriber image_sub,manual_calib_x,manual_calib_y,manual_calib_z,manual_calib_roll,manual_calib_pitch,manual_calib_yaw;

static bool isRegister_tf;
static bool isPublish_extrinsic;
static bool isPublish_cameraInfo;

static std::string camera_id_str_;
static std::string camera_frame_;
static std::string target_frame_;

static bool instrinsics_parsed_;
static bool extrinsics_parsed_;

static sensor_msgs::CameraInfo camera_info_msg_;
static autoware_msgs::ProjectionMatrix extrinsic_matrix_msg_;

double manual_roll = 0, manual_pitch = 0, manual_yaw = 0;
double manual_x=0,      manual_y=0,       manual_z=0;

double calibration_roll = 0, calibration_pitch = 0, calibration_yaw = 0;
double calibration_x=0,      calibration_y=0,       calibration_z=0;

bool manual_roll_changed,manual_pitch_changed,manual_yaw_changed,manual_x_changed,manual_y_changed,manual_z_changed;

void tfRegistration(const cv::Mat &camExtMat, const ros::Time &timeStamp)
{
  tf::Matrix3x3 rotation_mat;
  tf::Quaternion quaternion;
  tf::Transform transform;
  static tf::TransformBroadcaster broadcaster;
  double roll,pitch,yaw,x,y,z;

  rotation_mat.setValue(camExtMat.at<double>(0, 0), camExtMat.at<double>(0, 1), camExtMat.at<double>(0, 2),
                        camExtMat.at<double>(1, 0), camExtMat.at<double>(1, 1), camExtMat.at<double>(1, 2),
                        camExtMat.at<double>(2, 0), camExtMat.at<double>(2, 1), camExtMat.at<double>(2, 2));

  // std::cout<<"camExtMat.at<double>(0, 0) = "<<camExtMat.at<double>(0, 0)<<std::endl;

  rotation_mat.getRPY(roll, pitch, yaw, 1);
  x=camExtMat.at<double>(0, 3);
  y=camExtMat.at<double>(1, 3);
  z=camExtMat.at<double>(2, 3);

  if (manual_roll_changed)
  {
    roll=manual_roll;
  }

  if (manual_pitch_changed)
  {
    pitch=manual_pitch;
  }

  if (manual_yaw_changed)
  {
    yaw=manual_yaw;
  }

  if (manual_x_changed)
  {
    x=manual_x;
  }

  if (manual_y_changed)
  {
    y=manual_y;
  }

  if (manual_z_changed)
  {
    z=manual_z;
  }

  // quaternion.setRPY(0.253386, -1.559, 1.32198);
  // yaw 1.32198, pitch -1.559, roll 0.253386; from velodyne to cam02
  quaternion.setRPY(roll, pitch, yaw);
  transform.setOrigin(
  tf::Vector3(x, y, z));

  transform.setRotation(quaternion);

  // transform=transform.inverse();

  // tf::Matrix3x3 rotation_mat_inv;

  // tf::Vector3 translation_inv;

  // rotation_mat_inv=transform.getBasis();
  // translation_inv=transform.getOrigin();

  // std::cout << std::setprecision(4) << translation_inv[0] << "\t" << translation_inv[1] << "\t" << translation_inv[2] << "\n\n";

  // std::cout << std::setprecision(4) << rotation_mat_inv[0][0] << "\t" << rotation_mat_inv[0][1] << "\t" << rotation_mat_inv[0][2] << "\n"
  //       << rotation_mat_inv[1][0] << "\t" << rotation_mat_inv[1][1] << "\t" << rotation_mat_inv[1][2] << "\n"
  //       << rotation_mat_inv[2][0] << "\t" << rotation_mat_inv[2][1] << "\t" << rotation_mat_inv[2][2] << "\n\n";

  

  // broadcaster.sendTransform(tf::StampedTransform(transform, timeStamp,camera_frame_,target_frame_));//modified to adapt the kitti datasets
  broadcaster.sendTransform(tf::StampedTransform(transform, timeStamp,target_frame_, camera_frame_));//original autoware packcage

  std::cout<<"roll = "<<roll*57.3<<std::endl;
  std::cout<<"pitch = "<<pitch*57.3<<std::endl;
  std::cout<<"yaw = "<<yaw*57.3<<std::endl;
  std::cout<<"x = "<<x<<std::endl;
  std::cout<<"y = "<<y<<std::endl;
  std::cout<<"z = "<<z<<std::endl;

  

  // static tf::TransformBroadcaster broadcaster_pcd;
  // tf::Quaternion quaternion_;
  // tf::Transform transform_;

  // quaternion_.setRPY(0, 0, M_PI);
  // transform_.setOrigin(tf::Vector3(0, 0, 0));
  // transform_.setRotation(quaternion_);

  // broadcaster_pcd.sendTransform(tf::StampedTransform(transform_, timeStamp,"newrslidar_repub", "newrslidar_transformed"));//original autoware packcage


  // tf::StampedTransform::StampedTransform(const tf::Transform &input, const ros::Time &timestamp, const std::string &frame_id, const std::string &child_frame_id)
  // sendTransform(self, translation, rotation, time, child, parent)
}

void projectionMatrix_sender(const cv::Mat &projMat, const ros::Time &timeStamp)
{
  if (!extrinsics_parsed_)
  {
    for (int row = 0; row < 4; row++)
    {
      for (int col = 0; col < 4; col++)
      {
        extrinsic_matrix_msg_.projection_matrix[row * 4 + col] = projMat.at<double>(row, col);
      }
    }
    extrinsics_parsed_ = true;
  }
  extrinsic_matrix_msg_.header.stamp = timeStamp;
  extrinsic_matrix_msg_.header.frame_id = camera_frame_;
  projection_matrix_pub.publish(extrinsic_matrix_msg_);
}

void cameraInfo_sender(const cv::Mat &camMat, const cv::Mat &distCoeff, const cv::Size &imgSize,
                       const std::string &distModel, const ros::Time &timeStamp)
{
  if (!instrinsics_parsed_)
  {
    for (int row = 0; row < 3; row++)
    {
      for (int col = 0; col < 3; col++)
      {
        camera_info_msg_.K[row * 3 + col] = camMat.at<double>(row, col);
      }
    }

    for (int row = 0; row < 3; row++)
    {
      for (int col = 0; col < 4; col++)
      {
        if (col == 3)
        {
          camera_info_msg_.P[row * 4 + col] = 0.0f;
        } else
        {
          camera_info_msg_.P[row * 4 + col] = camMat.at<double>(row, col);
        }
      }
    }

    for (int row = 0; row < distCoeff.rows; row++)
    {
      for (int col = 0; col < distCoeff.cols; col++)
      {
        camera_info_msg_.D.push_back(distCoeff.at<double>(row, col));
      }
    }
    camera_info_msg_.distortion_model = distModel;
    camera_info_msg_.height = imgSize.height;
    camera_info_msg_.width = imgSize.width;

    instrinsics_parsed_ = true;
  }
  camera_info_msg_.header.stamp = timeStamp;
  camera_info_msg_.header.frame_id = camera_frame_;


  camera_info_pub.publish(camera_info_msg_);
}

static void image_raw_cb(const sensor_msgs::Image &image_msg)
{

  std::cout<<"test 1 "<<std::endl;
  // ros::Time timeStampOfImage = image_msg.header.stamp;

  ros::Time timeStampOfImage;
  timeStampOfImage.sec = image_msg.header.stamp.sec;
  timeStampOfImage.nsec = image_msg.header.stamp.nsec;

  /* create TF between velodyne and camera with time stamp of /image_raw */
  if (isRegister_tf)
  {
    std::cout<<"test 2 "<<std::endl;
    tfRegistration(CameraExtrinsicMat, timeStampOfImage);
  }

  if (isPublish_cameraInfo)
  {
    cameraInfo_sender(CameraMat, DistCoeff, ImageSize, DistModel, timeStampOfImage);
  }
  if (isPublish_extrinsic)
  {
    projectionMatrix_sender(CameraExtrinsicMat, timeStampOfImage);
  }
}

void manual_calib_x_cb(std_msgs::Float64 msg)
{
  manual_x=msg.data;
  manual_x_changed=true;

}

void manual_calib_y_cb(std_msgs::Float64 msg)
{
  manual_y=msg.data;
  manual_y_changed=true;
}

void manual_calib_z_cb(std_msgs::Float64 msg)
{
  manual_z=msg.data;
  manual_z_changed=true;
}

void manual_calib_roll_cb(std_msgs::Float64 msg)
{
  manual_roll=msg.data/180*M_PI;
  manual_roll_changed=true;
}
void manual_calib_pitch_cb(std_msgs::Float64 msg)
{
  manual_pitch=msg.data/180*M_PI;
  manual_pitch_changed=true;
}

void manual_calib_yaw_cb(std_msgs::Float64 msg)
{
  manual_yaw=msg.data/180*M_PI;
  manual_yaw_changed=true;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "calibration_publisher");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  char __APP_NAME__[] = "calibration_publisher";

  if (!private_nh.getParam("register_lidar2camera_tf", isRegister_tf))
  {
    isRegister_tf = true;
  }

  if (!private_nh.getParam("publish_extrinsic_mat", isPublish_extrinsic))
  {
    isPublish_extrinsic = true; /* publish extrinsic_mat in default */
  }

  if (!private_nh.getParam("publish_camera_info", isPublish_cameraInfo))
  {
    isPublish_cameraInfo = true; /* doesn't publish camera_info in default */
  }

  private_nh.param<std::string>("camera_frame", camera_frame_, "camera");
  ROS_INFO("[%s] camera_frame: '%s'", __APP_NAME__, camera_frame_.c_str());


  private_nh.param<std::string>("camera_frame", camera_frame_, "camera");
  ROS_INFO("[%s] camera_frame: '%s'", __APP_NAME__, camera_frame_.c_str());

  private_nh.param<std::string>("target_frame", target_frame_, "velodyne");
  ROS_INFO("[%s] target_frame: '%s'", __APP_NAME__, target_frame_.c_str());

  std::string calibration_file;
  private_nh.param<std::string>("calibration_file", calibration_file, "");
  ROS_INFO("[%s] calibration_file: '%s'", __APP_NAME__, calibration_file.c_str());
  if (calibration_file.empty())
  {
    ROS_ERROR("[%s] Missing calibration file path '%s'.", __APP_NAME__, calibration_file.c_str());
    ros::shutdown();
    return -1;
  }

  cv::FileStorage fs(calibration_file, cv::FileStorage::READ);


  if (!fs.isOpened())
  {
    ROS_ERROR("[%s] Cannot open file calibration file '%s'", __APP_NAME__, calibration_file.c_str());
    ros::shutdown();
    return -1;
  }

  fs["CameraExtrinsicMat"] >> CameraExtrinsicMat;
  fs["CameraMat"] >> CameraMat;
  fs["DistCoeff"] >> DistCoeff;
  fs["ImageSize"] >> ImageSize;
  fs["DistModel"] >> DistModel;

  std::string image_topic_name;
  std::string camera_info_name;
  std::string projection_matrix_topic;

  private_nh.param<std::string>("image_topic_src", image_topic_name, "/image_raw");
  ROS_INFO("[%s] image_topic_name: %s", __APP_NAME__, image_topic_name.c_str());

  private_nh.param<std::string>("camera_info_topic", camera_info_name, "/camera_info");
  ROS_INFO("[%s] camera_info_name: %s", __APP_NAME__, camera_info_name.c_str());

  private_nh.param<std::string>("projection_matrix_topic", projection_matrix_topic, "/projection_matrix");
  ROS_INFO("[%s] projection_matrix_topic: %s", __APP_NAME__, projection_matrix_topic.c_str());

  instrinsics_parsed_ = false;
  extrinsics_parsed_ = false;

  std::string name_space_str = ros::this_node::getNamespace();
  if (name_space_str != "/")
  {
    image_topic_name = name_space_str + image_topic_name;
    camera_info_name = name_space_str + camera_info_name;
    projection_matrix_topic = name_space_str + projection_matrix_topic;
    if (name_space_str.substr(0, 2) == "//")
    {
      /* if name space obtained by ros::this::node::getNamespace()
         starts with "//", delete one of them */
      name_space_str.erase(name_space_str.begin());
    }
    camera_id_str_ = name_space_str;
  }

  image_sub = n.subscribe(image_topic_name, 10, image_raw_cb);

  manual_calib_x= n.subscribe("manual_calib_x", 10, manual_calib_x_cb);
  manual_calib_y= n.subscribe("manual_calib_y", 10, manual_calib_y_cb);
  manual_calib_z= n.subscribe("manual_calib_z", 10, manual_calib_z_cb);
  manual_calib_roll= n.subscribe("manual_calib_roll", 10, manual_calib_roll_cb);
  manual_calib_pitch= n.subscribe("manual_calib_pitch", 10, manual_calib_pitch_cb);
  manual_calib_yaw= n.subscribe("manual_calib_yaw", 10, manual_calib_yaw_cb);

  camera_info_pub = n.advertise<sensor_msgs::CameraInfo>(camera_info_name, 10, true);

  projection_matrix_pub = n.advertise<autoware_msgs::ProjectionMatrix>(projection_matrix_topic, 10, true);

    // tf::TransformListener listener;
    // while (n.ok()){
    // tf::StampedTransform transform;
    // try{
    //       listener.lookupTransform("cam02", "velodyne",ros::Time(0), transform);
    //       // listener.lookupTransform("cam00", "velodyne",ros::Time(0), transform);
    //       std::cout<<"listener.lookupTransform(cam02, velodyne,ros::Time(0), transform); "<<std::endl;
    //       std::cout<<"transform.getOrigin().x() = "<<transform.getOrigin().x()<<std::endl;
    //       std::cout<<"transform.getOrigin().y() = "<<transform.getOrigin().y()<<std::endl;
    //       std::cout<<"transform.getOrigin().z() = "<<transform.getOrigin().z()<<std::endl;
    //       double x,y,z,w;
    //       double yaw,pitch,roll;
    //       transform.getBasis().getEulerYPR(yaw,pitch,roll);
    //       std::cout<<"transform.getBasis().getEulerYPR(yaw,pitch,roll); yaw = "<<yaw*57.3<<std::endl;
    //       std::cout<<"transform.getBasis().getEulerYPR(yaw,pitch,roll); pitch = "<<pitch*57.3<<std::endl;
    //       std::cout<<"transform.getBasis().getEulerYPR(yaw,pitch,roll); roll = "<<roll*57.3<<std::endl;
    // }
    // catch (tf::TransformException ex){
    //   ROS_ERROR("%s",ex.what());
    //   ros::Duration(1.0).sleep();
    // }
    // }

  ros::spin();

  return 0;
}