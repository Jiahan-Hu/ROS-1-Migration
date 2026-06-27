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
#include <sensor_msgs/PointCloud2.h>
#include "autoware_msgs/ProjectionMatrix.h"
#include <iostream>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>


ros::Publisher new_pcd_publisher;
ros::Publisher new_pcd_transformed_publisher;
ros::Subscriber image_sub,pcd_sub;

ros::Time timeStampOfImage;

std::string image_raw_topic_str,point_cloud_topic_name_str,point_cloud_output_topic_name_str,lidar_new_frame_id_str,point_cloud_transformed_topic_name_str;
int which_camera,cross_check;//front 1, left 2, rear 3, right 4;


void image_raw_cb(sensor_msgs::Image image_msg)
{

  timeStampOfImage.sec = image_msg.header.stamp.sec;
  timeStampOfImage.nsec = image_msg.header.stamp.nsec;

}

void pcd_cb(sensor_msgs::PointCloud2 pcd_msg)
{
  if (cross_check)
  {
    pcd_msg.header.stamp.sec = timeStampOfImage.sec;
    pcd_msg.header.stamp.nsec = timeStampOfImage.nsec;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(pcd_msg, *cloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZI>());

  for (int i = 0; i < cloud->points.size(); ++i)
  {
    pcl::PointXYZI point,p_lidar;
    point=cloud->points[i];
    if (which_camera==1)//front
    {
      p_lidar=point;
    }else if (which_camera==2)//left
    {
      p_lidar=point;
      p_lidar.x=point.y;
      p_lidar.y=-point.x;
    }else if (which_camera==3)//rear
    {
      p_lidar=point;
      p_lidar.x=-point.x;
      p_lidar.y=-point.y;
    }else //right
    {
      p_lidar=point;
      p_lidar.x=-point.y;
      p_lidar.y=point.x;
    }
    cloud_transformed->points.push_back(p_lidar);
  }

  sensor_msgs::PointCloud2 pcd_transformed_output_;
  pcl::toROSMsg(*cloud_transformed, pcd_transformed_output_);
  pcd_transformed_output_.header.stamp = pcd_msg.header.stamp;
  pcd_transformed_output_.header.frame_id = pcd_msg.header.frame_id;
  new_pcd_transformed_publisher.publish(pcd_transformed_output_);




  pcd_msg.header.stamp.sec = timeStampOfImage.sec;
  pcd_msg.header.stamp.nsec = timeStampOfImage.nsec;
  pcd_msg.header.frame_id=lidar_new_frame_id_str;
  new_pcd_publisher.publish(pcd_msg);



}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pcd_republisher");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  private_nh.param<std::string>("image_raw_topic_str", image_raw_topic_str, "/zed2i_1/zed_node1/left/image_rect_color");
  private_nh.param<std::string>("point_cloud_topic_name_str", point_cloud_topic_name_str, "/new/rslidar_points");
  private_nh.param<std::string>("point_cloud_output_topic_name_str", point_cloud_output_topic_name_str, "/new/rslidar_points_repub");
  private_nh.param<std::string>("lidar_new_frame_id_str", lidar_new_frame_id_str, "newrslidar_repub");

  private_nh.param<std::string>("point_cloud_transformed_topic_name_str", point_cloud_transformed_topic_name_str, "/new/rslidar_points_transformed");
  private_nh.param<int>("which_camera", which_camera, 1);

  private_nh.param<int>("cross_check", cross_check, 1);

  image_sub = n.subscribe(image_raw_topic_str, 10, image_raw_cb);
  pcd_sub =   n.subscribe(point_cloud_topic_name_str, 10, pcd_cb);

  new_pcd_publisher=n.advertise<sensor_msgs::PointCloud2>(point_cloud_output_topic_name_str, 1000);
  new_pcd_transformed_publisher=n.advertise<sensor_msgs::PointCloud2>(point_cloud_transformed_topic_name_str, 1000);

  ros::spin();

  return 0;
}