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
 */
#ifndef CNN_SEGMENTATION_H
#define CNN_SEGMENTATION_H

#include <chrono>
#include <numeric>

#include <ros/ros.h>

#include "caffe/caffe.hpp"

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <autoware_msgs/DetectedObjectArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <std_msgs/Header.h>

#include "cluster2d.h"
#include "feature_generator.h"

#include <ground_seg.h>
#include "lidar_cluster.h"

#include <yaml-cpp/yaml.h>

#define __APP_NAME__ "lidar_apollo_cnn_seg_detect"

struct CellStat
{
  CellStat() : point_num(0), valid_point_num(0)
  {
  }

  int point_num;
  int valid_point_num;
};

class CNNSegmentation
{
public:
  CNNSegmentation();

  void run();

  void test_run();

private:

  ground_seg ground_seg_model;
  lidar_cluster_class lidar_cluster_;

  double range_, score_threshold_;
  int width_;
  int height_;
  bool use_constant_feature_;  // add code
  std_msgs::Header message_header_;
  std::string topic_src_;
  std::string object_yolo_f_topic_src_,object_yolo_r_topic_src_;
  std::string pub_point_cluster_,pub_objects_,pub_points_cluster_,pub_cluster_centroids_,pub_cloud_clusters_;
  double adjustable_z;

  int gpu_device_id_;
  bool use_gpu_;

  // nodehandle
  ros::NodeHandle nh_;

  // publisher
  ros::Publisher points_pub_;
  ros::Publisher objects_pub_;

  // subscriber
  ros::Subscriber points_sub_,objects_from_yolo_f_sub,objects_from_yolo_r_sub;

  std::shared_ptr<caffe::Net<float>> caffe_net_;

  std::vector<autoware_msgs::DetectedObjectArray> objects_front_cam,objects_rear_cam;

  // center offset prediction
  boost::shared_ptr<caffe::Blob<float>> instance_pt_blob_;
  // objectness prediction
  boost::shared_ptr<caffe::Blob<float>> category_pt_blob_;
  // fg probability prediction
  boost::shared_ptr<caffe::Blob<float>> confidence_pt_blob_;
  // object height prediction
  boost::shared_ptr<caffe::Blob<float>> height_pt_blob_;
  // raw features to be input into network
  boost::shared_ptr<caffe::Blob<float>> feature_blob_;
  // class prediction
  boost::shared_ptr<caffe::Blob<float>> class_pt_blob_;

  // clustering model for post-processing
  std::shared_ptr<Cluster2D> cluster2d_;

  // bird-view raw feature generator
  std::shared_ptr<FeatureGenerator> feature_generator_;

  bool init();

  bool segment(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_ptr,
               const pcl::PointIndices &valid_idx,
               autoware_msgs::DetectedObjectArray &objects);

  void pointsCallback(const sensor_msgs::PointCloud2 &msg);

  void pointsCallback_use_camera(const sensor_msgs::PointCloud2 &msg);

  void pubColoredPoints(const autoware_msgs::DetectedObjectArray &objects);

  void combine_objects(autoware_msgs::DetectedObjectArray &objects_array1, 
      autoware_msgs::DetectedObjectArray &objects_array2);
  void objects_yolo_f_callback(const autoware_msgs::DetectedObjectArray &msg);
  void objects_yolo_r_callback(const autoware_msgs::DetectedObjectArray &msg);
};

#endif //CNN_SEGMENTATION_H
