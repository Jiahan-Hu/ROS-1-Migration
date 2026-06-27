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

#include "cnn_segmentation.h"
#include <math.h>

#if (CV_MAJOR_VERSION == 3)
#include "gencolors.cpp"
#else
#include <opencv2/contrib/contrib.hpp>
#endif

CNNSegmentation::CNNSegmentation() : nh_()
{
}

bool CNNSegmentation::init()
{

  ros::NodeHandle private_node_handle("~");//to receive args

  //***********ground removal parameters and variables***********************//
//************************************************************************//
  ground_seg_model.no_ground_topic=private_node_handle.param<std::string>("no_ground_point_topic", "/points_no_ground");
  ground_seg_model.ground_topic=private_node_handle.param<std::string>("ground_point_topic", "/points_ground");
  ground_seg_model.input_point_topic_ = private_node_handle.param<std::string>("input_point_topic", "/points_raw");
  ground_seg_model.DistanceThreshold = private_node_handle.param<double>("DistanceThreshold", 0.1);
  // ground_seg_model.points_cropped_topic = private_node_handle.param<std::string>("points_cropped_topic", "/cropped_points");

  std::cout<<"average_height = "<<ground_seg_model.average_height<<std::endl;
  ground_seg_model.average_height = private_node_handle.param<double>("average_height", -2);
  std::cout<<"average_height = "<<ground_seg_model.average_height<<std::endl;
  std::cout<<"height_offset = "<<ground_seg_model.height_offset<<std::endl;
  ground_seg_model.height_offset = private_node_handle.param<double>("height_offset", 0.5);
  std::cout<<"height_offset = "<<ground_seg_model.height_offset<<std::endl;

  //**********************************************************************//
  //**********************************************************************//

//***********lidar cluster parameters and variables***********************//
//************************************************************************//
  tf::StampedTransform transform;
  tf::TransformListener listener;
  tf::TransformListener vectormap_tf_listener;

  lidar_cluster_._vectormap_transform_listener = &vectormap_tf_listener;
  lidar_cluster_._transform = &transform;
  lidar_cluster_._transform_listener = &listener;

  #if (CV_MAJOR_VERSION == 3)
    generateColors(lidar_cluster_._colors, 255);
  #else
    cv::generateColors(lidar_cluster_._colors, 255);
  #endif

  private_node_handle.param("cluster_size_min", lidar_cluster_._cluster_size_min, 5);
  private_node_handle.param("cluster_size_max", lidar_cluster_._cluster_size_max, 100000);
  private_node_handle.param("pose_estimation", lidar_cluster_._pose_estimation, false);
  private_node_handle.param("cluster_merge_threshold", lidar_cluster_._cluster_merge_threshold, 1.5);
  private_node_handle.param<std::string>("output_frame", lidar_cluster_._output_frame, "rslidar");
  private_node_handle.param("clustering_distance", lidar_cluster_._clustering_distance, 0.75);
  private_node_handle.param("use_multiple_thres", lidar_cluster_._use_multiple_thres, false);
  std::cout<<"_clustering_distance = "<<lidar_cluster_._clustering_distance<<std::endl;
  std::string str_distances;
  std::string str_ranges;
  private_node_handle.param("clustering_distances", str_distances, std::string("[0.5,1.1,1.6,2.1,2.6]"));
  private_node_handle.param("clustering_ranges", str_ranges, std::string("[15,30,45,60]"));

  std::cout<<"str_distances = "<<str_distances<<std::endl;
  std::cout<<"str_ranges = "<<str_ranges<<std::endl;

  YAML::Node distances = YAML::Load(str_distances);
  YAML::Node ranges = YAML::Load(str_ranges);
  size_t distances_size = distances.size();
  size_t ranges_size = ranges.size();
  for (size_t i_distance = 0; i_distance < distances_size; i_distance++)
  {
    lidar_cluster_._clustering_distances.push_back(distances[i_distance].as<double>());
    std::cout<<distances[i_distance].as<double>()<<std::endl;
  }
  for (size_t i_range = 0; i_range < ranges_size; i_range++)
  {
    lidar_cluster_._clustering_ranges.push_back(ranges[i_range].as<double>());
    std::cout<<ranges[i_range].as<double>()<<std::endl;
  }
//**********************************************************************//
//**********************************************************************//

  private_node_handle.param("if_use_camera", lidar_cluster_.if_use_camera, false);

  std::string proto_file;
  std::string weight_file;


  if (private_node_handle.getParam("network_definition_file", proto_file))
  {
    ROS_INFO("[%s] network_definition_file: %s", __APP_NAME__, proto_file.c_str());
  } else
  {
    ROS_INFO("[%s] No Network Definition File was received. Finishing execution.", __APP_NAME__);
    return false;
  }
  if (private_node_handle.getParam("pretrained_model_file", weight_file))
  {
    ROS_INFO("[%s] Pretrained Model File: %s", __APP_NAME__, weight_file.c_str());
  } else
  {
    ROS_INFO("[%s] No Pretrained Model File was received. Finishing execution.", __APP_NAME__);
    return false;
  }

  

  private_node_handle.param<std::string>("object_yolo_f_topic_src", object_yolo_f_topic_src_, "/detection/image_detector/objects_f");
  private_node_handle.param<std::string>("object_yolo_r_topic_src", object_yolo_r_topic_src_, "/detection/image_detector/objects_r");


  private_node_handle.param<std::string>("points_src", topic_src_, "points_raw");
  ROS_INFO("[%s] points_src: %s", __APP_NAME__, topic_src_.c_str());

  private_node_handle.param<double>("range", range_, 60.);
  ROS_INFO("[%s] Pretrained Model File: %.2f", __APP_NAME__, range_);

  private_node_handle.param<double>("score_threshold", score_threshold_, 0.6);
  ROS_INFO("[%s] score_threshold: %.2f", __APP_NAME__, score_threshold_);

  private_node_handle.param<int>("width", width_, 512);
  ROS_INFO("[%s] width: %d", __APP_NAME__, width_);

  private_node_handle.param<int>("height", height_, 512);
  ROS_INFO("[%s] height: %d", __APP_NAME__, height_);

  private_node_handle.param<bool>("use_constant_feature", use_constant_feature_, false); // add code
  ROS_INFO("[%s] whether to use constant features: %d", __APP_NAME__, use_constant_feature_); // add code

  private_node_handle.param<bool>("use_gpu", use_gpu_, false);
  ROS_INFO("[%s] use_gpu: %d", __APP_NAME__, use_gpu_);

  private_node_handle.param<int>("gpu_device_id", gpu_device_id_, 0);
  ROS_INFO("[%s] gpu_device_id: %d", __APP_NAME__, gpu_device_id_);

  private_node_handle.param<std::string>("pub_detector_point_cluster", pub_point_cluster_, "/detection/lidar_detector/points_cluster");
  private_node_handle.param<std::string>("pub_detector_objects",pub_objects_,"/detection/lidar_detector/objects");
  private_node_handle.param<std::string>("pub_points_cluster",pub_points_cluster_,"/points_cluster");
  private_node_handle.param<std::string>("pub_cluster_centroids",pub_cluster_centroids_,"/cluster_centroids");
  private_node_handle.param<std::string>("pub_cloud_clusters",pub_cloud_clusters_,"/detection/lidar_detector/cloud_clusters");

  private_node_handle.param<double>("pc_adjustable_z",adjustable_z,-1.5);
  ROS_INFO("adjustable_z is: [%f] ", adjustable_z);

  /// Instantiate Caffe net
  if (!use_gpu_)
  {
    caffe::Caffe::set_mode(caffe::Caffe::CPU);
  }
  else
  {
    caffe::Caffe::SetDevice(gpu_device_id_);
    caffe::Caffe::set_mode(caffe::Caffe::GPU);
    caffe::Caffe::DeviceQuery();
  }

  caffe_net_.reset(new caffe::Net<float>(proto_file, caffe::TEST));
  caffe_net_->CopyTrainedLayersFrom(weight_file);


  std::string instance_pt_blob_name = "instance_pt";
  instance_pt_blob_ = caffe_net_->blob_by_name(instance_pt_blob_name);
  CHECK(instance_pt_blob_ != nullptr) << "`" << instance_pt_blob_name
                                      << "` layer required";

  std::string category_pt_blob_name = "category_score";
  category_pt_blob_ = caffe_net_->blob_by_name(category_pt_blob_name);
  CHECK(category_pt_blob_ != nullptr) << "`" << category_pt_blob_name
                                      << "` layer required";

  std::string confidence_pt_blob_name = "confidence_score";
  confidence_pt_blob_ = caffe_net_->blob_by_name(confidence_pt_blob_name);
  CHECK(confidence_pt_blob_ != nullptr) << "`" << confidence_pt_blob_name
                                        << "` layer required";

  std::string height_pt_blob_name = "height_pt";
  height_pt_blob_ = caffe_net_->blob_by_name(height_pt_blob_name);
  CHECK(height_pt_blob_ != nullptr) << "`" << height_pt_blob_name
                                    << "` layer required";

  std::string feature_blob_name = "data";
  feature_blob_ = caffe_net_->blob_by_name(feature_blob_name);
  CHECK(feature_blob_ != nullptr) << "`" << feature_blob_name
                                  << "` layer required";

  std::string class_pt_blob_name = "class_score";
  class_pt_blob_ = caffe_net_->blob_by_name(class_pt_blob_name);
  CHECK(class_pt_blob_ != nullptr) << "`" << class_pt_blob_name
                                   << "` layer required";

  cluster2d_.reset(new Cluster2D());
  if (!cluster2d_->init(height_, width_, range_))
  {
    ROS_ERROR("[%s] Fail to Initialize cluster2d for CNNSegmentation", __APP_NAME__);
    return false;
  }

  feature_generator_.reset(new FeatureGenerator());
  if (!feature_generator_->init(feature_blob_.get(), use_constant_feature_))  // add code
  {
    ROS_ERROR("[%s] Fail to Initialize feature generator for CNNSegmentation", __APP_NAME__);
    return false;
  }

  return true;
}

bool CNNSegmentation::segment(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_ptr,
                              const pcl::PointIndices &valid_idx,
                              autoware_msgs::DetectedObjectArray &objects)
{
  int num_pts = static_cast<int>(pc_ptr->points.size());
  if (num_pts == 0)
  {
    ROS_INFO("[%s] Empty point cloud.", __APP_NAME__);
    return true;
  }

  feature_generator_->generate(pc_ptr);

// network forward process
  caffe_net_->Forward();
#ifndef USE_CAFFE_GPU
//  caffe::Caffe::set_mode(caffe::Caffe::CPU);
#else
//  int gpu_id = 0;
//   caffe::Caffe::SetDevice(gpu_id);
//    caffe::Caffe::set_mode(caffe::Caffe::GPU);
//    caffe::Caffe::DeviceQuery();
#endif

  // clutser points and construct segments/objects
  float objectness_thresh = 0.5;
  bool use_all_grids_for_clustering = true;
  cluster2d_->cluster(*category_pt_blob_, *instance_pt_blob_, pc_ptr,
                      valid_idx, objectness_thresh,
                      use_all_grids_for_clustering);
  cluster2d_->filter(*confidence_pt_blob_, *height_pt_blob_);
  cluster2d_->classify(*class_pt_blob_);
  float confidence_thresh = score_threshold_;
  float height_thresh = 0.5;
  int min_pts_num = 3;
  cluster2d_->getObjects(confidence_thresh, height_thresh, min_pts_num,
                         objects, message_header_);
  return true;
}

void CNNSegmentation::test_run()
{
  std::string in_pcd_file = "uscar_12_1470770225_1470770492_1349.pcd";

  pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile(in_pcd_file, *in_pc_ptr);


  pcl::PointIndices valid_idx;
  auto &indices = valid_idx.indices;
  indices.resize(in_pc_ptr->size());
  std::iota(indices.begin(), indices.end(), 0);

  autoware_msgs::DetectedObjectArray objects;
  init();
  segment(in_pc_ptr, valid_idx, objects);


}

void CNNSegmentation::combine_objects(autoware_msgs::DetectedObjectArray &objects_array1, autoware_msgs::DetectedObjectArray &objects_array2)
{
  double range_threshold=40;
  double x,y,distance;
  for (int i = 0; i < objects_array2.objects.size(); ++i)
  {
    autoware_msgs::DetectedObject detected_object=objects_array2.objects[i];
    x=detected_object.pose.position.x;
    y=detected_object.pose.position.y;
    distance=sqrt(x*x+y*y);

    if (distance>=range_threshold)
    {
      objects_array1.objects.push_back(detected_object);
    }
    
  }
}


void CNNSegmentation::run()
{
  // init();
  if(this->init()){  // add code
    ROS_INFO("The network init successfully!");  // add code
  }else{  // add code
    ROS_ERROR("The network init fail!!!");  // add code
  }  // add code

  if (lidar_cluster_.if_use_camera)
  {
    // points_sub_ = nh_.subscribe(topic_src_, 1, &CNNSegmentation::pointsCallback_use_camera, this);
    points_sub_ = nh_.subscribe(topic_src_, 1, &CNNSegmentation::pointsCallback, this);
  }else
  {
    points_sub_ = nh_.subscribe(topic_src_, 1, &CNNSegmentation::pointsCallback, this);
  }
  
  objects_from_yolo_f_sub=nh_.subscribe(object_yolo_f_topic_src_, 1, &CNNSegmentation::objects_yolo_f_callback, this);
  objects_from_yolo_r_sub=nh_.subscribe(object_yolo_r_topic_src_, 1, &CNNSegmentation::objects_yolo_r_callback, this);

  points_sub_ = nh_.subscribe(topic_src_, 1, &CNNSegmentation::pointsCallback, this);


  // points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/detection/lidar_detector/points_cluster", 1);
  // objects_pub_ = nh_.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/objects", 1);
  points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pub_point_cluster_, 1);
  objects_pub_ = nh_.advertise<autoware_msgs::DetectedObjectArray>(pub_objects_, 1);

  ground_seg_model.points_no_ground_pub=nh_.advertise<sensor_msgs::PointCloud2>(ground_seg_model.no_ground_topic, 100);//xx
  ground_seg_model.points_ground_pub=nh_.advertise<sensor_msgs::PointCloud2>(ground_seg_model.ground_topic, 100);//xx
  ground_seg_model.points_cropped_pub=nh_.advertise<sensor_msgs::PointCloud2>("cropped_points", 100);//xx 
  // ground_seg_model.points_cropped_pub=nh_.advertise<sensor_msgs::PointCloud2>(ground_seg_model.points_cropped_topic, 100);

  // lidar_cluster_._pub_cluster_cloud = nh_.advertise<sensor_msgs::PointCloud2>("/points_cluster", 1);
  // lidar_cluster_._centroid_pub = nh_.advertise<autoware_msgs::Centroids>("/cluster_centroids", 1);
  // lidar_cluster_._pub_clusters_message = nh_.advertise<autoware_msgs::CloudClusterArray>("/detection/lidar_detector/cloud_clusters", 1);
  // lidar_cluster_._pub_detected_objects = nh_.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/objects", 1);
  lidar_cluster_._pub_cluster_cloud = nh_.advertise<sensor_msgs::PointCloud2>(pub_points_cluster_, 1);
  lidar_cluster_._centroid_pub = nh_.advertise<autoware_msgs::Centroids>(pub_cluster_centroids_, 1);
  lidar_cluster_._pub_clusters_message = nh_.advertise<autoware_msgs::CloudClusterArray>(pub_cloud_clusters_, 1);
  lidar_cluster_._pub_detected_objects = nh_.advertise<autoware_msgs::DetectedObjectArray>(pub_objects_, 1);


  ROS_INFO("[%s] Ready. Waiting for data...", __APP_NAME__);
}

void CNNSegmentation::objects_yolo_f_callback(const autoware_msgs::DetectedObjectArray &msg)
{
  objects_front_cam.push_back(msg);
}

void CNNSegmentation::objects_yolo_r_callback(const autoware_msgs::DetectedObjectArray &msg)
{
  objects_rear_cam.push_back(msg);
}

void CNNSegmentation::pointsCallback_use_camera(const sensor_msgs::PointCloud2 &msg)
{
  std::chrono::system_clock::time_point start, end;
  start = std::chrono::system_clock::now();

  pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(msg, *in_pc_ptr);
  pcl::PointIndices valid_idx;
  auto &indices = valid_idx.indices;
  indices.resize(in_pc_ptr->size());
  std::iota(indices.begin(), indices.end(), 0);
  message_header_ = msg.header;

  autoware_msgs::DetectedObjectArray objects;
  objects.header = message_header_;
  segment(in_pc_ptr, valid_idx, objects);

  pubColoredPoints(objects);

  objects_pub_.publish(objects);

  end = std::chrono::system_clock::now();
  double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();


  /* ground removal and clusering starts */
  // pcl::PointCloud<PointT>::Ptr output_plane_points (new pcl::PointCloud<PointT>());
  // pcl::PointCloud<PointT>::Ptr output_nonplane_points (new pcl::PointCloud<PointT>());
  // ground_seg_model.ground_segmentation(in_pc_ptr,output_plane_points,output_nonplane_points,msg);
  // autoware_msgs::DetectedObjectArray objects_euclidean;
  // lidar_cluster_._velodyne_header = msg.header;
  // if (!output_nonplane_points->empty())
  // {
  //   pcl::PointCloud<PointT>::Ptr ROI_nonplane_points (new pcl::PointCloud<PointT>());
  //   for (int i = 0; i < output_nonplane_points->points.size(); ++i)
  //   {
  //     double distance_=0;
  //     PointT point=output_nonplane_points->points[i];
  //     distance_=sqrt(point.x*point.x+point.y*point.y);
  //     if (distance_>38)
  //     {
  //       ROI_nonplane_points->points.push_back(point);
  //     }
  //   }
  //   if (ROI_nonplane_points->points.size()>50)
  //   {
  //     lidar_cluster_.lidar_cluster(ROI_nonplane_points, objects_euclidean);
  //   }
    
  // }
  
  /* ground removal and clusering ends*/

  // autoware_msgs::DetectedObjectArray objects_total=objects;
  // combine_objects(objects_total,objects_euclidean);



}

void CNNSegmentation::pointsCallback(const sensor_msgs::PointCloud2 &msg)
{
  std::chrono::system_clock::time_point start, end;
  start = std::chrono::system_clock::now();

  pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(msg, *in_pc_ptr);
  
  // adjust the z of the point cloud
  // ROS_INFO("adjustable_z is: [%f] ", adjustable_z);
  if (adjustable_z != 0.0){
    for (int i=0;i < in_pc_ptr->points.size();++i)
    {
      in_pc_ptr->points[i].z=in_pc_ptr->points[i].z + adjustable_z;
    }    
  }

  pcl::PointIndices valid_idx;
  auto &indices = valid_idx.indices;
  indices.resize(in_pc_ptr->size());
  std::iota(indices.begin(), indices.end(), 0);
  message_header_ = msg.header;

  autoware_msgs::DetectedObjectArray objects;
  objects.header = message_header_;
  segment(in_pc_ptr, valid_idx, objects);

  pubColoredPoints(objects);

  objects_pub_.publish(objects);

  end = std::chrono::system_clock::now();
  double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();


  /* ground removal and clusering starts */
  // pcl::PointCloud<PointT>::Ptr output_plane_points (new pcl::PointCloud<PointT>());
  // pcl::PointCloud<PointT>::Ptr output_nonplane_points (new pcl::PointCloud<PointT>());
  // ground_seg_model.ground_segmentation(in_pc_ptr,output_plane_points,output_nonplane_points,msg);
  // autoware_msgs::DetectedObjectArray objects_euclidean;
  // lidar_cluster_._velodyne_header = msg.header;
  // if (!output_nonplane_points->empty())
  // {
  //   pcl::PointCloud<PointT>::Ptr ROI_nonplane_points (new pcl::PointCloud<PointT>());
  //   for (int i = 0; i < output_nonplane_points->points.size(); ++i)
  //   {
  //     double distance_=0;
  //     PointT point=output_nonplane_points->points[i];
  //     distance_=sqrt(point.x*point.x+point.y*point.y);
  //     if (distance_>38)
  //     {
  //       ROI_nonplane_points->points.push_back(point);
  //     }
  //   }
  //   if (ROI_nonplane_points->points.size()>50)
  //   {
  //     lidar_cluster_.lidar_cluster(ROI_nonplane_points, objects_euclidean);
  //   }
    
  // }
  
  /* ground removal and clusering ends*/

  // autoware_msgs::DetectedObjectArray objects_total=objects;
  // combine_objects(objects_total,objects_euclidean);



}

void CNNSegmentation::pubColoredPoints(const autoware_msgs::DetectedObjectArray &objects_array)
{
  pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
  for (size_t object_i = 0; object_i < objects_array.objects.size(); object_i++)
  {
    // std::cout << "objct i" << object_i << std::endl;
    pcl::PointCloud<pcl::PointXYZI> object_cloud;
    pcl::fromROSMsg(objects_array.objects[object_i].pointcloud, object_cloud);
    int red = (object_i) % 256;
    int green = (object_i * 7) % 256;
    int blue = (object_i * 13) % 256;

    for (size_t i = 0; i < object_cloud.size(); i++)
    {
      // std::cout << "point i" << i << "/ size: "<<object_cloud.size()  << std::endl;
      pcl::PointXYZRGB colored_point;
      colored_point.x = object_cloud[i].x;
      colored_point.y = object_cloud[i].y;
      colored_point.z = object_cloud[i].z;
      colored_point.r = red;
      colored_point.g = green;
      colored_point.b = blue;
      colored_cloud.push_back(colored_point);
    }
  }
  sensor_msgs::PointCloud2 output_colored_cloud;
  pcl::toROSMsg(colored_cloud, output_colored_cloud);
  output_colored_cloud.header = message_header_;
  points_pub_.publish(output_colored_cloud);
}
