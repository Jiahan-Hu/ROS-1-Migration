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
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 */

#include "lidar_cluster.h"

bool init_matrix = false;
cv::Mat cameraExtrinsicMat;
cv::Mat cameraMat;
cv::Mat distCoeff;
cv::Size imageSize;
cv::Mat invRt, invTt;

void lidar_cluster_class::lidar_cluster(pcl::PointCloud<PointT>::Ptr cloud, autoware_msgs::DetectedObjectArray &objects_euclidean_)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clustered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  autoware_msgs::Centroids centroids;
  autoware_msgs::CloudClusterArray cloud_clusters;

  for (int i = 0; i < cloud->size(); ++i)
  {
    pcl::PointXYZ p;
    p.x=cloud->points[i].x;
    p.y=cloud->points[i].y;
    p.z=cloud->points[i].z;

    if (p.z<1)
    {
      cloud_xyz_ptr->points.push_back(p);
    }
    
  }

  std::cout<<"cloud_xyz_ptr size "<<cloud_xyz_ptr->size()<<std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clustered_cloud_total_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  autoware_msgs::CloudClusterArray cloud_clusters_total;

  if (!cloud_xyz_ptr->empty())
  {
    segmentByDistance(cloud_xyz_ptr, colored_clustered_cloud_ptr, centroids,cloud_clusters);
    if (!colored_clustered_cloud_ptr->empty())
    {
      for (int i = 0; i < colored_clustered_cloud_ptr->points.size(); ++i)
      {
        colored_clustered_cloud_total_ptr->points.push_back(colored_clustered_cloud_ptr->points[i]);
      }

      for (auto i = cloud_clusters.clusters.begin(); i != cloud_clusters.clusters.end(); i++)
      {
        cloud_clusters_total.clusters.push_back(*i);
      }

    }

  }

  if (!colored_clustered_cloud_total_ptr->empty())
  {
    publishColorCloud(&_pub_cluster_cloud, colored_clustered_cloud_total_ptr);
    // centroids.header = _velodyne_header;
    // publishCentroids(&_centroid_pub, centroids, _output_frame, _velodyne_header);
    cloud_clusters_total.header = _velodyne_header;
    publishCloudClusters(&_pub_clusters_message, cloud_clusters_total, _output_frame, _velodyne_header,objects_euclidean_);
  }
}

std::vector<ClusterPtr> lidar_cluster_class::clusterAndColor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                                        autoware_msgs::Centroids &in_out_centroids,
                                        double in_max_cluster_distance = 0.5)
{

  // std::cout<<"clustering algorithm is running"<<std::endl;

  // std::cout<<"in_cloud_ptr size = "<<in_cloud_ptr->points.size()<<std::endl;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  // create 2d pc
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*in_cloud_ptr, *cloud_2d);
  // make it flat
  for (size_t i = 0; i < cloud_2d->points.size(); i++)
  {
    cloud_2d->points[i].z = 0;
  }

  // std::cout<<"cloud_2d size = "<<cloud_2d->points.size()<<std::endl;

  if (cloud_2d->points.size() > 0)
    tree->setInputCloud(cloud_2d);

  // std::cout<<"test 1 "<<std::endl;

  std::vector<pcl::PointIndices> cluster_indices;

  // perform clustering on 2d cloud
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

  // std::cout<<"in_max_cluster_distance "<<in_max_cluster_distance<<std::endl;
  ec.setClusterTolerance(in_max_cluster_distance);  //
  // std::cout<<"_cluster_size_min "<<_cluster_size_min<<std::endl;
  ec.setMinClusterSize(_cluster_size_min);
  // std::cout<<"_cluster_size_max "<<_cluster_size_max<<std::endl;
  ec.setMaxClusterSize(_cluster_size_max);
  ec.setSearchMethod(tree);
  // std::cout<<"test 2 "<<std::endl;
  ec.setInputCloud(cloud_2d);
  // std::cout<<"test 3 "<<std::endl;
  ec.extract(cluster_indices);
  // use indices on 3d cloud
  // std::cout<<"test 4 "<<std::endl;

  // std::cout<<"cluster_indices size =  "<<cluster_indices.size()<<std::endl;

  // std::cout<<"_pose_estimation "<<_pose_estimation<<std::endl;

  /////////////////////////////////
  //---  3. Color clustered points
  /////////////////////////////////
  unsigned int k = 0;
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

  std::vector<ClusterPtr> clusters;
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);//coord + color
  // cluster
  for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    ClusterPtr cluster(new Cluster());
    // std::cout<<"test 5 "<<std::endl;

    // std::cout<<"_velodyne_header.frame_id =  "<<_velodyne_header.frame_id<<std::endl;

    // std::cout<<"it->indices size =  "<<it->indices.size()<<std::endl;
    
    cluster->SetCloud(in_cloud_ptr, it->indices, _velodyne_header, k, (int) _colors[k].val[0],
                      (int) _colors[k].val[1],
                      (int) _colors[k].val[2], "", _pose_estimation);
    // std::cout<<"test 6 "<<std::endl;
    clusters.push_back(cluster);

    k++;
  }
  // std::cout<<"test 7 "<<std::endl;
  // std::cout << "Clusters: " << k << std::endl;
  return clusters;
}

void lidar_cluster_class::checkClusterMerge(size_t in_cluster_id, std::vector<ClusterPtr> &in_clusters,
                       std::vector<bool> &in_out_visited_clusters, std::vector<size_t> &out_merge_indices,
                       double in_merge_threshold)
{
  // std::cout << "checkClusterMerge" << std::endl;
  pcl::PointXYZ point_a = in_clusters[in_cluster_id]->GetCentroid();
  for (size_t i = 0; i < in_clusters.size(); i++)
  {
    if (i != in_cluster_id && !in_out_visited_clusters[i])
    {
      pcl::PointXYZ point_b = in_clusters[i]->GetCentroid();
      double distance = sqrt(pow(point_b.x - point_a.x, 2) + pow(point_b.y - point_a.y, 2));
      if (distance <= in_merge_threshold)
      {
        in_out_visited_clusters[i] = true;
        out_merge_indices.push_back(i);
        // std::cout << "Merging " << in_cluster_id << " with " << i << " dist:" << distance << std::endl;
        checkClusterMerge(i, in_clusters, in_out_visited_clusters, out_merge_indices, in_merge_threshold);
      }
    }
  }
}

void lidar_cluster_class::mergeClusters(const std::vector<ClusterPtr> &in_clusters, std::vector<ClusterPtr> &out_clusters,
                   std::vector<size_t> in_merge_indices, const size_t &current_index,
                   std::vector<bool> &in_out_merged_clusters)
{
  // std::cout << "mergeClusters:" << in_merge_indices.size() << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB> sum_cloud;
  pcl::PointCloud<pcl::PointXYZ> mono_cloud;
  ClusterPtr merged_cluster(new Cluster());
  for (size_t i = 0; i < in_merge_indices.size(); i++)
  {
    sum_cloud += *(in_clusters[in_merge_indices[i]]->GetCloud());
    in_out_merged_clusters[in_merge_indices[i]] = true;
  }
  std::vector<int> indices(sum_cloud.points.size(), 0);
  for (size_t i = 0; i < sum_cloud.points.size(); i++)
  {
    indices[i] = i;
  }

  if (sum_cloud.points.size() > 0)
  {
    pcl::copyPointCloud(sum_cloud, mono_cloud);
    merged_cluster->SetCloud(mono_cloud.makeShared(), indices, _velodyne_header, current_index,
                             (int) _colors[current_index].val[0], (int) _colors[current_index].val[1],
                             (int) _colors[current_index].val[2], "", _pose_estimation);
    out_clusters.push_back(merged_cluster);
  }
}

void lidar_cluster_class::checkAllForMerge(std::vector<ClusterPtr> &in_clusters, std::vector<ClusterPtr> &out_clusters,
                      float in_merge_threshold)
{
  // std::cout << "checkAllForMerge" << std::endl;
  std::vector<bool> visited_clusters(in_clusters.size(), false);
  std::vector<bool> merged_clusters(in_clusters.size(), false);
  size_t current_index = 0;
  for (size_t i = 0; i < in_clusters.size(); i++)
  {
    if (!visited_clusters[i])
    {
      visited_clusters[i] = true;
      std::vector<size_t> merge_indices;
      checkClusterMerge(i, in_clusters, visited_clusters, merge_indices, in_merge_threshold);
      mergeClusters(in_clusters, out_clusters, merge_indices, current_index++, merged_clusters);
    }
  }
  for (size_t i = 0; i < in_clusters.size(); i++)
  {
    // check for clusters not merged, add them to the output
    if (!merged_clusters[i])
    {
      out_clusters.push_back(in_clusters[i]);
    }
  }

  // ClusterPtr cluster(new Cluster());
}

void lidar_cluster_class::segmentByDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                       autoware_msgs::Centroids &in_out_centroids, autoware_msgs::CloudClusterArray &in_out_clusters)
{
  // cluster the pointcloud according to the distance of the points using different thresholds (not only one for the
  // entire pc)
  // in this way, the points farther in the pc will also be clustered

  // 0 => 0-15m d=0.5
  // 1 => 15-30 d=1
  // 2 => 30-45 d=1.6
  // 3 => 45-60 d=2.1
  // 4 => >60   d=2.6

  std::vector<ClusterPtr> all_clusters;

  if (!_use_multiple_thres)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
    {
      pcl::PointXYZ current_point;
      current_point.x = in_cloud_ptr->points[i].x;
      current_point.y = in_cloud_ptr->points[i].y;
      current_point.z = in_cloud_ptr->points[i].z;

      cloud_ptr->points.push_back(current_point);
    }

    all_clusters =
        clusterAndColor(cloud_ptr, out_cloud_ptr, in_out_centroids, _clustering_distance);

  } 

  else
  {
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_segments_array(2);
    for (unsigned int i = 0; i < cloud_segments_array.size(); i++)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      cloud_segments_array[i] = tmp_cloud;
    }

    for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
    {
      pcl::PointXYZ current_point;
      current_point.x = in_cloud_ptr->points[i].x;
      current_point.y = in_cloud_ptr->points[i].y;
      current_point.z = in_cloud_ptr->points[i].z;

      float origin_distance = sqrt(pow(current_point.x, 2) + pow(current_point.y, 2));

      // std::cout<<"_clustering_ranges[0] = "<<_clustering_ranges[0]<<std::endl;

      if (origin_distance < _clustering_ranges[0])
      {
        cloud_segments_array[0]->points.push_back(current_point);
      }
      else
      {
        cloud_segments_array[1]->points.push_back(current_point);
      }

      // if (origin_distance < _clustering_ranges[0])
      // {
      //   cloud_segments_array[0]->points.push_back(current_point);
      // }
      // else if (origin_distance < _clustering_ranges[1])
      // {
      //   cloud_segments_array[1]->points.push_back(current_point);

      // }

      // else if (origin_distance < _clustering_ranges[2])
      // {
      //   cloud_segments_array[2]->points.push_back(current_point);

      // }else if (origin_distance < _clustering_ranges[3])
      // {
      //   cloud_segments_array[3]->points.push_back(current_point);

      // }else
      // {
      //   cloud_segments_array[4]->points.push_back(current_point);
      // }

    }

    std::vector<ClusterPtr> local_clusters;
    for (unsigned int i = 0; i < cloud_segments_array.size(); i++)
    {

      local_clusters = clusterAndColor(
          cloud_segments_array[i], out_cloud_ptr, in_out_centroids, _clustering_distances[i]);
      all_clusters.insert(all_clusters.end(), local_clusters.begin(), local_clusters.end());
    }
  }

  std::cout<<"all_clusters size  = "<<all_clusters.size()<<std::endl;


  // Clusters can be merged or checked in here
  //....
  // check for mergable clusters
  std::vector<ClusterPtr> mid_clusters;
  std::vector<ClusterPtr> final_clusters;

  if (all_clusters.size() > 0)
    checkAllForMerge(all_clusters, mid_clusters, _cluster_merge_threshold);
  else
    mid_clusters = all_clusters;

  if (mid_clusters.size() > 0)
    checkAllForMerge(mid_clusters, final_clusters, _cluster_merge_threshold);
  else
    final_clusters = mid_clusters;

    // Get final PointCloud to be published
    for (unsigned int i = 0; i < final_clusters.size(); i++)
    {
      *out_cloud_ptr = *out_cloud_ptr + *(final_clusters[i]->GetCloud());

      jsk_recognition_msgs::BoundingBox bounding_box = final_clusters[i]->GetBoundingBox();
      geometry_msgs::PolygonStamped polygon = final_clusters[i]->GetPolygon();
      jsk_rviz_plugins::Pictogram pictogram_cluster;
      pictogram_cluster.header = _velodyne_header;

      // PICTO
      pictogram_cluster.mode = pictogram_cluster.STRING_MODE;
      pictogram_cluster.pose.position.x = final_clusters[i]->GetMaxPoint().x;
      pictogram_cluster.pose.position.y = final_clusters[i]->GetMaxPoint().y;
      pictogram_cluster.pose.position.z = final_clusters[i]->GetMaxPoint().z;
      tf::Quaternion quat(0.0, -0.7, 0.0, 0.7);
      tf::quaternionTFToMsg(quat, pictogram_cluster.pose.orientation);
      pictogram_cluster.size = 4;
      std_msgs::ColorRGBA color;
      color.a = 1;
      color.r = 1;
      color.g = 1;
      color.b = 1;
      pictogram_cluster.color = color;
      pictogram_cluster.character = std::to_string(i);
      // PICTO

      // pcl::PointXYZ min_point = final_clusters[i]->GetMinPoint();
      // pcl::PointXYZ max_point = final_clusters[i]->GetMaxPoint();
      pcl::PointXYZ center_point = final_clusters[i]->GetCentroid();
      geometry_msgs::Point centroid;
      centroid.x = center_point.x;
      centroid.y = center_point.y;
      centroid.z = center_point.z;
      bounding_box.header = _velodyne_header;
      polygon.header = _velodyne_header;

      if (final_clusters[i]->IsValid())
      {

        in_out_centroids.points.push_back(centroid);

        autoware_msgs::CloudCluster cloud_cluster;
        final_clusters[i]->ToROSMessage(_velodyne_header, cloud_cluster);
        in_out_clusters.clusters.push_back(cloud_cluster);
      }
    }
}



void lidar_cluster_class::publishCloud(const ros::Publisher *in_publisher, const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr)
{
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
  cloud_msg.header = _velodyne_header;
  in_publisher->publish(cloud_msg);
}

void lidar_cluster_class::publishColorCloud(const ros::Publisher *in_publisher,
                       const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_to_publish_ptr)
{
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
  cloud_msg.header = _velodyne_header;
  in_publisher->publish(cloud_msg);
}

void lidar_cluster_class::publishCentroids(const ros::Publisher *in_publisher, const autoware_msgs::Centroids &in_centroids,
                      const std::string &in_target_frame, const std_msgs::Header &in_header)
{
  if (in_target_frame != in_header.frame_id)
  {
    autoware_msgs::Centroids centroids_transformed;
    centroids_transformed.header = in_header;
    centroids_transformed.header.frame_id = in_target_frame;
    for (auto i = centroids_transformed.points.begin(); i != centroids_transformed.points.end(); i++)
    {
      geometry_msgs::PointStamped centroid_in, centroid_out;
      centroid_in.header = in_header;
      centroid_in.point = *i;
      try
      {
        _transform_listener->transformPoint(in_target_frame, ros::Time(), centroid_in, in_header.frame_id,
                                            centroid_out);

        centroids_transformed.points.push_back(centroid_out.point);
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("publishCentroids: %s", ex.what());
      }
    }
    in_publisher->publish(centroids_transformed);
  } else
  {
    in_publisher->publish(in_centroids);
  }
}

void lidar_cluster_class::publishDetectedObjects(const autoware_msgs::CloudClusterArray &in_clusters,autoware_msgs::DetectedObjectArray &objects_euclidean_)
{
  autoware_msgs::DetectedObjectArray detected_objects;
  detected_objects.header = in_clusters.header;

  for (size_t i = 0; i < in_clusters.clusters.size(); i++)
  {
    autoware_msgs::DetectedObject detected_object;
    detected_object.header = in_clusters.header;
    detected_object.label = "unknown";
    detected_object.score = 1.;
    detected_object.space_frame = in_clusters.header.frame_id;
    detected_object.pose = in_clusters.clusters[i].bounding_box.pose;
    detected_object.dimensions = in_clusters.clusters[i].dimensions;
    detected_object.pointcloud = in_clusters.clusters[i].cloud;
    detected_object.convex_hull = in_clusters.clusters[i].convex_hull;
    detected_object.valid = true;

    detected_objects.objects.push_back(detected_object);
    objects_euclidean_.objects.push_back(detected_object);
  }
  _pub_detected_objects.publish(detected_objects);
}

void lidar_cluster_class::publishCloudClusters(const ros::Publisher *in_publisher, const autoware_msgs::CloudClusterArray &in_clusters,
                          const std::string &in_target_frame, const std_msgs::Header &in_header,autoware_msgs::DetectedObjectArray &objects_euclidean_)
{
  if (in_target_frame != in_header.frame_id)
  {
    autoware_msgs::CloudClusterArray clusters_transformed;
    clusters_transformed.header = in_header;
    clusters_transformed.header.frame_id = in_target_frame;
    for (auto i = in_clusters.clusters.begin(); i != in_clusters.clusters.end(); i++)
    {
      autoware_msgs::CloudCluster cluster_transformed;
      cluster_transformed.header = in_header;
      try
      {
        _transform_listener->lookupTransform(in_target_frame, _velodyne_header.frame_id, ros::Time(),
                                             *_transform);
        pcl_ros::transformPointCloud(in_target_frame, *_transform, i->cloud, cluster_transformed.cloud);
        _transform_listener->transformPoint(in_target_frame, ros::Time(), i->min_point, in_header.frame_id,
                                            cluster_transformed.min_point);
        _transform_listener->transformPoint(in_target_frame, ros::Time(), i->max_point, in_header.frame_id,
                                            cluster_transformed.max_point);
        _transform_listener->transformPoint(in_target_frame, ros::Time(), i->avg_point, in_header.frame_id,
                                            cluster_transformed.avg_point);
        _transform_listener->transformPoint(in_target_frame, ros::Time(), i->centroid_point, in_header.frame_id,
                                            cluster_transformed.centroid_point);

        cluster_transformed.dimensions = i->dimensions;
        cluster_transformed.eigen_values = i->eigen_values;
        cluster_transformed.eigen_vectors = i->eigen_vectors;

        cluster_transformed.convex_hull = i->convex_hull;
        cluster_transformed.bounding_box.pose.position = i->bounding_box.pose.position;
        if(_pose_estimation)
        {
          cluster_transformed.bounding_box.pose.orientation = i->bounding_box.pose.orientation;
        }
        else
        {
          cluster_transformed.bounding_box.pose.orientation.w = 1.0;
        }
        clusters_transformed.clusters.push_back(cluster_transformed);
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("publishCloudClusters: %s", ex.what());
      }
    }
    in_publisher->publish(clusters_transformed);
    publishDetectedObjects(clusters_transformed,objects_euclidean_);
  } else
  {
    in_publisher->publish(in_clusters);
    publishDetectedObjects(in_clusters,objects_euclidean_);
  }
}



