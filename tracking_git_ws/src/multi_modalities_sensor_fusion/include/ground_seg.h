#ifndef GROUND_SEG_H_
#define GROUND_SEG_H_

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
// #include <ros_utils.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_search.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

using std::atan2;
using std::cos;
using std::sin;

using std::cout;
using std::endl;
using std::vector;
using std::setprecision;

typedef pcl::PointXYZI PointT;

class ground_seg{

    public:
        ros::Publisher points_no_ground_pub,points_ground_pub,points_cropped_pub;

        std::string no_ground_topic, ground_topic,input_point_topic_ ;

        pcl::Filter<PointT>::Ptr downsample_filter;
        // boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());

        double average_height,height_offset;
        double DistanceThreshold;

        void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,pcl::PointCloud<PointT> &cloud_out, float thres);
        pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud);
        void ransac_plane_fitting(pcl::PointCloud<PointT>::Ptr input_points, 
              pcl::PointCloud<PointT>::Ptr &output_plane_points_, 
              pcl::PointCloud<PointT>::Ptr &output_nonplane_points_);
        void combine_cloud(pcl::PointCloud<PointT>::Ptr &cloud_1, 
           pcl::PointCloud<PointT>::Ptr cloud_2);

        void ground_segmentation(pcl::PointCloud<PointT>::Ptr cloud, 
             pcl::PointCloud<PointT>::Ptr &output_plane_points__,
             pcl::PointCloud<PointT>::Ptr &output_nonplane_points__,
             sensor_msgs::PointCloud2 laserCloudMsg);

        void velydyne_handler(sensor_msgs::PointCloud2 laserCloudMsg);
};



#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_SEGMENTATION_CNNSEG_UTIL_H_
