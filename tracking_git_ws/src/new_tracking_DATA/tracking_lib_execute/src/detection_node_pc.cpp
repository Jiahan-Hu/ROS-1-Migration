/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <memory>

#include "common/msgs/autosense_msgs/PointCloud2Array.h"
#include "common/msgs/autosense_msgs/TrackingFixedTrajectoryArray.h"
#include "common/msgs/autosense_msgs/TrackingObjectArray.h"
#include "common/msgs/autosense_msgs/TrackingObjectArray_local_frame.h"


#include "common/bounding_box.hpp"
#include "common/color.hpp"
#include "common/parameter.hpp"
#include "common/publisher.hpp"
#include "common/time.hpp"
#include "common/transform.hpp"
#include "common/types/object.hpp"
#include "common/types/type.h"
#include "object_builders/object_builder_manager.hpp"
#include "tracking/tracking_worker_manager.hpp"

#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

const std::string param_ns_prefix_ = "tracking";  // NOLINT
std::string local_frame_id_, global_frame_id_;    // NOLINT
int tf_timeout_ms_ = 0;
double threshold_contian_IoU_ = 0.0;
autosense::TrackingWorkerParams tracking_params_;
// ROS Subscriber
ros::Subscriber pcs_segmented_sub_;
ros::Subscriber detector_objects_sub_;
std::unique_ptr<tf::TransformListener> tf_listener_;
// ROS Publisher
ros::Publisher segments_coarse_pub_;
ros::Publisher det_objects_boxscorelabel_pub_; // new for testing
ros::Publisher segments_predict_pub_;
ros::Publisher segments_pub_;
ros::Publisher tracking_output_objects_pub_,tracking_output_objects_local_pub_;
ros::Publisher tracking_output_trajectories_pub_;
ros::Publisher tracking_objects_pub_;
ros::Publisher tracking_objects_cloud_pub_;
ros::Publisher tracking_objects_velocity_pub_;
ros::Publisher tracking_objects_trajectory_pub_;
ros::Publisher tracking_objects_box_pub_;
ros::Publisher tracking_objects_box_score_pub_;
ros::Publisher tracking_objects_fusion_box_score_pub_;
ros::Publisher tracking_objects_fusion_score_label_pub_;
ros::Publisher tracking_objects_header_box_score_pub_; // new for testing
ros::Publisher tracking_objects_header_box_score_label_pub_; // new for testing 

// global variables: detector_objects
autoware_msgs::DetectedObjectArray detector_objects_;
// objects colors topics
std::string objects_box_score_color_;

/// @note Core components
std::unique_ptr<autosense::object_builder::BaseObjectBuilder> object_builder_ =
    nullptr;
std::unique_ptr<autosense::tracking::BaseTrackingWorker> tracking_worker_ =
    nullptr;

// TODO(Zhaoliang Zheng): callback function for OnDetectorObjects
void OnDetectorObjects(const autoware_msgs::DetectedObjectArrayConstPtr &msg) {
    // put the msg into global variable detector_objects_
    detector_objects_ = *msg;
    ROS_INFO("Detector objects size: %d at %lf.", msg->objects.size(),
             msg->header.stamp.toSec());
}

// TODO(chenshengjie): callback function as fast as possible
void OnSegmentClouds(const autosense_msgs::PointCloud2ArrayConstPtr &segments_msg) 
{
    const double kTimeStamp = segments_msg->header.stamp.toSec();
    ROS_INFO("Clusters size: %d at %lf.", segments_msg->clouds.size(),
             kTimeStamp);

    std_msgs::Header header;
    header.frame_id = local_frame_id_;
    header.stamp = segments_msg->header.stamp;

    // initial coarse segments directly from segment node or after classified by
    // learning node
    std::vector<autosense::PointICloudPtr> segment_clouds;
    for (size_t i = 0u; i < segments_msg->clouds.size(); ++i) {
        autosense::PointICloudPtr cloud(new autosense::PointICloud);
        pcl::fromROSMsg(segments_msg->clouds[i], *cloud);
        segment_clouds.push_back(cloud);
    }

    // current pose
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    // getVelodynePose: transform.hpp 
    auto status = autosense::common::transform::getVelodynePose(
        *tf_listener_, local_frame_id_, global_frame_id_, kTimeStamp, &pose);
    if (!status) {
        ROS_WARN("Failed to fetch current pose, tracking skipped...");
        return;
    }
    auto velo2world = std::make_shared<Eigen::Matrix4d>(pose);

    // object builder
    autosense::common::Clock clock_builder;
    std::vector<autosense::ObjectPtr> objects;
    object_builder_->build(segment_clouds, &objects);
    ROS_INFO_STREAM("Objects built. Took " << clock_builder.takeRealTime()
                                           << "ms.");

    // define color:
    std_msgs::ColorRGBA box_color;
    if (objects_box_score_color_ == "GREEN"){
        box_color = autosense::common::GREEN.rgbA;
    }else if (objects_box_score_color_ == "RED"){
        box_color = autosense::common::RED.rgbA;
    }else if (objects_box_score_color_ == "BLUE"){
        box_color = autosense::common::BLUE.rgbA;
    }else if (objects_box_score_color_ == "YELLOW"){
        box_color = autosense::common::YELLOW.rgbA;
    }else if (objects_box_score_color_ == "CYAN"){
        box_color = autosense::common::CYAN.rgbA;
    }

    // visualize initial coarse segments (need to be checked)
    // topic name: /tracking/segments_coarse
    // from #include "common/publisher.hpp"
    autosense::common::publishObjectsMarkers(
        segments_coarse_pub_, header, autosense::common::MAGENTA.rgbA, objects);

    // zzl new 
    autosense::common::publishObjectsMarkersDetBoxScoreLabel(det_objects_boxscorelabel_pub_, header,
                                             box_color,
                                             objects, detector_objects_);
    
    ROS_INFO_STREAM("==================================================");
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "tracking_node");

    // Node handle
    ros::NodeHandle nh = ros::NodeHandle();
    ros::NodeHandle private_nh = ros::NodeHandle("~");
    ros::AsyncSpinner spiner(1);

    // Load ROS parameters from rosparam server
    private_nh.getParam(param_ns_prefix_ + "/local_frame_id", local_frame_id_);
    private_nh.getParam(param_ns_prefix_ + "/global_frame_id",
                        global_frame_id_);
    private_nh.getParam(param_ns_prefix_ + "/tf_timeout_ms", tf_timeout_ms_);

    std::string sub_pcs_segmented_topic,sub_lidar_detector_objects_topic;
    int sub_pcs_queue_size;
    private_nh.getParam(param_ns_prefix_ + "/sub_pcs_segmented_topic",
                        sub_pcs_segmented_topic);
    private_nh.getParam(param_ns_prefix_ + "/sub_pcs_queue_size",
                        sub_pcs_queue_size);
    private_nh.getParam(param_ns_prefix_ + "/sub_lidar_detector_objects_topic",
                        sub_lidar_detector_objects_topic);

    std::string pub_segments_coarse_topic, pub_segments_predict_topic,
        pub_segments_topic;
    private_nh.getParam(param_ns_prefix_ + "/pub_segments_coarse_topic",
                        pub_segments_coarse_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_segments_predict_topic",
                        pub_segments_predict_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_segments_topic",
                        pub_segments_topic);

    std::string pub_output_objects_topic,pub_output_objects_local_topic, pub_output_trajectories_topic;
    private_nh.getParam(param_ns_prefix_ + "/pub_output_objects_topic",
                        pub_output_objects_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_output_objects_local_topic",
                        pub_output_objects_local_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_output_trajectories_topic",
                        pub_output_trajectories_topic);

    std::string det_objects_boxscorelabel_topic,pub_tracking_objects_header_box_score_label_topic,
        pub_tracking_objects_header_box_score_topic,
        pub_tracking_objects_fusion_score_label_topic,
        pub_tracking_objects_fusion_box_score_topic,
        pub_tracking_objects_box_score_topic,
        pub_tracking_objects_box_topic,
        pub_tracking_objects_topic, 
        pub_tracking_objects_cloud_topic,
        pub_tracking_objects_velocity_topic,
        pub_tracking_objects_trajectory_topic;
    private_nh.getParam(param_ns_prefix_ + "/pub_tracking_objects_topic",
                        pub_tracking_objects_topic);
    
    private_nh.getParam(param_ns_prefix_ + "/pub_tracking_objects_box_topic",
                        pub_tracking_objects_box_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_tracking_objects_box_score_topic",
                        pub_tracking_objects_box_score_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_tracking_objects_fusion_box_score_topic",
                        pub_tracking_objects_fusion_box_score_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_tracking_objects_fusion_score_label_topic",
                        pub_tracking_objects_fusion_score_label_topic);    
                        
    private_nh.getParam(param_ns_prefix_ + "/pub_tracking_objects_cloud_topic",
                        pub_tracking_objects_cloud_topic);
    private_nh.getParam(
        param_ns_prefix_ + "/pub_tracking_objects_velocity_topic",
        pub_tracking_objects_velocity_topic);
    private_nh.getParam(
        param_ns_prefix_ + "/pub_tracking_objects_trajectory_topic",
        pub_tracking_objects_trajectory_topic);
    // new header
    private_nh.getParam(param_ns_prefix_ + "/pub_tracking_objects_header_box_score_topic",
                    pub_tracking_objects_header_box_score_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_tracking_objects_header_box_score_label_topic",
                    pub_tracking_objects_header_box_score_label_topic);
    // new detection 
    private_nh.getParam(param_ns_prefix_ + "/det_objects_boxscorelabel_topic",
                    det_objects_boxscorelabel_topic);

    // threshold
    private_nh.param<double>(param_ns_prefix_ + "/threshold_contian_IoU",
                             threshold_contian_IoU_, 1.0);
    
    // tracking objects colors
    private_nh.getParam(param_ns_prefix_ + "/objects_box_score_color_topic",
                        objects_box_score_color_);

    tracking_params_ =
        autosense::common::getTrackingWorkerParams(nh, param_ns_prefix_);

    // Init core compoments
    object_builder_ = autosense::object_builder::createObjectBuilder();
    if (nullptr == object_builder_) {
        ROS_FATAL("Failed to create object_builder_.");
        return -1;
    }
    tracking_worker_ =
        autosense::tracking::createTrackingWorker(tracking_params_);
    if (nullptr == tracking_worker_) {
        ROS_FATAL("Failed to create tracking_worker_.");
        return -1;
    }

    // Init subscribers and publishers
    pcs_segmented_sub_ = nh.subscribe<autosense_msgs::PointCloud2Array>(
        sub_pcs_segmented_topic, sub_pcs_queue_size, OnSegmentClouds);
    // subscribe autoware_msgs/DetectedObjectArray
    detector_objects_sub_ = nh.subscribe<autoware_msgs::DetectedObjectArray>(
        sub_lidar_detector_objects_topic, sub_pcs_queue_size,
        OnDetectorObjects);

    tf_listener_.reset(new tf::TransformListener);
    // segments
    segments_coarse_pub_ =
        private_nh.advertise<visualization_msgs::MarkerArray>(
            pub_segments_coarse_topic, 1);
    segments_predict_pub_ =
        private_nh.advertise<visualization_msgs::MarkerArray>(
            pub_segments_predict_topic, 1);
    segments_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>(
        pub_segments_topic, 1);
    // tracking infos for debugging
    // tracking_objects_pub_ =
    //     private_nh.advertise<visualization_msgs::MarkerArray>(
    //         pub_tracking_objects_topic, 1);
    // zzl 
    // tracking_objects_box_pub_ = 
    //     private_nh.advertise<visualization_msgs::MarkerArray>(
    //         pub_tracking_objects_box_topic, 1);
    // tracking_objects_box_score_pub_ = 
    //     private_nh.advertise<visualization_msgs::MarkerArray>(
    //         pub_tracking_objects_box_score_topic, 1);    
    // tracking_objects_fusion_box_score_pub_ = 
    //     private_nh.advertise<visualization_msgs::MarkerArray>(
    //         pub_tracking_objects_fusion_box_score_topic, 1);
    // tracking_objects_fusion_score_label_pub_ = 
    //     private_nh.advertise<visualization_msgs::MarkerArray>(
    //         pub_tracking_objects_fusion_score_label_topic, 1);
    // // new for testing 
    // tracking_objects_header_box_score_pub_ = 
    //     private_nh.advertise<visualization_header_msgs::MarkerArrayHeader>(
    //         pub_tracking_objects_header_box_score_topic, 1);
    // tracking_objects_header_box_score_label_pub_ = 
    //     private_nh.advertise<visualization_header_msgs::MarkerArrayHeader>(
    //         pub_tracking_objects_header_box_score_label_topic, 1);        
    // zzl new detection
    det_objects_boxscorelabel_pub_ =        
        private_nh.advertise<visualization_msgs::MarkerArray>(
            det_objects_boxscorelabel_topic, 1);


    // others
    // tracking_objects_cloud_pub_ =
    //     private_nh.advertise<sensor_msgs::PointCloud2>(
    //         pub_tracking_objects_cloud_topic, 1);
    // tracking_objects_velocity_pub_ =
    //     private_nh.advertise<visualization_msgs::MarkerArray>(
    //         pub_tracking_objects_velocity_topic, 1);
    // tracking_objects_trajectory_pub_ =
    //     private_nh.advertise<visualization_msgs::MarkerArray>(
    //         pub_tracking_objects_trajectory_topic, 1);

    // the whole tracking output in the global frame
    tracking_output_objects_pub_ =
        private_nh.advertise<autosense_msgs::TrackingObjectArray>(
            pub_output_objects_topic, 1);

    // the whole tracking output in the local frame
    tracking_output_objects_local_pub_ =
        private_nh.advertise<autosense_msgs::TrackingObjectArray_local_frame>(
            pub_output_objects_local_topic, 1);

    tracking_output_trajectories_pub_ =
        private_nh.advertise<autosense_msgs::TrackingFixedTrajectoryArray>(
            pub_output_trajectories_topic, 1);

    spiner.start();
    ROS_INFO("tracking_node started...");

    ros::waitForShutdown();
    ROS_INFO("tracking_node exited...");

    return 0;
}
