#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <unordered_map>
#include "common/time.hpp"
class MarkerInitializer
{
public:
    MarkerInitializer(ros::NodeHandle nh) : nh_(nh)
    {
        // Get the value of 'k' from the ROS parameter server, default is 5
        // nh_.param<int>("k", k_, 5);

        // Get the subscribed and published topic names from the ROS parameter server
        // std::string subscribe_topic, publish_topic;
        // nh_.param<std::string>("subscribe_topic", subscribe_topic, "/tracking/objects_fusion_box_score");
        // nh_.param<std::string>("publish_topic", publish_topic, "/tracking/init/objects_fusion_box_score");

        // Create a private node handle to access private parameters
        ros::NodeHandle private_nh("~");

        // Get the value of 'k' from the private ROS parameter server, default is 5
        private_nh.param<int>("k", k_, 5);

        // Get the subscribed and published topic names from the private ROS parameter server
        std::string subscribe_topic, publish_topic;
        private_nh.param<std::string>("subscribe_topic", subscribe_topic, "/tracking/objects_fusion_box_score");
        private_nh.param<std::string>("publish_topic", publish_topic, "/tracking/init/objects_fusion_box_score");

        // Subscribe to the MarkerArray topic
        marker_sub_ = nh_.subscribe(subscribe_topic, 10, &MarkerInitializer::markerCallback, this);

        // Publish the initialized MarkerArray topic
        init_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(publish_topic, 10);
    }

    void markerCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
    {
        visualization_msgs::MarkerArray init_markers;
        autosense::common::Clock clock_builder;
        autosense::common::Clock clock_final;
        ROS_INFO_STREAM("====================== Function Begin =========================");
        for (const auto& marker : msg->markers)
        {   
            if (marker.action != 3)
            {   
                

                // ROS_INFO("This is not empty");
                int id = getIDFromMarkerText(marker.text);
                /*
                if (id == 0){
                    ROS_INFO("This is 0");
                }*/
                /*
                if (id == 0){
                    ROS_INFO("This is 0");
                }else if(id > 0){
                    ROS_INFO("id: %d", id);
                }else{
                    ROS_INFO("id-1: %d", id);
                }
                */
               /*
                if (id == 0) {
                    if (id_to_markers.find(0) != id_to_markers.end()) {
                        for (const auto& marker : id_to_markers[0]) {

                            ROS_INFO("Marker ID: %d, Marker Text: %s", marker.id, marker.text.c_str());
                        }
                    } else {
                        ROS_INFO("id_to_markers does not contain key 0");
                    }
                }*/

                // If the ID appears for the first time, perform initialization
                if (id_to_markers.find(id) == id_to_markers.end())
                {
                    id_to_markers[id].push_back(marker);

                    if (id == 0){
                        ROS_INFO("Push 0 to the id_to_markers");
                    }
                }
                else
                {
                    // If the ID has appeared before, check if it has reached 'k'
                    if (id_to_markers[id].size() < k_)
                    {
                        id_to_markers[id].push_back(marker);

                        if (id == 0){
                            ROS_INFO("size of 0:%d",id_to_markers[id].size());
                        }
                    }
                    else
                    {
                        // print marker position
                        // ROS_INFO("-------------------");
                        // ROS_INFO("Marker ID: %d, Position: x=%.3f, y=%.3f, z=%.3f", marker.id, marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);

                        id_to_markers[id].push_back(marker);

                        // Calculate the scale average of the first 'k' markers
                        visualization_msgs::Marker averaged_marker = computeAverageMarker(id_to_markers[id]);
                        averaged_marker.color.r = 1.0;
                        averaged_marker.color.b = 0.0;

                        // averaged_marker.pose.position.x = marker.pose.position.x;
                        // averaged_marker.pose.position.y = marker.pose.position.y;
                        // averaged_marker.pose.position.z = marker.pose.position.z;
                        // averaged_marker.pose.orientation = marker.pose.orientation;
                        // print average marker position
                        // ROS_INFO("Average Marker ID: %d, Position: x=%.3f, y=%.3f, z=%.3f", id, averaged_marker.pose.position.x, averaged_marker.pose.position.y, averaged_marker.pose.position.z);

                        init_markers.markers.push_back(averaged_marker);

                        // Remove the oldest marker
                        id_to_markers[id].erase(id_to_markers[id].begin());
                        // id_to_markers[id].push_back(marker);
                        /*if (id == 0){
                            ROS_INFO("calculate average");
                        }*/
                    }
                }
                
            }else{
                publishClearMarker(marker.header);
            } 
            /*else{
                ROS_INFO("This is empty");
                init_markers.markers.push_back();
            }*/
        }

        ROS_INFO_STREAM("Smooth. Took " << clock_builder.takeRealTime()<< "ms.");

        // Check if 'init_markers' is not empty before publishing 'clear_marker' and the initialized MarkerArray
        if (!init_markers.markers.empty())
        {
            // Publish 'clear_marker'
            // publishClearMarker(init_markers.markers[0].header);

            // Publish the initialized MarkerArray
            init_marker_pub_.publish(init_markers);
        }

        ROS_INFO_STREAM("MarkerCallback Took " << clock_final.takeRealTime()<< "ms.");
        ROS_INFO_STREAM("===============================================");
    }

    void publishClearMarker(const std_msgs::Header& header)
    {
        // clear all markers before
        visualization_msgs::MarkerArray empty_markers;  
        visualization_msgs::Marker clear_marker;
        clear_marker.header = header;
        clear_marker.ns = "objects";
        clear_marker.id = 0;
        clear_marker.action = clear_marker.DELETEALL;
        clear_marker.lifetime = ros::Duration();
        empty_markers.markers.push_back(clear_marker);
        init_marker_pub_.publish(empty_markers);
    }
    /*
    int getIDFromMarkerText(const std::string& text)
    {
        int id=0;
        if (sscanf(text.c_str(), "<ID= %d>", &id) == 1)
        {   
            return id;
        }else{
            return -1;
        }
        
    }*/

    int getIDFromMarkerText(const std::string& text)
    {
        int id = 0;
        if (sscanf(text.c_str(), "%*f m/s, <ID= %d>,", &id) == 1)
        {   
            return id;
        }
        else
        {
            return -1;
        }
    }

    visualization_msgs::Marker computeAverageMarker(const std::vector<visualization_msgs::Marker>& markers)
    {
        visualization_msgs::Marker averaged_marker = markers.back();
        // ROS_INFO("Back Marker ID: %d, Position: x=%.3f, y=%.3f, z=%.3f", averaged_marker.id, averaged_marker.pose.position.x, averaged_marker.pose.position.y, averaged_marker.pose.position.z);

        // Create a new marker for the first marker in the list
        // visualization_msgs::Marker first_marker = markers.front();
        // ROS_INFO("First Marker ID: %d, Position: x=%.3f, y=%.3f, z=%.3f", first_marker.id, first_marker.pose.position.x, first_marker.pose.position.y, first_marker.pose.position.z);

        // Calculate the scale average of the first 'k' markers
        double scale_x = 0.0;
        double scale_y = 0.0;
        double scale_z = 0.0;
        for (const auto& marker : markers)
        {
            scale_x += marker.scale.x;
            scale_y += marker.scale.y;
            scale_z += marker.scale.z;
        }

        averaged_marker.scale.x = scale_x / (k_+1);
        averaged_marker.scale.y = scale_y / (k_+1);
        averaged_marker.scale.z = scale_z / (k_+1);

        return averaged_marker;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber marker_sub_;
    ros::Publisher init_marker_pub_;
    std::unordered_map<int, std::vector<visualization_msgs::Marker>> id_to_markers;
    int k_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_initializer");
    ros::NodeHandle nh;

    MarkerInitializer marker_initializer(nh);

    ros::spin();

    return 0;
}

