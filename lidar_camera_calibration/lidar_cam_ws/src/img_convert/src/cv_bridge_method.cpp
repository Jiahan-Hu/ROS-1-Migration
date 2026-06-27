#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ImageConverter
{
public:
    ImageConverter(const std::string& compressed_topic, const std::string& image_topic)
    {
        image_sub_ = nh_.subscribe(compressed_topic, 1, &ImageConverter::imageCb, this);
        image_pub_ = nh_.advertise<sensor_msgs::Image>(image_topic, 1);
    }

    void imageCb(const sensor_msgs::CompressedImageConstPtr& msg)
    {
        cv::Mat image = cv::imdecode(cv::Mat(msg->data), 1);
        if (!image.empty())
        {
            sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
            image_pub_.publish(out_msg);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Publisher image_pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    
    // Get subscriber and publisher topics from ROS parameters
    ros::NodeHandle nh("~");
    std::string compressed_topic, image_topic;
    nh.param<std::string>("compressed_topic", compressed_topic, "/compressed_image");
    nh.param<std::string>("image_topic", image_topic, "/image");
    
    ImageConverter ic(compressed_topic, image_topic);
    ros::spin();
    return 0;
}