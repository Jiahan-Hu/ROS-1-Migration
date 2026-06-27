#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/opencv.hpp>
#include "ros/console.h"

class ImageConverter
{
public:
    ImageConverter(const std::string& compressed_topic, const std::string& image_topic)
    {   std::cout << "\033[1;32m Converting CompressImg to RawImage! \033[0m" << std::endl;
        image_sub_ = nh_.subscribe(compressed_topic, 1, &ImageConverter::imageCb, this);
        image_pub_ = nh_.advertise<sensor_msgs::Image>(image_topic, 1);
    }

    void imageCb(const sensor_msgs::CompressedImageConstPtr& msg)
    {
        cv::Mat image = cv::imdecode(cv::Mat(msg->data), 1);
        if (!image.empty())
        {   
            ROS_INFO("Converting ImageCompress to Image...");
            // Convert OpenCV image to ROS message
            sensor_msgs::Image out_msg;
            out_msg.header = msg->header;
            out_msg.height = image.rows;
            out_msg.width = image.cols;
            out_msg.encoding = "bgr8";
            out_msg.is_bigendian = false;
            out_msg.step = image.cols * image.elemSize();
            size_t size = out_msg.step * image.rows;
            out_msg.data.resize(size);
            memcpy(&out_msg.data[0], image.data, size);

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
