# KF tracking code

```c++
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>

using namespace Eigen;

class KF_Tracker
{
  public:
    KF_Tracker()
    {
      // Initialize Kalman filter
      x_ = Vector2d::Zero();
      P_ = Matrix2d::Identity();
      F_ = Matrix2d::Identity();
      Q_ = Matrix2d::Identity() * 0.1;
      H_ = Matrix2d::Identity();
      R_ = Matrix2d::Identity() * 0.1;
    }

    void callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
    {
      // Get the latest marker position
      Vector2d z = Vector2d(msg->markers[0].pose.position.x, msg->markers[0].pose.position.y);

      // Prediction step
      x_ = F_ * x_;
      P_ = F_ * P_ * F_.transpose() + Q_;

      // Update step
      Vector2d y = z - H_ * x_;
      Matrix2d S = H_ * P_ * H_.transpose() + R_;
      Matrix2d K = P_ * H_.transpose() * S.inverse();
      x_ = x_ + K * y;
      P_ = (Matrix2d::Identity() - K * H_) * P_;
    }

  private:
    Vector2d x_;
    Matrix2d P_;
    Matrix2d F_;
    Matrix2d Q_;
    Matrix2d H_;
    Matrix2d R_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kf_tracker");
  ros::NodeHandle nh;

  KF_Tracker tracker;
  ros::Subscriber sub = nh.subscribe("marker_array", 1, &KF_Tracker::callback, &tracker);

  ros::spin();

  return 0;
}

```

```c++
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

class KF_Tracker
{
  public:
    ...
    KF_Tracker()
    {
      ...
      marker_pub_ = nh_.advertise<visualization_msgs::Marker>("kf_trajectory", 1);
      marker_.header.frame_id = "/world";
      marker_.type = visualization_msgs::Marker::LINE_STRIP;
      marker_.scale.x = 0.05;
      marker_.color.b = 1.0;
      marker_.color.a = 1.0;
    }

    void callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
    {
      ...

      // Publish KF trajectory
      marker_.points.push_back(geometry_msgs::Point());
      marker_.points.back().x = x_(0);
      marker_.points.back().y = x_(1);
      marker_pub_.publish(marker_);
    }

  private:
    ...
    ros::Publisher marker_pub_;
    visualization_msgs::Marker marker_;
};

```

```C++
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>

using namespace Eigen;

class KF_Tracker
{
  public:
    KF_Tracker()
    {
      // Initialize Kalman filter
      x_ = Vector2d::Zero();
      P_ = Matrix2d::Identity();
      F_ = Matrix2d::Identity();
      Q_ = Matrix2d::Identity() * 0.1;
      H_ = Matrix2d::Identity();
      R_ = Matrix2d::Identity() * 0.1;

      // Initialize publisher
      marker_pub_ = nh_.advertise<visualization_msgs::Marker>("kf_trajectory", 1);
      marker_.header.frame_id = "/world";
      marker_.type = visualization_msgs::Marker::LINE_STRIP;
      marker_.scale.x = 0.05;
      marker_.color.b = 1.0;
      marker_.color.a = 1.0;
    }

    void callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
    {
      // Get the latest marker position
      Vector2d z = Vector2d(msg->markers[0].pose.position.x, msg->markers[0].pose.position.y);

      // Prediction step
      x_ = F_ * x_;
      P_ = F_ * P_ * F_.transpose() + Q_;

      // Update step
      Vector2d y = z - H_ * x_;
      Matrix2d S = H_ * P_ * H_.transpose() + R_;
      Matrix2d K = P_ * H_.transpose() * S.inverse();
      x_ = x_ + K * y;
      P_ = (Matrix2d::Identity() - K * H_) * P_;

      // Publish KF trajectory
      marker_.points.push_back(geometry_msgs::Point());
      marker_.points.back().x = x_(0);
      marker_.points.back().y = x_(1);
      marker_pub_.publish(marker_);
    }

  private:
    Vector2d x_;
    Matrix2d P_;
    Matrix2d F_;
    Matrix2d Q_;
    Matrix2d H_;
    Matrix2d R_;
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    visualization_msgs::Marker marker_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kf_tracker");
  ros::NodeHandle nh;

  KF_Tracker tracker;
  ros::Subscriber sub = nh.subscribe("marker_array", 1, &KF_Tracker::callback, &tracker);

  ros::spin();

  return 0;
}
```

