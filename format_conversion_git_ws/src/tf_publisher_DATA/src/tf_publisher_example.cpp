#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  transform.setOrigin( tf::Vector3(0.1, 0.0, 0.2) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);

  ros::Rate rate(10.0);
  while (node.ok()){
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "link1"));
    rate.sleep();
  }
  return 0;
}

// #include <ros/ros.h>
// #include <tf/transform_broadcaster.h>

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "tf_publisher_example");
//     ros::NodeHandle nh;
    
//     // Define the translation and rotation data from the YAML file
//     tf::Vector3 translation(175.587300635, 181.411803641, -0.32539367676);
//     tf::Quaternion rotation(-0.00365339913944, -0.00208248063148, 
//                             0.968257754353, 0.24991846087);
    
//     // Construct a transform from the translation and rotation data
//     tf::Transform transform(rotation, translation);
    
//     // Create a TransformBroadcaster object
//     tf::TransformBroadcaster br;
    
//     ros::Rate rate(10);  // Set the rate at which the transform should be broadcast
//     while (ros::ok()) {
//         // Broadcast the transform
//         br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "imu"));
//         rate.sleep();
//     }
//     return 0;
// }

