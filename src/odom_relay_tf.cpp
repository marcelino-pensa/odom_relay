
// ROS
#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_broadcaster.h>

ros::Publisher pose_pub;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  geometry_msgs::Point pt = msg->pose.pose.position;
  geometry_msgs::Quaternion quat = msg->pose.pose.orientation;
  transform.setOrigin( tf::Vector3(pt.x, pt.y, pt.z) );
  tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "vislam", "base_link"));

  // geometry_msgs::PoseStamped pose;
  // pose.header = msg->header;
  // pose.header.frame_id = "fcu";
  // pose.pose.position = msg->pose.pose.position;
  // pose.pose.orientation = msg->pose.pose.orientation;
  // pose_pub.publish(pose);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_relay_tf");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/vislam/odometry", 5, odomCallback);
  
  ros::spin();

  return 0;
} 