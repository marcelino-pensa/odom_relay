
// ROS
#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

ros::Publisher pose_pub;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  pose.header.frame_id = "fcu";
  pose.pose.position = msg->pose.pose.position;
  pose.pose.orientation = msg->pose.pose.orientation;
  pose_pub.publish(pose);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_relay_lpe");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/camera/odom/sample_low_freq", 5, odomCallback);
  pose_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 5);
  
  ros::spin();

  return 0;
} 