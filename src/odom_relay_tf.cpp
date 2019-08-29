// ROS
#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_broadcaster.h>

ros::Publisher pose_pub;
std::string odom_topic, frame_id, child_frame_id;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  geometry_msgs::Point pt = msg->pose.pose.position;
  geometry_msgs::Quaternion quat = msg->pose.pose.orientation;
  transform.setOrigin( tf::Vector3(pt.x, pt.y, pt.z) );
  tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, frame_id, child_frame_id));

  // geometry_msgs::PoseStamped pose;
  // pose.header = msg->header;
  // pose.header.frame_id = "fcu";
  // pose.pose.position = msg->pose.pose.position;
  // pose.pose.orientation = msg->pose.pose.orientation;
  // pose_pub.publish(pose);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle n("~");

  n.getParam("odom_topic", odom_topic);
  n.getParam("frame_id", frame_id);
  n.getParam("child_frame_id", child_frame_id);

  std::cout << "parameters:" << std::endl;
  std::cout << odom_topic << " " << frame_id << " " << " " << child_frame_id << std::endl;

  ros::Subscriber sub = n.subscribe("/vislam/odometry", 5, odomCallback);
  
  ros::spin();

  return 0;
} 