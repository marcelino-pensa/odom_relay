
// ROS
#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/tf.h>

ros::Publisher odom_pub;
ros::Publisher laser_pub;

geometry_msgs::Vector3 set_vector3(const double &x, const double &y, const double &z) {
  geometry_msgs::Vector3 v;
  v.x = x; v.y = y; v.z = z;
  return v;
}

geometry_msgs::Vector3 convert_to_body_frame(
    const geometry_msgs::Quaternion &quat,
    const geometry_msgs::Vector3 &vec) {
  tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Quaternion q_world(vec.x, vec.y, vec.z, 0.0);
  tf::Quaternion q_vec_body = q.inverse()*q_world*q;
  geometry_msgs::Vector3 vec_body = set_vector3(q_vec_body.x(), q_vec_body.y(), q_vec_body.z());
  return vec_body;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  geometry_msgs::Vector3 vel = msg->twist.twist.linear;
  geometry_msgs::Vector3 ang_vel = msg->twist.twist.angular;
  geometry_msgs::Quaternion quat = msg->pose.pose.orientation;

  // prepare the message for publishing
  nav_msgs::Odometry odom = *msg;
  odom.child_frame_id = "imu_link";
  odom.twist.twist.linear = convert_to_body_frame(quat, vel);
  odom_pub.publish(odom);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  sensor_msgs::LaserScan laser;
  laser = *msg;
  laser.header.frame_id = "laser_link";
  laser_pub.publish(laser);

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_relay_tf");
  ros::NodeHandle n;

  ros::Subscriber sub_odom = n.subscribe("/vislam/odometry", 5, odomCallback);
  odom_pub = n.advertise<nav_msgs::Odometry>("/vislam/odometry_body", 5);

  // Republish laser_scan with different header
  ros::Subscriber sub_laser = n.subscribe("/laser/scan", 5, laserCallback);
  laser_pub = n.advertise<sensor_msgs::LaserScan>("/laser/scan_laser_link", 5);

  
  ros::spin();

  return 0;
} 