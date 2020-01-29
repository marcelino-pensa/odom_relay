// ROS
#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

ros::Publisher pose_pub;
std::string odom_topic, frame_id, child_frame_id;

// static inline bool convertPointCloudToPointCloud2 (const sensor_msgs::PointCloud &input, sensor_msgs::PointCloud2 &output) {
//  output.header = input.header;
//  output.width  = input.points.size ();
//  output.height = 1;
//  output.fields.resize (3 + input.channels.size ());
//  // Convert x/y/z to fields
//  output.fields[0].name = "x"; output.fields[1].name = "y"; output.fields[2].name = "z";
//  int offset = 0;
//  // All offsets are *4, as all field data types are float32
//  for (size_t d = 0; d < output.fields.size (); ++d, offset += 4)
//  {
//    output.fields[d].offset = offset;
//    output.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
//    output.fields[d].count  = 1;
//  }
//  output.point_step = offset;
//  output.row_step   = output.point_step * output.width;
//  // Convert the remaining of the channels to fields
//  for (size_t d = 0; d < input.channels.size (); ++d)
//    output.fields[3 + d].name = input.channels[d].name;
//  output.data.resize (input.points.size () * output.point_step);
//  output.is_bigendian = false;  // @todo ?
//  output.is_dense     = false;

//  // Copy the data points
//  for (size_t cp = 0; cp < input.points.size (); ++cp)
//  {
//    memcpy (&output.data[cp * output.point_step + output.fields[0].offset], &input.points[cp].x, sizeof (float));
//    memcpy (&output.data[cp * output.point_step + output.fields[1].offset], &input.points[cp].y, sizeof (float));
//    memcpy (&output.data[cp * output.point_step + output.fields[2].offset], &input.points[cp].z, sizeof (float));
//    for (size_t d = 0; d < input.channels.size (); ++d)
//    {
//      if (input.channels[d].values.size() == input.points.size())
//      {
//        memcpy (&output.data[cp * output.point_step + output.fields[3 + d].offset], &input.channels[d].values[cp], sizeof (float));
//      }
//    }
//  }
//  return (true);
// }

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

class My_Filter {
     public:
        My_Filter();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
     private:
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener listener_;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_;
};

My_Filter::My_Filter(){
        scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/laser/scan", 100, &My_Filter::scanCallback, this);
        point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/laser/cloud", 100, false);
        // tfListener_.setExtrapolationLimit(ros::Duration(0.1));
}

void My_Filter::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in) {
  if(!listener_.waitForTransform(
        scan_in->header.frame_id,
        "/laser_frame",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0))){
     return;
  }

  // sensor_msgs::PointCloud  cloud;
  sensor_msgs::PointCloud2 cloud2;
  projector_.transformLaserScanToPointCloud("/laser_frame",*scan_in,
          cloud2,listener_);
  cloud2.is_dense = true;
  // convertPointCloudToPointCloud2(cloud, cloud2);

  point_cloud_publisher_.publish(cloud2);
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

  // ros::Subscriber sub_laser = n.subscribe("/laser/scan", 5, laserCallback);
  // pcl_pub = n.advertise<sensor_msgs::LaserScan>("/laser/scan_laser_link", 5);
  My_Filter filter;

  ros::spin();

  return 0;
} 