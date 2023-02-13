#pragma once
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <eigen3/Eigen/Dense>

#include <geometry_msgs/TransformStamped.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>

namespace pointcloud_preprocessor {

class SensortoBodyTransform : public nodelet::Nodelet {
public:
  SensortoBodyTransform();
  ~SensortoBodyTransform();

  virtual void onInit();

  // callback function
  void pc_callback(const sensor_msgs::PointCloud2ConstPtr &pc_ptr);

private:
  ros::NodeHandle nh_, pnh_;

  ros::Publisher pub_pc_;

  ros::Subscriber sub_pc_;

  // tf2 buffer
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  // object
  pcl::PointCloud<pcl::PointXYZRGB> pc_in_;
  pcl::PointCloud<pcl::PointXYZRGB> pc_out_;

  // parameters
  std::string base_tf_;

  std::string original_tf_;
};

} // namespace stereo_pointcloud_preprocessor
