#include <sensor_to_body_transform/sensor_to_body_transform.h>

namespace pointcloud_preprocessor {
SensortoBodyTransform::SensortoBodyTransform() : tfListener_(tfBuffer_) {}

SensortoBodyTransform::~SensortoBodyTransform() {}

void SensortoBodyTransform::onInit() {
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();

  pub_pc_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "pc_change_coordinates/output", 10);

  sub_pc_ =
      nh_.subscribe("input", 1, &SensortoBodyTransform::pc_callback, this);

  pnh_.param<std::string>("base_tf", base_tf_, "base_link");
  pnh_.param<std::string>("original_tf", original_tf_, "base_link");
}

void SensortoBodyTransform::pc_callback(
    const sensor_msgs::PointCloud2ConstPtr &pc_ptr) {

  // Change Coordinates

  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped =
        tfBuffer_.lookupTransform(base_tf_, original_tf_, ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
    return;
  }

  Eigen::Matrix4f mat =
      tf2::transformToEigen(transformStamped.transform).matrix().cast<float>();

  sensor_msgs::PointCloud2Ptr pc_out_msg(new sensor_msgs::PointCloud2);
  pcl_ros::transformPointCloud(mat, *pc_ptr, *pc_out_msg);
  pc_out_msg->header.frame_id = base_tf_;
  pc_out_msg->height = pc_ptr->height;
  pc_out_msg->width = pc_ptr->width;

  pub_pc_.publish(pc_out_msg);
  // ROS_INFO("converted");
}

} // namespace stereo_pointcloud_preprocessor

PLUGINLIB_EXPORT_CLASS(pointcloud_preprocessor::SensortoBodyTransform,
                       nodelet::Nodelet);
