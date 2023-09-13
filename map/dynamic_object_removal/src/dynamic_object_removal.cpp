#include "dynamic_object_removal.hpp"

// Constructor
DynamicObjectRemoval::DynamicObjectRemoval() : Node("dynamic_object_removal_node")
{
  pcl_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output/pointcloud", 10);

  // Subscribe to topics
  pcl_sub_.subscribe(this, "input/pointcloud");
  obj_sub_.subscribe(this, "input/objects");

  // Synchronize the incoming messages
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,
                                                          autoware_auto_perception_msgs::msg::DetectedObjects>
      MySyncPolicy;
  sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), pcl_sub_, obj_sub_);
  sync_->registerCallback(&DynamicObjectRemoval::callback, this);
}

// Callback function
void DynamicObjectRemoval::callback(
    const std::shared_ptr<const sensor_msgs::msg::PointCloud2>& pcl_msg,
    const std::shared_ptr<const autoware_auto_perception_msgs::msg::DetectedObjects>& obj_msg)
{
  // Convert ROS2 PointCloud2 to PCL PointCloud
  PointCloudXYZI::Ptr cloud(new PointCloudXYZI);
  pcl::fromROSMsg(*pcl_msg, *cloud);

  // Loop through detected objects
  for (const auto& detected_obj : obj_msg->objects)
  {
    const auto& object_pose = detected_obj.kinematics.pose_with_covariance.pose;
    const auto& object_dimension = detected_obj.shape.dimensions;

    Eigen::Quaterniond quat(object_pose.orientation.w, object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z);
    Eigen::Affine3d detected_obj_pose = Eigen::Translation3d(object_pose.position.x, object_pose.position.y, object_pose.position.z) * quat;

    // Extract Euler angles
    Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(2, 1, 0);
    double yaw = euler[0];

    // Define object dimensions for cropping
    Eigen::Vector4f min_point, max_point;
    min_point << -abs(object_dimension.x) / 2 - 0.25, -abs(object_dimension.y) / 2 - 0.25, -abs(object_dimension.z) / 2 - 0.1, 1.0;
    max_point << abs(object_dimension.x) / 2 + 0.25, abs(object_dimension.y) / 2 + 0.25, abs(object_dimension.z) / 2 + 0.25, 1.0;

    // Remove object using objectRemoveCropBox function
    objectRemoveCropBox(cloud, min_point, max_point, detected_obj_pose.translation().cast<float>(), yaw);
  }

  // Convert back to ROS2 PointCloud2 and publish
  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(*cloud, output_msg);
  pcl_publisher_->publish(output_msg);
}

// Function to remove objects using CropBox
void DynamicObjectRemoval::objectRemoveCropBox(PointCloudXYZI::Ptr crop_cloud, const Eigen::Vector4f min_point,
                                               const Eigen::Vector4f max_point, const Eigen::Vector3f translation,
                                               double orientation_yaw)
{
  PointCloudXYZI xyz_filtered_cloud;
  pcl::CropBox<pcl::PointXYZI> ObjectRemoveCrop;

  ObjectRemoveCrop.setNegative(true);
  ObjectRemoveCrop.setInputCloud(crop_cloud);
  ObjectRemoveCrop.setMin(min_point);
  ObjectRemoveCrop.setMax(max_point);
  ObjectRemoveCrop.setRotation(Eigen::Vector3f(0.0, 0.0, orientation_yaw));
  ObjectRemoveCrop.setTranslation(translation);
  ObjectRemoveCrop.filter(*crop_cloud);
}

// Main function
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicObjectRemoval>());
  rclcpp::shutdown();
  return 0;
}