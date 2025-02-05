// Copyright 2024 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/probabilistic_occupancy_grid_map/utils/utils.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>

#include <string>

namespace autoware::occupancy_grid_map
{
namespace utils
{

// used in laserscan based occupancy grid map
bool transformPointcloud(
  const sensor_msgs::msg::PointCloud2 & input, const tf2_ros::Buffer & tf2,
  const std::string & target_frame, sensor_msgs::msg::PointCloud2 & output)
{
  geometry_msgs::msg::TransformStamped tf_stamped;
  // lookup transform
  try {
    tf_stamped = tf2.lookupTransform(
      target_frame, input.header.frame_id, input.header.stamp, rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      rclcpp::get_logger("probabilistic_occupancy_grid_map"), "Failed to lookup transform: %s",
      ex.what());
    return false;
  }
  // transform pointcloud
  Eigen::Matrix4f tf_matrix = tf2::transformToEigen(tf_stamped.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(tf_matrix, input, output);
  output.header.stamp = input.header.stamp;
  output.header.frame_id = target_frame;
  return true;
}

bool transformPointcloudAsync(
  CudaPointCloud2 & input, const tf2_ros::Buffer & tf2, const std::string & target_frame)
{
  geometry_msgs::msg::TransformStamped tf_stamped;
  // lookup transform
  try {
    tf_stamped = tf2.lookupTransform(
      target_frame, input.header.frame_id, input.header.stamp, rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      rclcpp::get_logger("probabilistic_occupancy_grid_map"), "Failed to lookup transform: %s",
      ex.what());
    return false;
  }
  // transform pointcloud
  Eigen::Matrix4f tf_matrix = tf2::transformToEigen(tf_stamped.transform).matrix().cast<float>();
  Eigen::Matrix3f rotation = tf_matrix.block<3, 3>(0, 0);
  Eigen::Vector3f translation = tf_matrix.block<3, 1>(0, 3);
  transformPointCloudLaunch(
    input.data.get(), input.width * input.height, input.point_step, rotation, translation,
    input.stream);
  input.header.frame_id = target_frame;
  return true;
}

Eigen::Matrix4f getTransformMatrix(const geometry_msgs::msg::Pose & pose)
{
  auto transform = autoware::universe_utils::pose2transform(pose);
  Eigen::Matrix4f tf_matrix = tf2::transformToEigen(transform).matrix().cast<float>();
  return tf_matrix;
}

bool cropPointcloudByHeight(
  const sensor_msgs::msg::PointCloud2 & input, const tf2_ros::Buffer & tf2,
  const std::string & target_frame, const float min_height, const float max_height,
  sensor_msgs::msg::PointCloud2 & output)
{
  rclcpp::Clock clock{RCL_ROS_TIME};
  // Transformed pointcloud on target frame
  sensor_msgs::msg::PointCloud2 trans_input_tmp;
  const bool is_target_frame = (input.header.frame_id == target_frame);
  if (!is_target_frame) {
    if (!transformPointcloud(input, tf2, target_frame, trans_input_tmp)) return false;
  }
  const sensor_msgs::msg::PointCloud2 & trans_input = is_target_frame ? input : trans_input_tmp;

  // Apply height filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(trans_input, "x"),
       iter_y(trans_input, "y"), iter_z(trans_input, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (min_height < *iter_z && *iter_z < max_height) {
      pcl_output->push_back(pcl::PointXYZ(*iter_x, *iter_y, *iter_z));
    }
  }

  // Convert to ros msg
  pcl::toROSMsg(*pcl_output, output);
  output.header = trans_input.header;
  return true;
}

geometry_msgs::msg::Pose getPose(
  const std_msgs::msg::Header & source_header, const tf2_ros::Buffer & tf2,
  const std::string & target_frame)
{
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::TransformStamped tf_stamped;
  tf_stamped = tf2.lookupTransform(
    target_frame, source_header.frame_id, source_header.stamp, rclcpp::Duration::from_seconds(0.5));
  pose = autoware::universe_utils::transform2pose(tf_stamped.transform);
  return pose;
}

geometry_msgs::msg::Pose getPose(
  const builtin_interfaces::msg::Time & stamp, const tf2_ros::Buffer & tf2,
  const std::string & source_frame, const std::string & target_frame)
{
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::TransformStamped tf_stamped;
  tf_stamped =
    tf2.lookupTransform(target_frame, source_frame, stamp, rclcpp::Duration::from_seconds(0.5));
  pose = autoware::universe_utils::transform2pose(tf_stamped.transform);
  return pose;
}

/**
 * @brief Get the Inverse Pose object
 *
 * @param input
 * @return geometry_msgs::msg::Pose inverted pose
 */
geometry_msgs::msg::Pose getInversePose(const geometry_msgs::msg::Pose & pose)
{
  tf2::Vector3 position(pose.position.x, pose.position.y, pose.position.z);
  tf2::Quaternion orientation(
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf2::Transform tf(orientation, position);
  const auto inv_tf = tf.inverse();
  geometry_msgs::msg::Pose inv_pose;
  inv_pose.position.x = inv_tf.getOrigin().x();
  inv_pose.position.y = inv_tf.getOrigin().y();
  inv_pose.position.z = inv_tf.getOrigin().z();
  inv_pose.orientation.x = inv_tf.getRotation().x();
  inv_pose.orientation.y = inv_tf.getRotation().y();
  inv_pose.orientation.z = inv_tf.getRotation().z();
  inv_pose.orientation.w = inv_tf.getRotation().w();
  return inv_pose;
}

}  // namespace utils
}  // namespace autoware::occupancy_grid_map
