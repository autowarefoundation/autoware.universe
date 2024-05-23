// Copyright 2020 Tier IV, Inc.
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

#include "pointcloud_preprocessor/distortion_corrector/undistorter.hpp"

#include "tier4_autoware_utils/math/trigonometry.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

#include <deque>
#include <optional>
#include <string>
#include <utility>

namespace pointcloud_preprocessor
{

void Undistorter::setIMUTransform(
  const std::string & base_link_frame, const std::string & imu_frame)
{
  tf2::Transform tf2_imu_to_base_link;
  if (base_link_frame == imu_frame) {
    tf2_imu_to_base_link.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    tf2_imu_to_base_link.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    is_imu_transfrom_exist = true;
  } else {
    try {
      const auto transform_msg =
        tf2_buffer.lookupTransform(base_link_frame, imu_frame, tf2::TimePointZero);
      tf2::convert(transform_msg.transform, tf2_imu_to_base_link);
      is_imu_transfrom_exist = true;
    } catch (const tf2::TransformException & ex) {
      // RCLCPP_WARN(get_logger(), "%s", ex.what());
      // RCLCPP_ERROR(
      //   get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

      tf2_imu_to_base_link.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
      tf2_imu_to_base_link.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    }
  }

  geometry_imu_to_base_link_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>();
  geometry_imu_to_base_link_ptr->transform.rotation =
    tf2::toMsg(tf2_imu_to_base_link.getRotation());
}

void Undistorter2D::initialize()
{
  x = 0.0f;
  y = 0.0f;
  theta = 0.0f;
}

void Undistorter3D::initialize()
{
  prev_transformation_matrix = Eigen::Matrix4f::Identity();
}

void Undistorter2D::setPointCloudTransform(
  const std::string & base_link_frame, const std::string & lidar_frame)
{
  if (base_link_frame == lidar_frame) {
    tf2_lidar_to_base_link.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    tf2_lidar_to_base_link.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    tf2_base_link_to_lidar = tf2_lidar_to_base_link;
    is_pointcloud_transfrom_exist = true;
  } else {
    try {
      const auto transform_msg =
        tf2_buffer.lookupTransform(base_link_frame, lidar_frame, tf2::TimePointZero);
      tf2::convert(transform_msg.transform, tf2_lidar_to_base_link);
      tf2_base_link_to_lidar = tf2_lidar_to_base_link.inverse();
      is_pointcloud_transfrom_exist = true;
      is_pointcloud_transform_needed = true;
    } catch (const tf2::TransformException & ex) {
      // RCLCPP_WARN(get_logger(), "%s", ex.what());
      // RCLCPP_ERROR(
      //   get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

      tf2_lidar_to_base_link.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
      tf2_lidar_to_base_link.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
      tf2_base_link_to_lidar = tf2_lidar_to_base_link;
    }
  }
}

void Undistorter3D::setPointCloudTransform(
  const std::string & base_link_frame, const std::string & lidar_frame)
{
  if (base_link_frame == lidar_frame) {
    eigen_lidar_to_base_link = Eigen::Matrix4f::Identity();
    eigen_base_link_to_lidar = Eigen::Matrix4f::Identity();
    is_pointcloud_transfrom_exist = true;
  }

  try {
    const auto transform_msg =
      tf2_buffer.lookupTransform(base_link_frame, lidar_frame, tf2::TimePointZero);
    eigen_lidar_to_base_link =
      tf2::transformToEigen(transform_msg.transform).matrix().cast<float>();
    eigen_base_link_to_lidar = eigen_lidar_to_base_link.inverse();
    is_pointcloud_transfrom_exist = true;
    is_pointcloud_transform_needed = true;
  } catch (const tf2::TransformException & ex) {
    // RCLCPP_WARN(get_logger(), "%s", ex.what());
    // RCLCPP_ERROR(
    //   get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());
    eigen_lidar_to_base_link = Eigen::Matrix4f::Identity();
    eigen_base_link_to_lidar = Eigen::Matrix4f::Identity();
  }
}

void Undistorter2D::undistortPoint(
  sensor_msgs::PointCloud2Iterator<float> & it_x, sensor_msgs::PointCloud2Iterator<float> & it_y,
  sensor_msgs::PointCloud2Iterator<float> & it_z,
  std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
  std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu, float time_offset,
  bool is_twist_valid, bool is_imu_valid)
{
  // Initialize linear velocity and angular velocity
  float v{0.0f}, w{0.0f};
  if (is_twist_valid) {
    v = static_cast<float>(it_twist->twist.linear.x);
    w = static_cast<float>(it_twist->twist.angular.z);
  }
  if (is_imu_valid) {
    w = static_cast<float>(it_imu->vector.z);
  }

  // Undistort point
  point_tf.setValue(*it_x, *it_y, *it_z);

  if (is_pointcloud_transform_needed) {
    point_tf = tf2_lidar_to_base_link * point_tf;
  }
  theta += w * time_offset;
  baselink_quat.setValue(
    0, 0, tier4_autoware_utils::sin(theta * 0.5f),
    tier4_autoware_utils::cos(theta * 0.5f));  // baselink_quat.setRPY(0.0, 0.0, theta);
  const float dis = v * time_offset;
  x += dis * tier4_autoware_utils::cos(theta);
  y += dis * tier4_autoware_utils::sin(theta);

  baselink_tf_odom.setOrigin(tf2::Vector3(x, y, 0.0));
  baselink_tf_odom.setRotation(baselink_quat);

  undistorted_point_tf = baselink_tf_odom * point_tf;

  if (is_pointcloud_transform_needed) {
    undistorted_point_tf = tf2_base_link_to_lidar * undistorted_point_tf;
  }

  *it_x = static_cast<float>(undistorted_point_tf.getX());
  *it_y = static_cast<float>(undistorted_point_tf.getY());
  *it_z = static_cast<float>(undistorted_point_tf.getZ());
}

void Undistorter3D::undistortPoint(
  sensor_msgs::PointCloud2Iterator<float> & it_x, sensor_msgs::PointCloud2Iterator<float> & it_y,
  sensor_msgs::PointCloud2Iterator<float> & it_z,
  std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
  std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu, float time_offset,
  bool is_twist_valid, bool is_imu_valid)
{
  // Initialize linear velocity and angular velocity
  float v_x_{0.0f}, v_y_{0.0f}, v_z_{0.0f}, w_x_{0.0f}, w_y_{0.0f}, w_z_{0.0f};
  if (is_twist_valid) {
    v_x_ = static_cast<float>(it_twist->twist.linear.x);
    v_y_ = static_cast<float>(it_twist->twist.linear.y);
    v_z_ = static_cast<float>(it_twist->twist.linear.z);
    w_x_ = static_cast<float>(it_twist->twist.angular.x);
    w_y_ = static_cast<float>(it_twist->twist.angular.y);
    w_z_ = static_cast<float>(it_twist->twist.angular.z);
  }
  if (is_imu_valid) {
    w_x_ = static_cast<float>(it_imu->vector.x);
    w_y_ = static_cast<float>(it_imu->vector.y);
    w_z_ = static_cast<float>(it_imu->vector.z);
  }

  // Undistort point
  point_eigen << *it_x, *it_y, *it_z, 1.0;
  if (is_pointcloud_transform_needed) {
    point_eigen = eigen_lidar_to_base_link * point_eigen;
  }

  Sophus::SE3f::Tangent twist(v_x_, v_y_, v_z_, w_x_, w_y_, w_z_);
  twist = twist * time_offset;
  transformation_matrix = Sophus::SE3f::exp(twist).matrix();
  transformation_matrix = transformation_matrix * prev_transformation_matrix;
  undistorted_point_eigen = transformation_matrix * point_eigen;

  if (is_pointcloud_transform_needed) {
    undistorted_point_eigen = eigen_base_link_to_lidar * undistorted_point_eigen;
  }
  *it_x = undistorted_point_eigen[0];
  *it_y = undistorted_point_eigen[1];
  *it_z = undistorted_point_eigen[2];

  prev_transformation_matrix = transformation_matrix;
}

}  // namespace pointcloud_preprocessor
