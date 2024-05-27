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

#ifndef POINTCLOUD_PREPROCESSOR__DISTORTION_CORRECTOR__UNDISTORTER_HPP_
#define POINTCLOUD_PREPROCESSOR__DISTORTION_CORRECTOR__UNDISTORTER_HPP_

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <sophus/se3.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <memory>
#include <string>

namespace pointcloud_preprocessor
{

class UndistorterBase
{
public:
  virtual void processTwistMessage(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_msg) = 0;
  virtual void processIMUMessage(
    const std::string & base_link_frame, const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg) = 0;
  virtual void setPointCloudTransform(
    const std::string & base_link_frame, const std::string & lidar_frame) = 0;
  virtual void initialize() = 0;
  virtual void undistortPointCloud(bool use_imu, sensor_msgs::msg::PointCloud2 & pointcloud) = 0;
};

template <class Derived>
class Undistorter : public UndistorterBase
{
public:
  bool is_pointcloud_transform_needed_{false};
  bool is_pointcloud_transform_exist_{false};
  bool is_imu_transform_exist_{false};
  rclcpp::Node * node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::deque<geometry_msgs::msg::TwistStamped> twist_queue_;
  std::deque<geometry_msgs::msg::Vector3Stamped> angular_velocity_queue_;

  explicit Undistorter(rclcpp::Node * node)
  : node_(node), tf_buffer_(node_->get_clock()), tf_listener_(tf_buffer_)
  {
  }
  void processTwistMessage(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_msg) override;

  void processIMUMessage(
    const std::string & base_link_frame,
    const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg) override;
  void getIMUTransformation(
    const std::string & base_link_frame, const std::string & imu_frame,
    geometry_msgs::msg::TransformStamped::SharedPtr geometry_imu_to_base_link_ptr);
  void storeIMUToQueue(
    const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg,
    geometry_msgs::msg::TransformStamped::SharedPtr geometry_imu_to_base_link_ptr);

  bool isInputValid(sensor_msgs::msg::PointCloud2 & pointcloud);
  void getIteratorOfTwistAndIMU(
    bool use_imu, double first_point_time_stamp_sec,
    std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
    std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu);
  void undistortPointCloud(bool use_imu, sensor_msgs::msg::PointCloud2 & pointcloud) override;
  void warnIfTimestampsTooLate(
    bool is_twist_time_stamp_too_late, bool is_imu_time_stamp_is_too_late);

  virtual void initialize() = 0;
  void undistortPoint(
    sensor_msgs::PointCloud2Iterator<float> & it_x, sensor_msgs::PointCloud2Iterator<float> & it_y,
    sensor_msgs::PointCloud2Iterator<float> & it_z,
    std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
    std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu, float & time_offset,
    bool & is_twist_valid, bool & is_imu_valid)
  {
    static_cast<Derived *>(this)->undistortPointImplementation(
      it_x, it_y, it_z, it_twist, it_imu, time_offset, is_twist_valid, is_imu_valid);
  };

  virtual void setPointCloudTransform(
    const std::string & base_link_frame, const std::string & lidar_frame) = 0;
};

class Undistorter2D : public Undistorter<Undistorter2D>
{
private:
  // defined outside of for loop for performance reasons.
  tf2::Quaternion baselink_quat_;
  tf2::Transform baselink_tf_odom_;
  tf2::Vector3 point_tf_;
  tf2::Vector3 undistorted_point_tf_;
  float theta_;
  float x_;
  float y_;

  // TF
  tf2::Transform tf2_lidar_to_base_link_;
  tf2::Transform tf2_base_link_to_lidar_;

public:
  explicit Undistorter2D(rclcpp::Node * node) : Undistorter(node) {}
  void initialize() override;
  void undistortPointImplementation(
    sensor_msgs::PointCloud2Iterator<float> & it_x, sensor_msgs::PointCloud2Iterator<float> & it_y,
    sensor_msgs::PointCloud2Iterator<float> & it_z,
    std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
    std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu, float & time_offset,
    bool & is_twist_valid, bool & is_imu_valid);

  void setPointCloudTransform(
    const std::string & base_link_frame, const std::string & lidar_frame) override;
};

class Undistorter3D : public Undistorter<Undistorter3D>
{
private:
  // defined outside of for loop for performance reasons.
  Eigen::Vector4f point_eigen_;
  Eigen::Vector4f undistorted_point_eigen_;
  Eigen::Matrix4f transformation_matrix_;
  Eigen::Matrix4f prev_transformation_matrix_;

  // TF
  Eigen::Matrix4f eigen_lidar_to_base_link_;
  Eigen::Matrix4f eigen_base_link_to_lidar_;

public:
  explicit Undistorter3D(rclcpp::Node * node) : Undistorter(node) {}
  void initialize() override;
  void undistortPointImplementation(
    sensor_msgs::PointCloud2Iterator<float> & it_x, sensor_msgs::PointCloud2Iterator<float> & it_y,
    sensor_msgs::PointCloud2Iterator<float> & it_z,
    std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
    std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu, float & time_offset,
    bool & is_twist_valid, bool & is_imu_valid);
  void setPointCloudTransform(
    const std::string & base_link_frame, const std::string & lidar_frame) override;
};

}  // namespace pointcloud_preprocessor

#endif  // POINTCLOUD_PREPROCESSOR__DISTORTION_CORRECTOR__UNDISTORTER_HPP_
