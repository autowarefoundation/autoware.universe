// Copyright 2022 TIER IV, Inc.
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

#include "autonomous_emergency_braking/node.hpp"

#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <pcl/filters/voxel_grid.h>
#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace autoware::motion::control::autonomous_emergency_braking
{
using diagnostic_msgs::msg::DiagnosticStatus;

AEB::AEB(const rclcpp::NodeOptions & node_options) : Node("AEB", node_options)
{
  // Subscribers
  sub_point_cloud_ = this->create_subscription<PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&AEB::onPointCloud, this, std::placeholders::_1));

  sub_velocity_ = this->create_subscription<VelocityReport>(
    "~/input/velocity", rclcpp::QoS{1}, std::bind(&AEB::onVelocity, this, std::placeholders::_1));

  sub_odometry_ = this->create_subscription<Odometry>(
    "~/input/odometry", rclcpp::QoS{1}, std::bind(&AEB::onOdometry, this, std::placeholders::_1));

  sub_imu_ = this->create_subscription<Imu>(
    "~/input/imu", rclcpp::QoS{1}, std::bind(&AEB::onImu, this, std::placeholders::_1));

  // Publisher
  pub_obstacle_pointcloud_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("~/debug/obstacle_pointcloud", 1);

  // Diagnostics
  updater_.setHardwareID("autonomous_emergency_braking");
  updater_.add("aeb_emergency_stop", this, &AEB::onCheckCollision);

  // parameter
  use_imu_data_ = false;

  // start timer
  const auto planning_hz = 10.0;  // declare_parameter("planning_hz", 10.0);
  const auto period_ns = rclcpp::Rate(planning_hz).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period_ns, std::bind(&AEB::onTimer, this));
}

void AEB::onTimer() { updater_.force_update(); }

void AEB::onVelocity(const VelocityReport::ConstSharedPtr input_msg)
{
  current_velocity_ptr_ = input_msg;
}

void AEB::onOdometry(const Odometry::ConstSharedPtr input_msg) { odometry_ptr_ = input_msg; }

void AEB::onImu(const Imu::ConstSharedPtr input_msg) { imu_ptr_ = input_msg; }

void AEB::onPointCloud(const PointCloud2::ConstSharedPtr input_msg)
{
  PointCloud::Ptr pointcloud_ptr(new PointCloud);
  pcl::fromROSMsg(*input_msg, *pointcloud_ptr);

  if (input_msg->header.frame_id != "base_link") {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "[AEB]: Input point cloud frame is not base_link and it is " << input_msg->header.frame_id);
    // transform pointcloud
    geometry_msgs::msg::TransformStamped transform_stamped{};
    try {
      transform_stamped = tf_buffer_.lookupTransform(
        "base_link", input_msg->header.frame_id, input_msg->header.stamp,
        rclcpp::Duration::from_seconds(0.5));
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "[AEB] Failed to look up transform from base_link to" << input_msg->header.frame_id);
      return;
    }

    // transform by using eigen matrix
    PointCloud2 transformed_points{};
    const Eigen::Matrix4f affine_matrix =
      tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
    pcl_ros::transformPointCloud(affine_matrix, *input_msg, transformed_points);
    pcl::fromROSMsg(transformed_points, *pointcloud_ptr);
  }

  constexpr double voxel_grid_x = 0.05;
  constexpr double voxel_grid_y = 0.05;
  constexpr double voxel_grid_z = 100000.0;
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  PointCloud::Ptr no_height_filtered_pointcloud_ptr(new PointCloud);
  filter.setInputCloud(pointcloud_ptr);
  filter.setLeafSize(voxel_grid_x, voxel_grid_y, voxel_grid_z);
  filter.filter(*no_height_filtered_pointcloud_ptr);

  obstacle_ros_pointcloud_ptr_ = std::make_shared<PointCloud2>();
  pcl::toROSMsg(*no_height_filtered_pointcloud_ptr, *obstacle_ros_pointcloud_ptr_);
  obstacle_ros_pointcloud_ptr_->header = input_msg->header;
  pub_obstacle_pointcloud_->publish(*obstacle_ros_pointcloud_ptr_);
}

bool AEB::isDataReady()
{
  const auto missing = [this](const auto & name) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "[AEB] waiting for %s", name);
    return false;
  };

  if (!current_velocity_ptr_) {
    return missing("ego velocity");
  }

  if (!use_imu_data_ && !odometry_ptr_) {
    return missing("odometry");
  }

  if (use_imu_data_ && !imu_ptr_) {
    return missing("imu");
  }

  if (!obstacle_ros_pointcloud_ptr_) {
    return missing("object pointcloud");
  }

  return true;
}

void AEB::onCheckCollision(DiagnosticStatusWrapper & stat)
{
  if (checkCollision()) {
    std::cerr << "Emergency" << std::endl;
    const std::string error_msg = "[AEB]: Emergency Brake";
    const auto diag_level = DiagnosticStatus::ERROR;
    stat.summary(diag_level, error_msg);
    return;
  }

  const std::string error_msg = "[AEB]: No Collision";
  const auto diag_level = DiagnosticStatus::OK;
  stat.summary(diag_level, error_msg);
}

bool AEB::checkCollision()
{
  if (!isDataReady()) {
    return false;
  }

  // create data
  const double v = current_velocity_ptr_->longitudinal_velocity;
  const double w =
    use_imu_data_ ? imu_ptr_->angular_velocity.z : odometry_ptr_->twist.twist.angular.z;
  PointCloud::Ptr obstacle_points_ptr(new PointCloud);
  pcl::fromROSMsg(*obstacle_ros_pointcloud_ptr_, *obstacle_points_ptr);

  // step1. create predicted path
  std::vector<geometry_msgs::msg::Pose> predicted_path;
  double curr_x = 0.0;
  double curr_y = 0.0;
  double curr_yaw = 0.0;
  geometry_msgs::msg::Pose ini_pose;
  ini_pose.position = tier4_autoware_utils::createPoint(curr_x, curr_y, 0.0);
  ini_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(curr_yaw);
  predicted_path.push_back(ini_pose);

  constexpr double epsilon = 1e-6;
  constexpr double dt = 0.1;
  constexpr double horizon = 3.0;
  for (double t = 0.0; t < horizon + epsilon; t += dt) {
    curr_x = curr_x + v * std::cos(curr_yaw) * dt;
    curr_y = curr_y + v * std::sin(curr_yaw) * dt;
    curr_yaw = curr_yaw + w * dt;
    geometry_msgs::msg::Pose current_pose;
    current_pose.position = tier4_autoware_utils::createPoint(curr_x, curr_y, 0.0);
    current_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(curr_yaw);
    if (tier4_autoware_utils::calcDistance2d(predicted_path.back(), current_pose) < 1e-3) {
      continue;
    }
    predicted_path.push_back(current_pose);
  }

  // check if the predicted path has valid number of points
  if (predicted_path.size() < 2) {
    return false;
  }

  // step2. create object
  constexpr double lateral_dist_threshold = 5.0;
  std::vector<ObjectData> objects;
  for (const auto & point : obstacle_points_ptr->points) {
    ObjectData obj;
    obj.position = tier4_autoware_utils::createPoint(point.x, point.y, point.z);
    obj.velocity = 0.0;
    obj.time = pcl_conversions::fromPCL(obstacle_points_ptr->header).stamp;
    obj.lon_dist = motion_utils::calcSignedArcLength(predicted_path, 0, obj.position);
    obj.lat_dist = motion_utils::calcLateralOffset(predicted_path, obj.position);
    if (obj.lon_dist < 0.0 || obj.lat_dist > lateral_dist_threshold) {
      continue;
    }
    objects.push_back(obj);
  }

  // step3. calculate time to collision(RSS)
  const double t_rss = 5.0;
  const double a_min = -3.0;
  const double a_obj_min = -1.0;
  const double safe_buffer = 2.0;
  for (const auto & obj : objects) {
    const double & obj_v = obj.velocity;
    const double rss_dist = v * t_rss + v * v / (2 * std::fabs(a_min)) -
                            obj_v * obj_v / (2 * std::fabs(a_obj_min)) + safe_buffer;
    if (obj.lon_dist < rss_dist) {
      // collision happens
      return true;
    }
  }

  // no collision
  return false;
}

}  // namespace autoware::motion::control::autonomous_emergency_braking

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::motion::control::autonomous_emergency_braking::AEB)
