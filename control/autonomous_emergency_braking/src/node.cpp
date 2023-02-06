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

AEB::AEB(const rclcpp::NodeOptions & node_options)
: Node("AEB", node_options),
  vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo())
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
  debug_ego_predicted_path_publisher_ =
    this->create_publisher<Marker>("~/debug/ego_predicted_path", 1);

  // Diagnostics
  updater_.setHardwareID("autonomous_emergency_braking");
  updater_.add("aeb_emergency_stop", this, &AEB::onCheckCollision);

  // parameter
  use_imu_data_ = declare_parameter<bool>("use_imu_data", false);
  voxel_grid_x_ = declare_parameter<double>("voxel_grid_x", 0.05);
  voxel_grid_y_ = declare_parameter<double>("voxel_grid_y", 0.05);
  voxel_grid_z_ = declare_parameter<double>("voxel_grid_z", 100000.0);
  lateral_offset_ = declare_parameter<double>("lateral_offset", 1.0);
  longitudinal_offset_ = declare_parameter<double>("longitudinal_offset", 1.0);
  t_response_ = declare_parameter<double>("t_response", 1.0);
  a_ego_min_ = declare_parameter<double>("a_ego_min", -3.0);
  a_obj_min_ = declare_parameter<double>("a_obj_min", -1.0);
  prediction_time_horizon_ = declare_parameter<double>("prediction_time_horizon", 1.0);
  prediction_time_interval_ = declare_parameter<double>("prediction_time_interval", 0.1);

  // start time
  const double aeb_hz = declare_parameter<double>("aeb_hz", 10.0);
  const auto period_ns = rclcpp::Rate(aeb_hz).period();
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

  pcl::VoxelGrid<pcl::PointXYZ> filter;
  PointCloud::Ptr no_height_filtered_pointcloud_ptr(new PointCloud);
  filter.setInputCloud(pointcloud_ptr);
  filter.setLeafSize(voxel_grid_x_, voxel_grid_y_, voxel_grid_z_);
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
  // step1. check data
  if (!isDataReady()) {
    return false;
  }

  // step2. create data
  const double current_v = current_velocity_ptr_->longitudinal_velocity;
  const double current_w =
    use_imu_data_ ? imu_ptr_->angular_velocity.z : odometry_ptr_->twist.twist.angular.z;

  // step3. create ego path
  std::vector<geometry_msgs::msg::Pose> ego_path;
  generateEgoPath(current_v, current_w, ego_path);
  publishEgoPath(ego_path);

  // check if the predicted path has valid number of points
  if (ego_path.size() < 2) {
    return false;
  }

  // step4. create object
  std::vector<ObjectData> objects;
  createObjectData(ego_path, objects);

  // step5. calculate time to collision(RSS)
  const double & t = t_response_;
  for (const auto & obj : objects) {
    const double & obj_v = obj.velocity;
    const double rss_dist = current_v * t + current_v * current_v / (2 * std::fabs(a_ego_min_)) -
                            obj_v * obj_v / (2 * std::fabs(a_obj_min_)) + longitudinal_offset_;
    const double dist_ego_to_object = obj.lon_dist - vehicle_info_.max_longitudinal_offset_m;
    if (dist_ego_to_object < rss_dist) {
      // collision happens
      return true;
    }
  }

  return false;
}

void AEB::generateEgoPath(
  const double curr_v, const double curr_w, std::vector<geometry_msgs::msg::Pose> & path)
{
  double curr_x = 0.0;
  double curr_y = 0.0;
  double curr_yaw = 0.0;
  geometry_msgs::msg::Pose ini_pose;
  ini_pose.position = tier4_autoware_utils::createPoint(curr_x, curr_y, 0.0);
  ini_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(curr_yaw);
  path.push_back(ini_pose);

  constexpr double epsilon = 1e-6;
  const double & dt = prediction_time_interval_;
  const double & horizon = prediction_time_horizon_;
  for (double t = 0.0; t < horizon + epsilon; t += dt) {
    curr_x = curr_x + curr_v * std::cos(curr_yaw) * dt;
    curr_y = curr_y + curr_v * std::sin(curr_yaw) * dt;
    curr_yaw = curr_yaw + curr_w * dt;
    geometry_msgs::msg::Pose current_pose;
    current_pose.position = tier4_autoware_utils::createPoint(curr_x, curr_y, 0.0);
    current_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(curr_yaw);
    if (tier4_autoware_utils::calcDistance2d(path.back(), current_pose) < 1e-3) {
      continue;
    }
    path.push_back(current_pose);
  }
}

void AEB::createObjectData(
  const std::vector<geometry_msgs::msg::Pose> & ego_path, std::vector<ObjectData> & objects)
{
  const double lateral_dist_threshold = vehicle_info_.vehicle_width_m / 2.0 + lateral_offset_;
  const double path_length = motion_utils::calcArcLength(ego_path);
  PointCloud::Ptr obstacle_points_ptr(new PointCloud);
  pcl::fromROSMsg(*obstacle_ros_pointcloud_ptr_, *obstacle_points_ptr);
  for (const auto & point : obstacle_points_ptr->points) {
    ObjectData obj;
    obj.position = tier4_autoware_utils::createPoint(point.x, point.y, point.z);
    obj.velocity = 0.0;
    obj.time = pcl_conversions::fromPCL(obstacle_points_ptr->header).stamp;
    obj.lon_dist = motion_utils::calcSignedArcLength(ego_path, 0, obj.position);
    obj.lat_dist = motion_utils::calcLateralOffset(ego_path, obj.position);
    if (obj.lon_dist < 0.0 || obj.lon_dist > path_length || obj.lat_dist > lateral_dist_threshold) {
      // ignore objects that are behind the base link and ahead of the end point of the path
      // or outside of the lateral distance
      continue;
    }
    objects.push_back(obj);
  }
}

void AEB::publishEgoPath(const std::vector<geometry_msgs::msg::Pose> & path)
{
  // marker parameter
  constexpr double scale_x = 0.2;
  constexpr double scale_y = 0.2;
  constexpr double scale_z = 0.2;
  constexpr double color_r = 0.0 / 256.0;
  constexpr double color_g = 148.0 / 256.0;
  constexpr double color_b = 205.0 / 256.0;
  constexpr double color_a = 0.999;

  const auto current_time = this->get_clock()->now();
  auto marker = tier4_autoware_utils::createDefaultMarker(
    "map", current_time, "ego_predicted_path", 0L, Marker::LINE_STRIP,
    tier4_autoware_utils::createMarkerScale(scale_x, scale_y, scale_z),
    tier4_autoware_utils::createMarkerColor(color_r, color_g, color_b, color_a));
  marker.points.resize(path.size());

  // transform to map
  geometry_msgs::msg::TransformStamped transform_stamped{};
  try {
    transform_stamped = tf_buffer_.lookupTransform(
      "map", "base_link", current_time, rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(get_logger(), "[AEB] Failed to look up transform from base_link to map");
    return;
  }

  for (size_t i = 0; i < path.size(); ++i) {
    const auto & pose = path.at(i);
    geometry_msgs::msg::Pose map_pose;
    tf2::doTransform(pose, map_pose, transform_stamped);
    marker.points.at(i) = map_pose.position;
  }

  debug_ego_predicted_path_publisher_->publish(marker);
}

}  // namespace autoware::motion::control::autonomous_emergency_braking

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::motion::control::autonomous_emergency_braking::AEB)
