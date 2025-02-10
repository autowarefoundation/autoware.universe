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

#include "node.hpp"

#include <pcl_ros/transforms.hpp>

#include <pcl/common/centroid.h>

#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace
{
Eigen::Vector3d getArbitraryOrthogonalVector(const Eigen::Vector3d & input)
{
  const double x = input.x();
  const double y = input.y();
  const double z = input.z();
  const double x2 = std::pow(x, 2);
  const double y2 = std::pow(y, 2);
  const double z2 = std::pow(z, 2);

  Eigen::Vector3d unit_vec{0, 0, 0};
  if (x2 <= y2 && x2 <= z2) {
    unit_vec.x() = 0;
    unit_vec.y() = z;
    unit_vec.z() = -y;
    unit_vec = unit_vec / std::sqrt(y2 + z2);
  } else if (y2 <= x2 && y2 <= z2) {
    unit_vec.x() = -z;
    unit_vec.y() = 0;
    unit_vec.z() = x;
    unit_vec = unit_vec / std::sqrt(z2 + x2);
  } else if (z2 <= x2 && z2 <= y2) {
    unit_vec.x() = y;
    unit_vec.y() = -x;
    unit_vec.z() = 0;
    unit_vec = unit_vec / std::sqrt(x2 + y2);
  }
  return unit_vec;
}

geometry_msgs::msg::Pose getDebugPose(const Eigen::Affine3d & plane_affine)
{
  geometry_msgs::msg::Pose debug_pose;
  const Eigen::Quaterniond quat(plane_affine.rotation());
  debug_pose.position.x = plane_affine.translation().x();
  debug_pose.position.y = plane_affine.translation().y();
  debug_pose.position.z = plane_affine.translation().z();
  debug_pose.orientation.x = quat.x();
  debug_pose.orientation.y = quat.y();
  debug_pose.orientation.z = quat.z();
  debug_pose.orientation.w = quat.w();
  return debug_pose;
}
}  // namespace

namespace autoware::ground_segmentation
{
PlaneBasis getPlaneBasis(const Eigen::Vector3d & plane_normal)
{
  PlaneBasis basis;
  basis.e_z = plane_normal;
  basis.e_x = getArbitraryOrthogonalVector(plane_normal);
  basis.e_y = basis.e_x.cross(basis.e_z);
  return basis;
}

using autoware::pointcloud_preprocessor::get_param;
using autoware::universe_utils::ScopedTimeTrack;

RANSACGroundFilterComponent::RANSACGroundFilterComponent(const rclcpp::NodeOptions & options)
: Filter("RANSACGroundFilter", options)
{
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  unit_axis_ = declare_parameter<std::string>("unit_axis");
  max_iterations_ = declare_parameter<int>("max_iterations");
  min_inliers_ = declare_parameter<int>("min_trial");
  min_points_ = declare_parameter<int>("min_points");
  outlier_threshold_ = declare_parameter<double>("outlier_threshold");
  plane_slope_threshold_ = declare_parameter<double>("plane_slope_threshold");
  voxel_size_x_ = declare_parameter<double>("voxel_size_x");
  voxel_size_y_ = declare_parameter<double>("voxel_size_y");
  voxel_size_z_ = declare_parameter<double>("voxel_size_z");
  height_threshold_ = declare_parameter<double>("height_threshold");
  debug_ = declare_parameter<bool>("debug");

  if (unit_axis_ == "x") {
    unit_vec_ = Eigen::Vector3d::UnitX();
  } else if (unit_axis_ == "y") {
    unit_vec_ = Eigen::Vector3d::UnitY();
  } else {  // including (unit_axis_ == "z")
    unit_vec_ = Eigen::Vector3d::UnitZ();
  }

  if (debug_) {
    setDebugPublisher();
  }

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RANSACGroundFilterComponent::paramCallback, this, _1));

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  managed_tf_buffer_ =
    std::make_unique<autoware::universe_utils::ManagedTransformBuffer>(this, has_static_tf_only_);

  bool use_time_keeper = declare_parameter<bool>("publish_processing_time_detail");
  if (use_time_keeper) {
    detailed_processing_time_publisher_ =
      this->create_publisher<autoware::universe_utils::ProcessingTimeDetail>(
        "~/debug/processing_time_detail_ms", 1);
    auto time_keeper = autoware::universe_utils::TimeKeeper(detailed_processing_time_publisher_);
    time_keeper_ = std::make_shared<autoware::universe_utils::TimeKeeper>(time_keeper);
  }
}

void RANSACGroundFilterComponent::setDebugPublisher()
{
  if (is_initialized_debug_message_) {
    return;
  }
  debug_pose_array_pub_ =
    create_publisher<geometry_msgs::msg::PoseArray>("debug/plane_pose_array", max_queue_size_);
  debug_ground_cloud_pub_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("debug/ground/pointcloud", max_queue_size_);
  is_initialized_debug_message_ = true;
}

void RANSACGroundFilterComponent::publishDebugMessage(
  const geometry_msgs::msg::PoseArray & debug_pose_array,
  const pcl::PointCloud<PointType> & ground_cloud, const std_msgs::msg::Header & header)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_ground_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (const auto & ground_point : ground_cloud.points) {
    pcl::PointXYZRGB p;
    p.x = ground_point.x;
    p.y = ground_point.y;
    p.z = ground_point.z;
    p.r = 255.0;
    p.g = 0.0;
    p.b = 0.0;
    colored_ground_ptr->points.push_back(p);
  }
  sensor_msgs::msg::PointCloud2::SharedPtr ground_cloud_msg_ptr(new sensor_msgs::msg::PointCloud2);
  ground_cloud_msg_ptr->header = header;
  pcl::toROSMsg(*colored_ground_ptr, *ground_cloud_msg_ptr);
  ground_cloud_msg_ptr->header.frame_id = base_frame_;
  debug_pose_array_pub_->publish(debug_pose_array);
  debug_ground_cloud_pub_->publish(*ground_cloud_msg_ptr);
}

bool RANSACGroundFilterComponent::transformPointCloud(
  const std::string & in_target_frame, const PointCloud2ConstPtr & in_cloud_ptr,
  const PointCloud2::SharedPtr & out_cloud_ptr)
{
  if (in_target_frame == in_cloud_ptr->header.frame_id) {
    *out_cloud_ptr = *in_cloud_ptr;
    return true;
  }

  return managed_tf_buffer_->transformPointcloud(in_target_frame, *in_cloud_ptr, *out_cloud_ptr);
}

void RANSACGroundFilterComponent::extractPointsIndices(
  const pcl::PointCloud<PointType>::Ptr in_cloud_ptr, const pcl::PointIndices & in_indices,
  pcl::PointCloud<PointType>::Ptr out_only_indices_cloud_ptr,
  pcl::PointCloud<PointType>::Ptr out_removed_indices_cloud_ptr)
{
  pcl::ExtractIndices<PointType> extract_ground;
  extract_ground.setInputCloud(in_cloud_ptr);
  extract_ground.setIndices(pcl::make_shared<pcl::PointIndices>(in_indices));

  extract_ground.setNegative(false);  // true removes the indices, false leaves only the indices
  extract_ground.filter(*out_only_indices_cloud_ptr);

  extract_ground.setNegative(true);  // true removes the indices, false leaves only the indices
  extract_ground.filter(*out_removed_indices_cloud_ptr);
}

Eigen::Affine3d RANSACGroundFilterComponent::getPlaneAffine(
  const pcl::PointCloud<PointType> segment_ground_cloud, const Eigen::Vector3d & plane_normal)
{
  pcl::CentroidPoint<pcl::PointXYZ> centroid;
  for (const auto p : segment_ground_cloud.points) {
    centroid.add(p);
  }
  pcl::PointXYZ centroid_point;
  centroid.get(centroid_point);
  Eigen::Translation<double, 3> trans(centroid_point.x, centroid_point.y, centroid_point.z);
  const PlaneBasis basis = getPlaneBasis(plane_normal);
  Eigen::Matrix3d rot;
  rot << basis.e_x.x(), basis.e_y.x(), basis.e_z.x(), basis.e_x.y(), basis.e_y.y(), basis.e_z.y(),
    basis.e_x.z(), basis.e_y.z(), basis.e_z.z();
  return trans * rot;
}

void RANSACGroundFilterComponent::applyRANSAC(
  const pcl::PointCloud<PointType>::Ptr & input, pcl::PointIndices::Ptr & output_inliers,
  pcl::ModelCoefficients::Ptr & output_coefficients)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  pcl::SACSegmentation<PointType> seg;
  seg.setOptimizeCoefficients(true);
  seg.setRadiusLimits(0.3, std::numeric_limits<double>::max());
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(outlier_threshold_);
  seg.setInputCloud(input);
  seg.setMaxIterations(max_iterations_);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.segment(*output_inliers, *output_coefficients);
}

void RANSACGroundFilterComponent::filter(
  const PointCloud2::ConstSharedPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  std::scoped_lock lock(mutex_);
  sensor_msgs::msg::PointCloud2::SharedPtr input_transformed_ptr(new sensor_msgs::msg::PointCloud2);
  if (!transformPointCloud(base_frame_, input, input_transformed_ptr)) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "Failed transform from " << base_frame_ << " to " << input->header.frame_id);
    return;
  }
  pcl::PointCloud<PointType>::Ptr current_sensor_cloud_ptr(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(*input_transformed_ptr, *current_sensor_cloud_ptr);

  // downsample pointcloud to reduce ransac calculation cost
  pcl::PointCloud<PointType>::Ptr downsampled_cloud(new pcl::PointCloud<PointType>);
  downsampled_cloud->points.reserve(current_sensor_cloud_ptr->points.size());
  pcl::VoxelGrid<PointType> filter;
  filter.setInputCloud(current_sensor_cloud_ptr);
  filter.setLeafSize(voxel_size_x_, voxel_size_y_, voxel_size_z_);
  filter.filter(*downsampled_cloud);

  // apply ransac
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  applyRANSAC(downsampled_cloud, inliers, coefficients);

  if (coefficients->values.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "failed to find a plane");
    output = *input;
    return;
  }

  // filter too tilt plane to avoid mis-fitting (e.g. fitting to wall plane)
  Eigen::Vector3d plane_normal(
    coefficients->values[0], coefficients->values[1], coefficients->values[2]);
  {
    const auto plane_slope = std::abs(
      std::acos(plane_normal.dot(unit_vec_) / (plane_normal.norm() * unit_vec_.norm())) * 180 /
      M_PI);
    if (plane_slope > plane_slope_threshold_) {
      output = *input;
      return;
    }
  }

  // extract pointcloud from indices
  pcl::PointCloud<PointType>::Ptr segment_ground_cloud_ptr(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr segment_no_ground_cloud_ptr(new pcl::PointCloud<PointType>);
  extractPointsIndices(
    downsampled_cloud, *inliers, segment_ground_cloud_ptr, segment_no_ground_cloud_ptr);
  const Eigen::Affine3d plane_affine = getPlaneAffine(*segment_ground_cloud_ptr, plane_normal);
  pcl::PointCloud<PointType>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<PointType>);

  int x_offset = input->fields[pcl::getFieldIndex(*input, "x")].offset;
  int y_offset = input->fields[pcl::getFieldIndex(*input, "y")].offset;
  int z_offset = input->fields[pcl::getFieldIndex(*input, "z")].offset;
  int point_step = input->point_step;

  sensor_msgs::msg::PointCloud2::SharedPtr no_ground_cloud_msg_ptr(
    new sensor_msgs::msg::PointCloud2);
  no_ground_cloud_msg_ptr->header = input->header;
  no_ground_cloud_msg_ptr->fields = input->fields;
  no_ground_cloud_msg_ptr->point_step = point_step;
  no_ground_cloud_msg_ptr->data.resize(input->data.size());
  size_t output_size = 0;
  // use not downsampled pointcloud for extract pointcloud that higher than height threshold
  for (size_t global_size = 0; global_size < input->data.size(); global_size += point_step) {
    float x = *reinterpret_cast<const float *>(&input->data[global_size + x_offset]);
    float y = *reinterpret_cast<const float *>(&input->data[global_size + y_offset]);
    float z = *reinterpret_cast<const float *>(&input->data[global_size + z_offset]);
    const Eigen::Vector3d transformed_point = plane_affine.inverse() * Eigen::Vector3d(x, y, z);
    if (std::abs(transformed_point.z()) > height_threshold_) {
      std::memcpy(
        &no_ground_cloud_msg_ptr->data[output_size], &input->data[global_size], point_step);
      output_size += point_step;
    }
  }
  no_ground_cloud_msg_ptr->data.resize(output_size);
  no_ground_cloud_msg_ptr->height = input->height;
  no_ground_cloud_msg_ptr->width = output_size / point_step / input->height;
  no_ground_cloud_msg_ptr->row_step = output_size / input->height;
  no_ground_cloud_msg_ptr->is_dense = input->is_dense;
  no_ground_cloud_msg_ptr->is_bigendian = input->is_bigendian;

  sensor_msgs::msg::PointCloud2::SharedPtr no_ground_cloud_transformed_msg_ptr(
    new sensor_msgs::msg::PointCloud2);
  if (!transformPointCloud(
        base_frame_, no_ground_cloud_msg_ptr, no_ground_cloud_transformed_msg_ptr)) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "Failed transform from " << base_frame_ << " to "
                               << no_ground_cloud_msg_ptr->header.frame_id);
    return;
  }
  output = *no_ground_cloud_transformed_msg_ptr;

  // output debug plane coords and ground pointcloud when debug flag is set
  if (debug_) {
    const geometry_msgs::msg::Pose debug_pose = getDebugPose(plane_affine);
    geometry_msgs::msg::PoseArray debug_pose_array;
    debug_pose_array.header.frame_id = base_frame_;
    debug_pose_array.poses.push_back(debug_pose);
    publishDebugMessage(debug_pose_array, *segment_ground_cloud_ptr, input->header);
  }
}

rcl_interfaces::msg::SetParametersResult RANSACGroundFilterComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  if (get_param(p, "base_frame", base_frame_)) {
    RCLCPP_DEBUG(get_logger(), "Setting base_frame to: %s.", base_frame_.c_str());
  }
  if (get_param(p, "max_iterations", max_iterations_)) {
    RCLCPP_DEBUG(get_logger(), "Setting max_iterations to: %d.", max_iterations_);
  }
  if (get_param(p, "min_inliers", min_inliers_)) {
    RCLCPP_DEBUG(get_logger(), "Setting min_inliers to: %d.", min_inliers_);
  }
  if (get_param(p, "min_points", min_points_)) {
    RCLCPP_DEBUG(get_logger(), "Setting min_points to: %d.", min_points_);
  }
  if (get_param(p, "outlier_threshold", outlier_threshold_)) {
    RCLCPP_DEBUG(get_logger(), "Setting outlier_threshold to: %lf.", outlier_threshold_);
  }
  if (get_param(p, "height_threshold", height_threshold_)) {
    RCLCPP_DEBUG(get_logger(), "Setting height_threshold_ to: %lf.", height_threshold_);
  }
  if (get_param(p, "plane_slope_threshold", plane_slope_threshold_)) {
    RCLCPP_DEBUG(get_logger(), "Setting plane_slope_threshold to: %lf.", plane_slope_threshold_);
  }
  if (get_param(p, "voxel_size_x", voxel_size_x_)) {
    RCLCPP_DEBUG(get_logger(), "Setting voxel_size_x to: %lf.", voxel_size_x_);
  }
  if (get_param(p, "voxel_size_y", voxel_size_y_)) {
    RCLCPP_DEBUG(get_logger(), "Setting voxel_size_y to: %lf.", voxel_size_y_);
  }
  if (get_param(p, "voxel_size_z", voxel_size_z_)) {
    RCLCPP_DEBUG(get_logger(), "Setting voxel_size_z to: %lf.", voxel_size_z_);
  }
  if (get_param(p, "debug", debug_)) {
    RCLCPP_DEBUG(get_logger(), "Setting debug to: %d.", debug_);
  }

  if (debug_) {
    setDebugPublisher();
  }

  if (get_param(p, "unit_axis", unit_axis_)) {
    if (unit_axis_ == "x") {
      unit_vec_ = Eigen::Vector3d::UnitX();
    } else if (unit_axis_ == "y") {
      unit_vec_ = Eigen::Vector3d::UnitY();
    } else {  // including (unit_axis_ == "z")
      unit_vec_ = Eigen::Vector3d::UnitZ();
    }
    RCLCPP_DEBUG(get_logger(), "Setting unit_axis to: %s.", unit_axis_.c_str());
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

}  // namespace autoware::ground_segmentation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::ground_segmentation::RANSACGroundFilterComponent)
