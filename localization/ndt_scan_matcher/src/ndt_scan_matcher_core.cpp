// Copyright 2015-2019 Autoware Foundation
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

#include "ndt_scan_matcher/ndt_scan_matcher_core.hpp"

#include "ndt_scan_matcher/debug.hpp"
#include "ndt_scan_matcher/matrix_type.hpp"
#include "ndt_scan_matcher/particle.hpp"
#include "ndt_scan_matcher/pose_array_interpolator.hpp"
#include "ndt_scan_matcher/util_func.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <pcl_conversions/pcl_conversions.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <thread>

NDTScanMatcher::NDTScanMatcher()
: Node("ndt_scan_matcher"),
  ndt_ptr_(new NormalDistributionsTransform),
  map_frame_("map")
{
  ndt_params_.trans_epsilon = this->declare_parameter<double>("trans_epsilon");
  ndt_params_.step_size = this->declare_parameter<double>("step_size");
  ndt_params_.resolution = this->declare_parameter<double>("resolution");
  ndt_params_.max_iterations = this->declare_parameter<int>("max_iterations");
  int search_method = this->declare_parameter<int>("neighborhood_search_method");
  ndt_params_.search_method = static_cast<pclomp::NeighborSearchMethod>(search_method);
  ndt_params_.num_threads = this->declare_parameter<int>("num_threads");
  ndt_params_.num_threads = std::max(ndt_params_.num_threads, 1);

  ndt_ptr_->setTransformationEpsilon(ndt_params_.trans_epsilon);
  ndt_ptr_->setStepSize(ndt_params_.step_size);
  ndt_ptr_->setResolution(ndt_params_.resolution);
  ndt_ptr_->setMaximumIterations(ndt_params_.max_iterations);
  ndt_ptr_->setRegularizationScaleFactor(ndt_params_.regularization_scale_factor);
  ndt_ptr_->setNeighborhoodSearchMethod(ndt_params_.search_method);
  ndt_ptr_->setNumThreads(ndt_params_.num_threads);

  RCLCPP_INFO(
    get_logger(), "trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d",
    ndt_params_.trans_epsilon, ndt_params_.step_size, ndt_params_.resolution,
    ndt_params_.max_iterations);

  initial_estimate_particles_num_ =
    this->declare_parameter("initial_estimate_particles_num", initial_estimate_particles_num_);

  rclcpp::CallbackGroup::SharedPtr initial_pose_callback_group;
  initial_pose_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::CallbackGroup::SharedPtr main_callback_group;
  main_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto main_sub_opt = rclcpp::SubscriptionOptions();
  main_sub_opt.callback_group = main_callback_group;

  map_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&NDTScanMatcher::callback_map_points, this, std::placeholders::_1), main_sub_opt);
  ndt_monte_carlo_aligned_points_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("monte_carlo_points_aligned", 10);
  ndt_monte_carlo_initial_pose_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "monte_carlo_initial_pose_marker", 10);

  diagnostics_pub_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

  service_ = this->create_service<tier4_localization_msgs::srv::PoseWithCovarianceStamped>(
    "ndt_align_srv",
    std::bind(
      &NDTScanMatcher::service_ndt_align, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(), main_callback_group);

  diagnostic_thread_ = std::thread(&NDTScanMatcher::timer_diagnostic, this);
  diagnostic_thread_.detach();

  ndt_ptr_ptr_ = std::make_shared<std::shared_ptr<NormalDistributionsTransform>>(ndt_ptr_);
  tf2_listener_module_ = std::make_shared<Tf2ListenerModule>(this);
  ndt_scan_matching_module_ = std::make_unique<NDTScanMatchingModule>(
    this,
    &ndt_ptr_mtx_,
    ndt_ptr_ptr_,
    tf2_listener_module_,
    map_frame_,
    main_callback_group,
    initial_pose_callback_group,
    &key_value_stdmap_);
  std::cout << "KOJI ndt_scan_matcher ctor " << *ndt_ptr_ptr_ << std::endl;

}

void NDTScanMatcher::timer_diagnostic()
{
  rclcpp::Rate rate(100);
  while (rclcpp::ok()) {
    diagnostic_msgs::msg::DiagnosticStatus diag_status_msg;
    diag_status_msg.name = "ndt_scan_matcher";
    diag_status_msg.hardware_id = "";

    for (const auto & key_value : key_value_stdmap_) {
      diagnostic_msgs::msg::KeyValue key_value_msg;
      key_value_msg.key = key_value.first;
      key_value_msg.value = key_value.second;
      diag_status_msg.values.push_back(key_value_msg);
    }

    diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diag_status_msg.message = "";
    if (key_value_stdmap_.count("state") && key_value_stdmap_["state"] == "Initializing") {
      diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      diag_status_msg.message += "Initializing State. ";
    }
    if (
      key_value_stdmap_.count("skipping_publish_num") &&
      std::stoi(key_value_stdmap_["skipping_publish_num"]) > 1) {
      diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      diag_status_msg.message += "skipping_publish_num > 1. ";
    }
    if (
      key_value_stdmap_.count("skipping_publish_num") &&
      std::stoi(key_value_stdmap_["skipping_publish_num"]) >= 5) {
      diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diag_status_msg.message += "skipping_publish_num exceed limit. ";
    }
    // Ignore local optimal solution
    if (
      key_value_stdmap_.count("is_local_optimal_solution_oscillation") &&
      std::stoi(key_value_stdmap_["is_local_optimal_solution_oscillation"])) {
      diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      diag_status_msg.message = "local optimal solution oscillation occurred";
    }

    diagnostic_msgs::msg::DiagnosticArray diag_msg;
    diag_msg.header.stamp = this->now();
    diag_msg.status.push_back(diag_status_msg);

    diagnostics_pub_->publish(diag_msg);

    rate.sleep();
  }
}

void NDTScanMatcher::service_ndt_align(
  const tier4_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
  tier4_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res)
{
  // get TF from pose_frame to map_frame
  auto TF_pose_to_map_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>();
  tf2_listener_module_->get_transform(
    this->now(), map_frame_, req->pose_with_covariance.header.frame_id, TF_pose_to_map_ptr);

  // transform pose_frame to map_frame
  const auto mapTF_initial_pose_msg = transform(req->pose_with_covariance, *TF_pose_to_map_ptr);

  if ((*ndt_ptr_ptr_)->getInputTarget() == nullptr) {
    res->success = false;
    RCLCPP_WARN(get_logger(), "No InputTarget");
    return;
  }

  if ((*ndt_ptr_ptr_)->getInputSource() == nullptr) {
    res->success = false;
    RCLCPP_WARN(get_logger(), "No InputSource");
    return;
  }

  // mutex Map
  std::lock_guard<std::mutex> lock(ndt_ptr_mtx_);

  key_value_stdmap_["state"] = "Aligning";
  res->pose_with_covariance = align_using_monte_carlo((*ndt_ptr_ptr_), mapTF_initial_pose_msg);
  key_value_stdmap_["state"] = "Sleeping";
  res->success = true;
  res->pose_with_covariance.pose.covariance = req->pose_with_covariance.pose.covariance;
}

void NDTScanMatcher::callback_map_points(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr map_points_msg_ptr)
{
  std::shared_ptr<NormalDistributionsTransform> new_ndt_ptr(new NormalDistributionsTransform);
  new_ndt_ptr->setTransformationEpsilon(ndt_params_.trans_epsilon);
  new_ndt_ptr->setStepSize(ndt_params_.step_size);
  new_ndt_ptr->setResolution(ndt_params_.resolution);
  new_ndt_ptr->setMaximumIterations(ndt_params_.max_iterations);
  new_ndt_ptr->setRegularizationScaleFactor(ndt_params_.regularization_scale_factor);
  new_ndt_ptr->setNeighborhoodSearchMethod(ndt_params_.search_method);
  new_ndt_ptr->setNumThreads(ndt_params_.num_threads);

  pcl::shared_ptr<pcl::PointCloud<PointTarget>> map_points_ptr(new pcl::PointCloud<PointTarget>);
  pcl::fromROSMsg(*map_points_msg_ptr, *map_points_ptr);
  new_ndt_ptr->setInputTarget(map_points_ptr);
  // create Thread
  // detach
  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();
  new_ndt_ptr->align(*output_cloud);

  // swap
  ndt_ptr_mtx_.lock();
  (*ndt_ptr_ptr_).swap(new_ndt_ptr);
  ndt_ptr_mtx_.unlock();
  std::cout << "KOJI ndt_scan_matcher callbackMap " << *ndt_ptr_ptr_  << std::endl;
}

geometry_msgs::msg::PoseWithCovarianceStamped NDTScanMatcher::align_using_monte_carlo(
  const std::shared_ptr<NormalDistributionsTransform> & ndt_ptr,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov)
{
  if (ndt_ptr->getInputTarget() == nullptr || ndt_ptr->getInputSource() == nullptr) {
    RCLCPP_WARN(get_logger(), "No Map or Sensor PointCloud");
    return geometry_msgs::msg::PoseWithCovarianceStamped();
  }

  // generateParticle
  const auto initial_poses =
    create_random_pose_array(initial_pose_with_cov, initial_estimate_particles_num_);

  std::vector<Particle> particle_array;
  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();

  for (unsigned int i = 0; i < initial_poses.size(); i++) {
    const auto & initial_pose = initial_poses[i];
    const Eigen::Matrix4f initial_pose_matrix = pose_to_matrix4f(initial_pose);
    ndt_ptr->align(*output_cloud, initial_pose_matrix);
    const pclomp::NdtResult ndt_result = ndt_ptr->getResult();

    Particle particle(
      initial_pose, matrix4f_to_pose(ndt_result.pose), ndt_result.transform_probability,
      ndt_result.iteration_num);
    particle_array.push_back(particle);
    const auto marker_array = make_debug_markers(
      this->now(), map_frame_, tier4_autoware_utils::createMarkerScale(0.3, 0.1, 0.1), particle, i);
    ndt_monte_carlo_initial_pose_marker_pub_->publish(marker_array);

    auto sensor_points_mapTF_ptr = std::make_shared<pcl::PointCloud<PointSource>>();
    pcl::transformPointCloud(*ndt_ptr->getInputSource(), *sensor_points_mapTF_ptr, ndt_result.pose);
    publish_monte_carlo_point_cloud(initial_pose_with_cov.header.stamp, map_frame_, sensor_points_mapTF_ptr);
  }

  auto best_particle_ptr = std::max_element(
    std::begin(particle_array), std::end(particle_array),
    [](const Particle & lhs, const Particle & rhs) { return lhs.score < rhs.score; });

  geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.stamp = initial_pose_with_cov.header.stamp;
  result_pose_with_cov_msg.header.frame_id = map_frame_;
  result_pose_with_cov_msg.pose.pose = best_particle_ptr->result_pose;
  // ndt_pose_with_covariance_pub_->publish(result_pose_with_cov_msg);

  return result_pose_with_cov_msg;
}


void NDTScanMatcher::publish_monte_carlo_point_cloud(
  const rclcpp::Time & sensor_ros_time, const std::string & frame_id,
  const std::shared_ptr<const pcl::PointCloud<PointSource>> & sensor_points_mapTF_ptr)
{
  sensor_msgs::msg::PointCloud2 sensor_points_mapTF_msg;
  pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_points_mapTF_msg);
  sensor_points_mapTF_msg.header.stamp = sensor_ros_time;
  sensor_points_mapTF_msg.header.frame_id = frame_id;
  ndt_monte_carlo_aligned_points_pub_->publish(sensor_points_mapTF_msg);
}
