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

#include "ndt_scan_matcher/matrix_type.hpp"
#include "ndt_scan_matcher/ndt_scan_matcher_diagnostics_updater_core.hpp"
#include "ndt_scan_matcher/particle.hpp"
#include "ndt_scan_matcher/pose_array_interpolator.hpp"
#include "ndt_scan_matcher/util_func.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/transform/transforms.hpp>

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

tier4_debug_msgs::msg::Float32Stamped make_float32_stamped(
  const builtin_interfaces::msg::Time & stamp, const float data)
{
  using T = tier4_debug_msgs::msg::Float32Stamped;
  return tier4_debug_msgs::build<T>().stamp(stamp).data(data);
}

tier4_debug_msgs::msg::Int32Stamped make_int32_stamped(
  const builtin_interfaces::msg::Time & stamp, const int32_t data)
{
  using T = tier4_debug_msgs::msg::Int32Stamped;
  return tier4_debug_msgs::build<T>().stamp(stamp).data(data);
}

// cspell: ignore degrounded
NDTScanMatcher::NDTScanMatcher()
: Node("ndt_scan_matcher"),
  tf2_broadcaster_(*this),
  ndt_ptr_(new NormalDistributionsTransform),
  base_frame_("base_link"),
  ndt_base_frame_("ndt_base_link"),
  map_frame_("map"),
  converged_param_type_(ConvergedParamType::TRANSFORM_PROBABILITY),
  converged_param_transform_probability_(4.5),
  converged_param_nearest_voxel_transformation_likelihood_(2.3),
  initial_estimate_particles_num_(100),
  lidar_topic_timeout_sec_(1.0),
  initial_pose_timeout_sec_(1.0),
  initial_pose_distance_tolerance_m_(10.0),
  inversion_vector_threshold_(-0.9),
  oscillation_threshold_(10),
  output_pose_covariance_(),
  regularization_enabled_(declare_parameter<bool>("regularization_enabled")),
  estimate_scores_for_degrounded_scan_(
    declare_parameter<bool>("estimate_scores_for_degrounded_scan")),
  z_margin_for_ground_removal_(declare_parameter<double>("z_margin_for_ground_removal"))
{
  is_activated_ = false;
  is_succeed_latest_ndt_aling_service_ = false;
  is_running_ndt_aling_service_ = false;
  latest_ndt_aling_service_best_score_ = 0.0;

  int points_queue_size = this->declare_parameter<int>("input_sensor_points_queue_size");
  points_queue_size = std::max(points_queue_size, 0);
  RCLCPP_INFO(get_logger(), "points_queue_size: %d", points_queue_size);

  base_frame_ = this->declare_parameter<std::string>("base_frame");
  RCLCPP_INFO(get_logger(), "base_frame_id: %s", base_frame_.c_str());

  ndt_base_frame_ = this->declare_parameter<std::string>("ndt_base_frame");
  RCLCPP_INFO(get_logger(), "ndt_base_frame_id: %s", ndt_base_frame_.c_str());

  pclomp::NdtParams ndt_params{};
  ndt_params.trans_epsilon = this->declare_parameter<double>("trans_epsilon");
  ndt_params.step_size = this->declare_parameter<double>("step_size");
  ndt_params.resolution = this->declare_parameter<double>("resolution");
  ndt_params.max_iterations = this->declare_parameter<int>("max_iterations");
  ndt_params.num_threads = this->declare_parameter<int>("num_threads");
  ndt_params.num_threads = std::max(ndt_params.num_threads, 1);
  ndt_params.regularization_scale_factor =
    static_cast<float>(this->declare_parameter<float>("regularization_scale_factor"));
  ndt_ptr_->setParams(ndt_params);

  RCLCPP_INFO(
    get_logger(), "trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d",
    ndt_params.trans_epsilon, ndt_params.step_size, ndt_params.resolution,
    ndt_params.max_iterations);

  int converged_param_type_tmp = this->declare_parameter<int>("converged_param_type");
  converged_param_type_ = static_cast<ConvergedParamType>(converged_param_type_tmp);

  converged_param_transform_probability_ =
    this->declare_parameter<double>("converged_param_transform_probability");
  converged_param_nearest_voxel_transformation_likelihood_ =
    this->declare_parameter<double>("converged_param_nearest_voxel_transformation_likelihood");

  lidar_topic_timeout_sec_ = this->declare_parameter<double>("lidar_topic_timeout_sec");

  initial_pose_timeout_sec_ = this->declare_parameter<double>("initial_pose_timeout_sec");

  initial_pose_distance_tolerance_m_ =
    this->declare_parameter<double>("initial_pose_distance_tolerance_m");

  std::vector<double> output_pose_covariance =
    this->declare_parameter<std::vector<double>>("output_pose_covariance");
  for (std::size_t i = 0; i < output_pose_covariance.size(); ++i) {
    output_pose_covariance_[i] = output_pose_covariance[i];
  }

  initial_estimate_particles_num_ = this->declare_parameter<int>("initial_estimate_particles_num");

  rclcpp::CallbackGroup::SharedPtr initial_pose_callback_group;
  initial_pose_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::CallbackGroup::SharedPtr main_callback_group;
  main_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto initial_pose_sub_opt = rclcpp::SubscriptionOptions();
  initial_pose_sub_opt.callback_group = initial_pose_callback_group;

  auto main_sub_opt = rclcpp::SubscriptionOptions();
  main_sub_opt.callback_group = main_callback_group;

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ekf_pose_with_covariance", 100,
    std::bind(&NDTScanMatcher::callback_initial_pose, this, std::placeholders::_1),
    initial_pose_sub_opt);
  sensor_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS().keep_last(points_queue_size),
    std::bind(&NDTScanMatcher::callback_sensor_points, this, std::placeholders::_1), main_sub_opt);
  regularization_pose_sub_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "regularization_pose_with_covariance", 100,
      std::bind(&NDTScanMatcher::callback_regularization_pose, this, std::placeholders::_1));

  sensor_aligned_pose_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("points_aligned", 10);
  no_ground_points_aligned_pose_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("points_aligned_no_ground", 10);
  ndt_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ndt_pose", 10);
  ndt_pose_with_covariance_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "ndt_pose_with_covariance", 10);
  initial_pose_with_covariance_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initial_pose_with_covariance", 10);
  exe_time_pub_ = this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>("exe_time_ms", 10);
  transform_probability_pub_ =
    this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>("transform_probability", 10);
  nearest_voxel_transformation_likelihood_pub_ =
    this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
      "nearest_voxel_transformation_likelihood", 10);
  no_ground_transform_probability_pub_ =
    this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
      "no_ground_transform_probability", 10);
  no_ground_nearest_voxel_transformation_likelihood_pub_ =
    this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
      "no_ground_nearest_voxel_transformation_likelihood", 10);
  iteration_num_pub_ =
    this->create_publisher<tier4_debug_msgs::msg::Int32Stamped>("iteration_num", 10);
  initial_to_result_distance_pub_ =
    this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>("initial_to_result_distance", 10);
  initial_to_result_distance_old_pub_ =
    this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
      "initial_to_result_distance_old", 10);
  initial_to_result_distance_new_pub_ =
    this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
      "initial_to_result_distance_new", 10);
  ndt_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("ndt_marker", 10);
  ndt_monte_carlo_initial_pose_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "monte_carlo_initial_pose_marker", 10);

  service_ = this->create_service<tier4_localization_msgs::srv::PoseWithCovarianceStamped>(
    "ndt_align_srv",
    std::bind(
      &NDTScanMatcher::service_ndt_align, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(), main_callback_group);
  service_trigger_node_ = this->create_service<std_srvs::srv::SetBool>(
    "trigger_node_srv",
    std::bind(
      &NDTScanMatcher::service_trigger_node, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(), main_callback_group);

  tf2_listener_module_ = std::make_shared<Tf2ListenerModule>(this);

  diagnostics_module_ =
    std::make_unique<DiagnosticsModule>(this, "localization", "sensor_points_callback");
  diagnostics_update_module_ = std::make_unique<NDTScanMatcherDiagnosticsUpdaterCore>(this);

  use_dynamic_map_loading_ = this->declare_parameter<bool>("use_dynamic_map_loading");
  if (use_dynamic_map_loading_) {
    map_update_module_ = std::make_unique<MapUpdateModule>(
      this, &ndt_ptr_mtx_, ndt_ptr_, tf2_listener_module_, map_frame_, main_callback_group);
  } else {
    map_module_ = std::make_unique<MapModule>(this, &ndt_ptr_mtx_, ndt_ptr_, main_callback_group);
  }
}

void NDTScanMatcher::callback_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr initial_pose_msg_ptr)
{
  if (!is_activated_) return;

  // lock mutex for initial pose
  std::lock_guard<std::mutex> initial_pose_array_lock(initial_pose_array_mtx_);
  // if rosbag restart, clear buffer
  if (!initial_pose_msg_ptr_array_.empty()) {
    const builtin_interfaces::msg::Time & t_front =
      initial_pose_msg_ptr_array_.front()->header.stamp;
    const builtin_interfaces::msg::Time & t_msg = initial_pose_msg_ptr->header.stamp;
    if (t_front.sec > t_msg.sec || (t_front.sec == t_msg.sec && t_front.nanosec > t_msg.nanosec)) {
      initial_pose_msg_ptr_array_.clear();
    }
  }

  if (initial_pose_msg_ptr->header.frame_id == map_frame_) {
    initial_pose_msg_ptr_array_.push_back(initial_pose_msg_ptr);
  } else {
    // get TF from pose_frame to map_frame
    auto tf_pose_to_map_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>();
    tf2_listener_module_->get_transform(
      this->now(), map_frame_, initial_pose_msg_ptr->header.frame_id, tf_pose_to_map_ptr);

    // transform pose_frame to map_frame
    auto initial_pose_msg_in_map_ptr =
      std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    *initial_pose_msg_in_map_ptr = transform(*initial_pose_msg_ptr, *tf_pose_to_map_ptr);
    initial_pose_msg_in_map_ptr->header.stamp = initial_pose_msg_ptr->header.stamp;
    initial_pose_msg_ptr_array_.push_back(initial_pose_msg_in_map_ptr);
  }
}

void NDTScanMatcher::callback_regularization_pose(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_conv_msg_ptr)
{
  regularization_pose_msg_ptr_array_.push_back(pose_conv_msg_ptr);
}

void NDTScanMatcher::callback_sensor_points(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_msg_in_sensor_frame)
{
  diagnostics_module_->clear();
  initialize_diagnostics_key_value();

  validate_is_node_activated();
  bool is_set_sensor_points = set_input_source(sensor_points_msg_in_sensor_frame);
  validate_is_set_sensor_points(is_set_sensor_points);

  bool is_map_sensor_points = validate_is_set_map_points();
  if (is_activated_) {
    bool is_published_topic = false;
    if (is_set_sensor_points && is_map_sensor_points) {
      is_published_topic = process_scan_matching(sensor_points_msg_in_sensor_frame);
    }

    static size_t skipping_publish_num = 0;
    const size_t error_skkping_publish_num = 5;
    skipping_publish_num = is_published_topic ? 0 : (skipping_publish_num + 1);
    validate_skipping_publish_num(skipping_publish_num, error_skkping_publish_num);
  }

  diagnostics_module_->publish();
}

bool NDTScanMatcher::set_input_source(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_msg_in_sensor_frame)
{
  std::lock_guard<std::mutex> lock(ndt_ptr_mtx_);

  if (!validate_sensor_points_empty(sensor_points_msg_in_sensor_frame->width)) {
    return false;
  }

  if (!validate_sensor_points_delay_time(
        sensor_points_msg_in_sensor_frame->header.stamp, this->now(), lidar_topic_timeout_sec_)) {
    // If the delay time of the LiDAR topic exceeds the delay compensation time of ekf_localizer,
    // even if further processing continues, the estimated result will be rejected by ekf_localizer.
    // Therefore, it would be acceptable to exit the function here.
    // However, for now, we will continue the processing as it is.

    // return false;
  }

  // preprocess input pointcloud
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_sensor_frame(
    new pcl::PointCloud<PointSource>);
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_baselink_frame(
    new pcl::PointCloud<PointSource>);
  const std::string & sensor_frame = sensor_points_msg_in_sensor_frame->header.frame_id;

  pcl::fromROSMsg(*sensor_points_msg_in_sensor_frame, *sensor_points_in_sensor_frame);
  transform_sensor_measurement(
    sensor_frame, base_frame_, sensor_points_in_sensor_frame, sensor_points_in_baselink_frame);

  ndt_ptr_->setInputSource(sensor_points_in_baselink_frame);

  return true;
}

bool NDTScanMatcher::process_scan_matching(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_msg_in_sensor_frame)
{
  std::lock_guard<std::mutex> lock(ndt_ptr_mtx_);

  const rclcpp::Time sensor_ros_time = sensor_points_msg_in_sensor_frame->header.stamp;
  const auto sensor_points_in_baselink_frame = ndt_ptr_->getInputSource();

  // calculate initial pose
  std::unique_lock<std::mutex> initial_pose_array_lock(initial_pose_array_mtx_);

  const auto exe_start_time = std::chrono::system_clock::now();

  if (!validate_initial_pose_array_size(initial_pose_msg_ptr_array_.size())) {
    return false;
  }

  PoseArrayInterpolator interpolator(this, sensor_ros_time, initial_pose_msg_ptr_array_);

  if (!validate_time_stamp_difference(
        "old_pose", interpolator.get_old_pose().header.stamp, sensor_ros_time,
        initial_pose_timeout_sec_)) {
    return false;
  }

  if (!validate_time_stamp_difference(
        "new_pose", interpolator.get_new_pose().header.stamp, sensor_ros_time,
        initial_pose_timeout_sec_)) {
    return false;
  }

  if (!validate_position_difference(
        interpolator.get_old_pose().pose.pose.position,
        interpolator.get_new_pose().pose.pose.position, initial_pose_distance_tolerance_m_)) {
    return false;
  }

  pop_old_pose(initial_pose_msg_ptr_array_, sensor_ros_time);
  initial_pose_array_lock.unlock();

  // if regularization is enabled and available, set pose to NDT for regularization
  if (regularization_enabled_) {
    add_regularization_pose(sensor_ros_time);
  }

  // perform ndt scan matching
  const Eigen::Matrix4f initial_pose_matrix =
    pose_to_matrix4f(interpolator.get_current_pose().pose.pose);
  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();
  ndt_ptr_->align(*output_cloud, initial_pose_matrix);
  const pclomp::NdtResult ndt_result = ndt_ptr_->getResult();

  const auto exe_end_time = std::chrono::system_clock::now();
  const auto duration_micro_sec =
    std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count();
  const auto exe_time = static_cast<float>(duration_micro_sec) / 1000.0f;

  const geometry_msgs::msg::Pose result_pose_msg = matrix4f_to_pose(ndt_result.pose);
  std::vector<geometry_msgs::msg::Pose> transformation_msg_array;
  for (const auto & pose_matrix : ndt_result.transformation_array) {
    geometry_msgs::msg::Pose pose_ros = matrix4f_to_pose(pose_matrix);
    transformation_msg_array.push_back(pose_ros);
  }

  // perform several validations
  /*****************************************************************************
  The reason the add 2 to the ndt_ptr_->getMaximumIterations() is that there are bugs in
  implementation of ndt.
  1. gradient descent method ends when the iteration is greater than max_iteration if it does not
  converge (be careful it's 'greater than' instead of 'greater equal than'.)
     https://github.com/tier4/autoware.iv/blob/2323e5baa0b680d43a9219f5fb3b7a11dd9edc82/localization/pose_estimator/ndt_scan_matcher/ndt_omp/include/ndt_omp/ndt_omp_impl.hpp#L212
  2. iterate iteration count when end of gradient descent function.
     https://github.com/tier4/autoware.iv/blob/2323e5baa0b680d43a9219f5fb3b7a11dd9edc82/localization/pose_estimator/ndt_scan_matcher/ndt_omp/include/ndt_omp/ndt_omp_impl.hpp#L217

  These bugs are now resolved in original pcl implementation.
  https://github.com/PointCloudLibrary/pcl/blob/424c1c6a0ca97d94ca63e5daff4b183a4db8aae4/registration/include/pcl/registration/impl/ndt.hpp#L73-L180
  *****************************************************************************/
  bool is_ok_iteration_num =
    validate_num_iteration(ndt_result.iteration_num, ndt_ptr_->getMaximumIterations() + 2);

  bool is_ok_local_optimal_solution_oscillation = validate_local_optimal_solution_oscillation(
    transformation_msg_array, oscillation_threshold_, inversion_vector_threshold_);

  bool is_ok_converged_param = validate_converged_param(
    ndt_result.transform_probability, ndt_result.nearest_voxel_transformation_likelihood);

  // bool is_converged = is_ok_iteration_num && is_ok_converged_param;
  bool is_converged =
    (is_ok_iteration_num || is_ok_local_optimal_solution_oscillation) && is_ok_converged_param;
  if (!is_converged) {
    RCLCPP_WARN(get_logger(), "Not Converged");
  }

  const auto distance_from_initial_to_result = static_cast<float>(
    norm(interpolator.get_new_pose().pose.pose.position, result_pose_msg.position));
  const double warn_distance_from_initial_to_result = 3.0;
  if (!validate_distance_from_initial_to_result(
        distance_from_initial_to_result, warn_distance_from_initial_to_result)) {
    // return;
  }

  const double critical_upper_bound_exe_time_ms = 100.0;
  if (!validate_execution_time(exe_time, critical_upper_bound_exe_time_ms)) {
    // return;
  }

  // publish
  initial_pose_with_covariance_pub_->publish(interpolator.get_current_pose());
  exe_time_pub_->publish(make_float32_stamped(sensor_ros_time, exe_time));
  transform_probability_pub_->publish(
    make_float32_stamped(sensor_ros_time, ndt_result.transform_probability));
  nearest_voxel_transformation_likelihood_pub_->publish(
    make_float32_stamped(sensor_ros_time, ndt_result.nearest_voxel_transformation_likelihood));
  iteration_num_pub_->publish(make_int32_stamped(sensor_ros_time, ndt_result.iteration_num));
  publish_tf(sensor_ros_time, result_pose_msg);
  publish_pose(sensor_ros_time, result_pose_msg, is_converged);
  publish_marker(sensor_ros_time, transformation_msg_array);
  publish_initial_to_result_distances(
    sensor_ros_time, result_pose_msg, interpolator.get_current_pose(), interpolator.get_old_pose(),
    interpolator.get_new_pose());

  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_map_ptr(
    new pcl::PointCloud<PointSource>);
  tier4_autoware_utils::transformPointCloud(
    *sensor_points_in_baselink_frame, *sensor_points_in_map_ptr, ndt_result.pose);
  publish_point_cloud(sensor_ros_time, map_frame_, sensor_points_in_map_ptr);

  // whether use de-grounded points calculate score
  if (estimate_scores_for_degrounded_scan_) {
    // remove ground
    pcl::shared_ptr<pcl::PointCloud<PointSource>> no_ground_points_in_map_ptr(
      new pcl::PointCloud<PointSource>);
    for (std::size_t i = 0; i < sensor_points_in_map_ptr->size(); i++) {
      const float point_z = sensor_points_in_map_ptr->points[i].z;  // NOLINT
      if (point_z - matrix4f_to_pose(ndt_result.pose).position.z > z_margin_for_ground_removal_) {
        no_ground_points_in_map_ptr->points.push_back(sensor_points_in_map_ptr->points[i]);
      }
    }
    // pub remove-ground points
    sensor_msgs::msg::PointCloud2 no_ground_points_msg_in_map;
    pcl::toROSMsg(*no_ground_points_in_map_ptr, no_ground_points_msg_in_map);
    no_ground_points_msg_in_map.header.stamp = sensor_ros_time;
    no_ground_points_msg_in_map.header.frame_id = map_frame_;
    no_ground_points_aligned_pose_pub_->publish(no_ground_points_msg_in_map);
    // calculate score
    const auto no_ground_transform_probability = static_cast<float>(
      ndt_ptr_->calculateTransformationProbability(*no_ground_points_in_map_ptr));
    const auto no_ground_nearest_voxel_transformation_likelihood = static_cast<float>(
      ndt_ptr_->calculateNearestVoxelTransformationLikelihood(*no_ground_points_in_map_ptr));
    // pub score
    no_ground_transform_probability_pub_->publish(
      make_float32_stamped(sensor_ros_time, no_ground_transform_probability));
    no_ground_nearest_voxel_transformation_likelihood_pub_->publish(
      make_float32_stamped(sensor_ros_time, no_ground_nearest_voxel_transformation_likelihood));
  }

  return is_converged;
}

void NDTScanMatcher::transform_sensor_measurement(
  const std::string & source_frame, const std::string & target_frame,
  const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_input_ptr,
  pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_output_ptr)
{
  auto tf_target_to_source_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>();
  tf2_listener_module_->get_transform(
    this->now(), target_frame, source_frame, tf_target_to_source_ptr);
  const geometry_msgs::msg::PoseStamped target_to_source_pose_stamped =
    tier4_autoware_utils::transform2pose(*tf_target_to_source_ptr);
  const Eigen::Matrix4f base_to_sensor_matrix =
    pose_to_matrix4f(target_to_source_pose_stamped.pose);
  tier4_autoware_utils::transformPointCloud(
    *sensor_points_input_ptr, *sensor_points_output_ptr, base_to_sensor_matrix);
}

void NDTScanMatcher::publish_tf(
  const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg)
{
  geometry_msgs::msg::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time;
  result_pose_stamped_msg.header.frame_id = map_frame_;
  result_pose_stamped_msg.pose = result_pose_msg;
  tf2_broadcaster_.sendTransform(
    tier4_autoware_utils::pose2transform(result_pose_stamped_msg, ndt_base_frame_));
}

void NDTScanMatcher::publish_pose(
  const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg,
  const bool is_converged)
{
  geometry_msgs::msg::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time;
  result_pose_stamped_msg.header.frame_id = map_frame_;
  result_pose_stamped_msg.pose = result_pose_msg;

  geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.stamp = sensor_ros_time;
  result_pose_with_cov_msg.header.frame_id = map_frame_;
  result_pose_with_cov_msg.pose.pose = result_pose_msg;
  result_pose_with_cov_msg.pose.covariance = output_pose_covariance_;

  if (is_converged) {
    ndt_pose_pub_->publish(result_pose_stamped_msg);
    ndt_pose_with_covariance_pub_->publish(result_pose_with_cov_msg);
  }
}

void NDTScanMatcher::publish_point_cloud(
  const rclcpp::Time & sensor_ros_time, const std::string & frame_id,
  const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_in_map_ptr)
{
  sensor_msgs::msg::PointCloud2 sensor_points_msg_in_map;
  pcl::toROSMsg(*sensor_points_in_map_ptr, sensor_points_msg_in_map);
  sensor_points_msg_in_map.header.stamp = sensor_ros_time;
  sensor_points_msg_in_map.header.frame_id = frame_id;
  sensor_aligned_pose_pub_->publish(sensor_points_msg_in_map);
}

void NDTScanMatcher::publish_marker(
  const rclcpp::Time & sensor_ros_time, const std::vector<geometry_msgs::msg::Pose> & pose_array)
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = sensor_ros_time;
  marker.header.frame_id = map_frame_;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale = tier4_autoware_utils::createMarkerScale(0.3, 0.1, 0.1);
  int i = 0;
  marker.ns = "result_pose_matrix_array";
  marker.action = visualization_msgs::msg::Marker::ADD;
  for (const auto & pose_msg : pose_array) {
    marker.id = i++;
    marker.pose = pose_msg;
    marker.color = exchange_color_crc((1.0 * i) / 15.0);
    marker_array.markers.push_back(marker);
  }

  // TODO(Tier IV): delete old marker
  for (; i < ndt_ptr_->getMaximumIterations() + 2;) {
    marker.id = i++;
    marker.pose = geometry_msgs::msg::Pose();
    marker.color = exchange_color_crc(0);
    marker_array.markers.push_back(marker);
  }
  ndt_marker_pub_->publish(marker_array);
}

void NDTScanMatcher::publish_initial_to_result_distances(
  const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_cov_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_old_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_new_msg)
{
  const auto initial_to_result_distance =
    static_cast<float>(norm(initial_pose_cov_msg.pose.pose.position, result_pose_msg.position));
  initial_to_result_distance_pub_->publish(
    make_float32_stamped(sensor_ros_time, initial_to_result_distance));

  const auto initial_to_result_distance_old =
    static_cast<float>(norm(initial_pose_old_msg.pose.pose.position, result_pose_msg.position));
  initial_to_result_distance_old_pub_->publish(
    make_float32_stamped(sensor_ros_time, initial_to_result_distance_old));

  const auto initial_to_result_distance_new =
    static_cast<float>(norm(initial_pose_new_msg.pose.pose.position, result_pose_msg.position));
  initial_to_result_distance_new_pub_->publish(
    make_float32_stamped(sensor_ros_time, initial_to_result_distance_new));
}

std::optional<Eigen::Matrix4f> NDTScanMatcher::interpolate_regularization_pose(
  const rclcpp::Time & sensor_ros_time)
{
  if (regularization_pose_msg_ptr_array_.empty()) {
    return std::nullopt;
  }

  // synchronization
  PoseArrayInterpolator interpolator(this, sensor_ros_time, regularization_pose_msg_ptr_array_);

  pop_old_pose(regularization_pose_msg_ptr_array_, sensor_ros_time);

  // if the interpolate_pose fails, 0.0 is stored in the stamp
  if (rclcpp::Time(interpolator.get_current_pose().header.stamp).seconds() == 0.0) {
    return std::nullopt;
  }

  return pose_to_matrix4f(interpolator.get_current_pose().pose.pose);
}

void NDTScanMatcher::add_regularization_pose(const rclcpp::Time & sensor_ros_time)
{
  ndt_ptr_->unsetRegularizationPose();
  std::optional<Eigen::Matrix4f> pose_opt = interpolate_regularization_pose(sensor_ros_time);
  if (pose_opt.has_value()) {
    ndt_ptr_->setRegularizationPose(pose_opt.value());
    RCLCPP_DEBUG_STREAM(get_logger(), "Regularization pose is set to NDT");
  }
}

void NDTScanMatcher::service_trigger_node(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  is_activated_ = req->data;
  if (is_activated_) {
    std::lock_guard<std::mutex> initial_pose_array_lock(initial_pose_array_mtx_);
    initial_pose_msg_ptr_array_.clear();
  }
  res->success = true;
}

void NDTScanMatcher::service_ndt_align(
  const tier4_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
  tier4_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res)
{
  is_running_ndt_aling_service_ = true;
  // get TF from pose_frame to map_frame
  auto tf_pose_to_map_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>();
  tf2_listener_module_->get_transform(
    get_clock()->now(), map_frame_, req->pose_with_covariance.header.frame_id, tf_pose_to_map_ptr);

  // transform pose_frame to map_frame
  const auto initial_pose_msg_in_map_frame =
    transform(req->pose_with_covariance, *tf_pose_to_map_ptr);
  if (use_dynamic_map_loading_) {
    map_update_module_->update_map(initial_pose_msg_in_map_frame.pose.pose.position);
  }

  if (ndt_ptr_->getInputTarget() == nullptr) {
    res->success = false;
    is_succeed_latest_ndt_aling_service_ = false;
    is_running_ndt_aling_service_ = false;
    RCLCPP_WARN(get_logger(), "No InputTarget");
    return;
  }

  if (ndt_ptr_->getInputSource() == nullptr) {
    res->success = false;
    is_succeed_latest_ndt_aling_service_ = false;
    is_running_ndt_aling_service_ = false;
    RCLCPP_WARN(get_logger(), "No InputSource");
    return;
  }

  // mutex Map
  std::lock_guard<std::mutex> lock(ndt_ptr_mtx_);

  res->pose_with_covariance = align_using_monte_carlo(ndt_ptr_, initial_pose_msg_in_map_frame);
  res->success = true;
  res->pose_with_covariance.pose.covariance = req->pose_with_covariance.pose.covariance;

  is_succeed_latest_ndt_aling_service_ = true;
  is_running_ndt_aling_service_ = false;
}

geometry_msgs::msg::PoseWithCovarianceStamped NDTScanMatcher::align_using_monte_carlo(
  const std::shared_ptr<NormalDistributionsTransform> & ndt_ptr,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov)
{
  if (ndt_ptr->getInputTarget() == nullptr || ndt_ptr->getInputSource() == nullptr) {
    RCLCPP_WARN(get_logger(), "No Map or Sensor PointCloud");
    return geometry_msgs::msg::PoseWithCovarianceStamped();
  }

  output_pose_with_cov_to_log(get_logger(), "align_using_monte_carlo_input", initial_pose_with_cov);

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
      get_clock()->now(), map_frame_, tier4_autoware_utils::createMarkerScale(0.3, 0.1, 0.1),
      particle, i);
    ndt_monte_carlo_initial_pose_marker_pub_->publish(marker_array);

    auto sensor_points_in_map_ptr = std::make_shared<pcl::PointCloud<PointSource>>();
    tier4_autoware_utils::transformPointCloud(
      *ndt_ptr->getInputSource(), *sensor_points_in_map_ptr, ndt_result.pose);
    publish_point_cloud(initial_pose_with_cov.header.stamp, map_frame_, sensor_points_in_map_ptr);
  }

  auto best_particle_ptr = std::max_element(
    std::begin(particle_array), std::end(particle_array),
    [](const Particle & lhs, const Particle & rhs) { return lhs.score < rhs.score; });

  geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.stamp = initial_pose_with_cov.header.stamp;
  result_pose_with_cov_msg.header.frame_id = map_frame_;
  result_pose_with_cov_msg.pose.pose = best_particle_ptr->result_pose;

  output_pose_with_cov_to_log(
    get_logger(), "align_using_monte_carlo_output", result_pose_with_cov_msg);
  RCLCPP_INFO_STREAM(get_logger(), "best_score," << best_particle_ptr->score);
  latest_ndt_aling_service_best_score_ = best_particle_ptr->score;

  return result_pose_with_cov_msg;
}
