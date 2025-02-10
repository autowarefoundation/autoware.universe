// Copyright 2018-2019 Autoware Foundation
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

#include "autoware/ekf_localizer/ekf_localizer.hpp"

#include "autoware/ekf_localizer/diagnostics.hpp"
#include "autoware/ekf_localizer/string.hpp"
#include "autoware/ekf_localizer/warning_message.hpp"
#include "autoware/localization_util/covariance_ellipse.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/math/unit_conversion.hpp>
#include <autoware/universe_utils/ros/msg_covariance.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>

#include <fmt/core.h>

#include <algorithm>
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace autoware::ekf_localizer
{

// clang-format off
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl // NOLINT
#define DEBUG_INFO(...) {if (params_.show_debug_info) {RCLCPP_INFO(__VA_ARGS__);}} // NOLINT
// clang-format on

using std::placeholders::_1;

EKFLocalizer::EKFLocalizer(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("ekf_localizer", node_options),
  warning_(std::make_shared<Warning>(this)),
  tf2_buffer_(this->get_clock()),
  tf2_listener_(tf2_buffer_),
  params_(this),
  ekf_dt_(params_.ekf_dt),
  pose_queue_(params_.pose_smoothing_steps),
  twist_queue_(params_.twist_smoothing_steps)
{
  is_activated_ = false;
  is_set_initialpose_ = false;

  /* initialize ros system */
  timer_control_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(ekf_dt_),
    std::bind(&EKFLocalizer::timer_callback, this));

  pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("ekf_pose", 1);
  pub_pose_cov_ =
    create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("ekf_pose_with_covariance", 1);
  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("ekf_odom", 1);
  pub_twist_ = create_publisher<geometry_msgs::msg::TwistStamped>("ekf_twist", 1);
  pub_twist_cov_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "ekf_twist_with_covariance", 1);
  pub_yaw_bias_ =
    create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>("estimated_yaw_bias", 1);
  pub_biased_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("ekf_biased_pose", 1);
  pub_biased_pose_cov_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ekf_biased_pose_with_covariance", 1);
  pub_diag_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  pub_processing_time_ = create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/processing_time_ms", 1);
  sub_initialpose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1, std::bind(&EKFLocalizer::callback_initial_pose, this, _1));
  sub_pose_with_cov_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "in_pose_with_covariance", 1,
    std::bind(&EKFLocalizer::callback_pose_with_covariance, this, _1));
  sub_twist_with_cov_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "in_twist_with_covariance", 1,
    std::bind(&EKFLocalizer::callback_twist_with_covariance, this, _1));
  service_trigger_node_ = create_service<std_srvs::srv::SetBool>(
    "trigger_node_srv",
    std::bind(
      &EKFLocalizer::service_trigger_node, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile());

  tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(
    std::shared_ptr<rclcpp::Node>(this, [](auto) {}));

  ekf_module_ = std::make_unique<EKFModule>(warning_, params_);
  logger_configure_ = std::make_unique<autoware::universe_utils::LoggerLevelConfigure>(this);
}

/*
 * update_predict_frequency
 */
void EKFLocalizer::update_predict_frequency(const rclcpp::Time & current_time)
{
  if (last_predict_time_) {
    if (current_time < *last_predict_time_) {
      warning_->warn("Detected jump back in time");
    } else {
      /* Measure dt */
      ekf_dt_ = (current_time - *last_predict_time_).seconds();
      DEBUG_INFO(
        get_logger(), "[EKF] update ekf_dt_ to %f seconds (= %f hz)", ekf_dt_, 1 / ekf_dt_);

      if (ekf_dt_ > 10.0) {
        ekf_dt_ = 10.0;
        RCLCPP_WARN(
          get_logger(), "Large ekf_dt_ detected!! (%f sec) Capped to 10.0 seconds", ekf_dt_);
      } else if (ekf_dt_ > static_cast<double>(params_.pose_smoothing_steps) / params_.ekf_rate) {
        RCLCPP_WARN(
          get_logger(), "EKF period may be too slow to finish pose smoothing!! (%f sec) ", ekf_dt_);
      }

      /* Register dt and accumulate time delay */
      ekf_module_->accumulate_delay_time(ekf_dt_);
    }
  }
  last_predict_time_ = std::make_shared<const rclcpp::Time>(current_time);
}

/*
 * timer_callback
 */
void EKFLocalizer::timer_callback()
{
  stop_watch_timer_cb_.tic();

  const rclcpp::Time current_time = this->now();

  if (!is_activated_) {
    warning_->warn_throttle(
      "The node is not activated. Provide initial pose to pose_initializer", 2000);
    publish_diagnostics(geometry_msgs::msg::PoseStamped{}, current_time);
    return;
  }

  if (!is_set_initialpose_) {
    warning_->warn_throttle(
      "Initial pose is not set. Provide initial pose to pose_initializer", 2000);
    publish_diagnostics(geometry_msgs::msg::PoseStamped{}, current_time);
    return;
  }

  DEBUG_INFO(get_logger(), "========================= timer called =========================");

  /* update predict frequency with measured timer rate */
  update_predict_frequency(current_time);

  /* predict model in EKF */
  stop_watch_.tic();
  DEBUG_INFO(get_logger(), "------------------------- start prediction -------------------------");
  ekf_module_->predict_with_delay(ekf_dt_);
  DEBUG_INFO(get_logger(), "[EKF] predictKinematicsModel calc time = %f [ms]", stop_watch_.toc());
  DEBUG_INFO(get_logger(), "------------------------- end prediction -------------------------\n");

  /* pose measurement update */
  pose_diag_info_.queue_size = pose_queue_.size();
  pose_diag_info_.is_passed_delay_gate = true;
  pose_diag_info_.delay_time = 0.0;
  pose_diag_info_.delay_time_threshold = 0.0;
  pose_diag_info_.is_passed_mahalanobis_gate = true;
  pose_diag_info_.mahalanobis_distance = 0.0;

  bool pose_is_updated = false;

  if (!pose_queue_.empty()) {
    DEBUG_INFO(get_logger(), "------------------------- start Pose -------------------------");
    stop_watch_.tic();

    // save the initial size because the queue size can change in the loop
    const size_t n = pose_queue_.size();
    for (size_t i = 0; i < n; ++i) {
      const auto pose = pose_queue_.pop_increment_age();
      bool is_updated = ekf_module_->measurement_update_pose(*pose, current_time, pose_diag_info_);
      if (is_updated) {
        pose_is_updated = true;
      }
    }
    DEBUG_INFO(
      get_logger(), "[EKF] measurement_update_pose calc time = %f [ms]", stop_watch_.toc());
    DEBUG_INFO(get_logger(), "------------------------- end Pose -------------------------\n");
  }
  pose_diag_info_.no_update_count = pose_is_updated ? 0 : (pose_diag_info_.no_update_count + 1);

  /* twist measurement update */
  twist_diag_info_.queue_size = twist_queue_.size();
  twist_diag_info_.is_passed_delay_gate = true;
  twist_diag_info_.delay_time = 0.0;
  twist_diag_info_.delay_time_threshold = 0.0;
  twist_diag_info_.is_passed_mahalanobis_gate = true;
  twist_diag_info_.mahalanobis_distance = 0.0;

  bool twist_is_updated = false;

  if (!twist_queue_.empty()) {
    DEBUG_INFO(get_logger(), "------------------------- start Twist -------------------------");
    stop_watch_.tic();

    // save the initial size because the queue size can change in the loop
    const size_t n = twist_queue_.size();
    for (size_t i = 0; i < n; ++i) {
      const auto twist = twist_queue_.pop_increment_age();
      bool is_updated =
        ekf_module_->measurement_update_twist(*twist, current_time, twist_diag_info_);
      if (is_updated) {
        twist_is_updated = true;
      }
    }
    DEBUG_INFO(
      get_logger(), "[EKF] measurement_update_twist calc time = %f [ms]", stop_watch_.toc());
    DEBUG_INFO(get_logger(), "------------------------- end Twist -------------------------\n");
  }
  twist_diag_info_.no_update_count = twist_is_updated ? 0 : (twist_diag_info_.no_update_count + 1);

  const geometry_msgs::msg::PoseStamped current_ekf_pose =
    ekf_module_->get_current_pose(current_time, false);
  const geometry_msgs::msg::PoseStamped current_biased_ekf_pose =
    ekf_module_->get_current_pose(current_time, true);
  const geometry_msgs::msg::TwistStamped current_ekf_twist =
    ekf_module_->get_current_twist(current_time);

  /* publish ekf result */
  publish_estimate_result(current_ekf_pose, current_biased_ekf_pose, current_ekf_twist);
  publish_diagnostics(current_ekf_pose, current_time);

  /* publish processing time */
  const double elapsed_time = stop_watch_timer_cb_.toc();
  pub_processing_time_->publish(
    autoware_internal_debug_msgs::build<autoware_internal_debug_msgs::msg::Float64Stamped>()
      .stamp(current_time)
      .data(elapsed_time));
}

/*
 * get_transform_from_tf
 */
bool EKFLocalizer::get_transform_from_tf(
  std::string parent_frame, std::string child_frame,
  geometry_msgs::msg::TransformStamped & transform)
{
  parent_frame = erase_leading_slash(parent_frame);
  child_frame = erase_leading_slash(child_frame);

  try {
    transform = tf2_buffer_.lookupTransform(parent_frame, child_frame, tf2::TimePointZero);
    return true;
  } catch (tf2::TransformException & ex) {
    warning_->warn(ex.what());
  }
  return false;
}

/*
 * callback_initial_pose
 */
void EKFLocalizer::callback_initial_pose(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped transform;
  if (!get_transform_from_tf(params_.pose_frame_id, msg->header.frame_id, transform)) {
    RCLCPP_ERROR(
      get_logger(), "[EKF] TF transform failed. parent = %s, child = %s",
      params_.pose_frame_id.c_str(), msg->header.frame_id.c_str());
  }
  ekf_module_->initialize(*msg, transform);

  is_set_initialpose_ = true;
}

/*
 * callback_pose_with_covariance
 */
void EKFLocalizer::callback_pose_with_covariance(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (!is_activated_ && !is_set_initialpose_) {
    return;
  }

  pose_queue_.push(msg);

  publish_callback_return_diagnostics("pose", msg->header.stamp);
}

/*
 * callback_twist_with_covariance
 */
void EKFLocalizer::callback_twist_with_covariance(
  geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  // Ignore twist if velocity is too small.
  // Note that this inequality must not include "equal".
  if (std::abs(msg->twist.twist.linear.x) < params_.threshold_observable_velocity_mps) {
    msg->twist.covariance[0 * 6 + 0] = 10000.0;
  }
  twist_queue_.push(msg);

  publish_callback_return_diagnostics("twist", msg->header.stamp);
}

/*
 * publish_estimate_result
 */
void EKFLocalizer::publish_estimate_result(
  const geometry_msgs::msg::PoseStamped & current_ekf_pose,
  const geometry_msgs::msg::PoseStamped & current_biased_ekf_pose,
  const geometry_msgs::msg::TwistStamped & current_ekf_twist)
{
  /* publish latest pose */
  pub_pose_->publish(current_ekf_pose);
  pub_biased_pose_->publish(current_biased_ekf_pose);

  /* publish latest pose with covariance */
  geometry_msgs::msg::PoseWithCovarianceStamped pose_cov;
  pose_cov.header.stamp = current_ekf_pose.header.stamp;
  pose_cov.header.frame_id = current_ekf_pose.header.frame_id;
  pose_cov.pose.pose = current_ekf_pose.pose;
  pose_cov.pose.covariance = ekf_module_->get_current_pose_covariance();
  pub_pose_cov_->publish(pose_cov);

  geometry_msgs::msg::PoseWithCovarianceStamped biased_pose_cov = pose_cov;
  biased_pose_cov.pose.pose = current_biased_ekf_pose.pose;
  pub_biased_pose_cov_->publish(biased_pose_cov);

  /* publish latest twist */
  pub_twist_->publish(current_ekf_twist);

  /* publish latest twist with covariance */
  geometry_msgs::msg::TwistWithCovarianceStamped twist_cov;
  twist_cov.header.stamp = current_ekf_twist.header.stamp;
  twist_cov.header.frame_id = current_ekf_twist.header.frame_id;
  twist_cov.twist.twist = current_ekf_twist.twist;
  twist_cov.twist.covariance = ekf_module_->get_current_twist_covariance();
  pub_twist_cov_->publish(twist_cov);

  /* publish yaw bias */
  autoware_internal_debug_msgs::msg::Float64Stamped yawb;
  yawb.stamp = current_ekf_twist.header.stamp;
  yawb.data = ekf_module_->get_yaw_bias();
  pub_yaw_bias_->publish(yawb);

  /* publish latest odometry */
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = current_ekf_pose.header.stamp;
  odometry.header.frame_id = current_ekf_pose.header.frame_id;
  odometry.child_frame_id = "base_link";
  odometry.pose = pose_cov.pose;
  odometry.twist = twist_cov.twist;
  pub_odom_->publish(odometry);

  /* publish tf */
  const geometry_msgs::msg::TransformStamped transform_stamped =
    autoware::universe_utils::pose2transform(current_ekf_pose, "base_link");
  tf_br_->sendTransform(transform_stamped);
}

void EKFLocalizer::publish_diagnostics(
  const geometry_msgs::msg::PoseStamped & current_ekf_pose, const rclcpp::Time & current_time)
{
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> diag_status_array;

  diag_status_array.push_back(check_process_activated(is_activated_));
  diag_status_array.push_back(check_set_initialpose(is_set_initialpose_));

  if (is_activated_ && is_set_initialpose_) {
    diag_status_array.push_back(check_measurement_updated(
      "pose", pose_diag_info_.no_update_count, params_.pose_no_update_count_threshold_warn,
      params_.pose_no_update_count_threshold_error));
    diag_status_array.push_back(check_measurement_queue_size("pose", pose_diag_info_.queue_size));
    diag_status_array.push_back(check_measurement_delay_gate(
      "pose", pose_diag_info_.is_passed_delay_gate, pose_diag_info_.delay_time,
      pose_diag_info_.delay_time_threshold));
    diag_status_array.push_back(check_measurement_mahalanobis_gate(
      "pose", pose_diag_info_.is_passed_mahalanobis_gate, pose_diag_info_.mahalanobis_distance,
      params_.pose_gate_dist));

    diag_status_array.push_back(check_measurement_updated(
      "twist", twist_diag_info_.no_update_count, params_.twist_no_update_count_threshold_warn,
      params_.twist_no_update_count_threshold_error));
    diag_status_array.push_back(check_measurement_queue_size("twist", twist_diag_info_.queue_size));
    diag_status_array.push_back(check_measurement_delay_gate(
      "twist", twist_diag_info_.is_passed_delay_gate, twist_diag_info_.delay_time,
      twist_diag_info_.delay_time_threshold));
    diag_status_array.push_back(check_measurement_mahalanobis_gate(
      "twist", twist_diag_info_.is_passed_mahalanobis_gate, twist_diag_info_.mahalanobis_distance,
      params_.twist_gate_dist));

    geometry_msgs::msg::PoseWithCovariance pose_cov;
    pose_cov.pose = current_ekf_pose.pose;
    pose_cov.covariance = ekf_module_->get_current_pose_covariance();
    const autoware::localization_util::Ellipse ellipse =
      autoware::localization_util::calculate_xy_ellipse(pose_cov, params_.ellipse_scale);
    diag_status_array.push_back(check_covariance_ellipse(
      "cov_ellipse_long_axis", ellipse.long_radius, params_.warn_ellipse_size,
      params_.error_ellipse_size));
    diag_status_array.push_back(check_covariance_ellipse(
      "cov_ellipse_lateral_direction", ellipse.size_lateral_direction,
      params_.warn_ellipse_size_lateral_direction, params_.error_ellipse_size_lateral_direction));
  }

  diagnostic_msgs::msg::DiagnosticStatus diag_merged_status;
  diag_merged_status = merge_diagnostic_status(diag_status_array);
  diag_merged_status.name = "localization: " + std::string(this->get_name());
  diag_merged_status.hardware_id = this->get_name();

  diagnostic_msgs::msg::DiagnosticArray diag_msg;
  diag_msg.header.stamp = current_time;
  diag_msg.status.push_back(diag_merged_status);
  pub_diag_->publish(diag_msg);
}

void EKFLocalizer::publish_callback_return_diagnostics(
  const std::string & callback_name, const rclcpp::Time & current_time)
{
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = "topic_time_stamp";
  key_value.value = std::to_string(current_time.nanoseconds());
  diagnostic_msgs::msg::DiagnosticStatus diag_status;
  diag_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  diag_status.name =
    "localization: " + std::string(this->get_name()) + ": callback_" + callback_name;
  diag_status.hardware_id = this->get_name();
  diag_status.message = "OK";
  diag_status.values.push_back(key_value);
  diagnostic_msgs::msg::DiagnosticArray diag_msg;
  diag_msg.header.stamp = current_time;
  diag_msg.status.push_back(diag_status);
  pub_diag_->publish(diag_msg);
}

/**
 * @brief trigger node
 */
void EKFLocalizer::service_trigger_node(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  if (req->data) {
    pose_queue_.clear();
    twist_queue_.clear();
    is_activated_ = true;
  } else {
    is_activated_ = false;
    is_set_initialpose_ = false;
  }
  res->success = true;
}

}  // namespace autoware::ekf_localizer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::ekf_localizer::EKFLocalizer)
