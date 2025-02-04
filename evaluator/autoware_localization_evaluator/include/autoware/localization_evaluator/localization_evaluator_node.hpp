// Copyright 2025 Tier IV, Inc.
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

#ifndef AUTOWARE__LOCALIZATION_EVALUATOR__LOCALIZATION_EVALUATOR_NODE_HPP_
#define AUTOWARE__LOCALIZATION_EVALUATOR__LOCALIZATION_EVALUATOR_NODE_HPP_

#include "autoware/localization_evaluator/metrics_calculator.hpp"
#include "autoware/universe_utils/math/accumulator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::localization_diagnostics
{
using autoware::universe_utils::Accumulator;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using nav_msgs::msg::Odometry;
/**
 * @brief Node for localization evaluation
 */
class LocalizationEvaluatorNode : public rclcpp::Node
{
public:
  explicit LocalizationEvaluatorNode(const rclcpp::NodeOptions & node_options);
  ~LocalizationEvaluatorNode();

  /**
   * @brief synchronized callback on current and gt localization
   * @param [in] msg odometry message
   * @param [in] msg_ref reference pose
   */
  void syncCallback(
    const Odometry::ConstSharedPtr & msg,
    const PoseWithCovarianceStamped::ConstSharedPtr & msg_ref);

  /**
   * @brief callback on current odometry
   * @param [in] msg odometry message
   */
  void onOdom(const Odometry::SharedPtr msg);

  /**
   * @brief publish the given metric statistic
   */
  DiagnosticStatus generateDiagnosticStatus(
    const Metric & metric, const Accumulator<double> & metric_stat) const;

private:
  // ROS
  message_filters::Subscriber<Odometry> odom_sub_;
  message_filters::Subscriber<PoseWithCovarianceStamped> pose_gt_sub_;

  typedef message_filters::sync_policies::ApproximateTime<Odometry, PoseWithCovarianceStamped>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> SyncExact;
  SyncExact sync_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr metrics_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

  // Parameters
  std::string output_file_str_;

  // Calculator
  MetricsCalculator metrics_calculator_;
  // Metrics
  std::vector<Metric> metrics_;
  std::deque<rclcpp::Time> stamps_;
  std::array<std::deque<Accumulator<double>>, static_cast<size_t>(Metric::SIZE)> metric_stats_;
  std::unordered_map<Metric, Accumulator<double>> metrics_dict_;
};
}  // namespace autoware::localization_diagnostics

#endif  // AUTOWARE__LOCALIZATION_EVALUATOR__LOCALIZATION_EVALUATOR_NODE_HPP_
