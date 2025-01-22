// Copyright 2024 TIER IV, Inc.
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

#ifndef PERCEPTION_ONLINE_EVALUATOR__PERCEPTION_ONLINE_EVALUATOR_NODE_HPP_
#define PERCEPTION_ONLINE_EVALUATOR__PERCEPTION_ONLINE_EVALUATOR_NODE_HPP_

#include "autoware/universe_utils/math/accumulator.hpp"
#include "perception_online_evaluator/metrics_calculator.hpp"
#include "perception_online_evaluator/parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "autoware_perception_msgs/msg/predicted_objects.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tier4_metric_msgs/msg/metric.hpp>
#include <tier4_metric_msgs/msg/metric_array.hpp>

#include <array>
#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace perception_diagnostics
{
using autoware::universe_utils::Accumulator;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObjects;
using nav_msgs::msg::Odometry;
using TFMessage = tf2_msgs::msg::TFMessage;

using MarkerArray = visualization_msgs::msg::MarkerArray;

/**
 * @brief Node for perception evaluation
 */
class PerceptionOnlineEvaluatorNode : public rclcpp::Node
{
public:
  explicit PerceptionOnlineEvaluatorNode(const rclcpp::NodeOptions & node_options);
  ~PerceptionOnlineEvaluatorNode() {}

  /**
   * @brief callback on receiving a dynamic objects array
   * @param [in] objects_msg received dynamic object array message
   */
  void onObjects(const PredictedObjects::ConstSharedPtr objects_msg);

  /**
   * @brief Convert metric statistic to `tier4_metric_msgs::msg::Metric` and append to
   * `tier4_metric_msgs::msg::MetricArray`.
   *
   * @param metric Metric name.
   * @param metric_stat Metric statistic.
   * @param metrics_msg Metrics value container.
   */
  void toMetricMsg(
    const std::string & metric, const Accumulator<double> & metric_stat,
    tier4_metric_msgs::msg::MetricArray & metrics_msg) const;

  /**
   * @brief Convert metric value to `tier4_metric_msgs::msg::Metric` and append to
   * `tier4_metric_msgs::msg::MetricArray
   *
   * @param metric Metric name.
   * @param metric_stat Metric value.
   * @param metrics_msg Metrics value container.
   */
  void toMetricMsg(
    const std::string & metric, const double metric_value,
    tier4_metric_msgs::msg::MetricArray & metrics_msg) const;

private:
  // Subscribers and publishers
  rclcpp::Subscription<PredictedObjects>::SharedPtr objects_sub_;
  rclcpp::Publisher<tier4_metric_msgs::msg::MetricArray>::SharedPtr metrics_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_;

  // TF
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Parameters
  std::shared_ptr<Parameters> parameters_;
  void initParameter();
  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & parameters);
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  // Metrics Calculator
  MetricsCalculator metrics_calculator_;
  std::deque<rclcpp::Time> stamps_;
  void publishMetrics();

  // Debug
  void publishDebugMarker();
};
}  // namespace perception_diagnostics

#endif  // PERCEPTION_ONLINE_EVALUATOR__PERCEPTION_ONLINE_EVALUATOR_NODE_HPP_
