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

#ifndef AUTOWARE__CONTROL_EVALUATOR__CONTROL_EVALUATOR_NODE_HPP_
#define AUTOWARE__CONTROL_EVALUATOR__CONTROL_EVALUATOR_NODE_HPP_

#include "autoware/control_evaluator/metrics/deviation_metrics.hpp"
#include "autoware/control_evaluator/metrics/metric.hpp"
#include "autoware/universe_utils/math/accumulator.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware/universe_utils/ros/polling_subscriber.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_metric_msgs/msg/metric.hpp>
#include <tier4_metric_msgs/msg/metric_array.hpp>

#include <deque>
#include <optional>
#include <string>
#include <vector>

namespace control_diagnostics
{
using autoware::universe_utils::Accumulator;
using autoware_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::Odometry;
using LaneletMapBin = autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::LaneletRoute;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using MetricMsg = tier4_metric_msgs::msg::Metric;
using MetricArrayMsg = tier4_metric_msgs::msg::MetricArray;

/**
 * @brief Node for control evaluation
 */
class ControlEvaluatorNode : public rclcpp::Node
{
public:
  explicit ControlEvaluatorNode(const rclcpp::NodeOptions & node_options);
  ~ControlEvaluatorNode() override;

  void AddMetricMsg(const Metric & metric, const double & metric_value);
  void AddLateralDeviationMetricMsg(const Trajectory & traj, const Point & ego_point);
  void AddYawDeviationMetricMsg(const Trajectory & traj, const Pose & ego_pose);
  void AddGoalLongitudinalDeviationMetricMsg(const Pose & ego_pose);
  void AddGoalLateralDeviationMetricMsg(const Pose & ego_pose);
  void AddGoalYawDeviationMetricMsg(const Pose & ego_pose);

  void AddLaneletMetricMsg(const Pose & ego_pose);
  void AddKinematicStateMetricMsg(
    const Odometry & odom, const AccelWithCovarianceStamped & accel_stamped);

  void onTimer();

private:
  autoware::universe_utils::InterProcessPollingSubscriber<Odometry> odometry_sub_{
    this, "~/input/odometry"};
  autoware::universe_utils::InterProcessPollingSubscriber<AccelWithCovarianceStamped> accel_sub_{
    this, "~/input/acceleration"};
  autoware::universe_utils::InterProcessPollingSubscriber<Trajectory> traj_sub_{
    this, "~/input/trajectory"};
  autoware::universe_utils::InterProcessPollingSubscriber<
    LaneletRoute, autoware::universe_utils::polling_policy::Newest>
    route_subscriber_{this, "~/input/route", rclcpp::QoS{1}.transient_local()};
  autoware::universe_utils::InterProcessPollingSubscriber<
    LaneletMapBin, autoware::universe_utils::polling_policy::Newest>
    vector_map_subscriber_{this, "~/input/vector_map", rclcpp::QoS{1}.transient_local()};

  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    processing_time_pub_;
  rclcpp::Publisher<MetricArrayMsg>::SharedPtr metrics_pub_;

  // update Route Handler
  void getRouteData();

  // Parameters
  bool output_metrics_;

  // Metric
  const std::vector<Metric> metrics_ = {
    // collect all metrics
    Metric::lateral_deviation,      Metric::yaw_deviation,      Metric::goal_longitudinal_deviation,
    Metric::goal_lateral_deviation, Metric::goal_yaw_deviation,
  };

  std::array<Accumulator<double>, static_cast<size_t>(Metric::SIZE)>
    metric_accumulators_;  // 3(min, max, mean) * metric_size

  MetricArrayMsg metrics_msg_;
  autoware::route_handler::RouteHandler route_handler_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<AccelWithCovarianceStamped> prev_acc_stamped_{std::nullopt};
};
}  // namespace control_diagnostics

#endif  // AUTOWARE__CONTROL_EVALUATOR__CONTROL_EVALUATOR_NODE_HPP_
