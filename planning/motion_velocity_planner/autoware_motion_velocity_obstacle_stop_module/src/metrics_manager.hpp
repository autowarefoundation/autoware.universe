// Copyright 2025 TIER IV, Inc.
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

#ifndef METRICS_MANAGER_HPP_
#define METRICS_MANAGER_HPP_

#include "type_alias.hpp"
#include "types.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner
{
class MetricsManager
{
public:
  void init() { metrics_.clear(); }

  void calculate_metrics(
    const std::string & module_name, const std::string & reason,
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points,
    const std::optional<geometry_msgs::msg::Pose> & stop_pose,
    const std::optional<StopObstacle> & stop_obstacle)
  {
    // Create status
    {
      // Decision
      Metric decision_metric;
      decision_metric.name = module_name + "/decision";
      decision_metric.unit = "string";
      decision_metric.value = reason;
      metrics_.push_back(decision_metric);
    }

    if (stop_pose.has_value() && planner_data) {  // Stop info
      Metric stop_position_metric;
      stop_position_metric.name = module_name + "/stop_position";
      stop_position_metric.unit = "string";
      const auto & p = stop_pose.value().position;
      stop_position_metric.value =
        "{" + std::to_string(p.x) + ", " + std::to_string(p.y) + ", " + std::to_string(p.z) + "}";
      metrics_.push_back(stop_position_metric);

      Metric stop_orientation_metric;
      stop_orientation_metric.name = module_name + "/stop_orientation";
      stop_orientation_metric.unit = "string";
      const auto & o = stop_pose.value().orientation;
      stop_orientation_metric.value = "{" + std::to_string(o.w) + ", " + std::to_string(o.x) +
                                      ", " + std::to_string(o.y) + ", " + std::to_string(o.z) + "}";
      metrics_.push_back(stop_orientation_metric);

      const auto dist_to_stop_pose = autoware::motion_utils::calcSignedArcLength(
        traj_points, planner_data->current_odometry.pose.pose.position, stop_pose.value().position);

      Metric dist_to_stop_pose_metric;
      dist_to_stop_pose_metric.name = module_name + "/distance_to_stop_pose";
      dist_to_stop_pose_metric.unit = "double";
      dist_to_stop_pose_metric.value = std::to_string(dist_to_stop_pose);
      metrics_.push_back(dist_to_stop_pose_metric);
    }

    if (stop_obstacle.has_value()) {
      // Obstacle info
      Metric collision_point_metric;
      const auto & p = stop_obstacle.value().collision_point;
      collision_point_metric.name = module_name + "/collision_point";
      collision_point_metric.unit = "string";
      collision_point_metric.value =
        "{" + std::to_string(p.x) + ", " + std::to_string(p.y) + ", " + std::to_string(p.z) + "}";
      metrics_.push_back(collision_point_metric);
    }
  }

  MetricArray create_metric_array(const rclcpp::Time & current_time)
  {
    MetricArray metrics_msg;
    metrics_msg.stamp = current_time;
    metrics_msg.metric_array.insert(
      metrics_msg.metric_array.end(), metrics_.begin(), metrics_.end());
    return metrics_msg;
  }

private:
  std::vector<Metric> metrics_;
};
}  // namespace autoware::motion_velocity_planner

#endif  // METRICS_MANAGER_HPP_
