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

#ifndef AUTOWARE__OBSTACLE_CRUISE_PLANNER__PLANNER_INTERFACE_HPP_
#define AUTOWARE__OBSTACLE_CRUISE_PLANNER__PLANNER_INTERFACE_HPP_

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/obstacle_cruise_planner/common_structs.hpp"
#include "autoware/obstacle_cruise_planner/stop_planning_debug_info.hpp"
#include "autoware/obstacle_cruise_planner/type_alias.hpp"
#include "autoware/obstacle_cruise_planner/utils.hpp"
#include "autoware/planning_factor_interface/planning_factor_interface.hpp"
#include "autoware/universe_utils/ros/update_param.hpp"
#include "autoware/universe_utils/system/stop_watch.hpp"

#include <tier4_metric_msgs/msg/metric.hpp>
#include <tier4_metric_msgs/msg/metric_array.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

using Metric = tier4_metric_msgs::msg::Metric;
using MetricArray = tier4_metric_msgs::msg::MetricArray;

class PlannerInterface
{
public:
  virtual ~PlannerInterface() = default;
  PlannerInterface(
    rclcpp::Node & node, const LongitudinalInfo & longitudinal_info,
    const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
    const EgoNearestParam & ego_nearest_param, const std::shared_ptr<DebugData> debug_data_ptr)
  : planning_factor_interface_{std::make_unique<
      autoware::planning_factor_interface::PlanningFactorInterface>(
      &node, "obstacle_cruise_planner")},
    longitudinal_info_(longitudinal_info),
    vehicle_info_(vehicle_info),
    ego_nearest_param_(ego_nearest_param),
    debug_data_ptr_(debug_data_ptr),
    slow_down_param_(SlowDownParam(node)),
    stop_param_(StopParam(node, longitudinal_info))
  {
    stop_speed_exceeded_pub_ =
      node.create_publisher<StopSpeedExceeded>("~/output/stop_speed_exceeded", 1);
    metrics_pub_ = node.create_publisher<MetricArray>("~/metrics", 10);

    moving_object_speed_threshold =
      node.declare_parameter<double>("slow_down.moving_object_speed_threshold");
    moving_object_hysteresis_range =
      node.declare_parameter<double>("slow_down.moving_object_hysteresis_range");
  }

  void setParam(
    const bool enable_debug_info, const bool enable_calculation_time_info,
    const bool use_pointcloud, const double min_behavior_stop_margin,
    const double enable_approaching_on_curve, const double additional_safe_distance_margin_on_curve,
    const double min_safe_distance_margin_on_curve, const bool suppress_sudden_obstacle_stop)
  {
    enable_debug_info_ = enable_debug_info;
    enable_calculation_time_info_ = enable_calculation_time_info;
    use_pointcloud_ = use_pointcloud;
    min_behavior_stop_margin_ = min_behavior_stop_margin;
    enable_approaching_on_curve_ = enable_approaching_on_curve;
    additional_safe_distance_margin_on_curve_ = additional_safe_distance_margin_on_curve;
    min_safe_distance_margin_on_curve_ = min_safe_distance_margin_on_curve;
    suppress_sudden_obstacle_stop_ = suppress_sudden_obstacle_stop;
  }

  std::vector<TrajectoryPoint> generateStopTrajectory(
    const PlannerData & planner_data, const std::vector<StopObstacle> & stop_obstacles);

  virtual std::vector<TrajectoryPoint> generateCruiseTrajectory(
    const PlannerData & planner_data, const std::vector<TrajectoryPoint> & stop_traj_points,
    const std::vector<CruiseObstacle> & cruise_obstacles,
    std::optional<VelocityLimit> & vel_limit) = 0;

  std::vector<TrajectoryPoint> generateSlowDownTrajectory(
    const PlannerData & planner_data, const std::vector<TrajectoryPoint> & stop_traj_points,
    const std::vector<SlowDownObstacle> & slow_down_obstacles,
    std::optional<VelocityLimit> & vel_limit);

  std::vector<Metric> makeMetrics(
    const std::string & module_name, const std::string & reason,
    const std::optional<PlannerData> & planner_data = std::nullopt,
    const std::optional<geometry_msgs::msg::Pose> & stop_pose = std::nullopt,
    const std::optional<StopObstacle> & stop_obstacle = std::nullopt);
  void publishMetrics(const rclcpp::Time & current_time);
  void publishPlanningFactors() { planning_factor_interface_->publish(); }
  void clearMetrics();

  void onParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    updateCommonParam(parameters);
    updateCruiseParam(parameters);
    slow_down_param_.onParam(parameters);
    stop_param_.onParam(parameters, longitudinal_info_);
  }

  Float32MultiArrayStamped getStopPlanningDebugMessage(const rclcpp::Time & current_time) const
  {
    return stop_planning_debug_info_.convertToMessage(current_time);
  }
  virtual Float32MultiArrayStamped getCruisePlanningDebugMessage(
    [[maybe_unused]] const rclcpp::Time & current_time) const
  {
    return Float32MultiArrayStamped{};
  }
  Float32MultiArrayStamped getSlowDownPlanningDebugMessage(const rclcpp::Time & current_time)
  {
    slow_down_debug_multi_array_.stamp = current_time;
    return slow_down_debug_multi_array_;
  }
  double getSafeDistanceMargin() const { return longitudinal_info_.safe_distance_margin; }

protected:
  std::unique_ptr<autoware::planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface_;

  // Parameters
  bool enable_debug_info_{false};
  bool enable_calculation_time_info_{false};
  bool use_pointcloud_{false};
  LongitudinalInfo longitudinal_info_;
  double min_behavior_stop_margin_;
  bool enable_approaching_on_curve_;
  double additional_safe_distance_margin_on_curve_;
  double min_safe_distance_margin_on_curve_;
  bool suppress_sudden_obstacle_stop_;

  // stop watch
  autoware::universe_utils::StopWatch<
    std::chrono::milliseconds, std::chrono::microseconds, std::chrono::steady_clock>
    stop_watch_;

  // Publishers
  rclcpp::Publisher<StopSpeedExceeded>::SharedPtr stop_speed_exceeded_pub_;
  rclcpp::Publisher<MetricArray>::SharedPtr metrics_pub_;

  // Vehicle Parameters
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  EgoNearestParam ego_nearest_param_;

  mutable std::shared_ptr<DebugData> debug_data_ptr_;

  // debug info
  StopPlanningDebugInfo stop_planning_debug_info_;
  Float32MultiArrayStamped slow_down_debug_multi_array_;

  double calcDistanceToCollisionPoint(
    const PlannerData & planner_data, const geometry_msgs::msg::Point & collision_point);

  double calcRSSDistance(
    const double ego_vel, const double obstacle_vel, const double margin = 0.0) const
  {
    const auto & i = longitudinal_info_;
    const double rss_dist_with_margin =
      ego_vel * i.idling_time + std::pow(ego_vel, 2) * 0.5 / std::abs(i.min_ego_accel_for_rss) -
      std::pow(obstacle_vel, 2) * 0.5 / std::abs(i.min_object_accel_for_rss) + margin;
    return rss_dist_with_margin;
  }

  void updateCommonParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    longitudinal_info_.onParam(parameters);
  }

  virtual void updateCruiseParam([[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
  {
  }

  size_t findEgoIndex(
    const std::vector<TrajectoryPoint> & traj_points,
    const geometry_msgs::msg::Pose & ego_pose) const
  {
    const auto & p = ego_nearest_param_;
    return autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
      traj_points, ego_pose, p.dist_threshold, p.yaw_threshold);
  }

  size_t findEgoSegmentIndex(
    const std::vector<TrajectoryPoint> & traj_points,
    const geometry_msgs::msg::Pose & ego_pose) const
  {
    const auto & p = ego_nearest_param_;
    return autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      traj_points, ego_pose, p.dist_threshold, p.yaw_threshold);
  }

private:
  struct SlowDownOutput
  {
    SlowDownOutput() = default;
    SlowDownOutput(
      const std::string & arg_uuid, const std::vector<TrajectoryPoint> & traj_points,
      const std::optional<size_t> & start_idx, const std::optional<size_t> & end_idx,
      const double arg_target_vel, const double arg_feasible_target_vel,
      const double arg_precise_lat_dist, const bool is_moving)
    : uuid(arg_uuid),
      target_vel(arg_target_vel),
      feasible_target_vel(arg_feasible_target_vel),
      precise_lat_dist(arg_precise_lat_dist),
      is_moving(is_moving)
    {
      if (start_idx) {
        start_point = traj_points.at(*start_idx).pose;
      }
      if (end_idx) {
        end_point = traj_points.at(*end_idx).pose;
      }
    }

    std::string uuid;
    double target_vel;
    double feasible_target_vel;
    double precise_lat_dist;
    std::optional<geometry_msgs::msg::Pose> start_point{std::nullopt};
    std::optional<geometry_msgs::msg::Pose> end_point{std::nullopt};
    bool is_moving;
  };
  double calculateMarginFromObstacleOnCurve(
    const PlannerData & planner_data, const StopObstacle & stop_obstacle) const;
  double calculateSlowDownVelocity(
    const SlowDownObstacle & obstacle, const std::optional<SlowDownOutput> & prev_output,
    const bool is_obstacle_moving) const;
  std::optional<std::tuple<double, double, double>> calculateDistanceToSlowDownWithConstraints(
    const PlannerData & planner_data, const std::vector<TrajectoryPoint> & traj_points,
    const SlowDownObstacle & obstacle, const std::optional<SlowDownOutput> & prev_output,
    const double dist_to_ego, const bool is_obstacle_moving) const;

  struct SlowDownInfo
  {
    // NOTE: Acceleration limit is applied to lon_dist not to occur sudden brake.
    const double lon_dist;  // from ego pose to slow down point
    const double vel;       // slow down velocity
  };

  struct SlowDownParam
  {
    std::vector<std::string> obstacle_labels{"default"};
    std::vector<std::string> obstacle_moving_classification{"static", "moving"};
    std::unordered_map<uint8_t, std::string> types_map;
    struct ObstacleSpecificParams
    {
      double max_lat_margin;
      double min_lat_margin;
      double max_ego_velocity;
      double min_ego_velocity;
    };
    explicit SlowDownParam(rclcpp::Node & node)
    {
      obstacle_labels =
        node.declare_parameter<std::vector<std::string>>("slow_down.labels", obstacle_labels);
      // obstacle label dependant parameters
      for (const auto & label : obstacle_labels) {
        for (const auto & movement_postfix : obstacle_moving_classification) {
          ObstacleSpecificParams params;
          params.max_lat_margin = node.declare_parameter<double>(
            "slow_down." + label + "." + movement_postfix + ".max_lat_margin");
          params.min_lat_margin = node.declare_parameter<double>(
            "slow_down." + label + "." + movement_postfix + ".min_lat_margin");
          params.max_ego_velocity = node.declare_parameter<double>(
            "slow_down." + label + "." + movement_postfix + ".max_ego_velocity");
          params.min_ego_velocity = node.declare_parameter<double>(
            "slow_down." + label + "." + movement_postfix + ".min_ego_velocity");
          obstacle_to_param_struct_map.emplace(
            std::make_pair(label + "." + movement_postfix, params));
        }
      }

      // common parameters
      time_margin_on_target_velocity =
        node.declare_parameter<double>("slow_down.time_margin_on_target_velocity");
      lpf_gain_slow_down_vel = node.declare_parameter<double>("slow_down.lpf_gain_slow_down_vel");
      lpf_gain_lat_dist = node.declare_parameter<double>("slow_down.lpf_gain_lat_dist");
      lpf_gain_dist_to_slow_down =
        node.declare_parameter<double>("slow_down.lpf_gain_dist_to_slow_down");

      types_map = {{ObjectClassification::UNKNOWN, "unknown"},
                   {ObjectClassification::CAR, "car"},
                   {ObjectClassification::TRUCK, "truck"},
                   {ObjectClassification::BUS, "bus"},
                   {ObjectClassification::TRAILER, "trailer"},
                   {ObjectClassification::MOTORCYCLE, "motorcycle"},
                   {ObjectClassification::BICYCLE, "bicycle"},
                   {ObjectClassification::PEDESTRIAN, "pedestrian"}};
    }

    ObstacleSpecificParams getObstacleParamByLabel(
      const ObjectClassification & label_id, const bool is_obstacle_moving) const
    {
      const std::string label =
        (types_map.count(label_id.label) > 0) ? types_map.at(label_id.label) : "default";
      const std::string movement_postfix = (is_obstacle_moving) ? "moving" : "static";
      return (obstacle_to_param_struct_map.count(label + "." + movement_postfix) > 0)
               ? obstacle_to_param_struct_map.at(label + "." + movement_postfix)
               : obstacle_to_param_struct_map.at("default." + movement_postfix);
    }

    void onParam(const std::vector<rclcpp::Parameter> & parameters)
    {
      // obstacle type dependant parameters
      for (const auto & label : obstacle_labels) {
        for (const auto & movement_postfix : obstacle_moving_classification) {
          if (obstacle_to_param_struct_map.count(label + "." + movement_postfix) < 1) continue;
          auto & param_by_obstacle_label =
            obstacle_to_param_struct_map.at(label + "." + movement_postfix);
          autoware::universe_utils::updateParam<double>(
            parameters, "slow_down." + label + "." + movement_postfix + ".max_lat_margin",
            param_by_obstacle_label.max_lat_margin);
          autoware::universe_utils::updateParam<double>(
            parameters, "slow_down." + label + "." + movement_postfix + ".min_lat_margin",
            param_by_obstacle_label.min_lat_margin);
          autoware::universe_utils::updateParam<double>(
            parameters, "slow_down." + label + "." + movement_postfix + ".max_ego_velocity",
            param_by_obstacle_label.max_ego_velocity);
          autoware::universe_utils::updateParam<double>(
            parameters, "slow_down." + label + "." + movement_postfix + ".min_ego_velocity",
            param_by_obstacle_label.min_ego_velocity);
        }
      }

      // common parameters
      autoware::universe_utils::updateParam<double>(
        parameters, "slow_down.time_margin_on_target_velocity", time_margin_on_target_velocity);
      autoware::universe_utils::updateParam<double>(
        parameters, "slow_down.lpf_gain_slow_down_vel", lpf_gain_slow_down_vel);
      autoware::universe_utils::updateParam<double>(
        parameters, "slow_down.lpf_gain_lat_dist", lpf_gain_lat_dist);
      autoware::universe_utils::updateParam<double>(
        parameters, "slow_down.lpf_gain_dist_to_slow_down", lpf_gain_dist_to_slow_down);
    }

    std::unordered_map<std::string, ObstacleSpecificParams> obstacle_to_param_struct_map;

    double time_margin_on_target_velocity;
    double lpf_gain_slow_down_vel;
    double lpf_gain_lat_dist;
    double lpf_gain_dist_to_slow_down;
  };
  SlowDownParam slow_down_param_;
  struct StopParam
  {
    struct ObstacleSpecificParams
    {
      double limit_min_acc;
      double sudden_object_acc_threshold;
      double sudden_object_dist_threshold;
      bool abandon_to_stop;
    };
    const std::unordered_map<uint8_t, std::string> types_maps = {
      {ObjectClassification::UNKNOWN, "unknown"}, {ObjectClassification::CAR, "car"},
      {ObjectClassification::TRUCK, "truck"},     {ObjectClassification::BUS, "bus"},
      {ObjectClassification::TRAILER, "trailer"}, {ObjectClassification::MOTORCYCLE, "motorcycle"},
      {ObjectClassification::BICYCLE, "bicycle"}, {ObjectClassification::PEDESTRIAN, "pedestrian"}};
    std::unordered_map<std::string, ObstacleSpecificParams> type_specified_param_list;
    explicit StopParam(rclcpp::Node & node, const LongitudinalInfo & longitudinal_info)
    {
      const std::string param_prefix = "stop.type_specified_params.";
      std::vector<std::string> obstacle_labels{"default"};
      obstacle_labels =
        node.declare_parameter<std::vector<std::string>>(param_prefix + "labels", obstacle_labels);

      for (const auto & type_str : obstacle_labels) {
        if (type_str != "default") {
          ObstacleSpecificParams param{
            node.declare_parameter<double>(param_prefix + type_str + ".limit_min_acc"),
            node.declare_parameter<double>(
              param_prefix + type_str + ".sudden_object_acc_threshold"),
            node.declare_parameter<double>(
              param_prefix + type_str + ".sudden_object_dist_threshold"),
            node.declare_parameter<bool>(param_prefix + type_str + ".abandon_to_stop")};

          param.sudden_object_acc_threshold =
            std::min(param.sudden_object_acc_threshold, longitudinal_info.limit_min_accel);
          param.limit_min_acc = std::min(param.limit_min_acc, param.sudden_object_acc_threshold);

          type_specified_param_list.emplace(type_str, param);
        }
      }
    }
    void onParam(
      const std::vector<rclcpp::Parameter> & parameters, const LongitudinalInfo & longitudinal_info)
    {
      const std::string param_prefix = "stop.type_specified_params.";
      for (auto & [type_str, param] : type_specified_param_list) {
        if (type_str == "default") {
          continue;
        }
        autoware::universe_utils::updateParam<double>(
          parameters, param_prefix + type_str + ".limit_min_acc", param.limit_min_acc);
        autoware::universe_utils::updateParam<double>(
          parameters, param_prefix + type_str + ".sudden_object_acc_threshold",
          param.sudden_object_acc_threshold);
        autoware::universe_utils::updateParam<double>(
          parameters, param_prefix + type_str + ".sudden_object_dist_threshold",
          param.sudden_object_dist_threshold);
        autoware::universe_utils::updateParam<bool>(
          parameters, param_prefix + type_str + ".abandon_to_stop", param.abandon_to_stop);

        param.sudden_object_acc_threshold =
          std::min(param.sudden_object_acc_threshold, longitudinal_info.limit_min_accel);
        param.limit_min_acc = std::min(param.limit_min_acc, param.sudden_object_acc_threshold);
      }
    }
    std::string getParamType(const ObjectClassification label)
    {
      const auto type_str = types_maps.at(label.label);
      if (type_specified_param_list.count(type_str) == 0) {
        return "default";
      }
      return type_str;
    }
    ObstacleSpecificParams getParam(const ObjectClassification label)
    {
      return type_specified_param_list.at(getParamType(label));
    }
  };
  StopParam stop_param_;
  double moving_object_speed_threshold;
  double moving_object_hysteresis_range;
  std::vector<SlowDownOutput> prev_slow_down_output_;
  // previous trajectory and distance to stop
  // NOTE: Previous trajectory is memorized to deal with nearest index search for overlapping or
  // crossing lanes.
  std::optional<std::pair<std::vector<TrajectoryPoint>, double>> prev_stop_distance_info_{
    std::nullopt};
};

#endif  // AUTOWARE__OBSTACLE_CRUISE_PLANNER__PLANNER_INTERFACE_HPP_
