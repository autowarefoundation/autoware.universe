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

#ifndef OBSTACLE_STOP_MODULE_HPP_
#define OBSTACLE_STOP_MODULE_HPP_

#include "autoware/motion_utils/marker/marker_helper.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/motion_velocity_planner_common_universe/polygon_utils.hpp"
#include "autoware/motion_velocity_planner_common_universe/utils.hpp"
#include "autoware/object_recognition_utils/predicted_path_utils.hpp"
#include "autoware/universe_utils/ros/parameter.hpp"
#include "autoware/universe_utils/ros/update_param.hpp"
#include "autoware/universe_utils/system/stop_watch.hpp"
#include "autoware/universe_utils/system/time_keeper.hpp"
#include "metrics_manager.hpp"
#include "parameters.hpp"
#include "stop_planning_debug_info.hpp"
#include "type_alias.hpp"
#include "types.hpp"

#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware/motion_velocity_planner_common_universe/plugin_module_interface.hpp>
#include <autoware/motion_velocity_planner_common_universe/velocity_planning_result.hpp>
#include <autoware/objects_of_interest_marker_interface/objects_of_interest_marker_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner
{
using autoware::universe_utils::getOrDeclareParameter;

namespace
{
template <typename T>
bool is_in_vector(const T variable, const std::vector<T> & vec)
{
  return std::find(vec.begin(), vec.end(), variable) != vec.end();
}

double calc_minimum_distance_to_stop(
  const double initial_vel, const double max_acc, const double min_acc)
{
  if (initial_vel < 0.0) {
    return -std::pow(initial_vel, 2) / 2.0 / max_acc;
  }

  return -std::pow(initial_vel, 2) / 2.0 / min_acc;
}

autoware::universe_utils::Point2d convert_point(const geometry_msgs::msg::Point & p)
{
  return autoware::universe_utils::Point2d{p.x, p.y};
}

std::vector<TrajectoryPoint> resample_trajectory_points(
  const std::vector<TrajectoryPoint> & traj_points, const double interval)
{
  const auto traj = autoware::motion_utils::convertToTrajectory(traj_points);
  const auto resampled_traj = autoware::motion_utils::resampleTrajectory(traj, interval);
  return autoware::motion_utils::convertToTrajectoryPointArray(resampled_traj);
}

std::vector<PredictedPath> resample_highest_confidence_predicted_paths(
  const std::vector<PredictedPath> & predicted_paths, const double time_interval,
  const double time_horizon, const size_t num_paths)
{
  std::vector<PredictedPath> sorted_paths = predicted_paths;

  // Sort paths by descending confidence
  std::sort(
    sorted_paths.begin(), sorted_paths.end(),
    [](const PredictedPath & a, const PredictedPath & b) { return a.confidence > b.confidence; });

  std::vector<PredictedPath> selected_paths;
  size_t path_count = 0;

  // Select paths that meet the confidence thresholds
  for (const auto & path : sorted_paths) {
    if (path_count < num_paths) {
      selected_paths.push_back(path);
      ++path_count;
    }
  }

  // Resample each selected path
  std::vector<PredictedPath> resampled_paths;
  for (const auto & path : selected_paths) {
    if (path.path.size() < 2) {
      continue;
    }
    resampled_paths.push_back(
      autoware::object_recognition_utils::resamplePredictedPath(path, time_interval, time_horizon));
  }

  return resampled_paths;
}

template <class T>
std::vector<T> concat_vectors(
  const std::vector<T> & first_vector, const std::vector<T> & second_vector)
{
  std::vector<T> concatenated_vector;
  concatenated_vector.insert(concatenated_vector.end(), first_vector.begin(), first_vector.end());
  concatenated_vector.insert(concatenated_vector.end(), second_vector.begin(), second_vector.end());
  return concatenated_vector;
}

double calc_dist_to_bumper(const bool is_driving_forward, const VehicleInfo & vehicle_info)
{
  if (is_driving_forward) {
    return std::abs(vehicle_info.max_longitudinal_offset_m);
  }
  return std::abs(vehicle_info.min_longitudinal_offset_m);
}

Float64Stamped create_float64_stamped(const rclcpp::Time & now, const float & data)
{
  Float64Stamped msg;
  msg.stamp = now;
  msg.data = data;
  return msg;
}
}  // namespace

class ObstacleStopModule : public PluginModuleInterface
{
public:
  void init(rclcpp::Node & node, const std::string & module_name) override;
  void update_parameters(const std::vector<rclcpp::Parameter> & parameters) override;
  std::string get_module_name() const override { return module_name_; }

  VelocityPlanningResult plan(
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & ego_trajectory_points,
    const std::shared_ptr<const PlannerData> planner_data) override;

private:
  std::string module_name_;
  rclcpp::Clock::SharedPtr clock_;

  // ros parameters
  bool ignore_crossing_obstacle_;
  bool suppress_sudden_stop_;
  CommonParam common_param_;
  StopPlanningParam stop_planning_param_;
  ObstacleFilteringParam obstacle_filtering_param_;

  // module publisher
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr debug_stop_planning_info_pub_;
  rclcpp::Publisher<MetricArray>::SharedPtr metrics_pub_;
  rclcpp::Publisher<autoware::universe_utils::ProcessingTimeDetail>::SharedPtr
    processing_time_detail_pub_;

  // interface publisher
  std::unique_ptr<autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>
    objects_of_interest_marker_interface_;

  // internal variables
  mutable StopPlanningDebugInfo stop_planning_debug_info_;
  mutable std::shared_ptr<DebugData> debug_data_ptr_;
  std::vector<StopObstacle> prev_closest_stop_obstacles_{};
  std::vector<StopObstacle> prev_stop_obstacles_;
  MetricsManager metrics_manager_;
  // previous trajectory and distance to stop
  // NOTE: Previous trajectory is memorized to deal with nearest index search for overlapping or
  // crossing lanes.
  std::optional<std::pair<std::vector<TrajectoryPoint>, double>> prev_stop_distance_info_{
    std::nullopt};
  autoware::universe_utils::StopWatch<std::chrono::milliseconds> stop_watch_;
  mutable std::unordered_map<double, std::vector<Polygon2d>> trajectory_polygon_for_inside_map_{};
  mutable std::optional<std::vector<Polygon2d>> trajectory_polygon_for_outside_{std::nullopt};
  mutable std::optional<std::vector<Polygon2d>> decimated_traj_polys_{std::nullopt};
  mutable std::shared_ptr<universe_utils::TimeKeeper> time_keeper_;

  std::vector<Polygon2d> get_trajectory_polygon_for_inside(
    const std::vector<TrajectoryPoint> & decimated_traj_points, const VehicleInfo & vehicle_info,
    const geometry_msgs::msg::Pose & current_ego_pose, const double lat_margin,
    const bool enable_to_consider_current_pose, const double time_to_convergence,
    const double decimate_trajectory_step_length) const;

  std::vector<Polygon2d> get_trajectory_polygon_for_outside(
    const std::vector<TrajectoryPoint> & decimated_traj_points, const VehicleInfo & vehicle_info,
    const geometry_msgs::msg::Pose & current_ego_pose, const double lat_margin,
    const bool enable_to_consider_current_pose, const double time_to_convergence,
    const double decimate_trajectory_step_length) const;

  std::vector<StopObstacle> filter_stop_obstacle_for_predicted_object(
    const Odometry & odometry, const double ego_nearest_dist_threshold,
    const double ego_nearest_yaw_threshold, const rclcpp::Time & predicted_objects_stamp,
    const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & decimated_traj_points,
    const std::vector<std::shared_ptr<PlannerData::Object>> & objects,
    const bool is_driving_forward, const VehicleInfo & vehicle_info, const double dist_to_bumper,
    const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check);

  std::optional<geometry_msgs::msg::Point> plan_stop(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<StopObstacle> & stop_obstacles, const double dist_to_bumper);
  double calc_desired_stop_margin(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points, const StopObstacle & stop_obstacle,
    const double dist_to_bumper, const size_t ego_segment_idx,
    const double dist_to_collide_on_ref_traj);
  std::optional<double> calc_candidate_zero_vel_dist(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points, const StopObstacle & stop_obstacle,
    const double dist_to_collide_on_ref_traj, const double desired_stop_margin);
  void hold_previous_stop_if_necessary(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points,
    std::optional<double> & determined_zero_vel_dist);
  std::optional<geometry_msgs::msg::Point> calc_stop_point(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points, const double dist_to_bumper,
    const std::optional<StopObstacle> & determined_stop_obstacle,
    const std::optional<double> & determined_zero_vel_dist);
  void set_stop_planning_debug_info(
    const std::optional<StopObstacle> & determined_stop_obstacle,
    const std::optional<double> & determined_desired_stop_margin) const;
  void publish_debug_info();

  std::optional<StopObstacle> filter_inside_stop_obstacle_for_predicted_object(
    const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & decimated_traj_points,
    const std::shared_ptr<PlannerData::Object> object, const rclcpp::Time & predicted_objects_stamp,
    const double dist_from_obj_poly_to_traj_poly, const VehicleInfo & vehicle_info,
    const double dist_to_bumper,
    const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check) const;
  bool is_inside_stop_obstacle_velocity(
    const std::shared_ptr<PlannerData::Object> object,
    const std::vector<TrajectoryPoint> & traj_points) const;
  bool is_crossing_transient_obstacle(
    const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & decimated_traj_points,
    const std::shared_ptr<PlannerData::Object> object, const double dist_to_bumper,
    const std::vector<Polygon2d> & decimated_traj_polys_with_lat_margin,
    const std::optional<std::pair<geometry_msgs::msg::Point, double>> & collision_point) const;

  std::optional<StopObstacle> filter_outside_stop_obstacle_for_predicted_object(
    const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & decimated_traj_points,
    const rclcpp::Time & predicted_objects_stamp, const std::shared_ptr<PlannerData::Object> object,
    const double dist_from_obj_poly_to_traj_poly, const bool is_driving_forward,
    const VehicleInfo & vehicle_info, const double dist_to_bumper,
    const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check) const;
  std::optional<std::pair<geometry_msgs::msg::Point, double>>
  create_collision_point_for_outside_stop_obstacle(
    const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & decimated_traj_points,
    const std::vector<Polygon2d> & decimated_traj_polys,
    const std::shared_ptr<PlannerData::Object> object, const rclcpp::Time & predicted_objects_stamp,
    const PredictedPath & resampled_predicted_path, double max_lat_margin,
    const bool is_driving_forward, const VehicleInfo & vehicle_info, const double dist_to_bumper,
    const double decimate_trajectory_step_length) const;
  double calc_collision_time_margin(
    const Odometry & odometry, const std::vector<polygon_utils::PointWithStamp> & collision_points,
    const std::vector<TrajectoryPoint> & traj_points, const double dist_to_bumper) const;
  void check_consistency(
    const rclcpp::Time & current_time,
    const std::vector<std::shared_ptr<PlannerData::Object>> & objects,
    std::vector<StopObstacle> & stop_obstacles);
  double calc_margin_from_obstacle_on_curve(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points, const StopObstacle & stop_obstacle,
    const double dist_to_bumper, const double default_stop_margin) const;
  std::vector<StopObstacle> get_closest_stop_obstacles(
    const std::vector<StopObstacle> & stop_obstacles);
  double get_max_lat_margin(const uint8_t obj_label) const;
  std::vector<Polygon2d> get_decimated_traj_polys(
    const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & current_pose,
    const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
    const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold,
    const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check) const;
};
}  // namespace autoware::motion_velocity_planner

#endif  // OBSTACLE_STOP_MODULE_HPP_
