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

#ifndef AUTOWARE__OBSTACLE_CRUISE_PLANNER__NODE_HPP_
#define AUTOWARE__OBSTACLE_CRUISE_PLANNER__NODE_HPP_

#include "autoware/obstacle_cruise_planner/common_structs.hpp"
#include "autoware/obstacle_cruise_planner/obstacle_cruise_module.hpp"
#include "autoware/obstacle_cruise_planner/obstacle_slow_down_module.hpp"
#include "autoware/obstacle_cruise_planner/obstacle_stop_module.hpp"
#include "autoware/obstacle_cruise_planner/optimization_based_planner/optimization_based_planner.hpp"
#include "autoware/obstacle_cruise_planner/pid_based_planner/pid_based_planner.hpp"
#include "autoware/obstacle_cruise_planner/type_alias.hpp"
#include "autoware/signal_processing/lowpass_filter_1d.hpp"
#include "autoware/universe_utils/ros/logger_level_configure.hpp"
#include "autoware/universe_utils/ros/polling_subscriber.hpp"
#include "autoware/universe_utils/system/stop_watch.hpp"

#include <autoware/universe_utils/ros/published_time_publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::motion_planning
{
class ObstacleCruisePlannerNode : public rclcpp::Node
{
public:
  explicit ObstacleCruisePlannerNode(const rclcpp::NodeOptions & node_options);

private:
  // callback functions
  rcl_interfaces::msg::SetParametersResult onParam(
    const std::vector<rclcpp::Parameter> & parameters);
  void onTrajectory(const Trajectory::ConstSharedPtr msg);
  void onSmoothedTrajectory(const Trajectory::ConstSharedPtr msg);

  // main functions
  std::vector<Obstacle> convertToObstacles(
    const Odometry & odometry, const PredictedObjects & objects,
    const std::vector<TrajectoryPoint> & traj_points) const;
  std::vector<Obstacle> convertToObstacles(
    const Odometry & odometry, const PointCloud2 & pointcloud,
    const std::vector<TrajectoryPoint> & traj_points,
    const std_msgs::msg::Header & traj_header) const;
  /*
std::tuple<std::vector<StopObstacle>, std::vector<CruiseObstacle>, std::vector<SlowDownObstacle>>
  determineEgoBehaviorAgainstPointCloudObstacles(
    const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<Obstacle> & obstacles);
  */
  std::vector<TrajectoryPoint> decimateTrajectoryPoints(
    const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points) const;
  std::optional<SlowDownObstacle> createSlowDownObstacleForPointCloud(
    const Obstacle & obstacle, const double precise_lat_dist);
  PlannerData createPlannerData(
    const Odometry & odometry, const AccelWithCovarianceStamped & acc,
    const std::vector<TrajectoryPoint> & traj_points) const;

  void publishVelocityLimit(
    const std::optional<VelocityLimit> & vel_limit, const std::string & module_name);
  void publishDebugMarker() const;
  void publishDebugInfo() const;
  void publishCalculationTime(const double calculation_time) const;

  bool enable_debug_info_;
  bool enable_calculation_time_info_;
  double min_behavior_stop_margin_;
  bool enable_approaching_on_curve_;
  double additional_safe_distance_margin_on_curve_;
  double min_safe_distance_margin_on_curve_;
  bool suppress_sudden_obstacle_stop_;

  // parameter callback result
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  // publisher
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<VelocityLimit>::SharedPtr vel_limit_pub_;
  rclcpp::Publisher<VelocityLimitClearCommand>::SharedPtr clear_vel_limit_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_marker_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_cruise_wall_marker_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_stop_wall_marker_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_slow_down_wall_marker_pub_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr debug_stop_planning_info_pub_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr debug_cruise_planning_info_pub_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr debug_slow_down_planning_info_pub_;
  rclcpp::Publisher<Float64Stamped>::SharedPtr debug_calculation_time_pub_;

  // subscriber
  rclcpp::Subscription<Trajectory>::SharedPtr traj_sub_;
  autoware::universe_utils::InterProcessPollingSubscriber<Odometry> ego_odom_sub_{
    this, "~/input/odometry"};
  autoware::universe_utils::InterProcessPollingSubscriber<PredictedObjects> objects_sub_{
    this, "~/input/objects"};
  autoware::universe_utils::InterProcessPollingSubscriber<PointCloud2> pointcloud_sub_{
    this, "~/input/pointcloud"};
  autoware::universe_utils::InterProcessPollingSubscriber<AccelWithCovarianceStamped> acc_sub_{
    this, "~/input/acceleration"};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

  // Vehicle Parameters
  VehicleInfo vehicle_info_;

  // planning algorithm
  enum class PlanningAlgorithm { OPTIMIZATION_BASE, PID_BASE, INVALID };
  PlanningAlgorithm getPlanningAlgorithmType(const std::string & param) const;
  PlanningAlgorithm planning_algorithm_;

  // stop watch
  mutable autoware::universe_utils::StopWatch<
    std::chrono::milliseconds, std::chrono::microseconds, std::chrono::steady_clock>
    stop_watch_;
  mutable std::shared_ptr<DebugData> debug_data_ptr_{nullptr};

  // planner
  std::unique_ptr<PlannerInterface> planner_ptr_{nullptr};

  // PointCloud-based stop obstacle history
  std::vector<StopObstacle> stop_pc_obstacle_history_;

  // behavior determination parameter
  BehaviorDeterminationParam behavior_determination_param_;

  std::unordered_map<std::string, bool> need_to_clear_vel_limit_{
    {"cruise", false}, {"slow_down", false}};

  EgoNearestParam ego_nearest_param_;

  bool is_driving_forward_{true};
  bool use_pointcloud_{false};

  std::unique_ptr<autoware::universe_utils::LoggerLevelConfigure> logger_configure_;

  std::unique_ptr<autoware::universe_utils::PublishedTimePublisher> published_time_publisher_;

  std::unique_ptr<ObstacleStopModule> obstacle_stop_module_;
  std::unique_ptr<ObstacleSlowDownModule> obstacle_slow_down_module_;
  std::unique_ptr<ObstacleCruiseModule> obstacle_cruise_module_;
};
}  // namespace autoware::motion_planning

#endif  // AUTOWARE__OBSTACLE_CRUISE_PLANNER__NODE_HPP_
