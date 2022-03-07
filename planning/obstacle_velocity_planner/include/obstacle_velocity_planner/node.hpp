// Copyright 2022 Tier IV, Inc.
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

#ifndef OBSTACLE_VELOCITY_PLANNER__NODE_HPP_
#define OBSTACLE_VELOCITY_PLANNER__NODE_HPP_

#include "obstacle_velocity_planner/box2d.hpp"
#include "obstacle_velocity_planner/common/s_boundary.hpp"
#include "obstacle_velocity_planner/common/st_point.hpp"
#include "obstacle_velocity_planner/velocity_optimizer.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/self_pose_listener.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>

#include <boost/optional.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <memory>
#include <mutex>
#include <tuple>
#include <vector>

struct TrajectoryData
{
  TrajectoryData() {}

  autoware_auto_planning_msgs::msg::Trajectory traj;
  std::vector<double> s;
};

struct ObjectData
{
  geometry_msgs::msg::Pose pose;
  double length;
  double width;
  double time;
};

class ObstacleVelocityPlanner : public rclcpp::Node
{
public:
  explicit ObstacleVelocityPlanner(const rclcpp::NodeOptions & node_options);

private:
  // Callback Functions
  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg);

  void objectsCallback(const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr msg);

  void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  void trajectoryCallback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg);

  void smoothedTrajectoryCallback(
    const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg);

  void onExternalVelocityLimit(const tier4_planning_msgs::msg::VelocityLimit::ConstSharedPtr msg);

  // Member Functions
  double getClosestStopDistance(
    const TrajectoryData & ego_traj_data, const std::vector<double> & resolutions);

  std::tuple<double, double> calcInitialMotion(
    const autoware_auto_planning_msgs::msg::Trajectory & input_traj, const size_t input_closest,
    const autoware_auto_planning_msgs::msg::Trajectory & prev_traj, const double closest_stop_dist);

  autoware_auto_planning_msgs::msg::TrajectoryPoint calcInterpolatedTrajectoryPoint(
    const autoware_auto_planning_msgs::msg::Trajectory & trajectory,
    const geometry_msgs::msg::Pose & target_pose);

  bool checkHasReachedGoal(
    const autoware_auto_planning_msgs::msg::Trajectory & traj, const size_t closest_idx,
    const double v0);

  TrajectoryData getTrajectoryData(
    const autoware_auto_planning_msgs::msg::Trajectory & traj, const size_t closest_idx);

  TrajectoryData resampleTrajectoryData(
    const TrajectoryData & base_traj_data, const double resampling_s_interval,
    const double max_traj_length, const double stop_dist);

  autoware_auto_planning_msgs::msg::Trajectory resampleTrajectory(
    const std::vector<double> & base_index,
    const autoware_auto_planning_msgs::msg::Trajectory & base_trajectory,
    const std::vector<double> & query_index, const bool use_spline_for_pose = false);

  boost::optional<SBoundaries> getSBoundaries(
    const geometry_msgs::msg::Pose & current_pose, const TrajectoryData & ego_traj_data,
    const std::vector<double> & time_vec);

  boost::optional<SBoundaries> getSBoundaries(
    const TrajectoryData & ego_traj_data,
    const autoware_auto_perception_msgs::msg::PredictedObject & object,
    const rclcpp::Time & obj_base_time, const std::vector<double> & time_vec);

  boost::optional<SBoundaries> getSBoundaries(
    const TrajectoryData & ego_traj_data, const std::vector<double> & time_vec,
    const double safety_distance,
    const autoware_auto_perception_msgs::msg::PredictedObject & object,
    const double dist_to_collision_point);

  boost::optional<SBoundaries> getSBoundaries(
    const TrajectoryData & ego_traj_data, const std::vector<double> & time_vec,
    const double safety_distance,
    const autoware_auto_perception_msgs::msg::PredictedObject & object,
    const rclcpp::Time & obj_base_time,
    const autoware_auto_perception_msgs::msg::PredictedPath & predicted_path);

  bool checkOnMapObject(
    const autoware_auto_perception_msgs::msg::PredictedObject & object,
    const lanelet::ConstLanelets & valid_lanelets);

  lanelet::ConstLanelets getSurroundingLanelets(const geometry_msgs::msg::Pose & current_pose);

  void addValidLanelet(
    const lanelet::routing::LaneletPaths & candidate_paths,
    lanelet::ConstLanelets & valid_lanelets);

  bool checkIsFrontObject(
    const autoware_auto_perception_msgs::msg::PredictedObject & object,
    const autoware_auto_planning_msgs::msg::Trajectory & traj);

  boost::optional<autoware_auto_perception_msgs::msg::PredictedPath> resampledPredictedPath(
    const autoware_auto_perception_msgs::msg::PredictedObject & object,
    const rclcpp::Time & obj_base_time, const rclcpp::Time & current_time,
    const std::vector<double> & resolutions, const double horizon);

  boost::optional<geometry_msgs::msg::Pose> getCurrentObjectPoseFromPredictedPath(
    const autoware_auto_perception_msgs::msg::PredictedPath & predicted_path,
    const rclcpp::Time & obj_base_time, const rclcpp::Time & current_time);

  boost::optional<double> getDistanceToCollisionPoint(
    const TrajectoryData & ego_traj_data, const ObjectData & obj_data,
    const double delta_yaw_threshold);

  boost::optional<size_t> getCollisionIdx(
    const TrajectoryData & ego_traj, const Box2d & obj_box, const size_t start_idx,
    const size_t end_idx);

  double getObjectLongitudinalPosition(
    const TrajectoryData & traj_data, const geometry_msgs::msg::Pose & obj_pose);

  geometry_msgs::msg::Pose transformBaseLink2Center(
    const geometry_msgs::msg::Pose & pose_base_link);

  boost::optional<VelocityOptimizer::OptimizationResult> processOptimizedResult(
    const double v0, const VelocityOptimizer::OptimizationResult & opt_result);

  void publishDebugTrajectory(
    const autoware_auto_planning_msgs::msg::Trajectory & traj, const size_t closest_idx,
    const std::vector<double> & time_vec, const SBoundaries & s_boundaries,
    const VelocityOptimizer::OptimizationResult & opt_result);

  // ROS related members
  // Subscriber
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr
    smoothed_trajectory_sub_;
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr
    objects_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<tier4_planning_msgs::msg::VelocityLimit>::SharedPtr
    sub_external_velocity_limit_;  //!< @brief subscriber for external velocity limit

  // Publisher
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr boundary_pub_;
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr optimized_sv_pub_;
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr
    optimized_st_graph_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr distance_to_closest_obj_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr debug_calculation_time_;

  // Calculation time watcher
  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch_;

  // Self Pose Listener
  tier4_autoware_utils::SelfPoseListener self_pose_listener_;

  // Lanelet Map Pointers
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr_;
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr_;

  std::mutex mutex_;

  // Data for callback functions
  autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr in_objects_ptr_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose_ptr_;  // current vehicle pose

  // Velocity and Acceleration data
  geometry_msgs::msg::TwistStamped::SharedPtr current_twist_ptr_;
  geometry_msgs::msg::TwistStamped::SharedPtr previous_twist_ptr_;
  double previous_acc_;

  autoware_auto_planning_msgs::msg::Trajectory prev_output_;  // previously published trajectory
  autoware_auto_planning_msgs::msg::Trajectory::SharedPtr smoothed_trajectory_ptr_;
  // previously published trajectory

  // Velocity Optimizer
  std::shared_ptr<VelocityOptimizer> velocity_optimizer_ptr_;

  // Vehicle Parameters
  double wheel_base_;
  double front_overhang_;
  double rear_overhang_;
  double left_overhang_;
  double right_overhang_;
  double vehicle_length_;
  double vehicle_width_;

  // Parameters
  double max_accel_;
  double min_accel_;
  double max_jerk_;
  double min_jerk_;
  double min_object_accel_;

  // Resampling Parameter
  double resampling_s_interval_;
  double max_trajectory_length_;
  double dense_resampling_time_interval_;
  double sparse_resampling_time_interval_;
  double dense_time_horizon_;
  double max_time_horizon_;

  double delta_yaw_threshold_of_nearest_index_;
  double delta_yaw_threshold_of_object_and_ego_;
  double object_zero_velocity_threshold_;
  double object_low_velocity_threshold_;
  double external_velocity_limit_;
  double collision_time_threshold_;
  double safe_distance_margin_;
  double t_dangerous_;
  double t_idling_;
  double initial_velocity_margin_;
  bool enable_adaptive_cruise_;
  bool use_object_acceleration_;
  bool use_hd_map_;

  double replan_vel_deviation_;
  double engage_velocity_;
  double engage_acceleration_;
  double engage_exit_ratio_;
  double stop_dist_to_prohibit_engage_;
};

#endif  // OBSTACLE_VELOCITY_PLANNER__NODE_HPP_
