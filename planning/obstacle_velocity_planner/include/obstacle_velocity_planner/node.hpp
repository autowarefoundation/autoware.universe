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

#ifndef OBSTACLE_VELOCITY_PLANNER__NODE_HPP_
#define OBSTACLE_VELOCITY_PLANNER__NODE_HPP_

#include "obstacle_velocity_planner/common_structs.hpp"
#include "obstacle_velocity_planner/optimization_based_planner/optimization_based_planner.hpp"
#include "obstacle_velocity_planner/pid_based_planner/pid_based_planner.hpp"
#include "signal_processing/lowpass_filter_1d.hpp"
#include "tier4_autoware_utils/system/stop_watch.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_perception_msgs/msg/predicted_object.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tier4_debug_msgs/msg/float32_stamped.hpp"
#include "tier4_planning_msgs/msg/velocity_limit.hpp"
#include "tier4_planning_msgs/msg/velocity_limit_clear_command.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <boost/optional.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::PredictedPath;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::Odometry;
using tier4_debug_msgs::msg::Float32Stamped;
using tier4_planning_msgs::msg::StopReasonArray;
using tier4_planning_msgs::msg::VelocityLimit;
using tier4_planning_msgs::msg::VelocityLimitClearCommand;
using vehicle_info_util::VehicleInfo;

namespace motion_planning
{
class ObstacleVelocityPlannerNode : public rclcpp::Node
{
public:
  explicit ObstacleVelocityPlannerNode(const rclcpp::NodeOptions & node_options);

private:
  // callback functions
  rcl_interfaces::msg::SetParametersResult onParam(
    const std::vector<rclcpp::Parameter> & parameters);
  void onObjects(const PredictedObjects::SharedPtr msg);
  void onOdometry(const Odometry::SharedPtr);
  void onTrajectory(const Trajectory::SharedPtr msg);
  void onSmoothedTrajectory(const Trajectory::SharedPtr msg);

  // member Functions
  ObstacleVelocityPlannerData createPlannerData(
    const Trajectory & trajectory, const geometry_msgs::msg::Pose & current_pose, DebugData & debug_data);
  double calcCurrentAccel() const;
  std::vector<TargetObstacle> filterObstacles(
    const PredictedObjects & predicted_objects, const Trajectory & traj,
    const geometry_msgs::msg::Pose & current_pose, const double current_vel,
    DebugData & debug_data);
  geometry_msgs::msg::Point calcNearestCollisionPoint(
    const size_t & first_within_idx,
    const std::vector<geometry_msgs::msg::Point> & collision_points,
    const Trajectory & decimated_traj);
  double calcCollisionTimeMargin(
    const geometry_msgs::msg::Pose & current_pose, const double current_vel,
    const geometry_msgs::msg::Point & nearest_collision_point,
    const PredictedObject & predicted_object, const size_t first_within_idx,
    const Trajectory & decimated_traj,
    const std::vector<tier4_autoware_utils::Polygon2d> & decimated_traj_polygons);
  void publishVelocityLimit(const boost::optional<VelocityLimit> & vel_limit);
  void publishDebugData(const DebugData & debug_data) const;
  void publishCalculationTime(const double calculation_time) const;

  bool is_showing_debug_info_;
  double min_behavior_stop_margin_;
  double max_nearest_dist_deviation_;
  double max_nearest_yaw_deviation_;

  // parameter callback result
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  // publisher
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<VelocityLimit>::SharedPtr vel_limit_pub_;
  rclcpp::Publisher<VelocityLimitClearCommand>::SharedPtr clear_vel_limit_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_cruise_wall_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_stop_wall_marker_pub_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr debug_calculation_time_pub_;

  // subscriber
  rclcpp::Subscription<Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<Trajectory>::SharedPtr smoothed_trajectory_sub_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr objects_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;

  // self pose listener
  tier4_autoware_utils::SelfPoseListener self_pose_listener_;

  // data for callback functions
  PredictedObjects::SharedPtr in_objects_ptr_;
  geometry_msgs::msg::TwistStamped::SharedPtr current_twist_ptr_;
  geometry_msgs::msg::TwistStamped::SharedPtr prev_twist_ptr_;

  // low pass filter of acceleration
  std::shared_ptr<LowpassFilter1d> lpf_acc_ptr_;

  // Vehicle Parameters
  VehicleInfo vehicle_info_;

  // planning algorithm
  enum class PlanningAlgorithm { OPTIMIZATION_BASE, PID_BASE, INVALID };
  PlanningAlgorithm getPlanningAlgorithmType(const std::string & param) const;
  PlanningAlgorithm planning_algorithm_;

  // stop watch
  mutable tier4_autoware_utils::StopWatch<
    std::chrono::milliseconds, std::chrono::microseconds, std::chrono::steady_clock>
    stop_watch_;

  // planner
  std::unique_ptr<PlannerInterface> planner_ptr_;

  // obstacle filtering parameter
  struct ObstacleFilteringParam
  {
    double rough_detection_area_expand_width;
    double detection_area_expand_width;
    double decimate_trajectory_step_length;
    double min_obstacle_crossing_velocity;
    double margin_for_collision_time;
    double max_ego_obj_overlap_time;
    double max_prediction_time_for_collision_check;
    double crossing_obstacle_traj_angle_threshold;
  };
  ObstacleFilteringParam obstacle_filtering_param_;

  bool need_to_clear_vel_limit_{false};
};
}  // namespace motion_planning

#endif  // OBSTACLE_VELOCITY_PLANNER__NODE_HPP_
