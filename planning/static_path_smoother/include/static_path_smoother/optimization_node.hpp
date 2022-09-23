// Copyright 2020 Tier IV, Inc.
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
#ifndef STATIC_PATH_SMOOTHER__OPTIMIZATION_NODE_HPP_
#define STATIC_PATH_SMOOTHER__OPTIMIZATION_NODE_HPP_

#include "motion_utils/trajectory/trajectory.hpp"
#include "obstacle_avoidance_planner/common_structs.hpp"
#include "obstacle_avoidance_planner/costmap_generator.hpp"
#include "obstacle_avoidance_planner/eb_path_optimizer.hpp"
#include "obstacle_avoidance_planner/mpt_optimizer.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tier4_autoware_utils/system/stop_watch.hpp"

#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "autoware_auto_planning_msgs/msg/path_point.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tier4_debug_msgs/msg/string_stamped.hpp"
#include "tier4_planning_msgs/msg/enable_avoidance.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "boost/optional.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;

class StaticPathSmoother : public rclcpp::Node
{
private:
  rclcpp::Clock logger_ros_clock_;
  int eb_solved_count_;
  bool is_driving_forward_{true};

  bool is_publishing_debug_visualization_marker_;
  bool is_publishing_area_with_objects_;
  bool is_publishing_object_clearance_map_;
  bool is_publishing_clearance_map_;
  bool is_showing_debug_info_;
  bool is_showing_calculation_time_;
  bool is_stopping_if_outside_drivable_area_;
  bool enable_avoidance_;
  bool enable_pre_smoothing_;
  bool skip_optimization_;
  bool reset_prev_optimization_;

  // vehicle circles info for for mpt constraints
  std::string vehicle_circle_method_;
  int vehicle_circle_num_for_calculation_;
  std::vector<double> vehicle_circle_radius_ratios_;

  // params for replan
  double max_path_shape_change_dist_for_replan_;
  double max_ego_moving_dist_for_replan_;
  double max_delta_time_sec_for_replan_;

  // logic
  std::unique_ptr<CostmapGenerator> costmap_generator_ptr_;
  std::unique_ptr<EBPathOptimizer> eb_path_optimizer_ptr_;
  std::unique_ptr<MPTOptimizer> mpt_optimizer_ptr_;

  // params
  TrajectoryParam traj_param_;
  EBParam eb_param_;
  VehicleParam vehicle_param_;
  MPTParam mpt_param_;
  int mpt_visualize_sampling_num_;

  // debug
  mutable DebugData debug_data_;

  geometry_msgs::msg::Pose current_ego_pose_;
  std::unique_ptr<Trajectories> prev_optimal_trajs_ptr_;
  std::unique_ptr<std::vector<autoware_auto_planning_msgs::msg::PathPoint>> prev_path_points_ptr_;

  std::unique_ptr<rclcpp::Time> latest_replanned_time_ptr_;

  // ROS
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr traj_pub_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Path>::SharedPtr path_sub_;

  // functions
  void resetPlanning();
  void resetPrevOptimization();

  // void insertZeroVelocityOutsideDrivableArea(
  //   std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & traj_points,
  //   const CVMaps & cv_maps);

public:
  explicit StaticPathSmoother(const rclcpp::NodeOptions & node_options);

  // subscriber callback functions
  std::vector<TrajectoryPoint> pathCallback(
    const autoware_auto_planning_msgs::msg::Path::SharedPtr);
};

#endif  // STATIC_PATH_SMOOTHER__OPTIMIZATION_NODE_HPP_
