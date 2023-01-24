// Copyright 2023 TIER IV, Inc.
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
#ifndef COLLISION_FREE_PATH_PLANNER__NODE_HPP_
#define COLLISION_FREE_PATH_PLANNER__NODE_HPP_

#include "collision_free_path_planner/common_structs.hpp"
#include "collision_free_path_planner/eb_path_smoother.hpp"
#include "collision_free_path_planner/mpt_optimizer.hpp"
#include "collision_free_path_planner/replan_checker.hpp"
#include "collision_free_path_planner/type_alias.hpp"
#include "motion_utils/motion_utils.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace collision_free_path_planner
{
class CollisionFreePathPlanner : public rclcpp::Node
{
  FRIEND_TEST(CollisionFreePathPlanner, MPTOptimizer);

public:
  explicit CollisionFreePathPlanner(const rclcpp::NodeOptions & node_options);

private:
  // TODO(murooka) move this node to common
  class DrivingDirectionChecker
  {
  public:
    bool isDrivingForward(const std::vector<PathPoint> & path_points)
    {
      const auto is_driving_forward = motion_utils::isDrivingForward(path_points);
      is_driving_forward_ = is_driving_forward ? is_driving_forward.get() : is_driving_forward_;
      return is_driving_forward_;
    }

  private:
    bool is_driving_forward_{true};
  };
  DrivingDirectionChecker driving_direction_checker_{};

  // argument variables
  vehicle_info_util::VehicleInfo vehicle_info_{};
  mutable std::shared_ptr<DebugData> debug_data_ptr_{nullptr};

  // flags for some functions
  bool enable_pub_debug_marker_;
  bool enable_debug_info_;
  bool enable_calculation_time_info_;
  bool enable_outside_drivable_area_stop_;
  bool enable_smoothing_;
  bool enable_skip_optimization_;
  bool enable_reset_prev_optimization_;

  // core algorithms
  std::shared_ptr<ReplanChecker> replan_checker_ptr_{nullptr};
  std::shared_ptr<EBPathSmoother> eb_path_smoother_ptr_{nullptr};
  std::shared_ptr<MPTOptimizer> mpt_optimizer_ptr_{nullptr};

  // parameters
  TrajectoryParam traj_param_{};
  EgoNearestParam ego_nearest_param_{};

  // variables for subscribers
  Odometry::SharedPtr ego_state_ptr_;

  // variables for previous information
  std::shared_ptr<std::vector<TrajectoryPoint>> prev_optimized_traj_points_ptr_;

  // interface publisher
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr virtual_wall_pub_;

  // interface subscriber
  rclcpp::Subscription<Path>::SharedPtr path_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;

  // debug publisher
  rclcpp::Publisher<Trajectory>::SharedPtr debug_extended_traj_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_markers_pub_;
  rclcpp::Publisher<StringStamped>::SharedPtr debug_calculation_time_pub_;

  // parameter callback
  rcl_interfaces::msg::SetParametersResult onParam(
    const std::vector<rclcpp::Parameter> & parameters);
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  // subscriber callback function
  void onPath(const Path::SharedPtr);

  // reset functions
  void initializePlanning();
  void resetPrevData();

  // main functions
  bool isDataReady(const Path & path, rclcpp::Clock clock) const;
  PlannerData createPlannerData(const Path & path) const;
  std::vector<TrajectoryPoint> generateOptimizedTrajectory(const PlannerData & planner_data);
  std::vector<TrajectoryPoint> extendTrajectory(
    const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & optimized_points);
  void publishDebugData(const Header & header) const;

  // functions in generateOptimizedTrajectory
  std::vector<TrajectoryPoint> optimizeTrajectory(const PlannerData & planner_data);
  std::vector<TrajectoryPoint> getPrevOptimizedTrajectory(
    const std::vector<TrajectoryPoint> & traj_points) const;
  // void applyInputVelocity(
  //   std::vector<TrajectoryPoint> & traj_points, const std::vector<TrajectoryPoint> & traj_points)
  //   const;
  void insertZeroVelocityOutsideDrivableArea(
    const PlannerData & planner_data, std::vector<TrajectoryPoint> & traj_points);
  void publishDebugMarkerInOptimization(
    const PlannerData & planner_data, const std::vector<TrajectoryPoint> & traj_points);
};
}  // namespace collision_free_path_planner

#endif  // COLLISION_FREE_PATH_PLANNER__NODE_HPP_
