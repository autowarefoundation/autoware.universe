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
#ifndef COLLISION_FREE_PATH_PLANNER__NODE_HPP_
#define COLLISION_FREE_PATH_PLANNER__NODE_HPP_

#include "collision_free_path_planner/common_structs.hpp"
#include "collision_free_path_planner/costmap_generator.hpp"
#include "collision_free_path_planner/eb_path_optimizer.hpp"
#include "collision_free_path_planner/mpt_optimizer.hpp"
#include "collision_free_path_planner/replan_checker.hpp"
#include "collision_free_path_planner/type_alias.hpp"
#include "motion_utils/motion_utils.hpp"
#include "opencv2/core.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include "visualization_msgs/msg/marker_array.hpp"

#include "boost/optional.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace collision_free_path_planner
{
namespace
{
template <typename T>
boost::optional<geometry_msgs::msg::Pose> lerpPose(
  const T & points, const geometry_msgs::msg::Point & target_pos, const size_t closest_seg_idx)
{
  constexpr double epsilon = 1e-6;

  const double closest_to_target_dist =
    motion_utils::calcSignedArcLength(points, closest_seg_idx, target_pos);
  const double seg_dist =
    motion_utils::calcSignedArcLength(points, closest_seg_idx, closest_seg_idx + 1);

  const auto & closest_pose = points[closest_seg_idx].pose;
  const auto & next_pose = points[closest_seg_idx + 1].pose;

  geometry_msgs::msg::Pose interpolated_pose;
  if (std::abs(seg_dist) < epsilon) {
    interpolated_pose.position.x = next_pose.position.x;
    interpolated_pose.position.y = next_pose.position.y;
    interpolated_pose.position.z = next_pose.position.z;
    interpolated_pose.orientation = next_pose.orientation;
  } else {
    const double ratio = closest_to_target_dist / seg_dist;
    if (ratio < 0 || 1 < ratio) {
      return {};
    }

    interpolated_pose.position.x =
      interpolation::lerp(closest_pose.position.x, next_pose.position.x, ratio);
    interpolated_pose.position.y =
      interpolation::lerp(closest_pose.position.y, next_pose.position.y, ratio);
    interpolated_pose.position.z =
      interpolation::lerp(closest_pose.position.z, next_pose.position.z, ratio);

    const double closest_yaw = tf2::getYaw(closest_pose.orientation);
    const double next_yaw = tf2::getYaw(next_pose.orientation);
    const double interpolated_yaw = interpolation::lerp(closest_yaw, next_yaw, ratio);
    interpolated_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(interpolated_yaw);
  }
  return interpolated_pose;
}

template <typename T>
double lerpTwistX(
  const T & points, const geometry_msgs::msg::Point & target_pos, const size_t closest_seg_idx)
{
  if (points.size() == 1) {
    return points.at(0).longitudinal_velocity_mps;
  }

  constexpr double epsilon = 1e-6;

  const double closest_to_target_dist =
    motion_utils::calcSignedArcLength(points, closest_seg_idx, target_pos);
  const double seg_dist =
    motion_utils::calcSignedArcLength(points, closest_seg_idx, closest_seg_idx + 1);

  const double closest_vel = points[closest_seg_idx].longitudinal_velocity_mps;
  const double next_vel = points[closest_seg_idx + 1].longitudinal_velocity_mps;

  if (std::abs(seg_dist) < epsilon) {
    return next_vel;
  }

  const double ratio = std::min(1.0, std::max(0.0, closest_to_target_dist / seg_dist));
  return interpolation::lerp(closest_vel, next_vel, ratio);
}

template <typename T>
double lerpPoseZ(
  const T & points, const geometry_msgs::msg::Point & target_pos, const size_t closest_seg_idx)
{
  if (points.size() == 1) {
    return points.at(0).pose.position.z;
  }

  constexpr double epsilon = 1e-6;

  const double closest_to_target_dist =
    motion_utils::calcSignedArcLength(points, closest_seg_idx, target_pos);
  const double seg_dist =
    motion_utils::calcSignedArcLength(points, closest_seg_idx, closest_seg_idx + 1);

  const double closest_z = points[closest_seg_idx].pose.position.z;
  const double next_z = points[closest_seg_idx + 1].pose.position.z;

  return std::abs(seg_dist) < epsilon
           ? next_z
           : interpolation::lerp(closest_z, next_z, closest_to_target_dist / seg_dist);
}
}  // namespace

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
  mutable std::shared_ptr<DebugData> debug_data_ptr_;
  vehicle_info_util::VehicleInfo vehicle_info_{};
  int eb_solved_count_;

  // flags for some functions
  bool enable_pub_debug_marker_;
  bool enable_debug_info_;
  bool enable_calculation_time_info_;
  bool enable_outside_drivable_area_stop_;
  bool enable_avoidance_;
  bool enable_smoothing_;
  bool enable_skip_optimization_;
  bool enable_reset_prev_optimization_;

  // core algorithm
  std::shared_ptr<CostmapGenerator> costmap_generator_ptr_;
  std::shared_ptr<EBPathOptimizer> eb_path_optimizer_ptr_;
  std::shared_ptr<MPTOptimizer> mpt_optimizer_ptr_;
  std::shared_ptr<ReplanChecker> replan_checker_ptr_;

  // parameters
  TrajectoryParam traj_param_{};
  EgoNearestParam ego_nearest_param_{};

  // variables for debugging
  mutable tier4_autoware_utils::StopWatch<
    std::chrono::milliseconds, std::chrono::microseconds, std::chrono::steady_clock>
    stop_watch_;

  // variables for subscribers
  Odometry::SharedPtr ego_state_ptr_;
  PredictedObjects::SharedPtr objects_ptr_;

  // variables for previous information
  std::shared_ptr<std::vector<TrajectoryPoint>> prev_mpt_traj_ptr_;
  std::shared_ptr<std::vector<TrajectoryPoint>> prev_eb_traj_ptr_;

  // publisher
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr debug_extended_fixed_traj_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr debug_extended_non_fixed_traj_pub_;

  rclcpp::Publisher<MarkerArray>::SharedPtr debug_markers_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_wall_markers_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::StringStamped>::SharedPtr debug_calculation_time_pub_;

  // subscriber
  rclcpp::Subscription<Path>::SharedPtr path_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr objects_sub_;

  // parameter callback functions
  rcl_interfaces::msg::SetParametersResult onParam(
    const std::vector<rclcpp::Parameter> & parameters);
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  // subscriber callback functions
  void onPath(const Path::SharedPtr);

  // reset functions
  void resetPlanning();
  void resetPrevOptimization();

  // main functions
  bool isDataReady();
  PlannerData createPlannerData(const Path & path);
  std::vector<TrajectoryPoint> generateOptimizedTrajectory(const PlannerData & planner_data);
  std::vector<TrajectoryPoint> extendTrajectory(
    const std::vector<PathPoint> & path_points,
    const std::vector<TrajectoryPoint> & optimized_points);
  std::vector<TrajectoryPoint> generatePostProcessedTrajectory(
    const PlannerData & planner_data, const std::vector<TrajectoryPoint> & merged_optimized_points);
  void publishDebugDataInMain(const Path & path) const;

  // functions for optimization
  std::vector<TrajectoryPoint> optimizeTrajectory(
    const PlannerData & planner_data, const CVMaps & cv_maps);
  std::vector<TrajectoryPoint> getPrevOptimizedTrajectory(
    const std::vector<PathPoint> & path_points) const;
  void calcVelocity(
    const std::vector<PathPoint> & path_points, std::vector<TrajectoryPoint> & traj_points) const;
  void insertZeroVelocityOutsideDrivableArea(
    const PlannerData & planner_data, std::vector<TrajectoryPoint> & traj_points,
    const CVMaps & cv_maps);
  void publishDebugMarkerInOptimization(
    const PlannerData & planner_data, const std::vector<TrajectoryPoint> & traj_points);

  // util functions
  void logInfo(const int duration_sec, const char * msg);
  void logWarnThrottle(const int duration_ms, const char * msg);
};
}  // namespace collision_free_path_planner

#endif  // COLLISION_FREE_PATH_PLANNER__NODE_HPP_
