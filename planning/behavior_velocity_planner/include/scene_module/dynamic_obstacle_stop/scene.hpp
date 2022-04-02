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

#ifndef SCENE_MODULE__DYNAMIC_OBSTACLE_STOP__SCENE_HPP_
#define SCENE_MODULE__DYNAMIC_OBSTACLE_STOP__SCENE_HPP_

#include "scene_module/dynamic_obstacle_stop/debug.hpp"
#include "scene_module/dynamic_obstacle_stop/dynamic_obstacle.hpp"
#include "scene_module/dynamic_obstacle_stop/utils.hpp"

#include <motion_velocity_smoother/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.hpp>

#include <boost/optional.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <lanelet2_extension/regulatory_elements/detection_area.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scene_module/scene_module_interface.hpp>
// #include <utilization/boost_geometry_helper.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <tf2/LinearMath/Transform.h>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using dynamic_obstacle_stop_utils::DetectionAreaSize;
using dynamic_obstacle_stop_utils::PlannerParam;
using dynamic_obstacle_stop_utils::PoseWithRange;
using dynamic_obstacle_stop_utils::PredictedPath;
using dynamic_obstacle_stop_utils::State;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using tier4_debug_msgs::msg::Float32Stamped;

class DynamicObstacleStopModule : public SceneModuleInterface
{
public:
  DynamicObstacleStopModule(
    const int64_t module_id, const PlannerParam & planner_param, const rclcpp::Logger logger,
    const rclcpp::Clock::SharedPtr clock);

  DynamicObstacleStopModule(
    const int64_t module_id, const PlannerParam & planner_param, const rclcpp::Logger logger,
    const std::shared_ptr<motion_velocity_smoother::SmootherBase> & smoother,
    const std::shared_ptr<DynamicObstacleStopDebug> & debug_ptr,
    const rclcpp::Clock::SharedPtr clock);

  bool modifyPathVelocity(
    autoware_auto_planning_msgs::msg::PathWithLaneId * path,
    tier4_planning_msgs::msg::StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;

private:
  // Parameter
  PlannerParam planner_param_;

  // Variable
  State state_{State::GO};
  rclcpp::Time stop_time_;
  std::shared_ptr<motion_velocity_smoother::SmootherBase> smoother_;
  double velocity_limit_mps_{15.0 / 3.6};  // todo
  std::shared_ptr<DynamicObstacleStopDebug> debug_ptr_;

  // Function
  std::vector<DynamicObstacle> createDynamicObstaclesFromObjects(
    const PredictedObjects & predicted_objects) const;

  std::vector<DynamicObstacle> createDynamicObstaclesFromObjects(
    const PredictedObjects & predicted_objects, const Trajectory & trajectory) const;

  std::vector<DynamicObstacle> createDynamicObstaclesFromPoints(
    const pcl::PointCloud<pcl::PointXYZ> & obstacle_pointcloud,
    const Trajectory & trajectory) const;

  pcl::PointCloud<pcl::PointXYZ> extractObstaclePointsWithRectangle(
    const pcl::PointCloud<pcl::PointXYZ> & input_points,
    const geometry_msgs::msg::Pose & current_pose) const;

  std::vector<geometry_msgs::msg::Point> createDetectionAreaPolygon(
    const geometry_msgs::msg::Pose & current_pose,
    const DetectionAreaSize detection_area_size) const;

  void visualizePassingArea(
    const Trajectory & trajectory, const geometry_msgs::msg::Pose & current_pose) const;

  std::vector<float> calcPassingDistanceForEachPoint(
    const Trajectory & trajectory, const float obstacle_vel_mps, const float dist_max) const;

  boost::optional<size_t> calcDecelerationLineIndex(
    const Trajectory & trajectory, const geometry_msgs::msg::Pose & current_pose) const;

  std::vector<std::vector<geometry_msgs::msg::Point>> calcPassingLines(
    const Trajectory & trajectory, const std::vector<float> & lateral_passing_dist) const;

  boost::optional<std::vector<geometry_msgs::msg::Point>> createDetectionAreaPolygon(
    const std::vector<std::vector<geometry_msgs::msg::Point>> & passing_lines,
    const size_t deceleration_line_idx) const;

  pcl::PointCloud<pcl::PointXYZ> pointsWithinPolygon(
    const std::vector<geometry_msgs::msg::Point> & polygon,
    const pcl::PointCloud<pcl::PointXYZ> & candidate_points) const;

  boost::optional<DynamicObstacle> detectCollision(
    const std::vector<DynamicObstacle> & dynamic_obstacles, const Trajectory & trajectory) const;

  float calcCollisionPositionOfVehicleSide(
    const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Pose & base_pose) const;

  geometry_msgs::msg::Point selectCollisionPoint(
    const DynamicObstacle & dynamic_obstacle, const geometry_msgs::msg::Pose & base_pose,
    const Trajectory & trajectory, const geometry_msgs::msg::Pose & current_pose) const;

  std::vector<geometry_msgs::msg::Point> createVehiclePolygon(
    const geometry_msgs::msg::Pose & base_pose) const;

  std::vector<DynamicObstacle> checkCollisionWithObstacles(
    const std::vector<DynamicObstacle> & dynamic_obstacles,
    std::vector<geometry_msgs::msg::Point> poly, const float travel_time) const;

  boost::optional<DynamicObstacle> findNearestCollisionObstacle(
    const Trajectory & trajectory, const geometry_msgs::msg::Pose & base_pose,
    std::vector<DynamicObstacle> & dynamic_obstacles) const;

  DynamicObstacle findLongitudinalNearestObstacle(
    const std::vector<DynamicObstacle> & dynamic_obstacles, const Trajectory & trajectory,
    const geometry_msgs::msg::Pose & current_pose) const;

  boost::optional<geometry_msgs::msg::Pose> calcPredictedObstaclePose(
    const std::vector<PredictedPath> & predicted_paths, const float travel_time,
    const float velocity_mps) const;

  bool checkCollisionWithShape(
    const tier4_autoware_utils::Polygon2d & vehicle_polygon, const PoseWithRange pose_with_range,
    const Shape & shape, std::vector<geometry_msgs::msg::Point> & collision_points) const;

  bool checkCollisionWithCylinder(
    const tier4_autoware_utils::Polygon2d & vehicle_polygon, const PoseWithRange pose_with_range,
    const float radius, std::vector<geometry_msgs::msg::Point> & collision_points) const;

  bool checkCollisionWithBoundingBox(
    const tier4_autoware_utils::Polygon2d & vehicle_polygon, const PoseWithRange pose_with_range,
    const geometry_msgs::msg::Vector3 & dimension,
    std::vector<geometry_msgs::msg::Point> & collision_points) const;

  bool checkCollisionWithPolygon() const;

  std::vector<geometry_msgs::msg::Point> createBoundingBoxForRangedPoints(
    const PoseWithRange & pose_with_range, const float x_offset, const float y_offset) const;

  Trajectory trimTrajectoryFromSelfPose(
    const Trajectory & input, const geometry_msgs::msg::Pose & self_pose,
    const double trim_distance) const;

  Trajectory extendTrajectory(const Trajectory & input, const double extend_distance) const;

  TrajectoryPoint createExtendTrajectoryPoint(
    double extend_distance, const TrajectoryPoint & goal_point) const;

  size_t calcTrajectoryIndexByLength(
    const Trajectory & trajectory, const geometry_msgs::msg::Pose & current_pose,
    const double length) const;

  size_t calcTrajectoryIndexByLengthReverse(
    const Trajectory & trajectory, const geometry_msgs::msg::Point & src_point,
    const float target_length) const;

  boost::optional<geometry_msgs::msg::Pose> calcStopPoint(
    const boost::optional<DynamicObstacle> & dynamic_obstacle, const Trajectory & trajectory,
    const geometry_msgs::msg::Pose & current_pose, const float current_vel,
    const float current_acc) const;

  void insertStopPoint(
    const boost::optional<geometry_msgs::msg::Pose> stop_point,
    autoware_auto_planning_msgs::msg::PathWithLaneId & path);

  void insertVelocityWithApproaching(
    const boost::optional<DynamicObstacle> & dynamic_obstacle,
    const geometry_msgs::msg::Pose & current_pose, const float current_vel, const float current_acc,
    const Trajectory & trajectory, autoware_auto_planning_msgs::msg::PathWithLaneId & path);

  void insertApproachingVelocity(
    const DynamicObstacle & dynamic_obstacle, const geometry_msgs::msg::Pose & current_pose,
    const float approaching_vel, const float approach_margin, const Trajectory & trajectory,
    autoware_auto_planning_msgs::msg::PathWithLaneId & path);

  void addPointsBehindBase(
    const Trajectory & input_traj, const geometry_msgs::msg::Pose & current_pose,
    Trajectory & output_traj) const;

  Trajectory toTrajectoryMsg(
    const TrajectoryPoints & points, const std_msgs::msg::Header & header) const;

  boost::optional<Trajectory> applySmoother(
    const Trajectory & input_trajectory, const geometry_msgs::msg::Pose & current_pose,
    const double initial_vel, const double initial_acc) const;

  bool smoothVelocity(
    const TrajectoryPoints & input, const geometry_msgs::msg::Pose & current_pose,
    const double initial_vel, const double initial_acc, TrajectoryPoints & traj_smoothed) const;

  void applyMaxJerkLimit(
    const Trajectory & trajectory, const geometry_msgs::msg::Pose & current_pose,
    const float current_vel, const float current_acc,
    autoware_auto_planning_msgs::msg::PathWithLaneId & path);
};
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__DYNAMIC_OBSTACLE_STOP__SCENE_HPP_
