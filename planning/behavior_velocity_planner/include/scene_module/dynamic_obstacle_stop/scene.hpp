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

#include <autoware_auto_planning_msgs/msg/path_point.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

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

#include <lanelet2_core/LaneletMap.h>
#include <tf2/LinearMath/Transform.h>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using dynamic_obstacle_stop_utils::DetectionAreaSize;
using dynamic_obstacle_stop_utils::PlannerParam;
using dynamic_obstacle_stop_utils::State;
using tier4_debug_msgs::msg::Float32Stamped;
using BasicPolygons2d = std::vector<lanelet::BasicPolygon2d>;

class DynamicObstacleStopModule : public SceneModuleInterface
{
public:
  DynamicObstacleStopModule(
    const int64_t module_id, const std::shared_ptr<const PlannerData> & planner_data,
    const PlannerParam & planner_param, const rclcpp::Logger logger,
    const std::shared_ptr<DynamicObstacleStopDebug> & debug_ptr,
    const rclcpp::Clock::SharedPtr clock);

  bool modifyPathVelocity(
    autoware_auto_planning_msgs::msg::PathWithLaneId * path,
    tier4_planning_msgs::msg::StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;

  void setPlannerParam(const PlannerParam & planner_param);

private:
  // Parameter
  PlannerParam planner_param_;

  // Variable
  State state_{State::GO};
  rclcpp::Time stop_time_;
  BasicPolygons2d partition_lanelets_;
  std::shared_ptr<DynamicObstacleStopDebug> debug_ptr_;

  // Function
  boost::optional<std::vector<DynamicObstacle>> createDynamicObstacles(
    const PredictedObjects & predicted_objects,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & points, const PathWithLaneId & path,
    const geometry_msgs::msg::Pose & current_pose, const std::string & detection_method) const;

  std::vector<DynamicObstacle> createDynamicObstaclesFromObjects(
    const PredictedObjects & predicted_objects) const;

  std::vector<DynamicObstacle> createDynamicObstaclesFromPoints(
    const pcl::PointCloud<pcl::PointXYZ> & obstacle_pointcloud, const PathWithLaneId & path) const;

  std::vector<DynamicObstacle> createDynamicObstaclesFromObjects(
    const PredictedObjects & predicted_objects, const PathWithLaneId & path) const;

  pcl::PointCloud<pcl::PointXYZ> applyVoxelGridFilter(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input_points) const;

  pcl::PointCloud<pcl::PointXYZ> extractObstaclePointsWithRectangle(
    const pcl::PointCloud<pcl::PointXYZ> & input_points,
    const geometry_msgs::msg::Pose & current_pose) const;

  void visualizeDetectionArea(const PathWithLaneId & smoothed_path) const;

  pcl::PointCloud<pcl::PointXYZ> pointsWithinPolygon(
    const std::vector<geometry_msgs::msg::Point> & polygon,
    const pcl::PointCloud<pcl::PointXYZ> & candidate_points) const;

  boost::optional<DynamicObstacle> detectCollision(
    const std::vector<DynamicObstacle> & dynamic_obstacles,
    const PathWithLaneId & path_points) const;

  float calcCollisionPositionOfVehicleSide(
    const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Pose & base_pose) const;

  std::vector<geometry_msgs::msg::Point> createVehiclePolygon(
    const geometry_msgs::msg::Pose & base_pose) const;

  std::vector<DynamicObstacle> checkCollisionWithObstacles(
    const std::vector<DynamicObstacle> & dynamic_obstacles,
    std::vector<geometry_msgs::msg::Point> poly, const float travel_time) const;

  boost::optional<DynamicObstacle> findNearestCollisionObstacle(
    const PathWithLaneId & path, const geometry_msgs::msg::Pose & base_pose,
    std::vector<DynamicObstacle> & dynamic_obstacles) const;

  boost::optional<geometry_msgs::msg::Pose> calcPredictedObstaclePose(
    const std::vector<DynamicObstacle::PredictedPath> & predicted_paths, const float travel_time,
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

  boost::optional<geometry_msgs::msg::Pose> calcStopPoint(
    const boost::optional<DynamicObstacle> & dynamic_obstacle, const PathWithLaneId & path,
    const geometry_msgs::msg::Pose & current_pose, const float current_vel,
    const float current_acc) const;

  void insertStopPoint(
    const boost::optional<geometry_msgs::msg::Pose> stop_point,
    autoware_auto_planning_msgs::msg::PathWithLaneId & path);

  void insertVelocityWithApproaching(
    const boost::optional<DynamicObstacle> & dynamic_obstacle,
    const geometry_msgs::msg::Pose & current_pose, const float current_vel, const float current_acc,
    const PathWithLaneId & resampled_path, PathWithLaneId & output_path);

  void insertApproachingVelocity(
    const DynamicObstacle & dynamic_obstacle, const geometry_msgs::msg::Pose & current_pose,
    const float approaching_vel, const float approach_margin, const PathWithLaneId & resampled_path,
    PathWithLaneId & output_path);

  void applyMaxJerkLimit(
    const geometry_msgs::msg::Pose & current_pose, const float current_vel, const float current_acc,
    PathWithLaneId & path) const;

  std::vector<DynamicObstacle> excludeObstaclesOutSideOfPartition(
    const std::vector<DynamicObstacle> & dynamic_obstacles, const PathWithLaneId & path,
    const geometry_msgs::msg::Pose & current_pose) const;

void publishDebugValue(
  const PathWithLaneId & path, const std::vector<DynamicObstacle> extracted_obstacles, const boost::optional<DynamicObstacle> & dynamic_obstacle,
  const geometry_msgs::msg::Pose & current_pose) const;
};
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__DYNAMIC_OBSTACLE_STOP__SCENE_HPP_
