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

#ifndef DYNAMIC_OBSTACLE_STOP_PLANNER_UTILS_HPP_
#define DYNAMIC_OBSTACLE_STOP_PLANNER_UTILS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
namespace dynamic_obstacle_stop_utils
{
namespace bg = boost::geometry;
using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using tier4_autoware_utils::Box2d;
using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Polygon2d;
using tier4_debug_msgs::msg::Float32Stamped;
using vehicle_info_util::VehicleInfo;
using PathPointsWithLaneId = std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>;

struct CommonParam
{
  double normal_min_jerk;  // min jerk limit for mild stop [m/sss]
  double normal_min_acc;   // min deceleration limit for mild stop [m/ss]
  double limit_min_jerk;   // min jerk limit [m/sss]
  double limit_min_acc;    // min deceleration limit [m/ss]
};

struct DynamicObstacleStopParam
{
  bool enable_dynamic_obstacle_stop;
  bool use_objects;
  bool use_predicted_path;
  double extend_distance;
  double stop_margin;
  double passing_margin;
  double stop_start_jerk_dec;
  double obstacle_velocity_kph;
  float detection_distance;
  float detection_span;
  float min_vel_ego_kmph;
  bool calc_collision_from_point;
};

struct VehicleParam
{
  float base_to_front;
  float base_to_rear;
  float width;
};

struct DetectionAreaSize
{
  float dist_ahead;
  float dist_behind;
  float dist_right;
  float dist_left;
};

struct ApproachingParam
{
  bool enable;
  float margin;
  float limit_vel_kmph;
  float stop_thresh;
  float stop_time_thresh;
  float dist_thresh;
};

struct DynamicObstacleParam
{
  float min_vel_kmph{0.0};
  float max_vel_kmph{5.0};

  // parameter to convert points to dynamic obstacle
  float diameter{0.1};  // [m]
  float height{2.0};    // [m]
  size_t path_size{20};
  float time_step{0.5};  // [sec]
};

struct SlowDownLimit
{
  bool enable;
  float max_jerk;
  float max_acc;
};

struct PlannerParam
{
  CommonParam common;
  DynamicObstacleStopParam dynamic_obstacle_stop;
  VehicleParam vehicle_param;
  DetectionAreaSize detection_area;
  ApproachingParam approaching;
  DynamicObstacleParam dynamic_obstacle;
  SlowDownLimit slow_down_limit;
};

struct TextWithPosition
{
  std::string text;
  geometry_msgs::msg::Point position;
};

// since we use the minimum and maximum velocity,
// define the PredictedPath without time_step
struct PredictedPath
{
  std::vector<geometry_msgs::msg::Pose> path;
  float confidence;
};

struct PoseWithRange
{
  geometry_msgs::msg::Pose pose_min;
  geometry_msgs::msg::Pose pose_max;
};

enum class State {
  GO = 0,
  APPROACH,
  STOP,
};

bool validCheckDecelPlan(
  const double v_end, const double a_end, const double v_target, const double a_target,
  const double v_margin, const double a_margin);

/**
 * @brief calculate distance until velocity is reached target velocity (TYPE1)
 * @param (v0) current velocity [m/s]
 * @param (vt) target velocity [m/s]
 * @param (a0) current acceleration [m/ss]
 * @param (am) minimum deceleration [m/ss]
 * @param (ja) maximum jerk [m/sss]
 * @param (jd) minimum jerk [m/sss]
 * @param (t_min) duration of constant deceleration [s]
 * @return moving distance until velocity is reached vt [m]
 * @detail TODO(Satoshi Ota)
 */
boost::optional<double> calcDecelDistPlanType1(
  const double v0, const double vt, const double a0, const double am, const double ja,
  const double jd, const double t_min);

/**
 * @brief calculate distance until velocity is reached target velocity (TYPE2)
 * @param (v0) current velocity [m/s]
 * @param (vt) target velocity [m/s]
 * @param (a0) current acceleration [m/ss]
 * @param (am) minimum deceleration [m/ss]
 * @param (ja) maximum jerk [m/sss]
 * @param (jd) minimum jerk [m/sss]
 * @return moving distance until velocity is reached vt [m]
 * @detail TODO(Satoshi Ota)
 */
boost::optional<double> calcDecelDistPlanType2(
  const double v0, const double vt, const double a0, const double ja, const double jd);

/**
 * @brief calculate distance until velocity is reached target velocity (TYPE3)
 * @param (v0) current velocity [m/s]
 * @param (vt) target velocity [m/s]
 * @param (a0) current acceleration [m/ss]
 * @param (ja) maximum jerk [m/sss]
 * @return moving distance until velocity is reached vt [m]
 * @detail TODO(Satoshi Ota)
 */
boost::optional<double> calcDecelDistPlanType3(
  const double v0, const double vt, const double a0, const double ja);

boost::optional<double> calcDecelDistWithJerkAndAccConstraints(
  const double current_vel, const double target_vel, const double current_acc, const double acc_min,
  const double jerk_acc, const double jerk_dec);

void applyMaximumVelocityLimit(const double max_vel_mps, Trajectory & trajectory);

Polygon2d createBoostPolyFromMsg(const std::vector<geometry_msgs::msg::Point> & input_poly);

std::uint8_t getHighestProbLabel(const std::vector<ObjectClassification> & classification);

std::vector<geometry_msgs::msg::Pose> getHighestConfidencePath(
  const std::vector<PredictedPath> & predicted_paths);

// apply linear interpolation to position
geometry_msgs::msg::Pose lerpByPose(
  const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2, const float t);

geometry_msgs::msg::Point findLongitudinalNearestPoint(
  const Trajectory & trajectory, const geometry_msgs::msg::Point & src_point,
  const std::vector<geometry_msgs::msg::Point> & target_points);

std::vector<geometry_msgs::msg::Point> findLateralSameSidePoints(
  const std::vector<geometry_msgs::msg::Point> & points, const geometry_msgs::msg::Pose & base_pose,
  const geometry_msgs::msg::Point & target_point);

TextWithPosition createDebugText(
  const std::string text, const geometry_msgs::msg::Pose pose, const float lateral_offset);

TextWithPosition createDebugText(const std::string text, const geometry_msgs::msg::Point position);

Trajectory convertPathToTrajectory(const autoware_auto_planning_msgs::msg::PathWithLaneId & path);

std::vector<geometry_msgs::msg::Point> toRosPoints(const PathPointsWithLaneId & path_points);

size_t findNearestIndex(
  const PathPointsWithLaneId & path_points, const geometry_msgs::msg::Point & point);

size_t findNearestSegmentIndex(
  const PathPointsWithLaneId & path_points, const geometry_msgs::msg::Point & point);

boost::optional<size_t> haveSamePoint(
  const PathPointsWithLaneId & path_points, const geometry_msgs::msg::Point & point);

bool isSamePoint(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2);

void insertPathVelocityFromIndexLimited(
  const size_t & start_idx, const float velocity_mps, PathPointsWithLaneId & path_points);

void insertPathVelocityFromIndex(
  const size_t & start_idx, const float velocity_mps, PathPointsWithLaneId & path_points);

boost::optional<size_t> findFirstStopPointIdx(PathPointsWithLaneId & path_points);
}  // namespace dynamic_obstacle_stop_utils
}  // namespace behavior_velocity_planner
#endif  // DYNAMIC_OBSTACLE_STOP_PLANNER_UTILS_HPP_