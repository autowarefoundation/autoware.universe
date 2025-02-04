// Copyright 2021 Tier IV, Inc.
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

#ifndef MAP_BASED_PREDICTION__PATH_GENERATOR_HPP_
#define MAP_BASED_PREDICTION__PATH_GENERATOR_HPP_

#include "map_based_prediction/data_structure.hpp"

#include <Eigen/Eigen>
#include <autoware/universe_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::map_based_prediction
{
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjectKinematics;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::PredictedPath;
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjectKinematics;
using autoware_perception_msgs::msg::TrackedObjects;

struct FrenetPoint
{
  double s;
  double d;
  float s_vel;
  float d_vel;
  float s_acc;
  float d_acc;
};

struct CrosswalkEdgePoints
{
  Eigen::Vector2d front_center_point;
  Eigen::Vector2d front_right_point;
  Eigen::Vector2d front_left_point;
  Eigen::Vector2d back_center_point;
  Eigen::Vector2d back_right_point;
  Eigen::Vector2d back_left_point;

  void swap()
  {
    const Eigen::Vector2d tmp_center_point = front_center_point;
    const Eigen::Vector2d tmp_right_point = front_right_point;
    const Eigen::Vector2d tmp_left_point = front_left_point;

    front_center_point = back_center_point;
    front_right_point = back_right_point;
    front_left_point = back_left_point;

    back_center_point = tmp_center_point;
    back_right_point = tmp_right_point;
    back_left_point = tmp_left_point;
  }
};

using FrenetPath = std::vector<FrenetPoint>;

class PathGenerator
{
public:
  explicit PathGenerator(const double sampling_time_interval);
  PathGenerator(const double sampling_time_interval, const double min_crosswalk_user_velocity);

  void setTimeKeeper(std::shared_ptr<autoware::universe_utils::TimeKeeper> time_keeper_ptr);

  PredictedPath generatePathForNonVehicleObject(
    const TrackedObject & object, const double duration) const;

  PredictedPath generatePathForLowSpeedVehicle(
    const TrackedObject & object, const double duration) const;

  PredictedPath generatePathForOffLaneVehicle(
    const TrackedObject & object, const double duration) const;

  PredictedPath generatePathForOnLaneVehicle(
    const TrackedObject & object, const PosePath & ref_path, const double duration,
    const double lateral_duration, const double path_width = 0.0,
    const double speed_limit = 0.0) const;

  PredictedPath generatePathForCrosswalkUser(
    const TrackedObject & object, const CrosswalkEdgePoints & reachable_crosswalk,
    const double duration) const;

  PredictedPath generatePathToTargetPoint(
    const TrackedObject & object, const Eigen::Vector2d & point) const;

  void setUseVehicleAcceleration(const bool use_vehicle_acceleration)
  {
    use_vehicle_acceleration_ = use_vehicle_acceleration;
  }

  void setAccelerationHalfLife(const double acceleration_exponential_half_life)
  {
    acceleration_exponential_half_life_ = acceleration_exponential_half_life;
  }

private:
  // Parameters
  double sampling_time_interval_;
  double min_crosswalk_user_velocity_;
  bool use_vehicle_acceleration_;
  double acceleration_exponential_half_life_;

  std::shared_ptr<autoware::universe_utils::TimeKeeper> time_keeper_;

  // Member functions
  PredictedPath generateStraightPath(const TrackedObject & object, const double duration) const;

  PredictedPath generatePolynomialPath(
    const TrackedObject & object, const PosePath & ref_path, const double duration,
    const double lateral_duration, const double path_width, const double backlash_width,
    const double speed_limit = 0.0) const;

  FrenetPath generateFrenetPath(
    const FrenetPoint & current_point, const FrenetPoint & target_point, const double max_length,
    const double duration, const double lateral_duration) const;
  Eigen::Vector3d calcLatCoefficients(
    const FrenetPoint & current_point, const FrenetPoint & target_point, const double T) const;
  Eigen::Vector2d calcLonCoefficients(
    const FrenetPoint & current_point, const FrenetPoint & target_point, const double T) const;

  std::vector<double> interpolationLerp(
    const std::vector<double> & base_keys, const std::vector<double> & base_values,
    const std::vector<double> & query_keys) const;
  std::vector<tf2::Quaternion> interpolationLerp(
    const std::vector<double> & base_keys, const std::vector<tf2::Quaternion> & base_values,
    const std::vector<double> & query_keys) const;

  PosePath interpolateReferencePath(
    const PosePath & base_path, const FrenetPath & frenet_predicted_path) const;

  PredictedPath convertToPredictedPath(
    const TrackedObject & object, const FrenetPath & frenet_predicted_path,
    const PosePath & ref_path) const;

  FrenetPoint getFrenetPoint(
    const TrackedObject & object, const geometry_msgs::msg::Pose & ref_pose, const double duration,
    const double speed_limit = 0.0) const;
};
}  // namespace autoware::map_based_prediction

#endif  // MAP_BASED_PREDICTION__PATH_GENERATOR_HPP_
