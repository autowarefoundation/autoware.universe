// Copyright 2024 TIER IV, inc.
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

#ifndef MAP_BASED_PREDICTION__UTILS_HPP_
#define MAP_BASED_PREDICTION__UTILS_HPP_

#include <autoware/universe_utils/math/normalization.hpp>
#include <autoware/universe_utils/math/unit_conversion.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <tf2/utils.h>

#include <utility>
#include <vector>

namespace autoware::map_based_prediction
{
namespace utils
{

using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::TrackedObject;

/**
 * @brief calc absolute normalized yaw difference between lanelet and object
 *
 * @param object
 * @param lanelet
 * @return double
 */
double calcAbsYawDiffBetweenLaneletAndObject(
  const TrackedObject & object, const lanelet::ConstLanelet & lanelet)
{
  const double object_yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
  const double lane_yaw =
    lanelet::utils::getLaneletAngle(lanelet, object.kinematics.pose_with_covariance.pose.position);
  const double delta_yaw = object_yaw - lane_yaw;
  const double normalized_delta_yaw = autoware::universe_utils::normalizeRadian(delta_yaw);
  const double abs_norm_delta = std::fabs(normalized_delta_yaw);
  return abs_norm_delta;
}

bool withinRoadLanelet(
  const TrackedObject & object,
  const std::vector<std::pair<double, lanelet::Lanelet>> & surrounding_lanelets_with_dist,
  const bool use_yaw_information = false)
{
  for (const auto & [dist, lanelet] : surrounding_lanelets_with_dist) {
    if (lanelet.hasAttribute(lanelet::AttributeName::Subtype)) {
      lanelet::Attribute attr = lanelet.attribute(lanelet::AttributeName::Subtype);
      if (
        attr.value() == lanelet::AttributeValueString::Crosswalk ||
        attr.value() == lanelet::AttributeValueString::Walkway) {
        continue;
      }
    }

    constexpr float yaw_threshold = 0.6;
    bool within_lanelet = std::abs(dist) < 1e-5;
    if (use_yaw_information) {
      within_lanelet =
        within_lanelet && calcAbsYawDiffBetweenLaneletAndObject(object, lanelet) < yaw_threshold;
    }
    if (within_lanelet) {
      return true;
    }
  }

  return false;
}

bool withinRoadLanelet(
  const TrackedObject & object, const lanelet::LaneletMapPtr & lanelet_map_ptr,
  const bool use_yaw_information = false)
{
  const auto & obj_pos = object.kinematics.pose_with_covariance.pose.position;
  lanelet::BasicPoint2d search_point(obj_pos.x, obj_pos.y);
  // nearest lanelet
  constexpr double search_radius = 10.0;  // [m]
  const auto surrounding_lanelets_with_dist =
    lanelet::geometry::findWithin2d(lanelet_map_ptr->laneletLayer, search_point, search_radius);

  return withinRoadLanelet(object, surrounding_lanelets_with_dist, use_yaw_information);
}

/**
 * @brief change label for prediction
 *
 * @param label
 * @return ObjectClassification::_label_type
 */
ObjectClassification::_label_type changeLabelForPrediction(
  const ObjectClassification::_label_type & label, const TrackedObject & object,
  const lanelet::LaneletMapPtr & lanelet_map_ptr_)
{
  // for car like vehicle do not change labels
  switch (label) {
    case ObjectClassification::CAR:
    case ObjectClassification::BUS:
    case ObjectClassification::TRUCK:
    case ObjectClassification::TRAILER:
    case ObjectClassification::UNKNOWN:
      return label;

    case ObjectClassification::MOTORCYCLE:
    case ObjectClassification::BICYCLE: {  // if object is within road lanelet and satisfies yaw
                                           // constraints
      const bool within_road_lanelet = withinRoadLanelet(object, lanelet_map_ptr_, true);
      // if the object is within lanelet, do the same estimation with vehicle
      if (within_road_lanelet) return ObjectClassification::MOTORCYCLE;

      constexpr float high_speed_threshold =
        autoware::universe_utils::kmph2mps(25.0);  // High speed bicycle 25 km/h
      // calc abs speed from x and y velocity
      const double abs_speed = std::hypot(
        object.kinematics.twist_with_covariance.twist.linear.x,
        object.kinematics.twist_with_covariance.twist.linear.y);
      const bool high_speed_object = abs_speed > high_speed_threshold;
      // high speed object outside road lanelet will move like unknown object
      // return ObjectClassification::UNKNOWN; // temporary disabled
      if (high_speed_object) return label;  // Do nothing for now
      return ObjectClassification::BICYCLE;
    }

    case ObjectClassification::PEDESTRIAN: {
      const float max_velocity_for_human_mps =
        autoware::universe_utils::kmph2mps(25.0);  // Max human being motion speed is 25km/h
      const double abs_speed = std::hypot(
        object.kinematics.twist_with_covariance.twist.linear.x,
        object.kinematics.twist_with_covariance.twist.linear.y);
      const bool high_speed_object = abs_speed > max_velocity_for_human_mps;
      // fast, human-like object: like segway
      // return ObjectClassification::MOTORCYCLE;
      if (high_speed_object) return label;  // currently do nothing
      // fast human outside road lanelet will move like unknown object
      // return ObjectClassification::UNKNOWN;
      return label;
    }

    default:
      return label;
  }
}

}  // namespace utils

}  // namespace autoware::map_based_prediction

#endif  // MAP_BASED_PREDICTION__UTILS_HPP_
