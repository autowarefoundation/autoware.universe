// Copyright 2024 Tier IV, Inc.
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

#ifndef OBJECT_MANAGER_HPP_
#define OBJECT_MANAGER_HPP_

#include <rclcpp/time.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <boost/unordered_map.hpp>
#include <boost/uuid/uuid.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <memory>
#include <optional>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace std
{
template <>
struct hash<unique_identifier_msgs::msg::UUID>
{
  size_t operator()(const unique_identifier_msgs::msg::UUID & uid) const
  {
    const auto & ids = uid.uuid;
    boost::uuids::uuid u = {ids[0], ids[1], ids[2],  ids[3],  ids[4],  ids[5],  ids[6],  ids[7],
                            ids[8], ids[9], ids[10], ids[11], ids[12], ids[13], ids[14], ids[15]};
    return boost::hash<boost::uuids::uuid>()(u);
  }
};
}  // namespace std

namespace behavior_velocity_planner
{

/**
 * @struct
 * @brief store collision information
 */
struct CollisionKnowledge
{
  rclcpp::Time stamp;
  enum LanePosition {
    FIRST,
    SECOND,
    ELSE,
  } expected_lane_position;
  std::vector<geometry_msgs::msg::Pose> path;
  std::pair<size_t, size_t>
    interval_position;                      //! possible collision interval position index on path
  std::pair<double, double> interval_time;  //! possible collision interval time(without TTC margin)
};

/**
 * @struct
 * @brief store collision information of object on the attention area
 */
class ObjectInfo
{
public:
  /**
   * @fn
   * @brief update observed_position, observed_position_arc_coords, predicted_path,
   * observed_velocity, attention_lanelet, stopline, dist_to_stopline
   */
  void update(
    std::optional<lanelet::ConstLanelet> attention_lanelet_opt,
    std::optional<lanelet::ConstLineString3d> stopline_opt,
    const autoware_auto_perception_msgs::msg::PredictedObject & predicted_object,
    const std::optional<CollisionKnowledge> & unsafe_knowledge_opt);

  /**
   * @fn
   * @brief find the estimated position of the object in the past
   */
  geometry_msgs::msg::Pose estimated_past_pose(const double past_duration) const;

  /**
   * @fn
   * @brief check if object can stop before stopline under the deceleration. return false if
   * stopline is null for conservative collision  checking
   */
  bool can_stop_before_stopline(const double brake_deceleration) const;

  /**
   * @fn
   * @brief check if object can stop before stopline within the overshoot margin. return false if
   * stopline is null for conservative collision checking
   */
  bool can_stop_before_ego_lane(
    const double brake_deceleration, const double tolerable_overshoot,
    lanelet::ConstLanelet ego_lane) const;

private:
  geometry_msgs::msg::Pose observed_position;
  lanelet::ArcCoordinates observed_position_arc_coords;  //! arc coordinate on the attention lanelet
  double observed_velocity;
  std::optional<lanelet::ConstLanelet> attention_lanelet_opt{
    std::nullopt};  //! null if the object in intersection_area but not in attention_area
  std::optional<lanelet::ConstLineString3d> stopline_opt{std::nullopt};
  std::optional<double> dist_to_stopline_opt{std::nullopt};
  std::optional<CollisionKnowledge> unsafe_decision_knowledge{
    std::nullopt};  //! store the information of if judged as UNSAFE
  /**
   * @fn
   * @brief calculate/update the distance to corresponding stopline
   */
  void calc_dist_to_stopline();
};

/**
 * @struct
 * @brief
 */
class ObjectInfoManager
{
public:
  std::unordered_map<unique_identifier_msgs::msg::UUID, std::shared_ptr<ObjectInfo>> objects_info;
  std::vector<std::shared_ptr<ObjectInfo>> all_objects;
};
}  // namespace behavior_velocity_planner

#endif  // OBJECT_MANAGER_HPP_
