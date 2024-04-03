// Copyright 2024 TIER IV, Inc.
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

#include "tensorrt_mtr/node.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>

#include <tf2/utils.h>

#include <utility>

namespace
{
/**
 * @brief Get the Lanelet subtype name.
 *
 * @param lanelet
 * @return std::string
 */
std::string getLaneletSubtype(const lanelet::Lanelet & lanelet)
{
  const auto & subtype = lanelet.attribute("subtype").as<std::string>();
  if (subtype) {
    return subtype.get();
  } else {
    return "";
  }
}

/**
 * @brief Get the LineString type name.
 *
 * @param linestring
 * @return std::string
 */
std::string getLineStringType(const lanelet::ConstLineString3d & linestring)
{
  const auto & type = linestring.attribute("subtype").as<std::string>();
  if (type) {
    return type.get();
  } else {
    return "";
  }
}

/**
 * @brief Get the LineString subtype name.
 *
 * @param linestring
 * @return std::string
 */
std::string getLineStringSubtype(const lanelet::ConstLineString3d & linestring)
{
  const auto & subtype = linestring.attribute("subtype").as<std::string>();
  if (subtype) {
    return subtype.get();
  } else {
    return "";
  }
}

/**
 * @brief Check whether lanelet has an attribute named `turn_direction`.
 *
 * @param lanelet
 * @return true
 * @return false
 */
bool isTurnIntersection(const lanelet::Lanelet & lanelet)
{
  if (lanelet.hasAttribute("turn_direction")) {
    return true;
  } else {
    return false;
  }
}

/**
 * @brief Insert source LanePoints into target LanePoints.
 *
 * @param src Source LanePoints.
 * @param dst Target LanePoints.
 */
void insertPoints(const std::vector<LanePoint> & src, std::vector<LanePoint> dst)
{
  dst.insert(dst.end(), src.cbegin(), src.cend());
}

AgentState trackedObjectToAgentState(const TrackedObject & object, const bool is_valid)
{
  const auto & pose = object.kinematics.pose_with_covariance.pose;
  const auto & twist = object.kinematics.twist_with_covariance.twist;
  const auto & accel = object.kinematics.accel_with_covariance.accel;
  const auto & dimensions = object.shape.dimensions;
  const auto yaw = tf2::getYaw(pose.orientation);
  const auto valid = is_valid ? 1.0f : 0.0f;

  return {pose.position.x,
          pose.position.y,
          pose.position.z,
          dimensions.x,
          dimensions.y,
          dimensions.z,
          yaw,
          twist.linear.x,
          twist.linear.y,
          accel.linear.x,
          accel.linear.y,
          valid};
}
}  // namespace

namespace trt_mtr
{
MTRNode::MTRNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("tensorrt_mtr_node", node_options), polyline_type_map_(this)
{
  // TODO(ktro2828)
}

void MTRNode::callback(const TrackedObjects::ConstSharedPtr object_msg)
{
  // TODO(ktro2828)
  if (!lanelet_map_ptr_ || !polyline_ptr_) {
    return;
  }

  const auto current_time = rclcpp::Time(object_msg->header.stamp).seconds();
  removeAncientAgentHistory(current_time, object_msg);
  updateAgentHistory(current_time, object_msg);

  std::vector<AgentHistory> histories;
  histories.reserve(agent_history_map_.size());
  for (const auto & [_, history] : agent_history_map_) {
    histories.emplace_back(history);
  }
  // TODO(ktro2828): set AgentData
}

void MTRNode::onMap(const HADMapBin::ConstSharedPtr map_msg)
{
  RCLCPP_DEBUG(get_logger(), "[TensorRT MTR]: Start loading lanelet");
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *map_msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  RCLCPP_DEBUG(get_logger(), "[TensorRT MTR]: Finish loading lanelet");

  RCLCPP_DEBUG(get_logger(), "[TensorRT MTR]: Start converting lanelet to polyline");
  if (convertLaneletToPolyline()) {
    RCLCPP_DEBUG(get_logger(), "[TensorRT MTR]: Success to convert lanelet to polyline");
  } else {
    RCLCPP_WARN(get_logger(), "[TensorRT MTR]: Fail to convert lanelet to polyline");
  }
}

bool MTRNode::convertLaneletToPolyline()
{
  std::vector<LanePoint> all_points;
  all_points.reserve(1000);
  for (const auto & lanelet : lanelet_map_ptr_->laneletLayer) {
    const auto lanelet_id = lanelet.id();
    const auto lanelet_subtype = getLaneletSubtype(lanelet);
    if (
      lanelet_subtype == "road" || lanelet_subtype == "highway" ||
      lanelet_subtype == "road_shoulder" || lanelet_subtype == "pedestrian_lane" ||
      lanelet_subtype == "bicycle_lane" || lanelet_subtype == "bicycle_lane" ||
      lanelet_subtype == "walkway") {
      if (
        lanelet_subtype == "road" || lanelet_subtype == "highway" ||
        lanelet_subtype == "road_shoulder") {
        const auto & type_id = polyline_type_map_.getTypeID(lanelet_subtype);
        auto points = getLanePointFromLineString(lanelet.centerline3d(), type_id);
        insertPoints(points, all_points);
      }
      if (!isTurnIntersection(lanelet)) {
        const auto & left = lanelet.leftBound3d();
        const auto left_type = getLineStringType(left);
        if (left_type == "line_thin" || left_type == "line_thick") {
          const auto left_subtype = getLineStringSubtype(left);
          const auto & type_id = polyline_type_map_.getTypeID(left_subtype);
          auto points = getLanePointFromLineString(left, type_id);
          insertPoints(points, all_points);
        }
        const auto & right = lanelet.rightBound3d();
        const auto right_type = getLineStringType(right);
        if (right_type == "line_thin" || right_type == "line_thick") {
          const auto right_subtype = getLineStringSubtype(right);
          const auto & type_id = polyline_type_map_.getTypeID(right_subtype);
          auto points = getLanePointFromLineString(right, type_id);
          insertPoints(points, all_points);
        }
      }
    } else if (lanelet_subtype == "crosswalk") {
      const auto & type_id = polyline_type_map_.getTypeID(lanelet_subtype);
      auto points = getLanePointFromPolygon(lanelet.polygon3d(), type_id);
      insertPoints(points, all_points);
    }
  }

  for (const auto & linestring : lanelet_map_ptr_->lineStringLayer) {
    const auto linestring_type = getLineStringType(linestring);
    if (linestring_type != "road_boarder" && linestring_type != "traffic_sign") {
      continue;
    }
    const auto & type_id = polyline_type_map_.getTypeID(linestring_type);
    auto points = getLanePointFromLineString(linestring, type_id);
    insertPoints(points, all_points);
  }

  if (all_points.size() == 0) {
    return false;
  } else {
    // TODO(ktro2828): load from model config
    polyline_ptr_ = std::make_shared<PolylineData>(all_points, 798, 20, 1.0f);
    return true;
  }
}

void MTRNode::removeAncientAgentHistory(
  const float current_time, const TrackedObjects::ConstSharedPtr objects_msg)
{
  // TODO(ktro2828): use ego info
  for (const auto & object : objects_msg->objects) {
    const auto & object_id = tier4_autoware_utils::toHexString(object.object_id);
    if (agent_history_map_.count(object) == 0) {
      continue;
    }

    const auto & history = agent_history_map_.at(object_id);
    if (history.is_ancient(current_time, 1.0)) {  // TODO(ktro2828): use parameter
      agent_history_map_.erase(object_id);
    }
  }
}

void MTRNode::updateAgentHistory(
  const float current_time, const TrackedObjects::ConstSharedPtr objects_msg)
{
  // TODO(ktro2828): use ego info
  std::vector<std::string> observed_ids;
  for (const auto & object : objects_msg->objects) {
    const auto & object_id = tier4_autoware_utils::toHexString(object.object_id);
    observed_ids.emplace_back(object_id);
    state = trackedObjectToAgentState(object, true);
    if (agent_history_map_.count(object_id) == 0) {
      history = AgentHistory(object_id, 10);
      history.update(current_time, state);
      agent_history_map_.emplace(object_id, history);
    } else {
      agent_history_map_.at(object_id).update(current_time, state);
    }
  }

  // update unobserved histories with empty
  for (auto & [object_id, history] : agent_history_map_) {
    if (std::find(observed_ids.cbegin(), observed_ids.cend(), object_id) != observed_ids.cend()) {
      continue;
    }
    history.update_empty();
  }
}

bool MTRNode::predictFuture()
{
  // TODO(ktro2828)
}

}  // namespace trt_mtr

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(trt_mtr::MTRNode);
