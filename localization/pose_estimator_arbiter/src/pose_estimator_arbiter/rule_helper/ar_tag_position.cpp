// Copyright 2023 Autoware Foundation
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

#include "pose_estimator_arbiter/rule_helper/ar_tag_position.hpp"

#include <Eigen/Core>
#include <lanelet2_extension/localization/landmark.hpp>
#include <rclcpp/parameter_client.hpp>

#include <algorithm>
#include <limits>
#include <map>
#include <vector>

namespace pose_estimator_arbiter::rule_helper
{
struct ArTagPosition::Impl
{
  std::vector<std::string> target_tag_ids;
  rclcpp::AsyncParametersClient::SharedPtr params_tf_caster;
  std::optional<std::multimap<std::string, Eigen::Vector3d>> landmark_map{std::nullopt};
};

ArTagPosition::ArTagPosition(rclcpp::Node * node) : logger_(node->get_logger())
{
  // The landmark_based_localizer has available landmarks IDs, and we can confirm the names of
  // these landmarks by inspecting the parameters held by the tag_based_localizer. Here we get the
  // ID of the landmark using AsyncParametersClient
  constexpr char target_tag_ids_parameter_name[] = "target_tag_ids";
  constexpr char ar_tag_node_name[] = "/localization/pose_estimator/ar_tag_based_localizer";

  const auto callback = [&, target_tag_ids_parameter_name](
                          const std::shared_future<std::vector<rclcpp::Parameter>> & future) {
    for (const auto & param : future.get()) {
      if (param.get_name() == target_tag_ids_parameter_name) {
        impl_->target_tag_ids = param.as_string_array();
        RCLCPP_INFO_STREAM(
          logger_, "AsyncParameters got " << param.as_string_array().size() << " landmark IDs");
      }
    }
  };

  impl_ = std::make_shared<Impl>();
  impl_->params_tf_caster = rclcpp::AsyncParametersClient::make_shared(node, ar_tag_node_name);

  // Wait for the service to be available
  const std::chrono::seconds timeout_sec(1);
  while (!impl_->params_tf_caster->wait_for_service(timeout_sec)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service.");
      break;
    }
    RCLCPP_INFO(
      logger_, "waiting for node: %s, param: %s\n", ar_tag_node_name,
      target_tag_ids_parameter_name);
  }

  // Get the parameter
  impl_->params_tf_caster->get_parameters({target_tag_ids_parameter_name}, callback);
}

void ArTagPosition::init(HADMapBin::ConstSharedPtr msg)
{
  const std::vector<lanelet::Polygon3d> landmarks =
    lanelet::localization::parseLandmarkPolygons(msg, "apriltag_16h5");

  std::multimap<std::string, Eigen::Vector3d> landmark_map;
  for (const lanelet::Polygon3d & poly : landmarks) {
    // Get landmark_id
    const std::string landmark_id = poly.attributeOr("marker_id", "none");

    // Compute landmark center
    const auto & vertices = poly.basicPolygon();
    if (vertices.size() != 4) {
      continue;
    }

    const Eigen::Vector3d center = (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4.0;
    landmark_map.emplace(landmark_id, center);
  }

  // Store the landmark map
  impl_->landmark_map = landmark_map;
}

double ArTagPosition::distance_to_nearest_ar_tag_around_ego(
  const geometry_msgs::msg::Point & ego_position) const
{
  double distance_to_nearest_marker = std::numeric_limits<double>::max();

  // If landmark_map has not been initialized, return the maximum value of double
  if (impl_->landmark_map.has_value() == false) {
    RCLCPP_WARN_STREAM(logger_, "landmark_map has not been initialized");
    return distance_to_nearest_marker;
  }

  const Eigen::Vector3d ego_vector(ego_position.x, ego_position.y, ego_position.z);

  for (const std::string & tag_id : impl_->target_tag_ids) {
    // Basically, if a tag is included in the target, it should be also included in the map.
    if (impl_->landmark_map->count(tag_id) == 0) {
      RCLCPP_DEBUG_STREAM(logger_, "tag_id(" << tag_id << ") is not in landmark_map");
      continue;
    }

    // Find the nearest landmark among the landmarks with the same tag_id
    const auto range = impl_->landmark_map->equal_range(tag_id);
    for (auto itr = range.first; itr != range.second; ++itr) {
      const Eigen::Vector3d map_to_tag = itr->second;
      distance_to_nearest_marker =
        std::min((map_to_tag - ego_vector).norm(), distance_to_nearest_marker);
    }
  }

  return distance_to_nearest_marker;
}

}  // namespace pose_estimator_arbiter::rule_helper
