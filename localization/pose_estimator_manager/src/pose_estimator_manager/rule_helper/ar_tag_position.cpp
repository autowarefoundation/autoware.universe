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

#include "pose_estimator_manager/rule_helper/ar_tag_position.hpp"

#include <Eigen/Core>
#include <landmark_manager/landmark_manager.hpp>
#include <rclcpp/parameter_client.hpp>

#include <algorithm>
#include <limits>
#include <vector>

namespace pose_estimator_manager::rule_helper
{
struct ArTagPosition::Impl
{
  std::vector<std::string> target_tag_ids;
  rclcpp::AsyncParametersClient::SharedPtr params_tf_caster;
  std::map<std::string, Pose> landmark_map;
};

ArTagPosition::ArTagPosition(rclcpp::Node * node) : logger_(node->get_logger())
{
  // The landmark_based_localizer has available landmarks IDs, and we can confirm the names of
  // these landmarks by inspecting the parameters held by the tag_based_localizer. Here we get the
  // ID of the landmark using AsyncParametersClient
  constexpr char target_tag_ids_parameter_name[] = "target_tag_ids";
  constexpr char ar_tag_node_name[] =
    "/localization/pose_estimator/ar_tag_based_localizer/ar_tag_based_localizer";

  const auto callback = [&, target_tag_ids_parameter_name](
                          const std::shared_future<std::vector<rclcpp::Parameter>> & future) {
    for (const auto & param : future.get()) {
      if (param.get_name() == target_tag_ids_parameter_name) {
        impl_->target_tag_ids = param.as_string_array();
        RCLCPP_INFO_STREAM(
          logger_, "AsyncParameters got " << param.as_string_array().size() << " ladnmark IDs");
      }
    }
  };

  impl_ = std::make_shared<Impl>();
  impl_->params_tf_caster = rclcpp::AsyncParametersClient::make_shared(node, ar_tag_node_name);
  impl_->params_tf_caster->wait_for_service();
  impl_->params_tf_caster->get_parameters({target_tag_ids_parameter_name}, callback);
}

void ArTagPosition::init(HADMapBin::ConstSharedPtr msg)
{
  vector_map_is_initialized_ = true;
  const std::vector<landmark_manager::Landmark> landmarks =
    landmark_manager::parse_landmarks(msg, "apriltag_16h5", logger_);

  // auto & landmark_map = impl_->landmark_map;
  // landmark_map.clear();
  // for (const landmark_manager::Landmark & landmark : landmarks) {
  //   landmark_map[landmark.id] = landmark.pose;
  // }
}

bool ArTagPosition::vector_map_initialized() const
{
  return vector_map_is_initialized_;
}

double ArTagPosition::distance_to_nearest_ar_tag_around_ego(
  const geometry_msgs::msg::Point & ego_position) const
{
  double distance_to_nearest_marker = std::numeric_limits<double>::max();
  const Eigen::Vector3d ego_vector(ego_position.x, ego_position.y, ego_position.z);

  for (const std::string & tag_id : impl_->target_tag_ids) {
    if (impl_->landmark_map.count(tag_id) == 0) {
      RCLCPP_INFO_STREAM(logger_, "tag_id(" << tag_id << ") is not in landmark_map_");
      continue;
    }

    const Pose & map_to_tag = impl_->landmark_map.at(tag_id);
    const auto & t = map_to_tag.position;
    const Eigen::Vector3d marker_vector(t.x, t.y, t.z);

    distance_to_nearest_marker =
      std::min((marker_vector - ego_vector).norm(), distance_to_nearest_marker);
  }

  return distance_to_nearest_marker;
}

}  // namespace pose_estimator_manager::rule_helper
