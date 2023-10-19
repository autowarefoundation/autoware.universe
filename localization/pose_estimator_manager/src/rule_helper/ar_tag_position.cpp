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
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_client.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace multi_pose_estimator
{
struct ArTagPosition::Impl
{
  std::vector<std::string> target_tag_ids_;
  rclcpp::AsyncParametersClient::SharedPtr params_tf_caster_;
};

ArTagPosition::ArTagPosition(rclcpp::Node * node)
: logger_(node->get_logger()), tf2_buffer_(node->get_clock()), tf2_listener_(tf2_buffer_)
{
  // The landmark_based_localizer broadcasts landmarks using TF, and we can confirm the names of
  // these landmarks by inspecting the parameters held by the tag_based_localizer. Here we get the
  // ID of the landmark using AsyncParametersClient
  constexpr char target_tag_ids_parameter_name[] = "target_tag_ids";
  constexpr char ar_tag_node_name[] =
    "/localization/pose_estimator/ar_tag_based_localizer/ar_tag_based_localizer";

  const auto callback = [&, target_tag_ids_parameter_name](
                          const std::shared_future<std::vector<rclcpp::Parameter>> & future) {
    for (const auto & param : future.get()) {
      if (param.get_name() == target_tag_ids_parameter_name) {
        impl_->target_tag_ids_ = param.as_string_array();
        RCLCPP_INFO_STREAM(
          logger_, "AsyncParameters got " << param.as_string_array().size() << " ladnmark IDs");
      }
    }
  };

  impl_ = std::make_shared<Impl>();
  impl_->params_tf_caster_ = rclcpp::AsyncParametersClient::make_shared(node, ar_tag_node_name);
  impl_->params_tf_caster_->wait_for_service();
  impl_->params_tf_caster_->get_parameters({target_tag_ids_parameter_name}, callback);
}

double ArTagPosition::distance_to_nearest_ar_tag_around_ego(
  const geometry_msgs::msg::Point & ego_position) const
{
  double distance_to_nearest_marker = std::numeric_limits<double>::max();
  const Eigen::Vector3d ego_vector(ego_position.x, ego_position.y, ego_position.z);

  for (const std::string & tag_id : impl_->target_tag_ids_) {
    const auto opt_transform = get_transform("map", "tag_" + tag_id);
    if (opt_transform.has_value()) {
      const auto t = opt_transform->transform.translation;
      const Eigen::Vector3d marker_vector(t.x, t.y, t.z);

      distance_to_nearest_marker =
        std::min((marker_vector - ego_vector).norm(), distance_to_nearest_marker);
    }
  }

  return distance_to_nearest_marker;
}

std::optional<ArTagPosition::TransformStamped> ArTagPosition::get_transform(
  const std::string & target_frame, const std::string & source_frame) const
{
  TransformStamped transform_stamped;
  try {
    transform_stamped = tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(logger_, "%s", ex.what());
    RCLCPP_ERROR(logger_, "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());
    return std::nullopt;
  }
  return transform_stamped;
}
}  // namespace multi_pose_estimator
