#include "pose_estimator_manager/switch_rule/ar_tag_position.hpp"

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
  const std::string ar_tag_node_name =
    "/localization/pose_estimator/ar_tag_based_localizer/ar_tag_based_localizer";

  impl_ = std::make_shared<Impl>();

  const std::string target_tag_ids_parameter_name = "target_tag_ids";

  const auto callback = [&](const std::shared_future<std::vector<rclcpp::Parameter>> & future) {
    RCLCPP_INFO_STREAM(logger_, "AsyncParameters callback");

    for (const auto & param : future.get()) {
      RCLCPP_INFO_STREAM(logger_, "param : " << param.get_name());
      // TODO(KYabuuchi): We cannot use target_tag_ids_parameter_name here?
      if (param.get_name() == "target_tag_ids") {
        impl_->target_tag_ids_ = param.as_string_array();
        RCLCPP_INFO_STREAM(logger_, "target tag ids: " << param.as_string_array().size());
      }
    }
  };
  impl_->params_tf_caster_ = rclcpp::AsyncParametersClient::make_shared(node, ar_tag_node_name);
  impl_->params_tf_caster_->wait_for_service();
  impl_->params_tf_caster_->get_parameters({target_tag_ids_parameter_name}, callback);
}

bool ArTagPosition::exist_ar_tag_around_ego(const geometry_msgs::msg::Point &) const
{
  RCLCPP_INFO_STREAM(
    logger_,
    "try to check availability: searching for " << impl_->target_tag_ids_.size() << " markers");

  double squared_distance_to_nearest_marker = std::numeric_limits<double>::max();

  for (const std::string & tag_id : impl_->target_tag_ids_) {
    RCLCPP_INFO_STREAM(logger_, "target tag id " << tag_id);
    const auto opt_transform = get_transform("base_link", "tag_" + tag_id);
    if (opt_transform.has_value()) {
      const auto t = opt_transform->transform.translation;
      const double squared_distance = t.x * t.x + t.x * t.x + t.x * t.z;
      squared_distance_to_nearest_marker =
        std::min(squared_distance, squared_distance_to_nearest_marker);
    }
  }

  RCLCPP_INFO_STREAM(logger_, "distance to nearest markers" << squared_distance_to_nearest_marker);
  return squared_distance_to_nearest_marker < (15 * 15);
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

std::string ArTagPosition::debug_string() const
{
  return {};
}

ArTagPosition::MarkerArray ArTagPosition::debug_marker_array() const
{
  return MarkerArray{};
}

}  // namespace multi_pose_estimator