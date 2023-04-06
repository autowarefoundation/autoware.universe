#include "camera_pose_initializer/marker_module.hpp"

#include <pcdless_common/color.hpp>

namespace pcdless::initializer
{

MarkerModule::MarkerModule(rclcpp::Node * node)
{
  pub_marker_ = node->create_publisher<MarkerArray>("init/candidates", 10);
}

void MarkerModule::publish_marker(
  const std::vector<float> scores, const std::vector<float> angles,
  const Eigen::Vector3f & position)
{
  const int N = scores.size();
  auto minmax = std::minmax_element(scores.begin(), scores.end());
  auto normalize = [minmax](int score) -> float {
    return static_cast<float>(score - *minmax.first) /
           static_cast<float>(*minmax.second - *minmax.first);
  };

  MarkerArray array;
  for (int i = 0; i < N; i++) {
    Marker marker;
    marker.header.frame_id = "map";
    marker.type = Marker::ARROW;
    marker.id = i;
    marker.ns = "arrow";
    marker.color = common::color_scale::rainbow(normalize(scores.at(i)));
    marker.color.a = 0.5;

    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();

    const float rad = angles.at(i);
    marker.pose.orientation.w = std::cos(rad / 2.f);
    marker.pose.orientation.z = std::sin(rad / 2.f);

    marker.scale.x = 2.0;  // arrow length
    marker.scale.y = 0.2;  // arrow width
    marker.scale.z = 0.3;  // arrow height

    array.markers.push_back(marker);
  }
  pub_marker_->publish(array);
}

}  // namespace pcdless::initializer