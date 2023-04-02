#pragma once
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

namespace pcdless::initializer
{
class MarkerModule
{
public:
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  MarkerModule(rclcpp::Node * node);

  void publish_marker(
    const std::vector<int> scores, const std::vector<float> angles,
    const Eigen::Vector3f & position);

private:
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_;
};
}  // namespace pcdless::initializer