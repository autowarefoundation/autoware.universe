// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__YABLOC_PARTICLE_FILTER__COMMON__VISUALIZE_HPP_
#define AUTOWARE__YABLOC_PARTICLE_FILTER__COMMON__VISUALIZE_HPP_

#include <autoware_yabloc_particle_filter/msg/particle_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace autoware::yabloc::modularized_particle_filter
{
class ParticleVisualizer
{
public:
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Particle = autoware_yabloc_particle_filter::msg::Particle;
  using ParticleArray = autoware_yabloc_particle_filter::msg::ParticleArray;

  explicit ParticleVisualizer(rclcpp::Node & node);
  void publish(const autoware_yabloc_particle_filter::msg::ParticleArray & msg);

private:
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_array_;
};
}  // namespace autoware::yabloc::modularized_particle_filter

#endif  // AUTOWARE__YABLOC_PARTICLE_FILTER__COMMON__VISUALIZE_HPP_
