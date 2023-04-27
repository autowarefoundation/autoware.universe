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

#ifndef MODULARIZED_PARTICLE_FILTER__CORRECTION__ABST_CORRECTOR_HPP_
#define MODULARIZED_PARTICLE_FILTER__CORRECTION__ABST_CORRECTOR_HPP_

#include "modularized_particle_filter/common/mean.hpp"
#include "modularized_particle_filter/common/visualize.hpp"

#include <rclcpp/rclcpp.hpp>

#include <modularized_particle_filter_msgs/msg/particle_array.hpp>

#include <optional>

namespace yabloc
{
namespace modularized_particle_filter
{
class AbstCorrector : public rclcpp::Node
{
public:
  using Particle = modularized_particle_filter_msgs::msg::Particle;
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;

  AbstCorrector(const std::string & node_name);

protected:
  const float acceptable_max_delay_;  // [sec]
  const bool visualize_;
  const rclcpp::Logger logger_;

  rclcpp::Subscription<ParticleArray>::SharedPtr particle_sub_;
  rclcpp::Publisher<ParticleArray>::SharedPtr particle_pub_;
  std::list<ParticleArray> particle_array_buffer_;

  std::optional<ParticleArray> get_synchronized_particle_array(const rclcpp::Time & stamp);
  std::shared_ptr<ParticleVisualizer> visualizer_;

  void set_weighted_particle_array(const ParticleArray & particle_array);

private:
  void on_particle_array(const ParticleArray & particle_array);
};
}  // namespace modularized_particle_filter
}  // namespace yabloc

#endif  // MODULARIZED_PARTICLE_FILTER__CORRECTION__ABST_CORRECTOR_HPP_
