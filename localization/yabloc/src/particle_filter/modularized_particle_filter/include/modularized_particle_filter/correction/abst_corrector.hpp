#ifndef MODULARIZED_PARTICLE_FILTER__CORRECTION__ABST_CORRECTOR_HPP_
#define MODULARIZED_PARTICLE_FILTER__CORRECTION__ABST_CORRECTOR_HPP_

#include "modularized_particle_filter/common/mean.hpp"
#include "modularized_particle_filter/common/visualize.hpp"

#include <rclcpp/rclcpp.hpp>

#include <modularized_particle_filter_msgs/msg/particle_array.hpp>

#include <optional>

namespace modularized_particle_filter
{
class AbstCorrector : public rclcpp::Node
{
public:
  using Particle = modularized_particle_filter_msgs::msg::Particle;
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;

  AbstCorrector(const std::string & node_name);

protected:
  rclcpp::Subscription<ParticleArray>::SharedPtr particle_sub_;
  rclcpp::Publisher<ParticleArray>::SharedPtr particle_pub_;
  std::list<ParticleArray> particle_array_buffer_;

  std::optional<ParticleArray> getSyncronizedParticleArray(const rclcpp::Time & stamp);
  std::shared_ptr<ParticleVisualizer> visualizer_;

  void setWeightedParticleArray(const ParticleArray & particle_array);

  const bool visualize_;

private:
  void particleArrayCallback(const ParticleArray & particle_array);
};
}  // namespace modularized_particle_filter

#endif  // MODULARIZED_PARTICLE_FILTER__CORRECTION__ABST_CORRECTOR_HPP_
