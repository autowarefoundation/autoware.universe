#ifndef MODULARIZED_PARTICLE_FILTER__CORRECTION__ABST_CORRECTOR_HPP_
#define MODULARIZED_PARTICLE_FILTER__CORRECTION__ABST_CORRECTOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <modularized_particle_filter_msgs/msg/particle_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <optional>

class AbstCorrector : public rclcpp::Node
{
public:
  using Particle = modularized_particle_filter_msgs::msg::Particle;
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  AbstCorrector(const std::string & node_name);

protected:
  rclcpp::Subscription<ParticleArray>::SharedPtr particle_sub_;
  rclcpp::Publisher<ParticleArray>::SharedPtr particle_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_array_;
  std::list<ParticleArray> particle_array_buffer_;

  std::optional<ParticleArray> getSyncronizedParticleArray(const rclcpp::Time & stamp);

  void setWeightedParticleArray(const ParticleArray & particle_array);

  void visualize(const ParticleArray & msg);

  const bool visualize_;

private:
  void particleArrayCallback(const ParticleArray & particle_array);
};

#endif  // MODULARIZED_PARTICLE_FILTER__CORRECTION__ABST_CORRECTOR_HPP_
