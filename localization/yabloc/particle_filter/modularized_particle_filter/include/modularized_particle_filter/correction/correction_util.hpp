#ifndef MODULARIZED_PARTICLE_FILTER__CORRECTION__CORRECTION_UTIL_HPP_
#define MODULARIZED_PARTICLE_FILTER__CORRECTION__CORRECTION_UTIL_HPP_

#include "rclcpp/rclcpp.hpp"

#include "modularized_particle_filter_msgs/msg/particle_array.hpp"

#include <boost/circular_buffer.hpp>

#include <optional>

namespace pcdless
{
namespace modularized_particle_filter
{
std::optional<modularized_particle_filter_msgs::msg::ParticleArray> find_synced_particles(
  boost::circular_buffer<modularized_particle_filter_msgs::msg::ParticleArray> circular_buffer,
  rclcpp::Time time);
}
}  // namespace pcdless

#endif  // MODULARIZED_PARTICLE_FILTER__CORRECTION__CORRECTION_UTIL_HPP_