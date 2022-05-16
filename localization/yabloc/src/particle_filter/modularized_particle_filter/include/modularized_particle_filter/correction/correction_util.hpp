#ifndef MODULARIZED_PARTICLE_FILTER__CORRECTION__CORRECTION_UTIL_HPP_
#define MODULARIZED_PARTICLE_FILTER__CORRECTION__CORRECTION_UTIL_HPP_

#include <optional>

#include <boost/circular_buffer.hpp>

#include "rclcpp/rclcpp.hpp"

#include "modularized_particle_filter_msgs/msg/particle_array.hpp"

std::optional<modularized_particle_filter_msgs::msg::ParticleArray> findSyncedParticles(
  boost::circular_buffer<modularized_particle_filter_msgs::msg::ParticleArray>
  circular_buffer,
  rclcpp::Time time)
{
  for (int i{1}; i < static_cast<int>(circular_buffer.size()); i++) {
    if (rclcpp::Time(circular_buffer[i].header.stamp) < time) {
      return circular_buffer[i];
    }
  }
  if (0 < static_cast<int>(circular_buffer.size())) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger(
        "modularized_particle_filter.correction_util"), "the sensor data is too old: " <<
        rclcpp::Time(
        circular_buffer[static_cast<int>(circular_buffer.size()) - 1].header.stamp).seconds() -
        time.seconds());
  }
  return std::nullopt;
}

#endif // MODULARIZED_PARTICLE_FILTER__CORRECTION__CORRECTION_UTIL_HPP_
