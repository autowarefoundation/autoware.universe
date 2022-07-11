#ifndef MODULARIZED_PARTICLE_FILTER__CORRECTION__CORRECTION_UTIL_HPP_
#define MODULARIZED_PARTICLE_FILTER__CORRECTION__CORRECTION_UTIL_HPP_

#include "rclcpp/rclcpp.hpp"

#include "modularized_particle_filter_msgs/msg/particle_array.hpp"
#include <std_msgs/msg/color_rgba.hpp>

#include <boost/circular_buffer.hpp>

#include <optional>

std::optional<modularized_particle_filter_msgs::msg::ParticleArray> findSyncedParticles(
  boost::circular_buffer<modularized_particle_filter_msgs::msg::ParticleArray> circular_buffer,
  rclcpp::Time time);

std_msgs::msg::ColorRGBA computeColor(float value);

#endif  // MODULARIZED_PARTICLE_FILTER__CORRECTION__CORRECTION_UTIL_HPP_
