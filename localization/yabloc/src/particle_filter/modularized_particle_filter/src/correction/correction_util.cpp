#include "modularized_particle_filter/correction/correction_util.hpp"

std::optional<modularized_particle_filter_msgs::msg::ParticleArray> findSyncedParticles(
  boost::circular_buffer<modularized_particle_filter_msgs::msg::ParticleArray> circular_buffer,
  rclcpp::Time time)
{
  for (int i{1}; i < static_cast<int>(circular_buffer.size()); i++) {
    if (rclcpp::Time(circular_buffer[i].header.stamp) < time) {
      return circular_buffer[i];
    }
  }
  if (0 < static_cast<int>(circular_buffer.size())) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("modularized_particle_filter.correction_util"),
      "the sensor data is too old: "
        << rclcpp::Time(circular_buffer[static_cast<int>(circular_buffer.size()) - 1].header.stamp)
               .seconds() -
             time.seconds());
  }
  return std::nullopt;
}

std_msgs::msg::ColorRGBA computeColor(float value)
{
  float r = 1.0f, g = 1.0f, b = 1.0f;
  // clang-format off
    value = std::clamp(value, 0.0f, 1.0f);
    if (value < 0.25f) {
      r = 0; g = 4 * (value);
    } else if (value < 0.5f) {
      r = 0; b = 1 + 4 * (0.25f - value);
    } else if (value < 0.75f) {
      r = 4 * (value - 0.5f); b = 0;
    } else {
      g = 1 + 4 * (0.75f - value); b = 0;
    }
  // clang-format on

  std_msgs::msg::ColorRGBA rgba;
  rgba.r = r;
  rgba.g = g;
  rgba.b = b;
  rgba.a = 1.0f;
  return rgba;
}
