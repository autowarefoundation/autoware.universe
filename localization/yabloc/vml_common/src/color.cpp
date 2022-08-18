#include "vml_common/color.hpp"

namespace vml_common
{
Color toJet(float value)
{
  // clang-format off
  float r = 1.0f, g = 1.0f, b = 1.0f;
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
  return {r, g, b};
}

std_msgs::msg::ColorRGBA color(float red, float green, float blue, float alpha)
{
  std_msgs::msg::ColorRGBA rgba;
  rgba.r = red;
  rgba.g = green;
  rgba.b = blue;
  rgba.a = alpha;
  return rgba;
}
}  // namespace vml_common
